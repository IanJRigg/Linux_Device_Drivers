#include <linux/module.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>

#define DEVICE_NAME "blink_led"
#define CLASS_NAME "blink_led"

#define LED_GPIO 21
#define GPIO_OFFSET 512

static struct gpio_desc* led;

static dev_t dev;
static struct class* dev_class;
static struct cdev cdev;

#define LED_ON_CHAR '1'
#define LED_OFF_CHAR '0'
#define LED_ON_INT 1
#define LED_OFF_INT 0

/**
 * blink_led_open() - Open the device.
 * @inode: Pointer to the inode structure.
 * @file: Pointer to the file structure.
 *
 * This function is intended to be a pass-through. No special logic has been
 * identified at this time.
 *
 * Return: Always 0
 */
static int blink_led_open(struct inode* inode, struct file* file)
{
    return 0;
}

/**
 * blink_led_release() - close the device.
 * @inode: Pointer to the inode structure.
 * @file: Pointer to the file structure.
 *
 * This function is intended to be a pass-through. No special logic has been
 * identified at this time.
 *
 * Return: 0 on success, negative error code on failure.
 */
static int blink_led_release(struct inode* inode, struct file* file)
{
    return 0;
}

/**
 * blink_led_read() - Read from the device
 * @file: Pointer to the file structure.
 * @buffer: Pointer to the user space buffer where the data will be read into
 * @length: The amount of data being requested.
 * @offset: The offset following completion of this command. (Must be 0)
 *
 * Queries the state of the GPIO pin that is driving the LED. The status,
 * either high or low, is converted to a character and copied into the
 * provided user-space buffer.
 *
 * Return: The number of bytes read on success, negative error code on failure.
 */
static ssize_t blink_led_read(struct file* file_pointer,
                              char __user * buffer,
                              size_t length,
                              loff_t* offset)
{
    if(*offset != 0)
    {
        return -ESPIPE;
    }

    const int gpio_status = gpiod_get_value(led);

    char status = (gpio_status == LED_ON_INT) ? LED_ON_CHAR : LED_OFF_CHAR;
    int copied_byte_count = copy_to_user(buffer, &status, 1);
    if(copied_byte_count == 1)
    {
        pr_warn("blink_led: could not copy status to userspace\n");
        return copied_byte_count;
    }

    *offset += 1;

    pr_info("Reading %d from the LED circuit\n", 0);
    return 1;
}

/**
 * blink_led_write() - Write to the device.
 * @file: Pointer to the file structure.
 * @buffer: Pointer to the user space buffer where the data will be written to
 * @length: The amount of data available for writing.
 * @offset: The offset following completion of this command. (Should be 0)
 *
 * Provided the input is either '1' or '0', the data is converted to an integer
 * representation of HIGH or LOW, and writes that to the configured GPIO pin.
 *
 * Return: 1 if the GPIO was updated, otherwise a negative error code
 */
static ssize_t blink_led_write(struct file* file_pointer, const char __user * buffer, size_t length, loff_t* offset)
{
    if(*offset != 0)
    {
        return -ESPIPE;
    }

    char status;
    int ret = copy_from_user(&status, buffer, 1);
    if(ret != 0)
    {
        pr_err("blink_led: failed to copy from user buffer\n");
        return -EFAULT;
    }

    if((status != LED_ON_CHAR) && (status != LED_OFF_CHAR))
    {
        pr_err("blink_led: invalid status provided: %d\n", status);
        return -EINVAL;
    }

    const int gpio_status = (status == LED_ON_CHAR) ? LED_ON_INT : LED_OFF_INT;

    gpiod_set_value(led, gpio_status);

    *offset += 1;

    pr_info("Writing %d to the LED circuit\n", gpio_status);
    return 1;
}

static struct file_operations fops = 
{
    .owner = THIS_MODULE,
    .read = blink_led_read,
    .write = blink_led_write,
    .open = blink_led_open,
    .release = blink_led_release,
};

/**
 * blink_led_init() - Entry point for the driver
 *
 * It allocates a device number, associates a set of file operations, registers
 * the device as a class, and initializes the GPIO to the LOW state. 
 *
 * Return: 0 on success, negative error code on failure.
 */
static int __init blink_led_init(void)
{
    int ret;

    ret = alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME);
    if(ret < 0)
    {
        pr_err("blink_led: Cannot allocate Major/Minor device numbers\n");
        return ret;
    }

    cdev_init(&cdev, &fops);
    ret = cdev_add(&cdev, dev, 1);
    if(ret < 0)
    {
        pr_err("blink_led: Cannot register the file operations\n");
        goto failed_before_class_create;
    }

    dev_class = class_create(CLASS_NAME);
    if(IS_ERR(dev_class))
    {
        pr_err("blink_led: Cannot create the class\n");
        goto failed_before_class_create;
    }

    if(IS_ERR(device_create(dev_class, NULL, dev, NULL, DEVICE_NAME)))
    {
        pr_err("blink_led: Cannot create the device\n");
        goto failed_after_class_create;
    }

    led = gpio_to_desc(LED_GPIO + GPIO_OFFSET);
    if(!led)
    {
        pr_err("blink_led: Error accessing pin %d\n", LED_GPIO);
        goto failed_after_device_create;
    }

    ret = gpiod_direction_output(led, LED_OFF_INT);
    if(ret)
    {
        pr_err("blink_led: Error setting pin 21 to output\n");
        goto failed_after_device_create;
    }

    pr_info("blink_led: driver has loaded. Major: %d, Minor: %d\n", MAJOR(dev), MINOR(dev));
    return 0;

failed_after_device_create:
    device_destroy(dev_class, dev);
failed_after_class_create:
    class_destroy(dev_class);
failed_before_class_create:
    cdev_del(&cdev);
    unregister_chrdev_region(dev, 1);
    return ret;
}

/**
 * blink_led_exit() - Exit point for the driver
 *
 * Destroys the device, class, and file operations association, and unregisters
 * the major and minor device numbers.
 */
static void __exit blink_led_exit(void)
{
    device_destroy(dev_class, dev);
    class_destroy(dev_class);
    cdev_del(&cdev);
    unregister_chrdev_region(dev, 1);
    pr_info("blink_led: driver has been removed\n");
}

module_init(blink_led_init);
module_exit(blink_led_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ian Rigg");
MODULE_DESCRIPTION("Basic On/Off controls for a basic LED circuit driven through a GPIO");
MODULE_VERSION("1.0");