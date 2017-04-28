/* This is a linux device driver for ADXL345 accelerometer module using raspberry pi,
   On the basis of tilting of the module, corresponding readings ( x- , y-, z- axis )
   are presented at the output */ 


#include <linux/kernel.h>      
#include <linux/module.h>      
#include <linux/fs.h>
#include <asm/uaccess.h>       
#include <linux/device.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/gpio.h>                
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/cdev.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h> 

#define DEVICE_NAME "adxl"
#define MAJOR_NO 242
#define i2c_address 0x53

//Register definitions
#define REG_INT_ENABLE 0x2E // INT_ENABLE register address
#define REG_INT_SOURCE 0x30 // source of interrupts
#define REG_DATAX0 0x32    // reading FIFO registers
#define REG_DATAX1 0x33    // x axis data 1
#define REG_DATAY0 0x34	   // y axis data 0
#define REG_DATAY1 0x35    // y axis data 1
#define REG_DATAZ0 0x36    // z axis data 0
#define REG_DATAZ1 0x37    // z axis data 1
#define REG_DEVID 0x00  //device id,access to accelerometer is verified by reading DEVID registe
#define REG_DATA_FORMAT 0x31  //data format control
#define REG_POWER_CTL 0x2D    // Power control register
#define REG_INT_MAP 0x2F      // Interrupt mapping control

#define INT_GPIO 27

struct class *cl;
static unsigned int irqNumber;
static struct i2c_client *device;
struct i2c_adapter * adxl_adap;
static dev_t first;
static struct cdev c_dev;
static u8 reading[3];
static void work_handler(struct work_struct *w);
static struct workqueue_struct *wq = 0;
static DECLARE_WORK(work, work_handler);


static int adxl34x_read(struct i2c_client *client, u8 reg)
{
	int ret;
	ret = i2c_smbus_read_byte_data(client, reg); // function used to fetch data from accelerometer module
	if (ret < 0)
		dev_err(&client->dev,
			"Unable to read register, %d\n", ret);

	return (u8)ret;
}
/*static int adxl34x_smbus_read(struct device *dev, unsigned char reg)
{
	struct i2c_client *client = to_i2c_client(dev);

	return i2c_smbus_read_byte_data(client, reg);
}*/

static int adxl34x_write(struct i2c_client *client, u8 reg, u8 data)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, data);
	if (ret < 0)
		dev_err(&client->dev,"can not write register, returned %d\n", ret);

	return ret;
}

/* file read operation - When a user space application does a read operation on the
   driver, device_read() is called.It copies the data from the adxl_data array containing 
   the reading of 3 coordinates in the kernel space to userspace using copy_to_user() and 
   returns the number of bytes copied.*/

static ssize_t device_read(struct file *f, char __user *buf, size_t len, loff_t *off) {	

	printk(KERN_INFO "Driver read()\n");

	if (*off == 0)
	{
		if (copy_to_user(buf, &reading,3) != 0)
		{
			printk(KERN_INFO "Driver read: Inside if\n");
			return -EFAULT;
		}
		else 
		{
			return 3;
		}
	}
	else
		return 0;
}


static int adxl34x_open(struct inode *i, struct file *f)
{
	  printk(KERN_INFO "Driver: open()\n");
	    return 0;
}
static int adxl34x_close(struct inode *i, struct file *f)
{
	  printk(KERN_INFO "Driver: close()\n");
	    return 0;
}

static ssize_t device_write(struct file *f, const char __user *buf,
		   size_t len, loff_t *off)
{
	  printk(KERN_INFO "Driver: write()\n");
	    return len;
}

static struct file_operations my_dev_ops = {.read = device_read,
					    .owner = THIS_MODULE,
					    .open = adxl34x_open,
				            .release = adxl34x_close,
					    .write = device_write};


/* Issuing ISR - As soon the interrupt comes to denote the availability of new 
   set of data, the interrupt subroutine is executed. Here it issues a workqueue(wq) 
   to read the data from the  accelerometer. This workerqueue constitutes the Bottom Half
   (BH) of the interrupt.*/
static irq_handler_t adxl_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs) 
{
	static unsigned int intCount = 0;
	intCount++;
	printk(KERN_INFO "ADXL: interrupt_handler(),Count=%d\n",intCount);

	if (wq)
		queue_work(wq, &work);

	return (irq_handler_t) IRQ_HANDLED;
}

/* As soon as the ISR is issued workqueue will be executed and it will fetch  
   data from accelerometer module using smbus_read() */

static void work_handler(struct work_struct *w) 
{
	printk(KERN_INFO "ADXL345: Inside_work_handler\n");

	reading[0] = adxl34x_read(device, REG_DATAX1); 
	reading[1] = adxl34x_read(device, REG_DATAY1); 
	reading[2] = adxl34x_read(device, REG_DATAZ1);
}

static int __init adxl34x_init(void)
{
	int result = 0;
	u8 dataread,temp; 
        printk(KERN_INFO "Welcome to ADXL345 Accelerometer!\n");

       /* dynamic alllocation of node */
        if (alloc_chrdev_region(&first, 0, 1, DEVICE_NAME) < 0) {
        printk(KERN_DEBUG "Can't register device\n");
        return -1;
        }

	if (IS_ERR(cl = class_create(THIS_MODULE, "chardrv"))){
			unregister_chrdev_region(first, 1);
			}
	/* Create a driver at /dev/adxl */
	if (IS_ERR(device_create(cl, NULL, first, NULL, "adxl"))) {
		class_destroy(cl);
		unregister_chrdev_region(first,1);
	}

	cdev_init(&c_dev,&my_dev_ops);
	if (cdev_add(&c_dev, first,1) < 0)
	{
	device_destroy(cl, first);
	class_destroy(cl);
	unregister_chrdev_region(first,1);
	return -1;
	}
        

	/* code below aquires the i2c bus*/ 

     	adxl_adap = i2c_get_adapter(1); // 1 means i2c-1 bus
	if (!(device = i2c_new_dummy(adxl_adap, i2c_address))){
		printk(KERN_INFO "Couldn't acquire i2c slave");
		unregister_chrdev(MAJOR_NO, DEVICE_NAME);
		device_destroy(cl, first);
		class_destroy(cl);
		return -1;
	}
		
	dataread = adxl34x_read(device,0x00);
	if (dataread == (0b11100101)) {
		printk("Accelerometer detected, value = %d\r\n",dataread);
	}

	//Setting Data format to be left justified
	temp = adxl34x_read(device,REG_DATA_FORMAT);
	temp = temp | (1<<2);
	adxl34x_write(device, REG_DATA_FORMAT, temp);
	
	//Enabling the interrupt
	
	
	temp = adxl34x_read(device,REG_INT_ENABLE);
	temp = temp | (1<<7);
	adxl34x_write(device, REG_INT_ENABLE, temp);

	
	
	//Start measurement
	temp = adxl34x_read(device,REG_POWER_CTL);
	temp = temp | (1<<3);
	adxl34x_write(device, REG_POWER_CTL, temp);

	//Interrupt config

	if (!wq)
		wq = create_singlethread_workqueue("mykmod");

	irqNumber = gpio_to_irq(INT_GPIO);               /* linux gpio number is passed to gpio_to_irq() 
							    to get unique IRQ no.for that gpio.*/ 
	printk(KERN_INFO"Irq number is %d",irqNumber);

	result = request_irq(irqNumber, (irq_handler_t) adxl_irq_handler,
			IRQF_TRIGGER_RISING,
			"ADXL_INT",
			NULL);
	printk(KERN_INFO "ADXL: The interrupt request result is: %d\n", result);

	reading[0] = adxl34x_read(device, REG_DATAX1);
	reading[1] = adxl34x_read(device, REG_DATAY1);
	reading[2] = adxl34x_read(device, REG_DATAZ1);

	return 0;
}
 

static void __exit adxl34x_exit(void)
{
        printk(KERN_INFO " Removing ADXL345 module");
 	
	free_irq(irqNumber,NULL); // releasing thr irq no. assigned
	destroy_workqueue(wq);	  //releasing workqueue
	i2c_unregister_device(device);
        cdev_del(&c_dev);
 	device_destroy(cl, first);
	class_destroy(cl);
	unregister_chrdev_region(first, 1);
 
}
module_init(adxl34x_init);
module_exit(adxl34x_exit);

MODULE_DESCRIPTION("ADXL_DRIVER");
MODULE_AUTHOR("Sanchit");
MODULE_LICENSE("GPL");
