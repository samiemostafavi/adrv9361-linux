#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
//#include <dt-bindings/dma/axi-dmac.h>
#include <linux/irq.h>
#include <asm/io.h>
#include <linux/fs.h>		//required for fops
#include <linux/uaccess.h>	//required for 'cpoy_from_user' and 'copy_to_user'
#include <linux/signal.h>	//required for kernel-to-userspace signals
#include <linux/string.h>

#include "axi_timer.h"

#define _OPEN_SYS_ITOA_EXT
#define TIMER_BASE	0x42800000

#define TIMER_TCSR0	0x42800000	// address of timer 0's control and status register
#define TIMER_TLR0	0x42800004	// address of timer 0's load register
#define TIMER_TCR0	0x42800008	// address of timer 0's counter register

#define TIMER_TCSR1	0x42800010	// address of timer 1's control and status register
#define TIMER_TLR1	0x42800014	// address of timer 1's load register
#define TIMER_TCR1	0x42800018	// address of timer 1's counter register

#define RESET_VAL0	0x00000000	// load value 0
#define RESET_VAL1	0x00000000	// load value 1

#define DEVICE_NAME     "axitimer"	// device name
#define SUCCESS         0		// success return value

#define BUF_LEN 	80		// max buffer length
#define CLK_FREQ 	100000000	// AXI CLock: 166.66 MHz

// Round division
#define ROUND_DIV(a, b)		((a + (b / 2)) / b)

// Convert clock frequency in hertz to period in nano seconds
#define HZ_TO_NS(Hz)		ROUND_DIV(1000000000, Hz)

// JUST FOR TEST REMOVE LATER
#define GPIO0_DATA0     0x41210000      // Enable signal

//unsigned long *pSYSCALL_Virtual;	// base address
static int Device_Open = 0;		// device status

static char msg[BUF_LEN];		// the msg the device will give when asked
static char *msg_Ptr;

unsigned long *pTIMER_TCSR0;		// pointer to timer 0 control and status register
unsigned long *pTIMER_TLR0;		// pointer to timer 0 load register
unsigned long *pTIMER_TCR0;		// pointer to timer 0 counter register

unsigned long *pTIMER_TCSR1;		// pointer to timer 0 control and status register
unsigned long *pTIMER_TLR1;		// pointer to timer 0 load register
unsigned long *pTIMER_TCR1;		// pointer to timer 0 counter register

// JUST FOR TEST REMOVE LATER
unsigned char* ptGPIO0_DATA0; // Enable signal

int majorNum;
dev_t devNo;  // Major and Minor device numbers combined into 32 bits
struct class *pClass;  // class_create will set this

// write routine (called when write() is used in user-space)
ssize_t axitimer_write(struct file *flip, const char *buf, size_t length, loff_t *offset)
{
	ssize_t ret;
	//printk("[DBG][drivers][timer][axi_timer][axitimer_write]");
	
	if ((ret = copy_from_user(msg, buf, length)) != 0)		// read buffer from user space
		return -EFAULT;					// return error if it failed
	
	//printk("[DBG][drivers][timer][axi_timer][axitimer_write][inp: %s]",msg);
	return ret;
}

// JUST FOR TEST REMOVE LATER
bool tmp = true;

// read routine (called when read() is used in user-space)
ssize_t axitimer_read(struct file *flip, char *buf, size_t length, loff_t *offset)
{
	uint32_t upper_val_tmp;
	uint32_t upper_val;
	uint32_t lower_val;
	uint64_t timerval;

	// read timer value
	upper_val_tmp = ioread32(pTIMER_TCR1);			// Read the upper velue
	lower_val = ioread32(pTIMER_TCR0);			// Read the lower value

	upper_val = ioread32(pTIMER_TCR1);			// Read the upper velue again
	if(upper_val != upper_val_tmp)				// If the value is different, read lower part again
		lower_val = ioread32(pTIMER_TCR0);		// Read the lower velue again

	timerval = (uint64_t) upper_val << 32 | lower_val;	// Combine 2 

	printk("[DBG][drivers][timer][axi_timer][axitimer_read][cycles: %llu]",timerval);
	//printk("[DBG][drivers][timer][axi_timer][axitimer_read][upper cycles: %u]",upper_val);
	//printk("[DBG][drivers][timer][axi_timer][axitimer_read][lower cycles: %u]",lower_val);
	
	// JUST FOR TEST REMOVE LATER		
	if(tmp)
	{
		iowrite32(0x0,ptGPIO0_DATA0); // disable everything
		tmp = !tmp;
	}
	else
	{
		iowrite32(0xf,ptGPIO0_DATA0); // disable everything
		tmp = !tmp;
	}
		

	/*ssize_t ret = snprintf (msg, BUF_LEN, "%llu", timerval);
	if (copy_to_user(buf, &msg, length) != 0)		// send counter value
		return -EFAULT;
	else 
		return ret;*/
	return 0;
}

// open routine (called when a device opens /dev/axitimer)
static int axitimer_open(struct inode *inode, struct file *filp)
{

	//printk("[DBG][drivers][timer][axi_timer][axitimer_open]");
	if (Device_Open)
	{
		printk("[ERR][drivers][timer][axi_timer][axitimer_open][One device is already created]");
		return -EBUSY;					// return with an error
	}
	Device_Open++;						// 'open' device
	msg_Ptr = msg;						
	try_module_get(THIS_MODULE);
	return SUCCESS;
}

// close routine (called whne a device closes /dev/axitimer)
static int axitimer_close(struct inode *inode, struct file *file)
{
	Device_Open--;						// 'close' device

	module_put(THIS_MODULE);
	return 0;
}

// device init and file operations
struct file_operations axitimer_fops = {
	.owner = THIS_MODULE,
	.read = axitimer_read,		// read()
	.write = axitimer_write,	// write()
	.open = axitimer_open,		// open()
	.release = axitimer_close,	// close()
};


// init module      
static int __init mod_init(void)
{
	struct device *pDev;
	uint32_t ControlStatusReg;

	printk("[DBG][drivers][timer][axi_timer][mod_init][Init axitimer module]");
	
	pTIMER_TCSR0 = ioremap_nocache(TIMER_TCSR0,0x4);	// map timer 0 control and status register
	pTIMER_TLR0 = ioremap_nocache(TIMER_TLR0,0x4);		// map timer 0 load register
	pTIMER_TCR0 = ioremap_nocache(TIMER_TCR0,0x4);		// map timer 0 count register

	pTIMER_TCSR1 = ioremap_nocache(TIMER_TCSR1,0x4);	// map timer 1 control and status register
	pTIMER_TLR1 = ioremap_nocache(TIMER_TLR1,0x4);		// map timer 1 load register
	pTIMER_TCR1 = ioremap_nocache(TIMER_TCR1,0x4);		// map timer 1 count register
	
	// JUST FOR TEST REMOVE LATER
	ptGPIO0_DATA0 = ioremap_nocache(GPIO0_DATA0,0x1); // Enable signal

	// Register character device
  	majorNum = register_chrdev(0,DEVICE_NAME, &axitimer_fops);
  	if (majorNum < 0) {
    		printk(KERN_ALERT "Error: Could not register axitimer device: %d\n", majorNum);
    		return majorNum;
  	}
	devNo = MKDEV(majorNum, 0);  // Create a dev_t, 32 bit version of numbers
	
	printk("[DBG][drivers][timer][axi_timer][mod_init][majorNum: %d]",majorNum);

  	// Create /sys/class/axitimer in preparation of creating /dev/axitimer
  	pClass = class_create(THIS_MODULE, DEVICE_NAME);
  	if (IS_ERR(pClass)) {
    		printk("Error: Could not create axitimer class");
    		unregister_chrdev_region(devNo, 1);
    		return -1;
  	}

  	// Create /dev/axitimer for this char dev
  	if (IS_ERR(pDev = device_create(pClass, NULL, devNo, NULL, DEVICE_NAME))) {
    		printk(KERN_WARNING "axi_timer.ko can't create device /dev/axitimer\n");
    		class_destroy(pClass);
    		unregister_chrdev_region(devNo, 1);
    		return -1;
  	}

	printk("[DBG][drivers][timer][axi_timer][mod_init][Dev path is created]");
	
	/*
	 *  (Re-)initialzes both timer counters which aren't started already.
	 *
	 */
	iowrite32(0x00000000,pTIMER_TCSR0);             // Clear the timer enable bits
	iowrite32(XTC_CSR_LOAD_MASK,pTIMER_TCSR0);
	iowrite32(0x00000000,pTIMER_TCSR0);             // Clear the timer enable bits
	
	iowrite32(0x00000000,pTIMER_TCSR1);             // Clear the timer enable bits
	iowrite32(XTC_CSR_LOAD_MASK,pTIMER_TCSR1);
	iowrite32(0x00000000,pTIMER_TCSR1);             // Clear the timer enable bits

	/*
	 * Set a reset value for the timer counter such that it will expire
	 * eariler than letting it roll over from 0, the reset value is loaded
	 * into the timer counter when it is started
	 */
        iowrite32(RESET_VAL0,pTIMER_TLR0);                // Write the initial value in the lower 32 bit timer register
	iowrite32(RESET_VAL1,pTIMER_TLR1);                // Write the initial value in the higher 32 bit timer register


	/*
	 * Enable auto reload mode such that the timer counter will reload
	 * itself automatically and continue repeatedly, without this option
	 * it would expire once only and set the Cascade mode.
	 */
	iowrite32(XTC_CSR_AUTO_RELOAD_MASK | 
		  XTC_CSR_CASC_MASK,
		  pTIMER_TCSR0);             		
	
	
	/*
	 * Reset the timer counters such that it's incrementing by default
	 */
	// reset timer 1
	ControlStatusReg = ioread32(pTIMER_TCSR0);
	iowrite32(ControlStatusReg | XTC_CSR_LOAD_MASK,pTIMER_TCSR0);
	iowrite32(ControlStatusReg,pTIMER_TCSR0);

	// reset timer 2
	ControlStatusReg = ioread32(pTIMER_TCSR1);
	iowrite32(ControlStatusReg | XTC_CSR_LOAD_MASK,pTIMER_TCSR1);
	iowrite32(ControlStatusReg,pTIMER_TCSR1);


	/*
	 * Start the timer counter 0 such that it's incrementing by default,
	 * then wait for it to timeout a number of times.
	 */
	// start timer 1
	ControlStatusReg = ioread32(pTIMER_TCSR0);
	iowrite32(XTC_CSR_LOAD_MASK,pTIMER_TCSR0);
	iowrite32(ControlStatusReg | XTC_CSR_ENABLE_TMR_MASK ,pTIMER_TCSR0);

	printk("[DBG][drivers][timer][axi_timer][mod_init][64 bit timer is enabled]");

	return SUCCESS;
} 

// exit module
static void __exit mod_exit(void)  		
{
	iounmap(pTIMER_TCSR0);				// unregister timer hardware
	iounmap(pTIMER_TLR0);
	iounmap(pTIMER_TCR0);

	device_destroy(pClass, devNo);  		// Remove the /dev/axitimer
  	class_destroy(pClass);  			// Remove class /sys/class/axitimer
  	unregister_chrdev(majorNum, DEVICE_NAME);  	// Unregister the device
	
	printk("[DBG][drivers][timer][axi_timer][axitimer_exit]");
}

static const struct of_device_id my_match_table[] = {
     {
             .compatible = "axitimer_driver",
     },
};


MODULE_DEVICE_TABLE(of, my_match_table);

module_init(mod_init);
module_exit(mod_exit);

MODULE_AUTHOR ("Seyed Samie Mostafavi");
MODULE_DESCRIPTION("Driver for the Xilinx AXI Timer IP core.");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("custom:axitimer");
