/******************************************************************************
  Copyright (C), 2011-2021, Hisilicon Tech. Co., Ltd.
 ******************************************************************************/
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <linux/pagemap.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/wait.h>


#define DEVICE_NAME	"irqkey"
#define REG_WRITE(addr,value)  ((*(volatile unsigned int *)(addr)) = (value))
#define REG_READ(Addr)  (*(volatile unsigned int *)(Addr))
#define SEL_xDCS()  do{ REG_WRITE(gpio3_6_val_addr, 0x00); REG_WRITE(gpio5_7_val_addr, 0x80); }while(0)
#define SEL_xCS()   do{ REG_WRITE(gpio3_6_val_addr, 0x40); REG_WRITE(gpio5_7_val_addr, 0x00); }while(0)
#define SEL_NULL()  do{ REG_WRITE(gpio3_6_val_addr, 0x40); REG_WRITE(gpio5_7_val_addr, 0x80); }while(0)
#define SEL_ALL()  do{ REG_WRITE(gpio3_6_val_addr, 0x00); REG_WRITE(gpio5_7_val_addr, 0x00); }while(0)


#define VS_MODE_xCS     '1'
#define VS_MODE_xDCS    '2'
#define VS_MODE_NULL    '3'
#define VS_MODE_ALL    '0'


static struct class *irqkey_class;
static struct device *irqkey_device;
static DECLARE_WAIT_QUEUE_HEAD(irqkey_waitq);
static volatile int ev_press = 0;
static struct fasync_struct *irqkey_async;

//  定义结构体类型，由它把按钮中断的信息综合起来
struct irq_pin_desc {
    int irq;/*中断号*/
    int pin;/*中断标志寄存器，有中断产生时为1，无中断时为0*/
    int number;/*编号*/
    char *name;/*名称*/
};

// vs_xCS   GPIO5_7
// vs_xDCS  GPIO3_6
// DREQ     GPIO1_6
static struct irq_pin_desc irqkey_irqs[] = {
    { 8,2,1,"DREQ" },
};
static unsigned int gpio1_virtual_addr = 0;
static unsigned int gpio3_6_val_addr = 0;
static unsigned int gpio5_7_val_addr = 0;
static unsigned int reg_virtual_addr = 0;


static irqreturn_t irqkey_handler(int irq, void *dev_id)
{
    int down = 0x40 & REG_READ(gpio1_virtual_addr +0x0414); // gpio_ris
    REG_WRITE(gpio1_virtual_addr +0x041C, 0x40);    // clear irq flag
    mdelay(5);
    if(down != 0x40)
    {
        ev_press = 1;
        wake_up_interruptible(&irqkey_waitq);	// wake up thread
        printk("irqkey !down\n");
    }
    else
        printk("irqkey down\n");

    REG_WRITE(gpio1_virtual_addr +0x0410, 0x40);    // IE enable key interrupt
    kill_fasync(&irqkey_async, SIGIO, POLL_IN);
    return IRQ_RETVAL(IRQ_HANDLED);
}

static void virtual_addr_map(void)
{
    reg_virtual_addr = (unsigned int)ioremap_nocache(0x200f0000,0x4000);
    if(!reg_virtual_addr)
    {
        printk("0x200f0000 ioremap addr failed !\n");
        return;
    }
    gpio1_virtual_addr = (unsigned int)ioremap_nocache(0x20150000,0x4000);
    gpio3_6_val_addr = (unsigned int)ioremap_nocache(0x20170000 +0x100, 4);
    gpio5_7_val_addr = (unsigned int)ioremap_nocache(0x20190000 +0x200, 4);
    if(!gpio1_virtual_addr)
    {
        printk("GPIO1 0x20150000 ioremap addr failed !\n");
        return;
    }
}

static void virtual_addr_unmap(void)
{
    iounmap((void*)gpio1_virtual_addr);
    iounmap((void*)gpio3_6_val_addr);
    iounmap((void*)gpio5_7_val_addr);
    iounmap((void*)reg_virtual_addr);
}


static void irqkey_pin_cfg(void)
{
    REG_WRITE(reg_virtual_addr + 0x0094,0x0);   // reg37管脚复用配置gpio1_6
    REG_WRITE(gpio1_virtual_addr + 0x0400,0x00);    // dir input
    REG_WRITE(gpio1_virtual_addr + 0x0404,0x00);    // is边沿触发中断
    REG_WRITE(gpio1_virtual_addr + 0x0408,0x00);    // ibe single edge interrupt
    REG_WRITE(gpio1_virtual_addr + 0x040c,0x00);    // iev 低电平触发
    REG_WRITE(gpio1_virtual_addr + 0x041c,0xff);    // ic 清除中断
    REG_WRITE(gpio1_virtual_addr + 0x0410,0x40);    // ie 启用中断
    printk("irqkey pin config finish.\n");
}

void VS_HD_Reset(void)
{/*
    unsigned char retry=0;
	VS_RST=0;
	delay_ms(20);
	VS_XDCS=1;//取消数据传输
	VS_XCS=1;//取消数据传输
	VS_RST=1;
	while(VS_DQ==0&&retry<200)//等待DREQ为高
	{
		retry++;
		delay_us(50);
	};
	delay_ms(20);
	if(retry>=200)  return 1;
	else return 0;*/
}

static int irqkey_open(struct inode *inode ,struct file *file)
{
    int err;  // interrupt register return value;
    virtual_addr_map();
    irqkey_pin_cfg();
//    VS_HD_Reset();

    SEL_xCS();
    mdelay(5);
    SEL_xDCS();
    mdelay(5);
    SEL_NULL();

    // register irq
    err = request_irq(31,irqkey_handler,IRQF_SHARED,"DREQ",(void *)&irqkey_irqs);
    if(err)/*如果注册中断失败，则释放已经成功注册的中断*/
    {
        printk(DEVICE_NAME" request_irq 31 busy!\n");
        return -EBUSY;
    }
    ev_press = 1;
    printk(DEVICE_NAME" open success!\n");
    return 0;
}

static int irqkey_release(struct inode *inode ,struct file *filp)
{
      free_irq(31, (void*)&irqkey_irqs);
      virtual_addr_unmap();
      printk(DEVICE_NAME " release!\n");
      return 0;
}

static ssize_t irqkey_read(struct file *filp, char *buff, size_t count, loff_t *ppos)
{
      buff[0] = REG_READ(gpio1_virtual_addr +0x100);   // gpio1_6 for dreq
 //     printk("dreq(gpio1_6) = %#x\n", buff[0]);
      return count;
}

static ssize_t irqkey_write(struct file *filp,const char *buff, size_t count, loff_t *ppos)
{
 //   int i;

 /*     if(buff[0] == VS_MODE_xCS)
        SEL_xCS();
      else if(buff[0] == VS_MODE_xDCS)
        SEL_xDCS(); */

      switch(buff[0])
      {
            case VS_MODE_NULL:  SEL_NULL(); break;
            case VS_MODE_ALL:   SEL_ALL();  break;
            case VS_MODE_xCS:   SEL_xCS();  break;
            case VS_MODE_xDCS:  SEL_xDCS(); break;
            default: SEL_NULL();    break;
      }

#if 0
      printk("user write:");
      for(i=0;i<count;i++)
        printk(" %#x", buff[i]);
      printk("\n");
#endif
      return count;
}

static unsigned int irqkey_poll(struct file *file, poll_table *wait)
{
    unsigned int mask = 0;
    poll_wait(file, &irqkey_waitq, wait);
    if(ev_press)
        mask |= POLLIN|POLLRDNORM;

    printk(DEVICE_NAME "irqkey_poll mask=%#x\n", mask);
    return mask;
}

static int irqkey_fasync(int fd, struct file *filp, int on)
{
    printk("irqkey_fasync\n");
    return fasync_helper(fd, filp, on, &irqkey_async);
}

static struct file_operations irqkey_fops = {
      .owner = THIS_MODULE,
      .read = irqkey_read,
      .write = irqkey_write,
      .poll = irqkey_poll,
      .fasync = irqkey_fasync,
      .open = irqkey_open,
      .release = irqkey_release,
};

static int major;
static int __init irqkey_init(void)
{
    major = register_chrdev(0, DEVICE_NAME, &irqkey_fops);
    if(major < 0)
    {
       printk(KERN_EMERG DEVICE_NAME "irqkey: irqkey_init failed with %d.\n", major);
       return major;
    }
    irqkey_class = class_create(THIS_MODULE, DEVICE_NAME);
    irqkey_device = device_create(irqkey_class, NULL, MKDEV(major,0), NULL, DEVICE_NAME);	// /dev/irqkey

    printk(DEVICE_NAME " initialized, major = %d\n", major);
    return 0;
}

static void __exit irqkey_exit(void)
{
    unregister_chrdev(major, DEVICE_NAME);
    device_unregister(irqkey_device);
    class_destroy(irqkey_class);
    printk(DEVICE_NAME " removed!\n");
    return;
}

module_init(irqkey_init);
module_exit(irqkey_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("czzn");

