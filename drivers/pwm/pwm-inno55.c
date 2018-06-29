#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/device.h>
#include <linux/ioctl.h>
#include <linux/parport.h>
#include <linux/ctype.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/major.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

#include <asm/io.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/seq_file.h>

#define MAGIC_NUMBER   'k'
#define SET_FREQ_FAN0   _IO(MAGIC_NUMBER, 0)
#define SET_DUTY_FAN0   _IO(MAGIC_NUMBER, 1)
#define SET_FREQ_FAN1   _IO(MAGIC_NUMBER, 2)
#define SET_DUTY_FAN1   _IO(MAGIC_NUMBER, 3)


#define PWM_FAN_MAJOR      (0)
#define PWM_FAN_DEVNAME    "pwm_fan_devnode"
#define PWM_FAN_CLASSNAME  "pwm_fan_classnode"
#define PWM_FAN_DRIVERNAME "pwmdrv_inno"
#define PWM_FAN_PROCNAME   "pwm_fan_procname"
//#define PROC_PWM_SIZE       128

#define PWM_FAN0_FREQ      "fan0_freq"
#define PWM_FAN0_DUTY      "fan0_duty"
#define PWM_FAN1_FREQ      "fan1_freq"
#define PWM_FAN1_DUTY      "fan1_duty"

struct class    *pwm_fan_class;
static dev_t    pwm_fan_devno;
struct device   *pwm_fan_device;
static void     *pMemMapIO;

struct proc_dir_entry *mydir = NULL;


static int nOpenFlag = 0;

static int pwm_open(struct inode *inode, struct file *file)
{
    if(nOpenFlag)
    {
        return -EBUSY;
    }
    nOpenFlag++;

    *((unsigned int *)(pMemMapIO + (0x1 << 2))) = 0x2;
    *((unsigned int *)(pMemMapIO + (0x2 << 2))) = 0xff;
    
    try_module_get(THIS_MODULE);
    printk("Open pwm fan node ok!\n");
    return 0;
}

static int pwm_release(struct inode *inode, struct file *file)
{
    nOpenFlag--;
    module_put(THIS_MODULE);
    return 0;
}

static void set_freq_fan0(uint32_t RegValue)
{
    printk("pwm set Freq reg=0x%08x , value=0x%08x\n",(unsigned int)(pMemMapIO + (0x12 << 2)),RegValue);
    *((unsigned int *)(pMemMapIO + (0x12 << 2))) = RegValue;
}

static void set_duty_fan0(uint32_t RegValue)
{
    printk("pwm set Duty reg=0x%08x , value=0x%08x\n",(unsigned int)(pMemMapIO + (0x13 << 2)),RegValue);
    *((unsigned int *)(pMemMapIO + (0x13 << 2))) = RegValue;
}

static void set_freq_fan1(uint32_t RegValue)
{
    printk("pwm set Freq reg=0x%08x , value=0x%08x\n",(unsigned int)(pMemMapIO + (0x14 << 2)),RegValue);
    *((unsigned int *)(pMemMapIO + (0x14 << 2))) = RegValue;
}

static void set_duty_fan1(uint32_t RegValue)
{
    printk("pwm set Duty reg=0x%08x , value=0x%08x\n",(unsigned int)(pMemMapIO + (0x15 << 2)),RegValue);
     *((unsigned int *)(pMemMapIO + (0x15 << 2))) = RegValue;
}

static long pwm_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    switch(cmd)
    {
        case SET_FREQ_FAN0:
            printk("start set fan 0 freq!\n");
            set_freq_fan0(arg);
            break;
        case SET_DUTY_FAN0:
            printk("start set fan 0 duty!\n");
            set_duty_fan0(arg);
            break;
        case SET_FREQ_FAN1:
            printk("start set fan 1 freq!\n");
            set_freq_fan1(arg);
            break;
        case SET_DUTY_FAN1:
            printk("start set fan 1 duty!\n");
            set_duty_fan1(arg);
            break;
        default:
            break;
    }
            
    return 0 ;
}

struct file_operations pwm_fan_fops = {
        .owner  = THIS_MODULE,
        .open   = pwm_open,
        .unlocked_ioctl = pwm_ioctl,
        .release= pwm_release,
};

struct resource *pwm_fan_mem = NULL;
static int pwm_device_probe(struct platform_device *pdev)
{
    struct resource *pwm_fan_res = NULL;
    
    pwm_fan_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if(!pwm_fan_mem)
    {
        printk("platform get resource of mem Failed!\n");
        return -1;
    }

    printk("platform_get_resource of fan mem success!\n");

    printk("start=0x%08x,size=%d\n",pwm_fan_mem->start,pwm_fan_mem->end - pwm_fan_mem->start + 1);
    
    pwm_fan_res = request_mem_region(pwm_fan_mem->start, pwm_fan_mem->end - pwm_fan_mem->start + 1, PWM_FAN_DRIVERNAME);
    if(!pwm_fan_res)
    {
        printk("request mem Failed!\n");
        return -1;
    }
    printk("request_mem_region of mem success!\n");

    pMemMapIO = ioremap(pwm_fan_mem->start, pwm_fan_mem->end - pwm_fan_mem->start + 1);
    printk("BaseAddr:0x%08x\n",(unsigned int __force)pMemMapIO);
    
    return 0;
}


static int proc_fan0_freq_open(struct inode *inode,struct file  *file)
{  
    return 0;
}

static ssize_t proc_fan0_freq_read(struct file *file, char __user *buffer,size_t count,loff_t *offp)
{
    printk("0x%08x\n",*((unsigned int *)(pMemMapIO + (0x12 << 2))));
    return 0;
}

static ssize_t proc_fan0_freq_write(struct file *file,const char __user *buffer,size_t count,loff_t *offp)
{
    uint32_t value = 0;
    char reg[128] = {0};
    if(copy_from_user((void *)reg,buffer,count))
    {
        printk("copy from user data Failed!\n");
    }
    else
    {
        if(sscanf(reg,"%d",&value) == 1)
        {
            printk("echo value=0x%08x\n",value);
            *((unsigned int *)(pMemMapIO + (0x12 << 2))) = value;
        }
    }

    return count;
}

static int proc_fan0_duty_open(struct inode *inode,struct file  *file)
{  
    return 0;
}

static ssize_t proc_fan0_duty_read(struct file *file, char __user *buffer,size_t count,loff_t *offp)
{
    printk("0x%08x\n",*((unsigned int *)(pMemMapIO + (0x13 << 2))));
    return 0;
}

static ssize_t proc_fan0_duty_write(struct file *file,const char __user *buffer,size_t count,loff_t *offp)
{
    uint32_t value = 0;
    char reg[128] = {0};
    if(copy_from_user((void *)reg,buffer,count))
    {
        printk("copy from user data Failed!\n");
    }
    else
    {
        if(sscanf(reg,"%d",&value) == 1)
        {
            printk("echo value=0x%08x\n",value);
            *((unsigned int *)(pMemMapIO + (0x13 << 2))) = value;
        }
    }

    return count;
}

static int proc_fan1_freq_open(struct inode *inode,struct file  *file)
{  
    return 0;
}

static ssize_t proc_fan1_freq_read(struct file *file, char __user *buffer,size_t count,loff_t *offp)
{
    printk("0x%08x\n",*((unsigned int *)(pMemMapIO + (0x14 << 2))));
    return 0;
}

static ssize_t proc_fan1_freq_write(struct file *file,const char __user *buffer,size_t count,loff_t *offp)
{
    uint32_t value = 0;
    char reg[128] = {0};
    if(copy_from_user((void *)reg,buffer,count))
    {
        printk("copy from user data Failed!\n");
    }
    else
    { 
        if(sscanf(reg,"%d",&value) == 1)
        {
            printk("echo value=0x%08x\n",value);
            *((unsigned int *)(pMemMapIO + (0x14 << 2))) = value;
        }
    }

    return count;
}

static int proc_fan1_duty_open(struct inode *inode,struct file  *file)
{  
    return 0;
}

static ssize_t proc_fan1_duty_read(struct file *file, char __user *buffer,size_t count,loff_t *offp)
{
    printk("0x%08x\n",*((unsigned int *)(pMemMapIO + (0x15 << 2))));
    return 0;
}

static ssize_t proc_fan1_duty_write(struct file *file,const char __user *buffer,size_t count,loff_t *offp)
{
    uint32_t value = 0;
    char reg[128] = {0};
    if(copy_from_user((void *)reg,buffer,count))
    {
        printk("copy from user data Failed!\n");
    }
    else
    {
        if(sscanf(reg,"%d",&value) == 1)
        {
            printk("echo value=0x%08x\n",value);
            *((unsigned int *)(pMemMapIO + (0x15 << 2))) = value;
        }
    }

    return count;
}

static struct file_operations proc_pwm_fan0_freq_fops = {
    .open  = proc_fan0_freq_open,
    .read  = proc_fan0_freq_read,
    .write = proc_fan0_freq_write,
};
static struct file_operations proc_pwm_fan0_duty_fops = {
    .open  = proc_fan0_duty_open,
    .read  = proc_fan0_duty_read,
    .write = proc_fan0_duty_write,
};
static struct file_operations proc_pwm_fan1_freq_fops = {
    .open  = proc_fan1_freq_open,
    .read  = proc_fan1_freq_read,
    .write = proc_fan1_freq_write,
};
static struct file_operations proc_pwm_fan1_duty_fops = {
    .open  = proc_fan1_duty_open,
    .read  = proc_fan1_duty_read,
    .write = proc_fan1_duty_write,
};

static int create_proc_node_pwm(void)
{
    struct proc_dir_entry *proc_entry_fan0_freq;
    struct proc_dir_entry *proc_entry_fan0_duty;
    struct proc_dir_entry *proc_entry_fan1_freq;
    struct proc_dir_entry *proc_entry_fan1_duty;

    mydir = proc_mkdir(PWM_FAN_PROCNAME,NULL);

    if(!mydir)
    {
        printk("Create /proc/%s Failed!\n",PWM_FAN_PROCNAME);
        return -1;
    }
    
    printk("Create /proc/%s Success!\n",PWM_FAN_PROCNAME);

    proc_entry_fan0_freq = proc_create(PWM_FAN0_FREQ,0644,mydir,&proc_pwm_fan0_freq_fops);
    if(NULL == proc_entry_fan0_freq)
    {
        remove_proc_entry(PWM_FAN_PROCNAME,NULL);
        printk("Initialize /proc/%s Failed!\n",PWM_FAN_PROCNAME);
        goto error_exit;
    }

    proc_entry_fan0_duty = proc_create(PWM_FAN0_DUTY,0644,mydir,&proc_pwm_fan0_duty_fops);
    if(NULL == proc_entry_fan0_duty)
    {
        remove_proc_entry(PWM_FAN_PROCNAME,NULL);
        printk("Initialize /proc/%s Failed!\n",PWM_FAN_PROCNAME);
        goto error_exit;
    }

    proc_entry_fan1_freq = proc_create(PWM_FAN1_FREQ,0644,mydir,&proc_pwm_fan1_freq_fops);
    if(NULL == proc_entry_fan1_freq)
    {
        remove_proc_entry(PWM_FAN_PROCNAME,NULL);
        printk("Initialize /proc/%s Failed!\n",PWM_FAN_PROCNAME);
        goto error_exit;
    }

    proc_entry_fan1_duty = proc_create(PWM_FAN1_DUTY,0644,mydir,&proc_pwm_fan1_duty_fops);
    if(NULL == proc_entry_fan1_duty)
    {
        remove_proc_entry(PWM_FAN_PROCNAME,NULL);
        printk("Initialize /proc/%s Failed!\n",PWM_FAN_PROCNAME);
        goto error_exit;
    }

    return 0;
    
error_exit:
    remove_proc_entry(PWM_FAN_PROCNAME,mydir);  
    return -1;
}


static int pwm_55_probe(struct platform_device *pdev)
{
    int pwm = 0;
    pwm = register_chrdev(PWM_FAN_MAJOR, PWM_FAN_DEVNAME, &pwm_fan_fops);
    
	if (pwm < 0) 
    {
		printk (KERN_WARNING PWM_FAN_DEVNAME ": unable to get major %d\n",PWM_FAN_MAJOR);
		unregister_chrdev(PWM_FAN_MAJOR, PWM_FAN_DEVNAME);
        return -1;
	}

    printk("register pwm fan chrdev success!\n");
    
	pwm_fan_class = class_create(THIS_MODULE, PWM_FAN_CLASSNAME);
	if (IS_ERR(pwm_fan_class)) 
    {
		class_destroy(pwm_fan_class);
        return -1;
	}

    printk("class pwm fan node create success!\n");
    
    pwm_fan_devno = MKDEV(pwm,0);
    
    pwm_fan_device = device_create(pwm_fan_class, NULL, pwm_fan_devno, NULL, PWM_FAN_DEVNAME);
    if (IS_ERR(pwm_fan_device))
    {
        class_destroy(pwm_fan_class);
        unregister_chrdev(PWM_FAN_MAJOR, PWM_FAN_DEVNAME);
        return -1;
    }
    
    printk("bind dev and class node success!\n");
    
    pwm_device_probe(pdev);

    create_proc_node_pwm();
    
    return 0;
}

static void remove_proc_node(void)
{
    remove_proc_entry(PWM_FAN0_FREQ,mydir); 
    remove_proc_entry(PWM_FAN0_DUTY,mydir); 
    remove_proc_entry(PWM_FAN1_FREQ,mydir); 
    remove_proc_entry(PWM_FAN1_DUTY,mydir); 
    remove_proc_entry(PWM_FAN_PROCNAME,NULL);  
}

static int pwm_55_remove(struct platform_device *pdev)
{
    printk("rmmod pwm module!\n");
    remove_proc_node();   
    iounmap(pMemMapIO);
    release_mem_region(pwm_fan_mem->start, pwm_fan_mem->end - pwm_fan_mem->start + 1);
    pwm_fan_mem = NULL;
    device_destroy(pwm_fan_class,pwm_fan_devno);
	class_destroy(pwm_fan_class);
	unregister_chrdev (PWM_FAN_MAJOR, PWM_FAN_DEVNAME);
    
    return 0;
}


static struct platform_driver pwm_driver = {
        .driver= {
            .name = PWM_FAN_DRIVERNAME,
         },
        .probe       = pwm_55_probe,
        .remove      = pwm_55_remove,
};

static int __init pwmdev_init(void)
{
    printk("pwm fan init...\n");
	return platform_driver_register(&pwm_driver);
}

static void __exit pwmdev_exit(void)
{
    printk("pwm fan exit...\n");
    platform_driver_unregister(&pwm_driver);
}

module_init(pwmdev_init);
module_exit(pwmdev_exit);

MODULE_DESCRIPTION("pwm control fan speed driver");
MODULE_LICENSE("GPL");