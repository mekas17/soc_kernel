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
#define SET_FREQ_CMD0   _IO(MAGIC_NUMBER, 0)
#define SET_DUTY_CMD0   _IO(MAGIC_NUMBER, 1)
#define SET_FREQ_CMD1   _IO(MAGIC_NUMBER, 2)
#define SET_DUTY_CMD1   _IO(MAGIC_NUMBER, 3)


#define PWM_VERSION    "pwmdev: userspace pwm driver"
#define CHRDEVNAME     "pwmdev_inno"
#define PWM_MAJOR      0

#define DRIVER_NAME_PWM    "pwmdrv_inno"

#define PWM_BASEADDR   0x602f0000
#define PWM_SIZE       0x10000


struct class *pwmdev_class;
static int pwm;
struct device *pwmdev_device;
static void *pMemMapIO;
static dev_t pwm_devno;


static int nOpenFlag = 0;

static int pwm_open(struct inode *inode, struct file *file)
{
    if(nOpenFlag)
    {
        return -EBUSY;
    }
    nOpenFlag++;
    
    *((unsigned int *)(pMemMapIO + (0x1 << 2))) = 0x02;
    *((unsigned int *)(pMemMapIO + (0x2 << 2))) = 0xff;
    
    try_module_get(THIS_MODULE);
    printk("Open pwm node ok!\n");
    return 0;
}

static int pwm_release(struct inode *inode, struct file *file)
{
    nOpenFlag--;
    module_put(THIS_MODULE);
    return 0;
}

static void set_pwm_freq_inno55_0(uint32_t RegValue)
{
    printk("pwm set Freq reg=0x%08x , value=0x%08x\n",(unsigned int)(pMemMapIO + (0x12 << 2)),RegValue);
    *((unsigned int *)(pMemMapIO + (0x12 << 2))) = RegValue;
}

static void set_pwm_duty_inno55_0(uint32_t RegValue)
{
    printk("pwm set Duty reg=0x%08x , value=0x%08x\n",(unsigned int)(pMemMapIO + (0x13 << 2)),RegValue);
    *((unsigned int *)(pMemMapIO + (0x13 << 2))) = RegValue;
}

static void set_pwm_freq_inno55_1(uint32_t RegValue)
{
    printk("pwm set Freq reg=0x%08x , value=0x%08x\n",(unsigned int)(pMemMapIO + (0x14 << 2)),RegValue);
    *((unsigned int *)(pMemMapIO + (0x14 << 2))) = RegValue;
}

static void set_pwm_duty_inno55_1(uint32_t RegValue)
{
    printk("pwm set Duty reg=0x%08x , value=0x%08x\n",(unsigned int)(pMemMapIO + (0x15 << 2)),RegValue);
     *((unsigned int *)(pMemMapIO + (0x15 << 2))) = RegValue;
}

static int pwm_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    switch(cmd)
    {
        case SET_FREQ_CMD0:
            printk("start set SET_FREQ_CMD0!\n");
            set_pwm_freq_inno55_0(arg);
            break;
        case SET_DUTY_CMD0:
            printk("start set SET_DUTY_CMD0!\n");
            set_pwm_duty_inno55_0(arg);
            break;
        case SET_FREQ_CMD1:
            printk("start set SET_FREQ_CMD1!\n");
            set_pwm_freq_inno55_1(arg);
            break;
        case SET_DUTY_CMD1:
            printk("start set SET_DUTY_CMD1!\n");
            set_pwm_duty_inno55_1(arg);
            break;
        default:
            break;
    }
            
    return 0 ;
}

struct file_operations pwm_fops = {
        .owner  = THIS_MODULE,
        .open   = pwm_open,
        .unlocked_ioctl = pwm_ioctl,
        .release= pwm_release,
};

/*
struct device {
    
};

struct platform_device pwm_device={
    const char *name;
    u32 id;
    struct device dev;
    u32 num_resourcesl
    struct resource *resource;
};
*/

/*static irqreturn_t pwm_irq_function(int irq,void *dev_id)
{
	printk("pwm interrupt ok\n");
	return IRQ_HANDLED;
}*/


static int pwm_device_probe(struct platform_device *pdev)
{
    //struct resource *pwm_irq;
    struct resource *pwm_mem;
    //struct device *dev = &pdev->dev;
    
    struct resource *res = NULL;
    
    pwm_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if(!pwm_mem)
    {
        printk("platform get resource of mem Failed!\n");
        return -1;
    }

    printk("platform_get_resource of mem success!\n");

    printk("start=0x%08x,size=%d\n",pwm_mem->start,pwm_mem->end - pwm_mem->start + 1);
    
    res = request_mem_region(pwm_mem->start, pwm_mem->end - pwm_mem->start + 1, DRIVER_NAME_PWM);
    if(!res)
    {
        printk("request mem Failed!\n");
        return -1;
    }
    printk("request_mem_region of mem success!\n");

    //void __iomem *baseaddr;

    pMemMapIO = ioremap(pwm_mem->start, pwm_mem->end - pwm_mem->start + 1);
    printk("BaseAddr:0x%08x\n",(unsigned int __force)pMemMapIO);

    #if 0
    pwm_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    if(!pwm_irq)
    {
        printk("platform get resource of irq Failed!\n");
        return -1;
    }

    printk("platform_get_resource of irq success!\n");
    printk("start=%d\n",pwm_irq->start);
    res = request_irq(pwm_irq->start, &pwm_irq_function, 0, DRIVER_NAME_PWM, NULL);
    if(!res)
    {
        printk("request irq Failed!\n");
        return -1;
    }
    printk("request_irq of irq success!\n");
    #endif
    
    return 0;
}

#define PROC_PWM_SIZE   128
#define PROC_PWM_NAME   "pwm55info"


static int pwm_stop(struct seq_file *m,void *p)
{
    return 0;
}

static int pwm_start(struct seq_file *m,loff_t *pos)
{
    return *pos < 1?(void *)1:NULL;
}


static struct seq_operations  proc_pwm_seq_ops = {
    .start = pwm_start,
    .stop  = pwm_stop
};

static void proc_pwm_open(struct inode *inode,struct file *file)
{
    int ret = 0;
    struct seq_file *m;

    ret = seq_open(file,&proc_pwm_seq_ops);
    m = file->private_data;
    m->private = file->f_path.dentry->d_iname;

    return ;
    
}

static int proc_pwm_read(struct seq_file *m,void *p)
{
    printk("proc_pwm_read reg addr:0x%08x, value: 0x%08x\n",((unsigned int )(pMemMapIO + (0x12 << 2))),*((unsigned int *)(pMemMapIO + (0x13 << 2))));
    printk("proc_pwm_read reg addr:0x%08x, value: 0x%08x\n",((unsigned int )(pMemMapIO + (0x14 << 2))),*((unsigned int *)(pMemMapIO + (0x15 << 2))));
    
    return 0;
}

static int proc_pwm_write(struct file *file,const char __user *buffer,size_t count,loff_t *offp)
{
    uint32_t value = 0;
    char reg[128] = {0};
    if(copy_from_user((void *)reg,buffer,count))
    {
        printk("copy from user data Failed!\n");
    }
    else
    {
        printk("proc_pwm_write reg=%s\n",reg);
    
        if(sscanf(reg,"%d",&value) == 1)
        {
            printk("echo value=0x%08x\n",value);
            *((unsigned int *)(pMemMapIO + (0x14 << 2))) = value;
        }
    }

    return count;
}

static struct file_operations proc_pwm_fops = {
    .open  = proc_pwm_open,
    .read  = proc_pwm_read,
    .write = proc_pwm_write,
    //.realse= proc_pwm_release,
};

static int create_proc_pwm_inode(char *name)
{
    struct proc_dir_entry *pwm_proc_entry;
    u8 proc_pwm_name[50] = {0};
    if(PROC_PWM_NAME == NULL)
    {
        sprintf(proc_pwm_name,"%s",name);
    }
    else
    {
        sprintf(proc_pwm_name,"%s/%s",PROC_PWM_NAME,name);
    }

    pwm_proc_entry = proc_create(proc_pwm_name,0644,NULL,&proc_pwm_fops);
    if(NULL == pwm_proc_entry)
    {
        remove_proc_entry(proc_pwm_name,NULL);
        printk("Initialize /proc/%s Failed!\n",proc_pwm_name);
        return -1;
    }

    return 0;
}
static int create_proc_pwm_file(void)
{
    struct proc_dir_entry *mydir = NULL;

    mydir = proc_mkdir(PROC_PWM_NAME,NULL);

    if(!mydir)
    {
        printk("Create /proc/%s Failed!\n",PROC_PWM_NAME);
        return -1;
    }
    
    printk("Create /proc/%s Success!\n",PROC_PWM_NAME);
    return 0;
}


static void create_proc_node_pwm(void)
{
    create_proc_pwm_file();
    create_proc_pwm_inode("pwm55");
}


static int pwm_55_probe(struct platform_device *pdev)
{
    pwm = register_chrdev(PWM_MAJOR, CHRDEVNAME, &pwm_fops);
    
	if (pwm < 0) 
    {
		printk (KERN_WARNING CHRDEVNAME ": unable to get major %d\n",PWM_MAJOR);
		unregister_chrdev(PWM_MAJOR, CHRDEVNAME);
        return -1;
	}

    printk("register_chrdev Success\n");
    
	pwmdev_class = class_create(THIS_MODULE, CHRDEVNAME);
	if (IS_ERR(pwmdev_class)) {
		class_destroy(pwmdev_class);
        return -1;
	}

    printk("class create Success\n");
    
    pwm_devno = MKDEV(pwm,0);
    
    pwmdev_device = device_create(pwmdev_class, NULL, pwm_devno, NULL, CHRDEVNAME);
    if (IS_ERR(pwmdev_device))
    {
        class_destroy(pwmdev_class);
        unregister_chrdev(PWM_MAJOR, CHRDEVNAME);
        return -1;
    }
    
    printk("device create Success\n");
    
    pwm_device_probe(pdev);

    create_proc_node_pwm();
    
    return 0;
}

static void remove_proc_node_pwm(char *name)
{
    u8 proc_name[50] = {0};

    if(PROC_PWM_NAME == NULL)
    {
        sprintf(proc_name,"%s",name);
    }
    else
    {
        sprintf(proc_name,"%s/%s",PROC_PWM_NAME,name);
    }

    remove_proc_entry(proc_name,NULL);
    
    if(PROC_PWM_NAME != NULL)
    {
        remove_proc_entry(PROC_PWM_NAME,NULL);
    }
    
}

static int pwm_55_remove(struct platform_device *pdev)
{
    printk("rmmod pwm module!\n");
    remove_proc_node_pwm("pwm55");
    iounmap(pMemMapIO);
    device_destroy(pwmdev_class,MKDEV(pwm,0));
	class_destroy(pwmdev_class);
	unregister_chrdev (PWM_MAJOR, CHRDEVNAME);
    
    return 0;
}


static struct platform_driver pwm_driver = {
        .driver= {
            .name = DRIVER_NAME_PWM,
         },
        .probe       = pwm_55_probe,
        .remove      = pwm_55_remove,
};

static int __init pwmdev_init(void)
{
    printk("pwm init...(%s) \n",PWM_VERSION);
	return platform_driver_register(&pwm_driver);
}

static void __exit pwmdev_exit(void)
{
    printk("pwm exit...\n");
    platform_driver_unregister(&pwm_driver);
}

module_init(pwmdev_init);
module_exit(pwmdev_exit);

MODULE_DESCRIPTION("pwm control fan speed driver");
MODULE_LICENSE("GPL");