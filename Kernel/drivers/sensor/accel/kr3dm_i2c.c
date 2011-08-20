#include <mach/instinctq.h>
#include "kr3dm_i2c.h"
#include <linux/wakelock.h>
#include <linux/input.h>

#define I2C_READ_FAIL_PROTECTION

#ifdef I2C_READ_FAIL_PROTECTION
#define I2C_READ_RETRY_CNT_DEF 3
#endif

#ifdef SECURITY_HOLE_SUPPORT
#define ACC_ENABLED 1
static void kr3dm_acc_enable(void);
static void kr3dm_acc_disable(void);
spinlock_t lock;
#endif

kr3dm_t *p_kr3dm;
kr3dm_t kr3dm;
kr3dmregs_t kr3dmregs;

static struct i2c_client *g_client;
static struct platform_device *kr3dm_accelerometer_device;
struct class *kr3dm_acc_class;

static int __devinit i2c_acc_kr3dm_probe(struct i2c_client *, const struct i2c_device_id *);
static int __devexit i2c_acc_kr3dm_remove(struct i2c_client *);
static int i2c_acc_kr3dm_detect(struct i2c_client *, int kind, struct i2c_board_info *);

static unsigned short ignore[] = {I2C_CLIENT_END };
static unsigned short normal_addr[] = {I2C_CLIENT_END };
static unsigned short probe_addr[] = { 6, SENS_ADD >> 1, I2C_CLIENT_END };

static bool is_kr3dm_sleep=false;

static struct i2c_client_address_data addr_data = {
	.normal_i2c		= normal_addr,
	.probe			= probe_addr,
	.ignore			= ignore,
};

static struct platform_driver kr3dm_accelerometer_driver = {
	.probe 	 = kr3dm_accelerometer_probe,
	.suspend = kr3dm_accelerometer_suspend,
	.resume  = kr3dm_accelerometer_resume,
	.driver  = {
		.name = "kr3dm-accelerometer",
	}
};

struct file_operations kr3dm_acc_fops =
{
	.owner   = THIS_MODULE,
	.read    = kr3dm_read,
	.write   = kr3dm_write,
	.open    = kr3dm_open,
	.ioctl   = kr3dm_ioctl,
	.release = kr3dm_release,
};


static struct i2c_device_id kr3dm_acc_id[] = {
	{ "kr3dm_i2c_driver", 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, kr3dm_acc_id);

struct i2c_driver acc_kr3dm_i2c_driver =
{
	.driver = {
		.name = "kr3dm_i2c_driver",
	},
	.class		= I2C_CLASS_HWMON,
	.probe		= i2c_acc_kr3dm_probe,
	.remove		= __devexit_p(i2c_acc_kr3dm_remove),
	.detect		= i2c_acc_kr3dm_detect,
	.id_table	= kr3dm_acc_id,
	.address_data 	= &addr_data
};

char i2c_acc_kr3dm_read(u8 reg, u8 *val, unsigned int len )
{
	int 	 err;
	struct 	 i2c_msg msg[1];
	unsigned char data[1];

	if( (g_client == NULL) || (!g_client->adapter) )
	{
		return -ENODEV;
	}

	msg->addr 	= g_client->addr;
	msg->flags 	= I2C_M_WR;
	msg->len 	= 1;
	msg->buf 	= data;
	*data       = reg;

	err = i2c_transfer(g_client->adapter, msg, 1);

	if (err >= 0)
	{
		msg->flags = I2C_M_RD;
		msg->len   = len;
		msg->buf   = val;
		err = i2c_transfer(g_client->adapter, msg, 1);
	}

	if (err >= 0)
	{
		return 0;
	}
#if DEBUG
	printk(KERN_ERR "%s %d i2c transfer error\n", __func__, __LINE__);
#endif
	return err;;

}
char i2c_acc_kr3dm_write( u8 reg, u8 *val )
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];

	if( (g_client == NULL) || (!g_client->adapter) ){
		return -ENODEV;
	}

	data[0] = reg;
	data[1] = *val;

	msg->addr = g_client->addr;
	msg->flags = I2C_M_WR;
	msg->len = 2;
	msg->buf = data;

	err = i2c_transfer(g_client->adapter, msg, 1);

	if (err >= 0) return 0;
#if DEBUG
	printk(KERN_ERR "%s %d i2c transfer error[%d]\n", __func__, __LINE__, err);
#endif
	return err;
}


static int __devinit i2c_acc_kr3dm_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;


	if ( !i2c_check_functionality(client->adapter,I2C_FUNC_SMBUS_BYTE_DATA) ) {
		printk(KERN_INFO "byte op is not permited.\n");
		goto ERROR0;
	}

	client->addr = SENS_ADD >> 1;
	client->driver = &acc_kr3dm_i2c_driver;
	client->flags = I2C_DF_NOTIFY | I2C_M_IGNORE_NAK;

	g_client = client;

#if DEBUG
	printk("i2c_acc_kr3dm_probe_client() completed!!!!!!!!!!!!!!!!!!\n");
#endif
	return 0;

ERROR0:
	printk(KERN_ERR "[KR3DM][%s] probe failed!\n", __func__);
	return err;
}


static int __devexit i2c_acc_kr3dm_remove(struct i2c_client *client)
{
	return 0;
}


int i2c_acc_kr3dm_init(void)
{
	int ret;

	if ( (ret = i2c_add_driver(&acc_kr3dm_i2c_driver)) )
	{
		printk(KERN_ERR "Driver registration failed, module not inserted.\n");
		return ret;
	}

	return 0;
}


void i2c_acc_kr3dm_exit(void)
{
	i2c_del_driver(&acc_kr3dm_i2c_driver);
}

static int i2c_acc_kr3dm_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
	strlcpy(info->type, "kr3dm_i2c_driver", I2C_NAME_SIZE);
	return 0;
}

int kr3dm_set_range(char range)
{
   int comres = 0;
   unsigned char data;

   if (p_kr3dm==0)
	    return E_KR3DM_NULL_PTR;

   if (range<3){
   		comres = p_kr3dm->KR3DM_BUS_READ_FUNC(p_kr3dm->dev_addr, CTRL_REG4, &data, 1 );
		data = data | (range << 4);
		comres += p_kr3dm->KR3DM_BUS_WRITE_FUNC(p_kr3dm->dev_addr, CTRL_REG4, &data, 1);
   }
   return comres;

}

int kr3dm_set_mode(unsigned char mode)
{

	int comres=0;
	unsigned char normal = 0x27;
	unsigned char sleep = 0x00;

	if (p_kr3dm==0)
		return E_KR3DM_NULL_PTR;

	switch(mode)
	{
		case KR3DM_MODE_NORMAL:
		case KR3DM_MODE_WAKE_UP:
			comres += p_kr3dm->KR3DM_BUS_WRITE_FUNC(p_kr3dm->dev_addr, CTRL_REG1, &normal, 1);
			break;
		case KR3DM_MODE_SLEEP:
			comres += p_kr3dm->KR3DM_BUS_WRITE_FUNC(p_kr3dm->dev_addr, CTRL_REG1, &sleep, 1);
			break;
		default:
			return E_OUT_OF_RANGE;
	}
	p_kr3dm->mode = mode;

	return comres;

}

unsigned char kr3dm_get_mode(void)
{
    if (p_kr3dm==0)
    	return E_KR3DM_NULL_PTR;

	return p_kr3dm->mode;

}

int kr3dm_set_bandwidth(char bw)
{
	int comres = 0;
	unsigned char data = 0x27;

	if (p_kr3dm==0)
		return E_KR3DM_NULL_PTR;

	if (bw<8)
	{
	  data = data | (3 << bw);
	  comres += p_kr3dm->KR3DM_BUS_WRITE_FUNC(p_kr3dm->dev_addr, CTRL_REG1, &data, 1 );
	}

	return comres;
}


int kr3dm_get_bandwidth(unsigned char *bw)
{
	int comres = 1;

	if (p_kr3dm==0)
		return E_KR3DM_NULL_PTR;

	comres = p_kr3dm->KR3DM_BUS_READ_FUNC(p_kr3dm->dev_addr, CTRL_REG1, bw, 1 );

	*bw = (*bw & 0x18);

	return comres;
}

int kr3dm_open (struct inode *inode, struct file *filp)
{
	return 0;
}

ssize_t kr3dm_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	return 0;
}

ssize_t kr3dm_write (struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
	return 0;
}

int kr3dm_release (struct inode *inode, struct file *filp)
{
	return 0;
}

int kr3dm_ioctl(struct inode *inode, struct file *filp, unsigned int cmd,  unsigned long arg)
{
	int err = 0;
	unsigned char data[3];
	kr3dmacc_t accels;
	unsigned char val1 = 0x27;

	/* check cmd */
	if(_IOC_TYPE(cmd) != KR3DM_IOC_MAGIC)
	{
		printk("cmd magic type error\n");
		return -ENOTTY;
	}
	if(_IOC_NR(cmd) > KR3DM_IOC_MAXNR)
	{
		printk("cmd number error\n");
		return -ENOTTY;
	}

	if(_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
	if(err)
	{
		printk("cmd access_ok error\n");
		return -EFAULT;
	}

	switch(cmd)
	{
		case KR3DM_READ_ACCEL_XYZ:
			err = kr3dm_read_accel_xyz(&accels);
			if(copy_to_user((kr3dmacc_t*)arg, &accels, sizeof(kr3dmacc_t))!=0)
			{
				printk("copy_to error\n");
				return -EFAULT;
			}
			return err;

		case KR3DM_SET_RANGE:
			if(copy_from_user(data,(unsigned char*)arg,1)!=0)
			{
				printk("[KR3DM] copy_from_user error\n");
				return -EFAULT;
			}
			err = kr3dm_set_range(*data);
			return err;

		case KR3DM_SET_MODE:
			if(copy_from_user(data,(unsigned char*)arg,1)!=0)
			{
				printk("[KR3DM] copy_from_user error\n");
				return -EFAULT;
			}
			err = kr3dm_set_mode(*data);
			return err;

		case KR3DM_SET_BANDWIDTH:
			if(copy_from_user(data,(unsigned char*)arg,1)!=0)
			{
				printk("[KR3DM] copy_from_user error\n");
				return -EFAULT;
			}
			err = kr3dm_set_bandwidth(*data);
			return err;

		default:
			return 0;
	}
	return 0;
}


int kr3dm_read_accel_xyz(kr3dmacc_t * acc)
{
	int comres;
	unsigned char data[3];

#ifdef I2C_READ_FAIL_PROTECTION
	int i2c_read_retry_cnt;
#endif
	
	if (p_kr3dm==0)
		return E_KR3DM_NULL_PTR;

#ifdef I2C_READ_FAIL_PROTECTION
	i2c_read_retry_cnt = I2C_READ_RETRY_CNT_DEF;

	while(i2c_read_retry_cnt--)
	{
		comres = p_kr3dm->KR3DM_BUS_READ_FUNC(p_kr3dm->dev_addr, OUT_X_L, &data[0], 1);
		if(!comres)
			break;
	}

	i2c_read_retry_cnt = I2C_READ_RETRY_CNT_DEF;
	
	while(i2c_read_retry_cnt--)
	{
		comres = p_kr3dm->KR3DM_BUS_READ_FUNC(p_kr3dm->dev_addr, OUT_Y_L, &data[1], 1);
		if(!comres)
			break;
	}

	i2c_read_retry_cnt = I2C_READ_RETRY_CNT_DEF;
	
	while(i2c_read_retry_cnt--)
	{
		comres = p_kr3dm->KR3DM_BUS_READ_FUNC(p_kr3dm->dev_addr, OUT_Z_L, &data[2], 1);
		if(!comres)
			break;
	}
#else
	comres = p_kr3dm->KR3DM_BUS_READ_FUNC(p_kr3dm->dev_addr, OUT_X_L, &data[0], 1);
	comres = p_kr3dm->KR3DM_BUS_READ_FUNC(p_kr3dm->dev_addr, OUT_Y_L, &data[1], 1);
	comres = p_kr3dm->KR3DM_BUS_READ_FUNC(p_kr3dm->dev_addr, OUT_Z_L, &data[2], 1);
#endif

	data[0] = (~data[0] + 1);
	data[1] = (~data[1] + 1);
	data[2] = (~data[2] + 1);

    	if(data[0] & 0x80)
    		acc->x = (0x100-data[0]);
    	else
    		acc->x = ((data[0]) & 0xFF)*(-1);
    	if(data[1]& 0x80)
    		acc->y = (0x100-data[1]);
    	else
    		acc->y = ((data[1]) & 0xFF)*(-1);
    	if(data[2]& 0x80)
    		acc->z = (0x100-data[2]);
    	else
    		acc->z = ((data[2]) & 0xFF)*(-1);

#if DEBUG
	printk("[KR3DM_convert] x = %d  /  y =  %d  /  z = %d converted data \n", acc->x, acc->y, acc->z ); 
#endif
	return comres;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void kr3dm_early_suspend(struct early_suspend *handler)
{
#ifdef SECURITY_HOLE_SUPPORT
		printk(" %s \n",__func__); 
		kr3dm_set_mode(KR3DM_MODE_SLEEP);
	
		if (kr3dm.state & ACC_ENABLED) 
			kr3dm_acc_disable();
#else
	is_kr3dm_sleep=true;
	kr3dm_set_mode( KR3DM_MODE_SLEEP );
#endif
}

static void kr3dm_late_resume(struct early_suspend *handler)
{
#ifdef SECURITY_HOLE_SUPPORT
		printk(" %s \n",__func__); 
	
		if (kr3dm.state & ACC_ENABLED)
			kr3dm_acc_enable();
	
		kr3dm_set_mode(KR3DM_MODE_NORMAL);
#else
		kr3dm_set_mode( KR3DM_MODE_NORMAL );
		msleep(3);
		is_kr3dm_sleep=false;
#endif
}
#endif /* CONFIG_HAS_EARLYSUSPEND */


void kr3dm_chip_init(void)
{
	kr3dm.kr3dm_bus_write = i2c_acc_kr3dm_write;
	kr3dm.kr3dm_bus_read  = i2c_acc_kr3dm_read;

#ifdef CONFIG_HAS_EARLYSUSPEND
	kr3dm.early_suspend.suspend = kr3dm_early_suspend;
	kr3dm.early_suspend.resume = kr3dm_late_resume;
	register_early_suspend(&kr3dm.early_suspend);
#endif
	kr3dm_init( &kr3dm );
}


int kr3dm_init(kr3dm_t *kr3dm)
{
	unsigned char val1 = 0x27;
	unsigned char val2 = 0x00;

	p_kr3dm = kr3dm;
	p_kr3dm->dev_addr = SENS_ADD;										/* preset KR3DM I2C_addr */
	p_kr3dm->KR3DM_BUS_WRITE_FUNC(p_kr3dm->dev_addr, CTRL_REG1, &val1, 1 );
	p_kr3dm->KR3DM_BUS_WRITE_FUNC(p_kr3dm->dev_addr, CTRL_REG2, &val2, 1 );
	p_kr3dm->KR3DM_BUS_WRITE_FUNC(p_kr3dm->dev_addr, CTRL_REG3, &val2, 1 );
	p_kr3dm->KR3DM_BUS_WRITE_FUNC(p_kr3dm->dev_addr, CTRL_REG4, &val2, 1 );
	p_kr3dm->KR3DM_BUS_WRITE_FUNC(p_kr3dm->dev_addr, CTRL_REG5, &val2, 1 );

	return 0;
}

#ifdef SECURITY_HOLE_SUPPORT
/////////////////////////////////////////////////////////////////////////////////////
static void kr3dm_acc_enable(void)
{
	unsigned long flag;
	printk("starting poll timer, delay %lldns\n", ktime_to_ns(kr3dm.acc_poll_delay));
	spin_lock_irqsave(lock,flag);
	hrtimer_start(&kr3dm.timer, kr3dm.acc_poll_delay, HRTIMER_MODE_REL);
	spin_unlock_irqrestore(lock,flag);
}

static void kr3dm_acc_disable(void)
{
	unsigned long flag;

	printk("cancelling poll timer\n");
	spin_lock_irqsave(lock,flag);
	hrtimer_cancel(&kr3dm.timer);
	spin_unlock_irqrestore(lock,flag);
	cancel_work_sync(&kr3dm.work_acc);
}

static ssize_t poll_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%lld\n", ktime_to_ns(kr3dm.acc_poll_delay));
}


static ssize_t poll_delay_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t new_delay;
	int err;

	err = strict_strtoll(buf, 10, &new_delay);
	if (err < 0)
		return err;

	printk("new delay = %lldns, old delay = %lldns\n",
		    new_delay, ktime_to_ns(kr3dm.acc_poll_delay));
	//mutex_lock(&kr3dm.power_lock);
	if (new_delay != ktime_to_ns(kr3dm.acc_poll_delay)) {
		kr3dm_acc_disable();
		kr3dm.acc_poll_delay = ns_to_ktime(new_delay);
		if (kr3dm.state & ACC_ENABLED) {
			kr3dm_acc_enable();
		}
	}
	//mutex_unlock(&kr3dm.power_lock);

	return size;
}

static ssize_t acc_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", (kr3dm.state & ACC_ENABLED) ? 1 : 0);
}


static ssize_t acc_enable_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	bool new_value;

	if (sysfs_streq(buf, "1"))
		new_value = true;
	else if (sysfs_streq(buf, "0"))
		new_value = false;
	else {
		pr_err("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	//mutex_lock(&kr3dm.power_lock);
	printk("new_value = %d, old state = %d\n", new_value, (kr3dm.state & ACC_ENABLED) ? 1 : 0);
	if (new_value && !(kr3dm.state & ACC_ENABLED)) {
		kr3dm.state |= ACC_ENABLED;
		kr3dm_acc_enable();
	} else if (!new_value && (kr3dm.state & ACC_ENABLED)) {
		kr3dm_acc_disable();
		kr3dm.state = 0;
	}
	//mutex_unlock(&kr3dm.power_lock);
	return size;
}

static DEVICE_ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
		   poll_delay_show, poll_delay_store);

static struct device_attribute dev_attr_acc_enable =
	__ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
	       acc_enable_show, acc_enable_store);

static struct attribute *acc_sysfs_attrs[] = {
	&dev_attr_acc_enable.attr,
	&dev_attr_poll_delay.attr,
	NULL
};

static struct attribute_group acc_attribute_group = {
	.attrs = acc_sysfs_attrs,
};
///////////////////////////////////////////////////////////////////////////////////

static void kr3dm_work_func_acc(struct work_struct *work)
{
	kr3dmacc_t acc;
	int err;
/*		
	err = kr3dm_read_accel_xyz(&acc);
	
	input_report_abs(kr3dm.acc_input_dev, ABS_X, acc.x);
	input_report_abs(kr3dm.acc_input_dev, ABS_Y, acc.y);
	input_report_abs(kr3dm.acc_input_dev, ABS_Z, acc.z);
	input_sync(kr3dm.acc_input_dev);
*/	
}

/* This function is for light sensor.  It operates every a few seconds.
 * It asks for work to be done on a thread because i2c needs a thread
 * context (slow and blocking) and then reschedules the timer to run again.
 */
static enum hrtimer_restart kr3dm_timer_func(struct hrtimer *timer)
{
	unsigned long flag;

	queue_work(kr3dm.wq, &kr3dm.work_acc);
	spin_lock_irqsave(lock,flag);
	hrtimer_forward_now(&kr3dm.timer, kr3dm.acc_poll_delay);
	spin_unlock_irqrestore(lock,flag);
	return HRTIMER_RESTART;
}
#endif

int kr3dm_acc_start(void)
{
	int result;
	struct device *dev_t;
	
#ifdef SECURITY_HOLE_SUPPORT	
		int err;
		struct input_dev *input_dev;
#endif

	kr3dmacc_t accels; /* only for test */

	result = register_chrdev( KR3DM_MAJOR, "kr3dm", &kr3dm_acc_fops);

	if (result < 0)
	{
		return result;
	}

	kr3dm_acc_class = class_create (THIS_MODULE, "KR3DM-dev");

	if (IS_ERR(kr3dm_acc_class))
	{
		unregister_chrdev( KR3DM_MAJOR, "kr3dm" );
		return PTR_ERR( kr3dm_acc_class );
	}

	dev_t = device_create( kr3dm_acc_class, NULL, MKDEV(KR3DM_MAJOR, 0), "%s", "kr3dm");

	if (IS_ERR(dev_t))
	{
		return PTR_ERR(dev_t);
	}

#ifdef SECURITY_HOLE_SUPPORT	
		//mutex_init(&kr3dm.power_lock);
	
		/* hrtimer settings.  we poll for light values using a timer. */
		hrtimer_init(&kr3dm.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		kr3dm.acc_poll_delay = ns_to_ktime(240 * NSEC_PER_MSEC);
		kr3dm.timer.function = kr3dm_timer_func;
	
		/* the timer just fires off a work queue request.  we need a thread
		   to read the i2c (can be slow and blocking). */
		kr3dm.wq = create_singlethread_workqueue("kr3dm_wq");
		if (!kr3dm.wq) {
			err = -ENOMEM;
			printk("%s: could not create workqueue\n", __func__);
			goto err_create_workqueue;
		}
		/* this is the thread function we run on the work queue */
		INIT_WORK(&kr3dm.work_acc, kr3dm_work_func_acc);
	
		/* allocate lightsensor-level input_device */
		input_dev = input_allocate_device();
		if (!input_dev) {
			printk("%s: could not allocate input device\n", __func__);
			err = -ENOMEM;
			goto err_input_allocate_device_light;
		}
		input_set_drvdata(input_dev, &kr3dm);
		input_dev->name = "accel";
	
	
		set_bit(EV_ABS, input_dev->evbit);	
		/* acceleration x-axis */
		input_set_capability(input_dev, EV_ABS, ABS_X);
		input_set_abs_params(input_dev, ABS_X, -1024, 1024, 0, 0);
		/* acceleration y-axis */
		input_set_capability(input_dev, EV_ABS, ABS_Y);
		input_set_abs_params(input_dev, ABS_Y, -1024, 1024, 0, 0);
		/* acceleration z-axis */
		input_set_capability(input_dev, EV_ABS, ABS_Z);
		input_set_abs_params(input_dev, ABS_Z, -1024, 1024, 0, 0);
	
		printk("registering lightsensor-level input device\n");
		err = input_register_device(input_dev);
		if (err < 0) {
			printk("%s: could not register input device\n", __func__);
			input_free_device(input_dev);
			goto err_input_register_device_light;
		}
		kr3dm.acc_input_dev = input_dev;
	
	
		err = sysfs_create_group(&input_dev->dev.kobj,&acc_attribute_group);
		if (err) {
			printk("Creating kr3dm attribute group failed");
			goto error_device;
		}
	
	//////////////////////////////////////////////////////////////////////////////
#endif	

	result = i2c_acc_kr3dm_init();

	if(result)
	{
		return result;
	}

	kr3dm_chip_init();
#if DEBUG
	printk("[KR3DM] read_xyz ==========================\n");
	kr3dm_read_accel_xyz( &accels );
	printk("[KR3DM] x = %d  /  y =  %d  /  z = %d\n", accels.x, accels.y, accels.z );
	printk("[KR3DM] ======================kr3dm_acc_start Ready for use !!!!! =============\n");
#endif
	return 0;

error_device:
	sysfs_remove_group(&input_dev->dev.kobj, &acc_attribute_group);
err_input_register_device_light:
	input_unregister_device(kr3dm.acc_input_dev);
err_input_allocate_device_light:	
	destroy_workqueue(kr3dm.wq);
err_create_workqueue:
	//mutex_destroy(&kr3dm.power_lock);
exit:
	return err;

}



static int kr3dm_accelerometer_suspend( struct platform_device* pdev, pm_message_t state )
{
#ifdef SECURITY_HOLE_SUPPORT
		printk(" %s \n",__func__); 
		kr3dm_set_mode(KR3DM_MODE_SLEEP);
	
		if (kr3dm.state & ACC_ENABLED) 
			kr3dm_acc_disable();
		
		return 0;
	
#else
		return 0;
#endif
}


static int kr3dm_accelerometer_resume( struct platform_device* pdev )
{
#ifdef SECURITY_HOLE_SUPPORT
		printk(" %s \n",__func__); 
	
		if (kr3dm.state & ACC_ENABLED)
			kr3dm_acc_enable();
	
		kr3dm_set_mode(KR3DM_MODE_NORMAL);
		return 0;
	
#else
		return 0;
#endif
}

void kr3dm_acc_end(void)
{
	unregister_chrdev( KR3DM_MAJOR, "kr3dm" );

	i2c_acc_kr3dm_exit();

	device_destroy( kr3dm_acc_class, MKDEV(KR3DM_MAJOR, 0) );
	class_destroy( kr3dm_acc_class );
	unregister_early_suspend(&kr3dm.early_suspend);
}

static int kr3dm_accelerometer_probe( struct platform_device* pdev )
{
	return kr3dm_acc_start();

}


static int __init kr3dm_acc_init(void)
{
	int result;

	result = platform_driver_register( &kr3dm_accelerometer_driver);
#if DEBUG
	printk("[KR3DM] ********** kr3dm_acc_init =====================\n");
#endif
	if( result )
	{
		return result;
	}

	kr3dm_accelerometer_device  = platform_device_register_simple( "kr3dm-accelerometer", -1, NULL, 0 );

	if( IS_ERR( kr3dm_accelerometer_device ) )
	{
		return PTR_ERR( kr3dm_accelerometer_device );
	}

	return 0;
}


static void __exit kr3dm_acc_exit(void)
{
	kr3dm_acc_end();
	platform_device_unregister( kr3dm_accelerometer_device );
	platform_driver_unregister( &kr3dm_accelerometer_driver );
}


module_init( kr3dm_acc_init );
module_exit( kr3dm_acc_exit );

MODULE_AUTHOR("vishnu.p");
MODULE_DESCRIPTION("accelerometer driver for KR3DM");
MODULE_LICENSE("GPL");
