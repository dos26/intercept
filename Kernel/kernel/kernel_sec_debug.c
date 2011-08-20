/* kernel_sec_debug.c
 *
 * Exception handling in kernel by SEC
 *
 * Copyright (c) 2010 Samsung Electronics
 *                http://www.samsung.com/
 *
 */

#ifdef CONFIG_KERNEL_DEBUG_SEC

#include <linux/kernel_sec_common.h>
#include <asm/cacheflush.h>           // cacheflush

#ifdef CONFIG_CIQ_CRASH_CATCHER
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/hrtimer.h>
#include <asm/uaccess.h>
#endif

void __iomem * kernel_sec_viraddr_wdt_reset_reg;
__used t_kernel_sec_arm_core_regsiters kernel_sec_core_reg_dump;
__used t_kernel_sec_mmu_info           kernel_sec_mmu_reg_dump;
__used kernel_sec_upload_cause_type     gkernel_sec_upload_cause;

__used char* p_upload_info;

void kernel_sec_set_upload_magic_number(void)
{
	int *magic_virt_addr = (int*) UPLOAD_MAGIC_VIRT_ADDR;

	*magic_virt_addr = UPLOAD_MAGIC_NUMBER; // SET
	printk(KERN_EMERG"KERNEL:magic_number=%x SET_UPLOAD_MAGIC_NUMBER\n",*magic_virt_addr);
}
EXPORT_SYMBOL(kernel_sec_set_upload_magic_number);

void kernel_sec_clear_upload_magic_number(void)
{
	int *magic_virt_addr = (int*) UPLOAD_MAGIC_VIRT_ADDR;

	*magic_virt_addr = 0;  // CLEAR
	printk(KERN_EMERG"KERNEL:magic_number=%x CLEAR_UPLOAD_MAGIC_NUMBER\n",*magic_virt_addr);
}
EXPORT_SYMBOL(kernel_sec_clear_upload_magic_number);

void kernel_sec_map_wdog_reg(void)
{
	/* Virtual Mapping of Watchdog register */
	kernel_sec_viraddr_wdt_reset_reg = ioremap_nocache(S3C_PA_WDT,0x400);

	if (kernel_sec_viraddr_wdt_reset_reg == NULL)
	{
		printk(KERN_EMERG"Failed to ioremap() region in forced upload keystring\n");
	}
}
EXPORT_SYMBOL(kernel_sec_map_wdog_reg);

void kernel_sec_set_upload_cause(kernel_sec_upload_cause_type uploadType)
{
	gkernel_sec_upload_cause=uploadType;
	__raw_writel(uploadType, UPLOAD_CAUSE_REG_ADDR);
	printk(KERN_EMERG"(kernel_sec_set_upload_cause) : upload_cause set %x\n",uploadType);	
}
EXPORT_SYMBOL(kernel_sec_set_upload_cause);

static void kernel_sec_info_ioremap(void) {
	UPLOAD_INFO_VIRT_ADDR = (char *)ioremap(UPLOAD_INFO_PHYS_ADDR, UPLOAD_INFO_SIZE);
}

void kernel_sec_set_log_ptrs_for_getlog(void* p)
{
	// This is a workaround for getLog
	// getLog seems to search a narrow address ranges for the magic codes.
	// But since we dump the modem ram first, we need to copy those variables
	// to the upper address of the dump in order to make the getLog happy!
	//
	// NOTE : If we don't dump the modem or if we don't dump the modem first
	// we just block be codes to make this function do nothing!
	
	static int location=0;

	if(UPLOAD_INFO_VIRT_ADDR == NULL) {
		kernel_sec_info_ioremap();
	}
	
	if(location==0) {
		memset((void*)UPLOAD_GETLOG_INFO_ADDR, 0x0, UPLOAD_GETLOG_INFO_SIZE);
	}
	
	// store physical address
	__raw_writel((u32)virt_to_phys(p), UPLOAD_GETLOG_INFO_ADDR+location);
	
	location+=sizeof(u32);
}
EXPORT_SYMBOL(kernel_sec_set_log_ptrs_for_getlog);

void kernel_sec_init(void)
{
	extern void* get_kernel_log_mark(void);
	
	kernel_sec_info_ioremap();
	
	kernel_sec_set_upload_magic_number();
	kernel_sec_set_upload_cause(UPLOAD_CAUSE_INIT);	
	kernel_sec_map_wdog_reg();

	kernel_sec_set_log_ptrs_for_getlog(get_kernel_log_mark());
}
EXPORT_SYMBOL(kernel_sec_init);

/* core reg dump function*/
void kernel_sec_get_core_reg_dump(t_kernel_sec_arm_core_regsiters* regs)
{
	asm(
		// we will be in SVC mode when we enter this function. Collect SVC registers along with cmn registers.
		"str r0, [%0,#0] \n\t"		// R0
		"str r1, [%0,#4] \n\t"		// R1
		"str r2, [%0,#8] \n\t"		// R2
		"str r3, [%0,#12] \n\t"		// R3
		"str r4, [%0,#16] \n\t"		// R4
		"str r5, [%0,#20] \n\t"		// R5
		"str r6, [%0,#24] \n\t"		// R6
		"str r7, [%0,#28] \n\t"		// R7
		"str r8, [%0,#32] \n\t"		// R8
		"str r9, [%0,#36] \n\t"		// R9
		"str r10, [%0,#40] \n\t"	// R10
		"str r11, [%0,#44] \n\t"	// R11
		"str r12, [%0,#48] \n\t"	// R12

		/* SVC */
		"str r13, [%0,#52] \n\t"	// R13_SVC
		"str r14, [%0,#56] \n\t"	// R14_SVC
		"mrs r1, spsr \n\t"			// SPSR_SVC
		"str r1, [%0,#60] \n\t"

		/* PC and CPSR */
		"sub r1, r15, #0x4 \n\t"	// PC
		"str r1, [%0,#64] \n\t"	
		"mrs r1, cpsr \n\t"			// CPSR
		"str r1, [%0,#68] \n\t"

		/* SYS/USR */
		"mrs r1, cpsr \n\t"			// switch to SYS mode
		"and r1, r1, #0xFFFFFFE0\n\t"
		"orr r1, r1, #0x1f \n\t"
		"msr cpsr,r1 \n\t"

		"str r13, [%0,#72] \n\t"	// R13_USR
		"str r14, [%0,#76] \n\t"	// R13_USR

		/*FIQ*/
		"mrs r1, cpsr \n\t"			// switch to FIQ mode
		"and r1,r1,#0xFFFFFFE0\n\t"
		"orr r1,r1,#0x11\n\t"
		"msr cpsr,r1 \n\t"

		"str r8, [%0,#80] \n\t"		// R8_FIQ
		"str r9, [%0,#84] \n\t"		// R9_FIQ
		"str r10, [%0,#88] \n\t"	// R10_FIQ
		"str r11, [%0,#92] \n\t"	// R11_FIQ
		"str r12, [%0,#96] \n\t"	// R12_FIQ
		"str r13, [%0,#100] \n\t"	// R13_FIQ
		"str r14, [%0,#104] \n\t"	// R14_FIQ
		"mrs r1, spsr \n\t"			// SPSR_FIQ
		"str r1, [%0,#108] \n\t"

		/*IRQ*/
		"mrs r1, cpsr \n\t"			// switch to IRQ mode
		"and r1, r1, #0xFFFFFFE0\n\t"
		"orr r1, r1, #0x12\n\t"
		"msr cpsr,r1 \n\t"

		"str r13, [%0,#112] \n\t"	// R13_IRQ
		"str r14, [%0,#116] \n\t"	// R14_IRQ
		"mrs r1, spsr \n\t"			// SPSR_IRQ
		"str r1, [%0,#120] \n\t"

		/*MON*/
		"mrs r1, cpsr \n\t"			// switch to monitor mode
		"and r1, r1, #0xFFFFFFE0\n\t"
		"orr r1, r1, #0x16\n\t"
		"msr cpsr,r1 \n\t"

		"str r13, [%0,#124] \n\t"	// R13_MON
		"str r14, [%0,#128] \n\t"	// R14_MON
		"mrs r1, spsr \n\t"			// SPSR_MON
		"str r1, [%0,#132] \n\t"

		/*ABT*/
		"mrs r1, cpsr \n\t"			// switch to Abort mode
		"and r1, r1, #0xFFFFFFE0\n\t"
		"orr r1, r1, #0x17\n\t"
		"msr cpsr,r1 \n\t"

		"str r13, [%0,#136] \n\t"	// R13_ABT
		"str r14, [%0,#140] \n\t"	// R14_ABT
		"mrs r1, spsr \n\t"			// SPSR_ABT
		"str r1, [%0,#144] \n\t"

		/*UND*/
		"mrs r1, cpsr \n\t"			// switch to undef mode
		"and r1, r1, #0xFFFFFFE0\n\t"
		"orr r1, r1, #0x1B\n\t"
		"msr cpsr,r1 \n\t"

		"str r13, [%0,#148] \n\t"	// R13_UND
		"str r14, [%0,#152] \n\t"	// R14_UND
		"mrs r1, spsr \n\t"			// SPSR_UND
		"str r1, [%0,#156] \n\t"

		/* restore to SVC mode */
		"mrs r1, cpsr \n\t"			// switch to undef mode
		"and r1, r1, #0xFFFFFFE0\n\t"
		"orr r1, r1, #0x13\n\t"
		"msr cpsr,r1 \n\t"
		
		:				/* output */
        :"r"(regs)    	/* input */
        :"%r1"     		/* clobbered register */
        );

	return;	
}
EXPORT_SYMBOL(kernel_sec_get_core_reg_dump);

int kernel_sec_get_mmu_reg_dump(t_kernel_sec_mmu_info *mmu_info)
{
	asm("mrc    p15, 0, r1, c1, c0, 0 \n\t"	//SCTLR
		"str r1, [%0] \n\t"
		"mrc    p15, 0, r1, c2, c0, 0 \n\t"	//TTBR0
		"str r1, [%0,#4] \n\t"
		"mrc    p15, 0, r1, c2, c0,1 \n\t"	//TTBR1
		"str r1, [%0,#8] \n\t"
		"mrc    p15, 0, r1, c2, c0,2 \n\t"	//TTBCR
		"str r1, [%0,#12] \n\t"
		"mrc    p15, 0, r1, c3, c0,0 \n\t"	//DACR
		"str r1, [%0,#16] \n\t"
		"mrc    p15, 0, r1, c5, c0,0 \n\t"	//DFSR
		"str r1, [%0,#20] \n\t"
		"mrc    p15, 0, r1, c6, c0,0 \n\t"	//DFAR
		"str r1, [%0,#24] \n\t"
		"mrc    p15, 0, r1, c5, c0,1 \n\t"	//IFSR
		"str r1, [%0,#28] \n\t"
		"mrc    p15, 0, r1, c6, c0,2 \n\t"	//IFAR
		"str r1, [%0,#32] \n\t"
		/*Dont populate DAFSR and RAFSR*/
		"mrc    p15, 0, r1, c10, c2,0 \n\t"	//PMRRR
		"str r1, [%0,#44] \n\t"
		"mrc    p15, 0, r1, c10, c2,1 \n\t"	//NMRRR
		"str r1, [%0,#48] \n\t"
		"mrc    p15, 0, r1, c13, c0,0 \n\t"	//FCSEPID
		"str r1, [%0,#52] \n\t"
		"mrc    p15, 0, r1, c13, c0,1 \n\t"	//CONTEXT
		"str r1, [%0,#56] \n\t"
		"mrc    p15, 0, r1, c13, c0,2 \n\t"	//URWTPID
		"str r1, [%0,#60] \n\t"
		"mrc    p15, 0, r1, c13, c0,3 \n\t"	//UROTPID
		"str r1, [%0,#64] \n\t"
		"mrc    p15, 0, r1, c13, c0,4 \n\t"	//POTPIDR
		"str r1, [%0,#68] \n\t"
		:					/* output */
        :"r"(mmu_info)    /* input */
        :"%r1","memory"         /* clobbered register */
        ); 
	return 0;
}
EXPORT_SYMBOL(kernel_sec_get_mmu_reg_dump);

#ifdef CONFIG_CIQ_CRASH_CATCHER
int crash_catcher_info_store(
	t_kernel_sec_arm_core_regsiters *pregs, 
	t_kernel_sec_mmu_info *pmmu)
{
	t_crash_catcher_info *pinfo = (t_crash_catcher_info*)UPLOAD_CRASH_CATCHER_INFO_ADDR;

	pinfo->magic1 = 0xDEADDEAD;
	pinfo->magic2 = 0x01234567;
	
	pinfo->timestamp = ktime_to_ns(ktime_get());
	
	memcpy((void*)&pinfo->regs, pregs, sizeof(t_kernel_sec_arm_core_regsiters));
	memcpy((void*)&pinfo->mmuinfo, pregs, sizeof(t_kernel_sec_mmu_info));
	memcpy((void*)&pinfo->taskinfo.task, current, sizeof(pinfo->taskinfo));

	memcpy((void*)&pinfo->taskinfo.stack[0], current->stack, CC_STACK_SZ);
	
	printk(KERN_EMERG "Crash Catcher info stored\n");
	
	return 0;
}
#endif

void kernel_sec_save_final_context(void)
{
	if(	kernel_sec_get_mmu_reg_dump(&kernel_sec_mmu_reg_dump) < 0)
	{
		printk(KERN_EMERG"(kernel_sec_save_final_context) kernel_sec_get_mmu_reg_dump faile.\n");
	}
	kernel_sec_get_core_reg_dump(&kernel_sec_core_reg_dump);

	printk(KERN_EMERG "(kernel_sec_save_final_context) Final context was saved before the system reset.\n");
	
#ifdef CONFIG_CIQ_CRASH_CATCHER
	crash_catcher_info_store(&kernel_sec_core_reg_dump, &kernel_sec_mmu_reg_dump);
#endif
	
}
EXPORT_SYMBOL(kernel_sec_save_final_context);

/*
 *  bSilentReset
 *    TRUE  : Silent reset - clear the magic code.
 *    FALSE : Reset to upload mode - not clear the magic code.
 *
 *  TODO : DebugLevel consideration should be added.
 */
extern void Ap_Cp_Switch_Config(u16 ap_cp_mode);
void kernel_sec_hw_reset(bool bSilentReset)
{
	//Ap_Cp_Switch_Config(0);
	
	if (bSilentReset)
	{
		kernel_sec_clear_upload_magic_number();
		printk(KERN_EMERG "(kernel_sec_hw_reset) Upload Magic Code is cleared.\n");
	}
	
	printk(KERN_EMERG "(kernel_sec_hw_reset) The forced reset was called. The system will be reset !!\n");

	/* flush cache back to ram */
	flush_cache_all();
	
	__raw_writel(0x8000,kernel_sec_viraddr_wdt_reset_reg +0x4 );
	__raw_writel(0x1,   kernel_sec_viraddr_wdt_reset_reg +0x4 );
	__raw_writel(0x8,   kernel_sec_viraddr_wdt_reset_reg +0x8 );
	__raw_writel(0x8021,kernel_sec_viraddr_wdt_reset_reg);

    /* Never happened because the reset will occur before this. */
	while(1);	
}
EXPORT_SYMBOL(kernel_sec_hw_reset);

#ifdef CONFIG_CIQ_CRASH_CATCHER

static int crash_info_show(struct seq_file *m, void *v)
{
	t_crash_catcher_info *pinfo = (t_crash_catcher_info*)UPLOAD_CRASH_CATCHER_INFO_ADDR;
	int i;

	if(	pinfo->magic1 != 0xDEADDEAD || pinfo->magic2 != 0x01234567) {
		seq_printf(m, "Empty\n");
		return 0;
	}

	seq_printf(m, "=======================================================\n");
	seq_printf(m, "Crashed at %Lu\n", pinfo->timestamp);
	seq_printf(m, "=======================================================\n");

	seq_printf(m, "@ Registers\n");
#define P(x) seq_printf(m, " %-40s: %08x\n", #x, (x))

	// print registers
	P(pinfo->regs.r0);
	P(pinfo->regs.r1);
	P(pinfo->regs.r2);
	P(pinfo->regs.r3);
	P(pinfo->regs.r4);
	P(pinfo->regs.r5);
	P(pinfo->regs.r6);
	P(pinfo->regs.r7);
	P(pinfo->regs.r8);
	P(pinfo->regs.r9);
	P(pinfo->regs.r10);
	P(pinfo->regs.r11);
	P(pinfo->regs.r12);
	
	P(pinfo->regs.r13_svc);
	P(pinfo->regs.r14_svc);
	P(pinfo->regs.spsr_svc);

	P(pinfo->regs.pc);
	P(pinfo->regs.cpsr);
	
	P(pinfo->regs.r13_usr);
	P(pinfo->regs.r14_usr);

	P(pinfo->regs.r8_fiq);
	P(pinfo->regs.r9_fiq);
	P(pinfo->regs.r10_fiq);
	P(pinfo->regs.r11_fiq);
	P(pinfo->regs.r12_fiq);
	P(pinfo->regs.r13_fiq);
	P(pinfo->regs.r14_fiq);
	P(pinfo->regs.spsr_fiq);

	P(pinfo->regs.r13_irq);
	P(pinfo->regs.r14_irq);
	P(pinfo->regs.spsr_irq);

	P(pinfo->regs.r13_mon);
	P(pinfo->regs.r14_mon);
	P(pinfo->regs.spsr_mon);

	P(pinfo->regs.r13_abt);
	P(pinfo->regs.r14_abt);
	P(pinfo->regs.spsr_abt);

	P(pinfo->regs.r13_und);
	P(pinfo->regs.r14_und);
	P(pinfo->regs.spsr_und);	
#undef P

	// Task Info
	seq_printf(m, "@ Task Info \n");
	seq_printf(m, " %-20s : %s\n", "Name", pinfo->taskinfo.task.comm);
	seq_printf(m, " %-20s : %d\n", "PID", pid_vnr(task_pid(&pinfo->taskinfo.task)));
	seq_printf(m, " %-20s : %lu\n", "State", pinfo->taskinfo.task.state);
	seq_printf(m, " %-20s : %08X\n", "Flags", pinfo->taskinfo.task.flags);	
	seq_printf(m, " %-20s : %lu\n", "Start time", (long)pinfo->taskinfo.task.real_start_time.tv_sec);
	seq_printf(m, " %-20s : \n", "Stack data");
	for(i=0;i<CC_STACK_SZ;i++) {
		seq_printf(m, " %08X", pinfo->taskinfo.stack[i]);
		if((i+1)%8==0 & i!=0) {
			seq_printf(m, "\n");
		}
	}
	
	seq_printf(m, "\n=======================================================\n");

	return 0;
}

static int crash_info_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, crash_info_show, NULL);
}

static ssize_t crash_info_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *ppos)
{
	t_crash_catcher_info *pinfo = (t_crash_catcher_info*)UPLOAD_CRASH_CATCHER_INFO_ADDR;
	char c;
	int rc;

	rc = get_user(c, buffer);
	if (rc)
		return rc;
	if (c == '0') {
		pinfo->magic1 = 0xBEEFBEEF;
	}

	return count;
}

static const struct file_operations crash_info_fops = {
	.open		= crash_info_open,
	.read		= seq_read,
	.write      = crash_info_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init init_crash_info_procfs(void)
{
	struct proc_dir_entry *pe;

	pe = proc_create("crashinfo", 0666, NULL, &crash_info_fops);
	if (!pe)
		return -ENOMEM;
	return 0;
}

__initcall(init_crash_info_procfs);
#endif

#define CONFIG_MAKE_NICE
#ifdef CONFIG_MAKE_NICE
static int make_nice_show(struct seq_file *m, void *v)
{
	struct task_struct *tsk;

	seq_printf(m, "======================\n");
	seq_printf(m, " Task list (name:nice) \n");
	seq_printf(m, "======================\n");
	
	read_lock(&tasklist_lock);
	for_each_process (tsk) {
		//printk(KERN_ERR "!@# %s %s\n", tsk->comm);
		seq_printf(m, "%s:%d\n", tsk->comm, task_nice(tsk));
	}
	read_unlock(&tasklist_lock);
	seq_printf(m, "======================\n");

	return 0;
}
static int make_nice_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, make_nice_show, NULL);
}

#include <linux/security.h>

struct make_nice_struct {
	struct delayed_work delayed_work;
	char procname[TASK_COMM_LEN];
};

struct make_nice_struct make_nice_data;

static void make_nice_delayed_work_handler(struct work_struct *work)
{
	struct make_nice_struct *data = container_of(work, struct make_nice_struct, delayed_work.work);
	struct task_struct *tsk;
	
	printk(KERN_ERR "[make_nice] %s\n", __func__, data->procname);

	read_lock(&tasklist_lock);
	for_each_process(tsk) {
		//printk(KERN_ERR "[make_nice] %16s:%d \n", tsk->comm, task_nice(tsk));
		if(!strcmp(tsk->comm, data->procname)) {
			
			#if 1
			printk(KERN_ERR "[make_nice] change nice of %s:%d\n", tsk->comm, task_nice(tsk));
			// make it nicest
			set_user_nice(tsk,19);
			printk(KERN_ERR "[make_nice] nice of %s changed to %d\n", tsk->comm, task_nice(tsk));
			#else
			printk(KERN_ERR "KILL:  %s\n", tsk->comm);
			kill_pid(task_pid(tsk), SIGTERM, 1);
			#endif
			break;
		}
	}
	read_unlock(&tasklist_lock);

}

static ssize_t make_nice_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *ppos)
{
	char procname[TASK_COMM_LEN] = {0,};
	int stringlen = count;
	bool killed = 0;
	struct task_struct *tsk;

	if(count>TASK_COMM_LEN-1) {
		buffer = buffer + count - (TASK_COMM_LEN-1);
		stringlen = TASK_COMM_LEN-1;
	}
	
	copy_from_user((void*)&procname[0], (void*)buffer, stringlen);
	
	printk(KERN_ERR "[make_nice] requested proc : %s (called from %s)\n", procname, current->comm);

	strncpy(make_nice_data.procname, procname, TASK_COMM_LEN);
	schedule_delayed_work(&make_nice_data.delayed_work, 3000);

	return count;
}

static const struct file_operations make_nice_fops = {
	.open		= make_nice_open,
	.read		= seq_read,
	.write      = make_nice_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init init_make_nice_procfs(void)
{
	struct proc_dir_entry *pe;
	
	proc_mkdir("debug", NULL);
	pe = proc_create("debug/make_nice", 0666, NULL, &make_nice_fops);
	if (!pe)
		return -ENOMEM;
	
	INIT_DELAYED_WORK(&make_nice_data.delayed_work, make_nice_delayed_work_handler);

	return 0;
}

__initcall(init_make_nice_procfs);
#endif



#ifdef CONFIG_FORCED_PANIC

static int forced_panic_show(struct seq_file *m, void *v)
{
	struct forced_panic_info *pinfo = (struct forced_panic_info*)FORCED_PANIC_ST_ADDR;
	
	if(	pinfo->magic1 != 0x00DEAD00 || pinfo->magic2 != 0x12000034) {
		seq_printf(m, "Empty\n");
		return 0;
	}

	seq_printf(m, "=======================================================\n");
	seq_printf(m, "Forced panic message\n");
	seq_printf(m, "=======================================================\n");

	seq_printf(m, "\n%s\n\n", pinfo->msg);
	
	seq_printf(m, "=======================================================\n");

	return 0;
}
static int forced_panic_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, forced_panic_show, NULL);
}

static ssize_t forced_panic_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *ppos)
{
	struct forced_panic_info *pinfo = (struct forced_panic_info*)FORCED_PANIC_ST_ADDR;
	
	if(count >= sizeof(pinfo->msg)) {
		count = sizeof(pinfo->msg) - 1;
	}
	
	copy_from_user((void*)&pinfo->msg[0], (void*)buffer, count);

	pinfo->msg[count] = 0x0;
	
	pinfo->magic1 = 0x00DEAD00;
	pinfo->magic2 = 0x12000034;
	
	kernel_sec_save_final_context();
	kernel_sec_set_upload_cause(UPLOAD_CAUSE_FORCED_UPLOAD);
	kernel_sec_hw_reset(0);

	return count;
}

static const struct file_operations forced_panic_fops = {
	.open		= forced_panic_open,
	.read		= seq_read,
	.write      = forced_panic_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init init_forced_panic_procfs(void)
{
	struct proc_dir_entry *pe;

	pe = proc_create("forcepanic", 0666, NULL, &forced_panic_fops);
	if (!pe)
		return -ENOMEM;
	return 0;
}

__initcall(init_forced_panic_procfs);
#endif

#endif // CONFIG_KERNEL_DEBUG_SEC
