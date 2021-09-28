#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>
#include <linux/fs.h>

#ifndef __MDRV_DVFS_H__
#include "mdrv_dvfs.h"
#endif

#ifndef __MHAL_DVFS_H__
#include "mhal_dvfs.h"
#endif

#ifdef CONFIG_MSTAR_DVFS_DEBUG
extern void trigger_ir_boost_event(void);
extern bool del_all_boost_client(int cpu_id);
extern int _CPU_calibrating_proc_write(const unsigned long, const unsigned long, int);
static int Pass_Flag = 0;
static int Ret_Cmd = 0;
atomic_t dvfs_test_proc_is_open = ATOMIC_INIT(0);

static int dvfs_test_seq_show(struct seq_file *s, void *v)
{
    if (Pass_Flag == 0)
        seq_printf(s,"CMD: %d, Fail\n",Ret_Cmd);
    else if (Pass_Flag == 1)
        seq_printf(s,"CMD: %d, Pass\n",Ret_Cmd);
    Pass_Flag = 0;
    Ret_Cmd = 0;
    return 0;
}

static int dvfs_test_proc_open(struct inode *inode, struct file *file)
{
    if (atomic_read(&dvfs_test_proc_is_open))
        return -EACCES;

    atomic_set(&dvfs_test_proc_is_open, 1);

    return single_open(file, &dvfs_test_seq_show, NULL);
}

static int dvfs_test_proc_release(struct inode *inode, struct file * file)
{

    WARN_ON(!atomic_read(&dvfs_test_proc_is_open));
    atomic_set(&dvfs_test_proc_is_open, 0);
    return single_release(inode, file);
}

static void clear(void)
{
    int i = 0;
    for_each_online_cpu(i)
    {
        if (i == getCpuCluster(i) )
        {
            del_all_boost_client(i);
            MDrvDvfsSetTemperatureOffset(i,0);
        }
    }
    return;
}

static void setup(int status)
{
    int ret = 0;
    int i = 0;
    for_each_online_cpu(i)
    {
        if ( i == getCpuCluster(i) )
            ret = MDrvDvfsSetStatus(status,i);
    }
    if(ret)
        printk("Not Supprot Status %d\n",status);
    return;
}

#define FREQ_DEVIATION 3000 // tolerance between set freq and exact freq
static int verify_frequency(int target_freq,int cpu)
{
    int ret = 0;
    int freq = 0;
    if ( cpu == getCpuCluster(cpu) )
    {
       freq = MDrvDvfsGetCpuFreq(cpu)*1000;
       printk("target = %d, freq = %d\n",target_freq,freq);
       if (freq <= (target_freq - FREQ_DEVIATION) || freq >= (target_freq + FREQ_DEVIATION) )
           ret = 1;
    }
    return ret;
}

static int verify_status(unsigned int cpu,int status)
{
    int cluster = getCpuCluster(cpu);
    if (status == CONFIG_DVFS_FREEZE_MODE)
        return 0;
    else
        return (MDrvDvfsGetStatus(cluster) != status)?1:0;
}

static int verify_voltage(unsigned int cpu, unsigned freq)
{
    int cluster = getCpuCluster(cpu);
    return MDrvDvfsVerifyVoltage(cluster,freq);
}



/*
1: open file
2: close file
3: set normal
4: set freeze
5: set over temp
6: clear

7: trigger ir
8  trigger app

9: verify ir/app in over temp
10: verify ir in normal/freeze
11: verify app in normal/freeze

12: verify table
13: verify dhrystone
14: verify low loading

*/
#define FREQ_LEVEL 50000 // 50 MHz, for verify dvfs table
#define MAX_DMSG_WRITE_BUFFER 64
static int dvfs_test_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    char buffer[MAX_DMSG_WRITE_BUFFER];
    unsigned int garbage = 0;
    unsigned int cmd = 0;
    unsigned int ret = 0;
    unsigned int i = 0;
    unsigned int min_freq = 0, max_freq = 0, cur_freq = 0;

    if (!count)
        return count;

    if (count >= MAX_DMSG_WRITE_BUFFER)
        count = MAX_DMSG_WRITE_BUFFER - 1;

    if (copy_from_user(buffer, buf, count))
        return -EFAULT;

    buffer[count] = '\0';

    if(sscanf(buffer,"%d %d",&cmd, &garbage) == 2)
    {
       printk("usage: echo cmd > /proc/dvfs_test\n");
    }
    else if (sscanf(buffer, "%d", &cmd) == 1)
    {
       switch(cmd)
       {
           case 1:
           {
               char path[MAX_DMSG_WRITE_BUFFER]="";
               sscanf(buffer,"%d %s",&cmd,path);
               printk("path = %s\n",path);
               MDrvDvfsOpenFile(path);
               break;
           }
           case 2:
           {
               MDrvDvfsCloseFile();
               break;
           }
           case 3:
           {
               setup(CONFIG_DVFS_NORMAL_MODE);
               for_each_online_cpu(i)
               {
                   if (i == getCpuCluster(i))
                   {
                       ret += verify_status(i,CONFIG_DVFS_NORMAL_MODE);
                   }
               }
               break;
           }
           case 4:
           {
               setup(CONFIG_DVFS_FREEZE_MODE);
               for_each_online_cpu(i)
               {
                   if (i == getCpuCluster(i))
                   {
                       ret += verify_status(i,CONFIG_DVFS_FREEZE_MODE);
                   }
               }
               break;
           }
           case 5:
           {
               setup(CONFIG_DVFS_OVER_TEMPERATURE_MODE);
               for_each_online_cpu(i)
               {
                   if (i == getCpuCluster(i))
                   {
                       ret += verify_status(i,CONFIG_DVFS_OVER_TEMPERATURE_MODE);
                   }
               }
               break;
           }
           case 6:
           {
               clear();
               break;
           }
           case 7:
           {
               trigger_ir_boost_event();
               break;
           }
           case 8:
           {
               for_each_online_cpu(i)
               {
                   if (i == getCpuCluster(i))
                   {
                       _CPU_calibrating_proc_write(32, CONFIG_DVFS_CPU_CLOCK_MAX(i), i);
                   }
               }
               break;
           }
           case 9:
           {
               for_each_online_cpu(i)
               {
                   if (i == getCpuCluster(i))
                   {
                       ret += verify_frequency(CONFIG_DVFS_CPU_CLOCK_PROTECT(i),i);
                   }
               }
               break;
           }
           case 10:
           {
               for_each_online_cpu(i)
               {
                   if (i == getCpuCluster(i))
                   {
                      ret = verify_frequency(CONFIG_DVFS_CPU_IRBOOST_CLOCK(i),i);
                   }
               }
               break;
           }
           case 11:
           {
               for_each_online_cpu(i)
               {
                   if (i == getCpuCluster(i))
                   {
                      ret = verify_frequency(CONFIG_DVFS_CPU_CLOCK_MAX(i),i);
                   }
               }
               break;
           }
           case 12:
           {
               setup(CONFIG_DVFS_NORMAL_MODE);
               for_each_online_cpu(i)
               {
                   if (i == getCpuCluster(i))
                   {
                      min_freq = CONFIG_DVFS_CPU_CLOCK_MIN(i);
                      max_freq = CONFIG_DVFS_CPU_CLOCK_MAX(i);
                      for (cur_freq = min_freq; cur_freq <=max_freq; cur_freq += FREQ_LEVEL)
                      {
                          _CPU_calibrating_proc_write(0,cur_freq, i);
                          ret += verify_voltage(i,cur_freq);
                          _CPU_calibrating_proc_write(0,0, i);
                          //printk("freq = %d, pass\n",cur_freq);
                      }
                   }
               }
               clear();
               break;
           }
           case 13:
           {
               setup(CONFIG_DVFS_NORMAL_MODE);
               for_each_online_cpu(i)
               {
                   if (i == getCpuCluster(i))
                   {
                   #if defined(CONFIG_DVFS_DETACH_BOOST_FREQ_AND_MAX_FREQ)
                       ret += verify_frequency(CONFIG_DVFS_CPU_CLOCK_TABLE_MAX(i),i);
                   #else
                       ret += verify_frequency(CONFIG_DVFS_CPU_IRBOOST_CLOCK(i),i);
                   #endif
                   }
               }
               clear();
               break;
            }
           case 14:
           {
               setup(CONFIG_DVFS_NORMAL_MODE);
               for_each_online_cpu(i)
               {
                   if (i == getCpuCluster(i))
                       ret += verify_frequency(CONFIG_DVFS_CPU_CLOCK_MIN(i),i);
               }
               break;
           }
           default:
               printk("unknown cmd\n");
       }

       if(ret)
       {
           Pass_Flag = 0;
           printk("Test Case %d, FAIL\n",cmd);
       }
       else
       {
           Pass_Flag = 1;
           printk("Test Case %d, PASS\n",cmd);
       }
       Ret_Cmd = cmd;
    }
    return count;
}

const struct file_operations proc_dvfs_test_operations = {
        .write    = dvfs_test_proc_write,
        .read     = seq_read,
        .llseek   = seq_lseek,
        .open     = dvfs_test_proc_open,
        .release  = dvfs_test_proc_release,
};


int dvfs_debug_init(void)
{
    struct proc_dir_entry *entry;
    entry = proc_create("dvfs_test",S_IRUSR | S_IWUSR, NULL ,&proc_dvfs_test_operations);
    if (!entry)
        return 0;
    else
        return 1;
}
EXPORT_SYMBOL(dvfs_debug_init);
#endif
