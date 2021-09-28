////////////////////////////////////////////////////////////////////////////////////////////////////
//
// * Copyright (c) 2006 - 2017 MStar Semiconductor, Inc.
// This program is free software.
// You can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation;
// either version 2 of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with this program;
// if not, write to the Free Software Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
//
///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @file   mdrv_util.c
/// @brief  mdrv_util Driver Interface
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/anon_inodes.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/version.h>
#include "mdrv_util.h"
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,53)
#include <linux/sched/signal.h>
#include <uapi/linux/sched/types.h>
#endif

//-------------------------------------------------------------------------------------------------
//  Local Variables
//-------------------------------------------------------------------------------------------------
LIST_HEAD(OriginTable);
LIST_HEAD(TuneList);

//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------
void ClearTuneList(void)
{
       struct tune_node *tune_iterator;
       struct tune_node *tmp_tune;
       list_for_each_entry_safe(tune_iterator, tmp_tune, &TuneList,node)
       {
               list_del(&tune_iterator->node);
               kfree(tune_iterator);
       }
}

void ClearOriginTable(void)
{
       struct origin_node *tmp_origin;
       struct origin_node *origin_iterator;
       list_for_each_entry_safe(origin_iterator, tmp_origin, &OriginTable,node)
       {
               list_del(&origin_iterator->node);
               kfree(origin_iterator);
       }
}

void MDrv_UTIL_SaveCurrentScheduler(void)
{
    struct task_struct *task;
    struct task_struct *gtask;

    do_each_thread(gtask,task)
    {
        struct origin_node* pOriginNode;
        pOriginNode = kmalloc(sizeof(struct origin_node),GFP_KERNEL);
        if(pOriginNode == NULL)
        {
                printk (KERN_INFO "\033[1;31m allocate memory fail \033[m\n");
                if(!list_empty(&OriginTable))
                {
                        ClearOriginTable();
                }
                return;
        }
        pOriginNode->pid = task->pid;
        pOriginNode->prio = task->rt_priority;
        pOriginNode->policy = task->policy;
        list_add(&pOriginNode->node,&OriginTable);
    }
    while_each_thread(gtask,task);
}

int MDrv_UTIL_SetScheduler(unsigned long arg)
{
    int ret = -1;
    struct tune_table TuneTable;
    struct tune_node *tune_iterator;
    struct task_struct *task;
    struct task_struct *gtask;
    struct sched_param target;
    struct tune_task other;

    if(list_empty(&OriginTable))
    {
        printk (KERN_INFO "\033[1;31m Please save current schedule first!! \033[m\n");
        return -EFAULT;
    }

    if (copy_from_user(&TuneTable, (void __user *)arg, sizeof(struct tune_table)))
    {
        printk (KERN_INFO "\033[1;31m user arg is a bad address  \033[m\n");
        return -EFAULT;
    }

    if(!TuneTable.list || !TuneTable.other)
    {
        printk (KERN_INFO "\033[1;31m both list and other can't be empty  \033[m\n");
        return -EFAULT;
    }

    //store set value of other thread
    if (copy_from_user(&other, (void __user *)TuneTable.other, sizeof(struct tune_task)))
    {
        printk (KERN_INFO "\033[1;31m other is a bad address  \033[m\n");
        return -EFAULT;
    }

    //clear last set value of specific thread
    if(!list_empty(&TuneList))
    {
               ClearTuneList();
    }

    //store set value of specific thread
    do
    {
        struct tune_node* pTuneNode;
        pTuneNode = kmalloc(sizeof(struct tune_node),GFP_KERNEL);
        if(pTuneNode == NULL)
        {
                printk (KERN_INFO "\033[1;31m allocate memory fail \033[m\n");
                if(!list_empty(&TuneList))
                {
                        ClearTuneList();
                }
                return -EFAULT;
        }
        if (copy_from_user(&pTuneNode->tune, (void __user *)TuneTable.list, sizeof(struct tune_task)))
        {
            printk (KERN_INFO "\033[1;31m there is a bad node in list  \033[m\n");
            if(!list_empty(&TuneList))
            {
                    ClearTuneList();
            }
            kfree(pTuneNode);
            return -EFAULT;
        }
        list_add(&pTuneNode->node,&TuneList);
        if (copy_from_user(&TuneTable.list, (void __user *)TuneTable.list->next, sizeof(struct tune_task*)))
        {
            printk (KERN_INFO "\033[1;31m there is a bad node in list  \033[m\n");
            if(!list_empty(&TuneList))
            {
                    ClearTuneList();
            }
            kfree(pTuneNode);
            return -EFAULT;
        }
    }while(TuneTable.list);

    //set new status to task
    do_each_thread(gtask,task)
    {
        rcu_read_lock();
        target.sched_priority = other.prio;
        ret = sched_setscheduler(task, other.policy, &target);
        rcu_read_unlock();
        list_for_each_entry(tune_iterator, &TuneList,node)
        {
            if(strcmp(task->comm,tune_iterator->tune.comm) == 0)
            {
                rcu_read_lock();
                target.sched_priority = tune_iterator->tune.prio;
                ret = sched_setscheduler(task, tune_iterator->tune.policy, &target);
                rcu_read_unlock();
                if(ret != 0) printk (KERN_INFO "\033[1;31m Set task(comm = %s , prio = %d , policy = %d) scheduler fail!!\033[m\n",tune_iterator->tune.comm,tune_iterator->tune.prio,tune_iterator->tune.policy);
                break;
            }
        }
    }
    while_each_thread(gtask,task);
    return ret;
}

int MDrv_UTIL_RestoreScheduler(void)
{
    int ret = -1;
    struct origin_node *origin_iterator;
    struct task_struct *task;
    struct sched_param target;

    if(list_empty(&OriginTable))
    {
        printk(KERN_INFO "[mdrv_util] Please save current schedule first!!\n");
        return -EFAULT;
    }

    list_for_each_entry(origin_iterator, &OriginTable,node)
    {
        struct pid * kpid=find_get_pid(origin_iterator->pid);
        rcu_read_lock();
        task = pid_task(kpid,PIDTYPE_PID);
        rcu_read_unlock();
        if (task != NULL)
        {
            rcu_read_lock();
            target.sched_priority = origin_iterator->prio;
            ret = sched_setscheduler(task, origin_iterator->policy, &target);
            rcu_read_unlock();
        }
        else
        {
            printk (KERN_INFO "[mdrv_util] Warning!! pid: %d No such process\n", origin_iterator->pid);
        }
    }

    //clear tune list
    if(!list_empty(&TuneList))
    {
         ClearTuneList();
    }

    //clear origin table
    if(!list_empty(&OriginTable))
    {
         ClearOriginTable();
    }
    return ret;
}

