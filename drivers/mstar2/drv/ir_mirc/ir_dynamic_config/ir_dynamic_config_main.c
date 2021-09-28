
//
// * Copyright (c) 2006 - 2007 MStar Semiconductor, Inc.
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

///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// file    ir_dynamic_config_main.c
/// @brief  Load IR config from INI config file
/// @author Vick.Sun@MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////



#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/kfifo.h>
#include <linux/reboot.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "ir_dynamic_config.h"
#include "iniparser.h"
#include "linux/init.h"

static int debug = 1;
#if defined(CONFIG_DATA_SEPARATION)
#define PROJECT_ID_LEN 5
#define PROJECT_ID_BUF_SIZE  50 //project_id.ini length
#define DATA_INDEX_NAME_BUF_SIZE  100//dataIndex.ini path length
#define IR_CONFIG_FILE_NAME_BUF_SIZE  50//ir_config.ini path length
#endif
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug:\n\t0: error only\n\t1:debug info\n\t2:all\n");

#if defined(CONFIG_DATA_SEPARATION)
static char *config_file = NULL;
#else
static inline int isRecoveryBoot(void)
{
	return strstr(saved_command_line, "recovery_mode=true") != NULL;
}
static char ir_config_path[IR_CONFIG_PATH_LEN] = CONFIG_IR_CONFIG_PATH;
static char ir_config_path_recovery[IR_CONFIG_PATH_LEN] = "/tvconfig_recovery/config/ir_config.ini";
static char *config_file;
#endif

#if defined(CONFIG_IR_SKYWORTH_FACTORY)
static char ir_config_path_diag[IR_CONFIG_PATH_LEN] = "/vendor/tvconfig/config/ir_config_diag.ini";
extern unsigned int idme_get_bootmode(void);
#endif
module_param(config_file,charp, 0644);
MODULE_PARM_DESC(config_file, "config file path");

#define DBG_ALL(__fmt,args...)              do{if (2 <= debug){printk("[IR config]"__fmt,##args);}}while(0)
#define DBG_INFO(__fmt,args...)             do{if (1 <= debug){printk("[IR config]"__fmt,##args);}}while(0)
#define DBG_ERR(__fmt,args...)              do{if (0 <= debug){printk("[IR config]"__fmt,##args);}}while(0)

#define SIZE(array)  (sizeof(array)/sizeof(array[0]))

static IR_Profile_t *ir_profiles = NULL;
static int ir_profile_num = 0;
static int ir_profile_parse_num = 0;
struct key_map_list *key_map_lists = NULL;
#if defined(CONFIG_MSTAR_PM)
static IR_PM51_Profile_t *ir_PM51_profiles = NULL;
static int ir_PM51_profile_num = 0;
static int ir_PM51_profile_parse_num = 0;
static u32 *ir_PM51WK_profiles;
static int ir_PM51WK_profile_parse_num;
static unsigned char ir_PM51_cfg_data[32] = {0};
static unsigned char ir2_PM51_cfg_data[16] = {0};
static unsigned char *ir_p16 = &ir_PM51_cfg_data[IR_HEAD16_KEY_PM_CFG_OFFSET];
static unsigned char *ir_p32 = &ir_PM51_cfg_data[IR_HEAD32_KEY_PM_CFG_OFFSET];
#endif

key_value_t IR_KeysList[] = {
#include "input_keys_available.h"
};

static void load_ir_config_thread(struct work_struct *work);
static int load_ir_config_thread_need_stop = 0;
static DECLARE_DELAYED_WORK(load_ir_config_work, load_ir_config_thread);



unsigned int get_ir_keycode(unsigned int scancode)
{
	int i = 0;
	unsigned int ir_header_code = 0;
	unsigned int keycode = KEY_RESERVED;

	if (ir_PM51_profiles == NULL) {
		DBG_ERR("ir_PM51_profiles is NULL\n");
		return keycode;
	}

	DBG_ALL("ir_PM51_profile_parse_num:%d\n", ir_PM51_profile_parse_num);
	//Search IR Header from PM51 profiles first
	for (i = 0; i < ir_PM51_profile_parse_num; i++) {
		if (scancode == ir_PM51_profiles[i].u32PowerKey) {
			ir_header_code = ir_PM51_profiles[i].u32Header;
			DBG_ALL("Match ir_PM51_profiles success, scancode:%u, ir_header_code:%u\n", scancode, ir_header_code);
			break;
		}
	}
	if (i == ir_PM51_profile_parse_num) {
		DBG_ERR("Match ir_PM51_profiles failed\n");
		return keycode;
	}

	//Get keycode from ir_header_code and scancode
	keycode = MIRC_Get_Keycode(ir_header_code, scancode);
	return keycode;
}
EXPORT_SYMBOL(get_ir_keycode);

static void fileread(char *filename, char **data)
{
    struct file *filp;
    mm_segment_t fs;
    loff_t size;
    ssize_t ret;
    filp = filp_open(filename, O_RDONLY, 0644);
    if (IS_ERR(filp))
    {
		DBG_ALL("open %s error...\n", filename);
        return;
    }
    fs = get_fs();
    set_fs(KERNEL_DS);
    size = filp->f_op->llseek(filp, 0, SEEK_END);
    *data = (char *)kmalloc(size + 1, GFP_KERNEL);
    filp->f_op->llseek(filp, 0, SEEK_SET);
    ret = vfs_read(filp, *data, size, &filp->f_pos);
    if (ret != size)
    {
        DBG_ERR(KERN_WARNING"read %s error!\n", filename);
        kfree(*data);
        *data = NULL;
        goto out;
    }
    (*data)[size] = 0;
out:
    set_fs(fs);
    filp_close(filp, NULL);
}
static int str2ul(const char *str, u32 *val)
{
    if (sscanf(str, "%i", val) == -1)
    {
        return -1;
    }
    return 0;
}

static void parse_all_keymaps(IniSectionNode *root, const char *name, const char *value)
{
    IniKeyNode *k;
    const char *enable;
    const char *protocol;
    const char *header;
    const char *speed;
    const char *keymap_name;
    IniSectionNode *keymap_sec;
    int keymap_size;
    struct key_map_table *table;
    int i;
    IR_Profile_t *curr_ir_profile = &ir_profiles[ir_profile_parse_num];
    IniSectionNode *ir_config = get_section(root, value);
    if(ir_profile_parse_num > 15)return;
    if (ir_config == NULL)
    {
        DBG_ERR("no section named %s\n", value);
        return;
    }
    /**
     * Keymap is Enabled?
     */
    enable = get_key_value(ir_config,"Enable");
    if(enable && (*enable == 'f' || *enable == 'F' || *enable == '0' || *enable == 'n' || *enable == 'N'))
    {
        return;
    }
    curr_ir_profile->u8Enable = true;
    /**
     * Parse Protocol
     */
    protocol = get_key_value(ir_config, "Protocol");
    if (protocol == NULL)
    {
        DBG_ERR("no Protocol for %s\n", ir_config->name);
        return;
    }
    if(str2ul(protocol,&curr_ir_profile->eIRType))
    {
        DBG_ERR("Protocol for %s format error.\n", ir_config->name);
        return;
    }
    DBG_INFO("Protocol value:0x%02x\n",curr_ir_profile->eIRType);
    curr_ir_profile->eIRType--;
    /**
     * Parse Header code
     */
    header = get_key_value(ir_config, "Header");
    if (header == NULL)
    {
        DBG_ERR("no Header for %s\n", ir_config->name);
        return;
    }
    if (str2ul(header, &curr_ir_profile->u32HeadCode))
    {
        DBG_ERR("header code format err for %s\n", ir_config->name);
        return;
    }
    DBG_INFO("Header code:0x%x\n", curr_ir_profile->u32HeadCode);
    /**
     * Parse IR Speed
     */
    speed = get_key_value(ir_config, "Speed");
    if (speed == NULL)
    {
        DBG_ERR("Speed for %s is empty. use default.\n", ir_config->name);
    }
    else
    {
        if (str2ul(speed, &curr_ir_profile->u32IRSpeed))
        {
            DBG_ERR("IR Speed format err for %s\n", ir_config->name);
            return;
        }
        DBG_INFO("Speed:0x%x\n", curr_ir_profile->u32IRSpeed);
    }
    /**
     * Parse Keymap
     */
    keymap_name = get_key_value(ir_config, "Keymap");
    if (keymap_name == NULL)
    {
        DBG_ERR("no Keymap key for %s\n", ir_config->name);
        return;
    }
    keymap_sec = get_section(root, keymap_name);
    if (keymap_sec == NULL)
    {
        DBG_ERR("no Keymap section for %s\n", keymap_name);
        return;
    }
    keymap_size = get_key_num(keymap_sec);
    if (keymap_size == 0)
    {
        DBG_ERR("no keys in section %s\n", keymap_name);
        return;
    }
    table = kmalloc(sizeof(struct key_map_table) * keymap_size, GFP_KERNEL);
    if (table == NULL)
    {
        DBG_ERR(KERN_ERR"OOM!\n");
        return;
    }
    memset(table, 0, sizeof(struct key_map_table) * keymap_size);
    key_map_lists[ir_profile_parse_num].map.scan = table;
    for (k = keymap_sec->keys;k != NULL; k = k->next)
    {
        for ( i = 0; i < SIZE(IR_KeysList); i++ )
        {
            if (!strcmp(k->name, IR_KeysList[i].key))
            {
                table->keycode = IR_KeysList[i].value;

                if (str2ul(k->value, &table->scancode))
                {
                    DBG_ERR("scan code format err for %s\n", keymap_sec->name);
                    return;
                }
                DBG_ALL("KEY name:%s,KEY val:0x%x,scan code:0x%x\n", k->name, table->keycode, table->scancode);
                table++;
                break;
            }
        }
        if(i == SIZE(IR_KeysList))
        {
            if(str2ul(k->name,&table->keycode))
            {
                DBG_ERR("non-standard key format err for %s\n", k->name);
                return;
            }
            if(str2ul(k->value,&table->scancode))
            {
                DBG_ERR("non-standard key's value format err for %s=%s\n", k->name,k->value);
                return;
            }
            DBG_ALL("KEY name:%s,KEY val:0x%x,scan code:0x%x\n", k->name, table->keycode, table->scancode);
            table++;
        }
    }
    key_map_lists[ir_profile_parse_num].map.headcode = curr_ir_profile->u32HeadCode;
    key_map_lists[ir_profile_parse_num].map.size = keymap_size;
    key_map_lists[ir_profile_parse_num].map.name = kstrdup(keymap_name, GFP_KERNEL);
    DBG_INFO("register config:%s with Header code 0x%x\n", ir_config->name, curr_ir_profile->u32HeadCode);
    DBG_INFO("register keymap:%s\n", keymap_name);
    MIRC_IRCustomer_Add(curr_ir_profile);
    MIRC_Map_Register(&key_map_lists[ir_profile_parse_num]);
    ir_profile_parse_num++;
}

#if defined(CONFIG_MSTAR_PM)
static void parse_PM51_all_keymaps(IniSectionNode *root, const char *name, const char *value)
{
    const char *protocol;
    const char *header;
    const char *power_key;
    IR_PM51_Profile_t *curr_ir_PM51_profile = &ir_PM51_profiles[ir_PM51_profile_parse_num];
    IniSectionNode *ir_PM51_config = get_section(root, value);
    if (ir_PM51_config == NULL)
    {
        DBG_ERR("no section named %s\n", value);
        return;
    }
    /**
     * Parse Protocol
     */
    protocol = get_key_value(ir_PM51_config, "Protocol");
    if (protocol == NULL)
    {
        DBG_ERR("no Protocol for %s\n", ir_PM51_config->name);
        return;
    }
    if(str2ul(protocol,&curr_ir_PM51_profile->eIRType))
    {
        DBG_ERR("Protocol for %s format error.\n", ir_PM51_config->name);
        return;
    }
    DBG_INFO("Protocol value:0x%02x\n",curr_ir_PM51_profile->eIRType);
    /**
     * Parse Header code
     */
    header = get_key_value(ir_PM51_config, "Header");
    if (header == NULL)
    {
        DBG_ERR("no Header for %s\n", ir_PM51_config->name);
        return;
    }
    if (str2ul(header, &curr_ir_PM51_profile->u32Header))
    {
        DBG_ERR("header code format err for %s\n", ir_PM51_config->name);
        return;
    }
    DBG_INFO("Header code:0x%x\n", curr_ir_PM51_profile->u32Header);

    /**
     * Parse PowerKey
     */
    power_key = get_key_value(ir_PM51_config, "Key");
    if (power_key == NULL)
    {
        DBG_ERR("no PowerKey for %s\n", ir_PM51_config->name);
        return;
    }
    if (str2ul(power_key, &curr_ir_PM51_profile->u32PowerKey))
    {
        DBG_ERR("PowerKey code format err for %s\n", ir_PM51_config->name);
        return;
    }
    DBG_INFO("PowerKey code:0x%x\n", curr_ir_PM51_profile->u32PowerKey);

	//IR_SUPPORT_PM_NUM_MAX	only restore 5 PM51 key maps, need to get the rest PM51 data from ir_config.ini
	if (ir_PM51_profile_parse_num < IR_SUPPORT_PM_NUM_MAX) {
		if ((curr_ir_PM51_profile->u32Header > 0x0000ffff) || (ir_PM51_profile_parse_num == (IR_SUPPORT_PM_NUM_MAX - 1))) {
			*ir_p32++ = (u8)(curr_ir_PM51_profile->eIRType & 0xff);
			*ir_p32++ = (u8)((curr_ir_PM51_profile->u32Header >> 24) & 0xff);
			*ir_p32++ = (u8)((curr_ir_PM51_profile->u32Header >> 16) & 0xff);
			*ir_p32++ = (u8)((curr_ir_PM51_profile->u32Header >> 8) & 0xff);
			*ir_p32++ = (u8)(curr_ir_PM51_profile->u32Header & 0xff);
			*ir_p32++ = (u8)((curr_ir_PM51_profile->u32PowerKey >> 8) & 0xff);
			*ir_p32++ = (u8)(curr_ir_PM51_profile->u32PowerKey & 0xff);
		} else {
			*ir_p16++ = (u8)(curr_ir_PM51_profile->eIRType & 0xff);
			*ir_p16++ = (u8)((curr_ir_PM51_profile->u32Header >> 8) & 0xff);
			*ir_p16++ = (u8)(curr_ir_PM51_profile->u32Header & 0xff);
			*ir_p16++ = (u8)((curr_ir_PM51_profile->u32PowerKey >> 8) & 0xff);
			*ir_p16++ = (u8)(curr_ir_PM51_profile->u32PowerKey & 0xff);
		}
	}

    ir_PM51_profile_parse_num++;
}

static void parse_PM51WK_all_keymaps(IniSectionNode *root, const char *name, const char *value)
{
	const char *power_key;
	u32 *curr_ir_PM51_profile = &ir_PM51WK_profiles[ir_PM51WK_profile_parse_num];
	IniSectionNode *ir_PM51_config = get_section(root, value);
	if (ir_PM51_profile_parse_num > (IR2_SUPPORT_PM_NUM_MAX - 1))
		return;

	if (ir_PM51_config == NULL) {
		DBG_ERR("no section named %s\n", value);
		return;
	}

	/**
	 * Parse WakeupKey
	 */
	power_key = get_key_value(ir_PM51_config, "Key");
	if (power_key == NULL) {
		DBG_ERR("no WakeupKey for %s\n", ir_PM51_config->name);
		return;
	}
	if (str2ul(power_key, curr_ir_PM51_profile)) {
		DBG_ERR("PowerKey code format err for %s\n", ir_PM51_config->name);
		return;
	}
	ir2_PM51_cfg_data[ir_PM51WK_profile_parse_num] = ((u8)*curr_ir_PM51_profile & 0xff);
	DBG_INFO("PowerKey code:0x%x\n", ir2_PM51_cfg_data[ir_PM51WK_profile_parse_num]);

	ir_PM51WK_profile_parse_num++;
}
#endif

#if defined(CONFIG_DATA_SEPARATION)
static void getProjectId(char **data)
{
    char *str = NULL;
    extern char *saved_command_line;
    char * tmp_command_line = NULL;
    tmp_command_line = (char *)kmalloc(strlen(saved_command_line) + 1, GFP_KERNEL);
    memset(tmp_command_line, 0, strlen(saved_command_line) + 1);
    memcpy(tmp_command_line, saved_command_line, strlen(saved_command_line) + 1);
    str = strstr(tmp_command_line, "cusdata_projectid");
    if (str)
    {
        *data = (char *)kmalloc(PROJECT_ID_LEN + 1, GFP_KERNEL);
        memset(*data, 0, PROJECT_ID_LEN + 1);
        char* pTypeStart = strchr(str, '=');
        if(pTypeStart)
        {
            if(strlen(pTypeStart) > PROJECT_ID_LEN)
            {
                char* pTypeEnd = strchr(pTypeStart+1, ' ');
                if(!pTypeEnd)
                {
                    kfree(tmp_command_line);
                    kfree(*data);
                    return;
                }
                *pTypeEnd = '\0';
            }
            memcpy(*data, pTypeStart+1, strlen(pTypeStart));
            printk("projectID = %s \n", *data);
            return;
        }
        kfree(*data);
    }
    kfree(tmp_command_line);
    printk("can not get project id from bootargs\n");
    return;
}


static void getDataIndexIniFile(char **data)
{
    char *project_id = NULL;
    //When accessing user memory, we need to make sure the entire area really is in user-level space.
    //KERNEL_DS addr user-level space need less than TASK_SIZE
    mm_segment_t fs=get_fs();
    set_fs(KERNEL_DS);
    *data = (char *)kzalloc(DATA_INDEX_NAME_BUF_SIZE+1, GFP_KERNEL);
    if(!(*data))
    {
        set_fs(fs);
        kfree(project_id);
        return false;
    }
    getProjectId(&project_id);
    snprintf(*data,DATA_INDEX_NAME_BUF_SIZE,"/cusdata/config/dataIndex/dataIndex_%s.ini",project_id);
    kfree(project_id);
    set_fs(fs);
    return true;
}

static void getIrConfigIniFile(char **data)
{
    char *data_index_buf = NULL;
    char *data_index_name = NULL;
    int u16BufIdx=0;
    char *str=NULL;
    int u8CntSwitch=0;
    //When accessing user memory, we need to make sure the entire area really is in user-level space.
    //KERNEL_DS addr user-level space need less than TASK_SIZE
    mm_segment_t fs=get_fs();
    set_fs(KERNEL_DS);

    //get dataIndex.ini path
    getDataIndexIniFile(&data_index_name);
    printk("dataIndex path = %s\n", data_index_name);
    //read dataIndex.ini
    fileread(data_index_name, &data_index_buf);
    kfree(data_index_name);
    if (data_index_buf == NULL)
    {
        set_fs(fs);
        return;
    }
    *data = (char*)kmalloc(IR_CONFIG_FILE_NAME_BUF_SIZE, GFP_KERNEL);
    memset(*data, 0, IR_CONFIG_FILE_NAME_BUF_SIZE + 1);
    //parser dataIndex.ini
    str = strstr(data_index_buf,"m_pIrConfig_File");
    if (str)
    {
        *data = (char*)kmalloc(IR_CONFIG_FILE_NAME_BUF_SIZE, GFP_KERNEL);
        char* pTypeStart = strchr(str, '"');
        if(pTypeStart)
        {
            char* pTypeEnd = strchr(pTypeStart+1, '"');
            if(pTypeEnd)
            {
                *pTypeEnd = '\0';
                memcpy(*data, pTypeStart+1, strlen(pTypeStart));
                printk("m_pIrConfig_File = %s \n", *data);
                set_fs(fs);
                kfree(data_index_buf);
                return;
            }
        }
        kfree(*data);
    }
    set_fs(fs);
    kfree(data_index_buf);
}
#endif

extern int MIRC_support_ir_max_timeout(void);

static int ir_config_state_open(struct inode *inode, struct file *file)
{
	pr_info("ir_config_state_open is open\n");
	return 0;
}

static const struct file_operations ir_config_ops = {
	.open = ir_config_state_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static void load_ir_config_thread(struct work_struct *work)
{
	struct proc_dir_entry *entry;
    while (!load_ir_config_thread_need_stop)
    {
        char *contents = NULL;
        IniSectionNode *root;
        IniSectionNode *kernel_sec;
#if defined(CONFIG_MSTAR_PM)
        IniSectionNode *PM51_sec;
#endif
#if defined(CONFIG_DATA_SEPARATION)
        getIrConfigIniFile(&config_file);
        printk("ir_config.ini path = %s\n", config_file);
#else
#if defined(CONFIG_IR_SKYWORTH_FACTORY)
		if (idme_get_bootmode() == 2) { // Diag mode
			config_file = ir_config_path_diag;
		} else {
			config_file = isRecoveryBoot() ? ir_config_path_recovery : ir_config_path;
		}
#else
		config_file = isRecoveryBoot() ? ir_config_path_recovery : ir_config_path;
#endif
#endif
		if (config_file != NULL) {
			fileread(config_file, &contents);
		}
        if (contents == NULL)
        {
            msleep(400);
            continue;
        }


        if (alloc_and_init_ini_tree(&root, contents))
        {
            DBG_ERR(KERN_ERR"OOM!\n");
            goto out;
        }
        if (debug > 1)
        {
            dump_ini(root);
        }
        kernel_sec = get_section(root, "Kernel");
        if (kernel_sec == NULL)
        {
            DBG_ERR("no section named Kernel\n");
            goto out;
        }
        ir_profile_num = get_section_num(kernel_sec);
        if (ir_profile_num == 0)
        {
            DBG_ERR("no no keys in section Kernel\n");
            goto out;
        }
        ir_profiles = kmalloc(sizeof(IR_Profile_t) * ir_profile_num, GFP_KERNEL);
        key_map_lists = kmalloc(sizeof(struct key_map_list) * ir_profile_num, GFP_KERNEL);
        if ((key_map_lists == NULL) || (ir_profiles == NULL))
        {
            DBG_ERR(KERN_ERR"OOM!\n");
        }
        memset(ir_profiles, 0, sizeof(IR_Profile_t) * ir_profile_num);
        memset(key_map_lists, 0, sizeof(struct key_map_list) * ir_profile_num);
        foreach_key(root, kernel_sec, parse_all_keymaps);
        MIRC_IRCustomer_Init();
        mstar_ir_reinit();
        MIRC_support_ir_max_timeout();
#if defined(CONFIG_MSTAR_PM)
        PM51_sec = get_section(root, "PM51");
        if (PM51_sec == NULL)
        {
            DBG_ERR("no section named PM51\n");
            goto out;
        }
        ir_PM51_profile_num = get_key_num(PM51_sec);
        if (ir_PM51_profile_num == 0)
        {
            DBG_ERR("no no keys in section PM51\n");
            goto out;
        }
        ir_PM51_profiles = kmalloc(sizeof(IR_PM51_Profile_t) * ir_PM51_profile_num, GFP_KERNEL);
        if (ir_PM51_profiles == NULL)
        {
            DBG_ERR(KERN_ERR"OOM!\n");
        }
	ir_PM51WK_profiles = kmalloc(sizeof(u32) * ir_PM51_profile_num, GFP_KERNEL);
	if (ir_PM51WK_profiles == NULL) {
		DBG_ERR(KERN_ERR"OOM!\n");
	}
        memset(ir_PM51_profiles, 0, sizeof(IR_PM51_Profile_t) * ir_PM51_profile_num);
	memset(ir_PM51WK_profiles, 0, sizeof(u8) * ir_PM51_profile_num);
        foreach_key(root, PM51_sec, parse_PM51_all_keymaps);
	foreach_key(root, PM51_sec, parse_PM51WK_all_keymaps);
        MDrv_PM_Set_IRCfg(ir_PM51_cfg_data, sizeof(ir_PM51_cfg_data));
	MDrv_PM_Set_IR2Cfg(ir2_PM51_cfg_data, sizeof(ir2_PM51_cfg_data));
	if (ir_PM51WK_profiles) {
		kfree(ir_PM51WK_profiles);
		ir_PM51WK_profiles = NULL;
	}
#endif
    out:
        release_ini_tree(root);
        kfree(contents);
	msleep(500);
		pr_info("Create a temporary proc file to inidicate ini parse is done \n");
		entry = proc_create_data("ir_config_done", 0444, NULL, &ir_config_ops, NULL);
		if (!entry) {
			pr_err("Could not create /proc/ir_config_done \n");
		}
        return;
    }
}

static int __init ir_dynamic_config_init(void)
{
    schedule_delayed_work(&load_ir_config_work,10);
    return 0;
}

static void __exit ir_dynamic_config_exit(void)
{
    int i;
    load_ir_config_thread_need_stop = 1;
    cancel_delayed_work_sync(&load_ir_config_work);
    for ( i = 0; i < ir_profile_parse_num; i++ )
    {
        if (key_map_lists != NULL)
        {
            if (key_map_lists[i].map.scan != NULL)
            {
                MIRC_Map_UnRegister(&key_map_lists[i]);
                kfree(key_map_lists[i].map.scan);
                key_map_lists[i].map.scan = NULL;
            }
            if (key_map_lists[i].map.name != NULL)
            {
                DBG_INFO("unregister keymap:%s\n", key_map_lists[i].map.name);
                kfree(key_map_lists[i].map.name);
                key_map_lists[i].map.name = NULL;
            }
        }
    }

    if (key_map_lists)
    {
        kfree(key_map_lists);
        key_map_lists = NULL;
    }
    if (ir_profiles)
    {
        for ( i = 0; i < ir_profile_parse_num; i++ )
        {
            DBG_INFO("register config with Header code:%x\n", ir_profiles[i].u32HeadCode);
            MIRC_IRCustomer_Remove(&ir_profiles[i]);
        }
        kfree(ir_profiles);
        ir_profiles = NULL;
    }
}

module_init(ir_dynamic_config_init);
module_exit(ir_dynamic_config_exit);

MODULE_DESCRIPTION("IR dynamic config module for MSTAR");
MODULE_AUTHOR("Vick Sun <vick.sun@mstarsemi.com>");
MODULE_LICENSE("GPL v2");
