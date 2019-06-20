/* drivers/sharp/shlog/shrlog.c
 * 
 * Copyright (C) 2010 Sharp Corporation
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*==============================================================================
  Includes
  ==============================================================================*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <sharp/shrlog.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <asm-generic/sections.h>
#include <asm/memory.h>
#ifdef CONFIG_ARM
#include <asm/unwind.h>
#endif /* CONFIG_ARM */

/*==============================================================================
  Constants
  ==============================================================================*/
static shrlog_fixed_apps_info fixed_info;
static struct shrlog_ram_fixed_T* shrlog_ram_fixed = NULL;

extern unsigned long get_xtime_sec_addr(void);
extern unsigned long get_log_buf_addr(void);
extern unsigned long get_log_buf_len(void);
extern unsigned long get_log_first_seq_addr(void);
extern unsigned long get_log_first_idx_addr(void);
extern unsigned long get_log_next_seq_addr(void);
#ifdef CONFIG_OOPS_LOG_BUFFER
extern unsigned long get_log_oops_buf_addr(void);
extern unsigned long get_log_oops_first_seq_addr(void);
extern unsigned long get_log_oops_last_seq_addr(void);
#endif /* CONFIG_OOPS_LOG_BUFFER */
#ifdef SHLOG_ENABLE_LOGGER
extern unsigned long get_buf_log_main_addr(void);
extern unsigned long get_buf_log_events_addr(void);
#endif /* SHLOG_ENABLE_LOGGER */
extern void* shrlog_get_smem_pos(resource_size_t* pRetSize);
extern struct task_struct *latest_process[];

#ifdef CONFIG_ARM
extern void* get_origin_unwind_addr(void);
extern struct unwind_idx __start_unwind_idx[];
extern struct unwind_idx __stop_unwind_idx[];
#endif

static ulong logd_info = 0;

static int logd_info_set(const char *buffer, const struct kernel_param *kp)
{
        ulong v;
        int ret = -EINVAL;

        if( kstrtoul(buffer, 0, &v) == 0) {
                fixed_info.adr.st.logd_task_addr = (unsigned long)(current);
                fixed_info.logd_info = v;
                ret = 0;
        }

        return ret;
}

static struct kernel_param_ops param_ops_logd_info = {
        .set = logd_info_set,
};

module_param_cb(logd_info, &param_ops_logd_info, &logd_info, 0200);

/*==============================================================================
  [Function]
  shrlog_set_fixed_info
  ==============================================================================*/
static int shrlog_set_fixed_info( void )
{
        if( shrlog_ram_fixed == NULL ) return -ENOMEM;

        fixed_info.adr.st.init_task_addr          = (unsigned long)(&init_task);
        fixed_info.adr.st.xtime_sec_addr          = get_xtime_sec_addr();
        fixed_info.adr.st.log_buf_addr            = get_log_buf_addr();
        fixed_info.adr.st.log_first_seq_addr      = get_log_first_seq_addr();
        fixed_info.adr.st.log_first_idx_addr      = get_log_first_idx_addr();
        fixed_info.adr.st.log_next_seq_addr       = get_log_next_seq_addr();
#ifdef CONFIG_OOPS_LOG_BUFFER
        fixed_info.adr.st.log_oops_buf_addr       = get_log_oops_buf_addr();
        fixed_info.adr.st.log_oops_first_seq_addr = get_log_oops_first_seq_addr();
        fixed_info.adr.st.log_oops_last_seq_addr  = get_log_oops_last_seq_addr();
        fixed_info.oops_logbuf_size = (1 << CONFIG_OOPS_LOG_BUF_SHIFT);
#endif /* CONFIG_OOPS_LOG_BUFFER */
#ifdef SHLOG_ENABLE_LOGGER
        fixed_info.adr.st.buf_log_main_addr       = get_buf_log_main_addr();
        fixed_info.adr.st.buf_log_events_addr     = get_buf_log_events_addr();
#endif /* SHLOG_ENABLE_LOGGER */
        fixed_info.adr.st._text_addr              = (unsigned long)(_text);
        fixed_info.adr.st._stext_addr             = (unsigned long)(_stext);
        fixed_info.adr.st._etext_addr             = (unsigned long)(_etext);
        fixed_info.adr.st.latest_process_addr     = (unsigned long)latest_process;
        fixed_info.size_of_task_struct = sizeof(struct task_struct);
        fixed_info.size_of_thread_struct = sizeof(struct thread_struct);
        fixed_info.size_of_mm_struct = sizeof(struct mm_struct);
        fixed_info.stack_offset        = offsetof(struct task_struct, stack);
        fixed_info.tasks_offset        = offsetof(struct task_struct, tasks);
        fixed_info.pid_offset          = offsetof(struct task_struct, pid);
        fixed_info.thread_group_offset = offsetof(struct task_struct, thread_group);
        fixed_info.comm_offset         = offsetof(struct task_struct, comm);
#ifdef CONFIG_ARM64
        fixed_info.size_of_cpu_context = sizeof(struct cpu_context);
        fixed_info.cpu_context_fp_offset =
                offsetof(struct task_struct, thread)
                + offsetof(struct thread_struct, cpu_context)
                + offsetof(struct cpu_context, fp);
        fixed_info.cpu_context_sp_offset =
                offsetof(struct task_struct, thread)
                + offsetof(struct thread_struct, cpu_context)
                + offsetof(struct cpu_context, sp);
        fixed_info.cpu_context_pc_offset =
                offsetof(struct task_struct, thread)
                + offsetof(struct thread_struct, cpu_context)
                + offsetof(struct cpu_context, pc);
#endif /* CONFIG_ARM64 */
#ifdef CONFIG_ARM
        fixed_info.adr.st.__start_unwind_idx_addr = (shrlog_kernel_t)__start_unwind_idx;
        fixed_info.adr.st.__origin_unwind_idx_addr = (shrlog_kernel_t)get_origin_unwind_addr();
        fixed_info.adr.st.__stop_unwind_idx_addr = (shrlog_kernel_t)__stop_unwind_idx;
        fixed_info.size_of_thread_info = sizeof(struct thread_info);
        fixed_info.cpu_context_in_thread_info = offsetof(struct thread_info, cpu_context);
        fixed_info.size_of_cpu_context_save = sizeof(struct cpu_context_save);
        fixed_info.cpu_context_save_fp_offset = offsetof(struct cpu_context_save, fp);
        fixed_info.cpu_context_save_sp_offset = offsetof(struct cpu_context_save, sp);
        fixed_info.cpu_context_save_pc_offset = offsetof(struct cpu_context_save, pc);
#endif /* CONFIG_ARM */
        fixed_info.mm_offset = offsetof(struct task_struct, mm);
        fixed_info.pgd_offset = offsetof(struct mm_struct, pgd);
        fixed_info.logbuf_size = get_log_buf_len();
        fixed_info.info_size = sizeof(fixed_info);
        fixed_info.info_magic = SHRLOG_FIXED_LINUX_MAGIC_NUM;

        shrlog_ram_fixed->shrlog_ram_fixed_addr = (RLOGUINT64)((shrlog_kernel_t)&fixed_info);
        shrlog_ram_fixed->linux_phys_offset = PHYS_OFFSET;
        shrlog_ram_fixed->magic_num1 = (RLOGUINT64)SHRLOG_FIXED_MAGIC_NUM; // magic number
        shrlog_ram_fixed->magic_num2 = (RLOGUINT64)SHRLOG_FIXED_MAGIC_NUM; // magic number

        return 0;
}

static int shrlog_probe(struct platform_device *pdev)
{
        int ret = 0;
        u32 modem_addr;
        u64 modem_size;
        if ( pdev->dev.of_node == NULL ){
        } else {
                bool use_smem = false;
                uint32_t offset_in_region = 0;
                int ret = 0;
                u32 modem_range[2];
                struct device_node* pnode = NULL;
                void* pV = NULL;
                ret = of_property_read_u32(pdev->dev.of_node, "sharp,shrlog-offset", &offset_in_region);
                if( ret ) offset_in_region = 0;
                if( (ret = of_property_read_u32_array(pdev->dev.of_node, "modem-range", modem_range, sizeof(modem_range)/sizeof(modem_range[0]))) == 0 ){
                        modem_addr = modem_range[0];
                        modem_size = modem_range[1];
                }else if( (pnode = of_parse_phandle(pdev->dev.of_node, "modem-region", 0)) != NULL ){
                        const u32 *addr;
                        addr = of_get_address(pnode, 0, &modem_size, NULL);
                        if( IS_ERR_OR_NULL(addr) ){
                                modem_size = 0;
                                modem_addr = 0;
                        }else{
                                modem_addr = (u32) of_read_ulong(addr, 2);
                        }
                        of_node_put(pnode);
                }else{
                        modem_size = 0;
                        modem_addr = 0;
                }
                if( (pnode = of_parse_phandle(pdev->dev.of_node, "memory-region", 0)) != NULL  ){
                        if( (pV = of_iomap(pnode, 0)) == NULL  ){
                                of_node_put(pnode);
                        }
                }else if( (pnode = of_parse_phandle(pdev->dev.of_node, "linux,contiguous-region", 0)) != NULL  ){
                        const u32 *addr;
                        u64 mem_size;
                        addr = of_get_address(pnode, 0, &mem_size, NULL);
                        if( IS_ERR_OR_NULL(addr) ){
                                pV = NULL;
                        }else{
                                u32 mem_addr;
                                mem_addr = (u32) of_read_ulong(addr, 2);
                                pV = ioremap(mem_addr, mem_size);
                        }
                        of_node_put(pnode);
                }else if( (use_smem = of_property_read_bool(pdev->dev.of_node, "sharp,shrlog-smem")) ){
                        resource_size_t size = 0;
                        if( (pV = shrlog_get_smem_pos(&size)) == NULL ){
                        }else if( (size - offset_in_region) < sizeof(struct shrlog_ram_fixed_T) ){
                                pr_debug("reserved memory is too small.\n");
                                pV = NULL;
                        }
                }
                if( pV != NULL ){
                        shrlog_ram_fixed = (struct shrlog_ram_fixed_T*)(((char*)pV)+offset_in_region);
                }
                else{
                        pr_debug("region is not reserved for %s\n", pdev->name);
                }
        }
        if( shrlog_ram_fixed == NULL ){
                return -ENOMEM;
        }
        if( (ret = shrlog_set_fixed_info()) != 0 ) return ret;

        shrlog_ram_fixed->modem_region_start = modem_addr;
        shrlog_ram_fixed->modem_region_size = modem_size;

        return 0;
}

static int shrlog_remove(struct platform_device *pdev)
{
        return 0;
}

/*
 *
 *
 *
 */
#define SHRLOG_COMPAT_STR       "sharp,shrlog"

static struct of_device_id shrlog_match_table[] = {
        {
                .compatible = SHRLOG_COMPAT_STR
        },
        {},
};

static struct platform_driver shrlog_driver = {
        .probe = shrlog_probe,
        .remove = shrlog_remove,
        .driver = {
                .name = "shrlog",
                .of_match_table = shrlog_match_table
        },
};

static int __init shrlog_init(void)
{
        return platform_driver_register(&shrlog_driver);
}
static void __exit shrlog_exit(void)
{
        platform_driver_unregister(&shrlog_driver);
}

subsys_initcall(shrlog_init);
module_exit(shrlog_exit);

