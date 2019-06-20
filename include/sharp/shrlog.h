/* include/sharp/shrlog.h
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

#ifndef _SHRLOG_H_
#define _SHRLOG_H_

#include <linux/sched.h>

#define SHRLOG_FIXED_MAGIC_NUM (0x88990011)
#define SHRLOG_FIXED_LINUX_MAGIC_NUM (0x88992211)

#if defined(CONFIG_ARM64)
typedef unsigned long RLOGUINT64;
#elif defined(CONFIG_ARM)
typedef unsigned long long RLOGUINT64;
#else
#error "types in linux not specified."
#endif

struct shrlog_ram_fixed_T {
        RLOGUINT64 shrlog_ram_fixed_addr;
        RLOGUINT64 magic_num1;
        RLOGUINT64 modem_region_start;
        RLOGUINT64 modem_region_size;
        RLOGUINT64 linux_phys_offset;
        RLOGUINT64 pad[2];
        RLOGUINT64 magic_num2;
};

typedef unsigned long shrlog_kernel_t;

typedef struct
{
        /* Be sure to add virtual addresses, which must be checked first, by using shrlog_kernel_t */
        shrlog_kernel_t xtime_sec_addr;
        shrlog_kernel_t _text_addr;
        shrlog_kernel_t _stext_addr;
        shrlog_kernel_t _etext_addr;

#if defined(CONFIG_SHLOG_SHLOG_LOGBUF)
        shrlog_kernel_t log_buf_addr;
        shrlog_kernel_t log_first_seq_addr;
        shrlog_kernel_t log_first_idx_addr;
        shrlog_kernel_t log_next_seq_addr;
#ifdef CONFIG_OOPS_LOG_BUFFER
        shrlog_kernel_t log_oops_buf_addr;
        shrlog_kernel_t log_oops_first_seq_addr;
        shrlog_kernel_t log_oops_last_seq_addr;
#endif /* CONFIG_OOPS_LOG_BUFFER */
#endif /* CONFIG_SHLOG_SHLOG_LOGBUF */

#if defined(CONFIG_SHLOG_SHLOG_TASKLIST)
        shrlog_kernel_t init_task_addr;
#ifdef CONFIG_ARM
        shrlog_kernel_t __start_unwind_idx_addr;
        shrlog_kernel_t __origin_unwind_idx_addr;
        shrlog_kernel_t __stop_unwind_idx_addr;
#endif
#endif /* CONFIG_SHLOG_SHLOG_TASKLIST */

#if defined(CONFIG_SHLOG_SHLOG_LATESTSTACK)
        shrlog_kernel_t latest_process_addr;
#endif /* CONFIG_SHLOG_SHLOG_LATESTSTACK */

#if defined(CONFIG_SHLOG_SHLOG_LOGGER)
        shrlog_kernel_t buf_log_main_addr;
        shrlog_kernel_t buf_log_events_addr;
#endif /* CONFIG_SHLOG_SHLOG_LOGGER */

#if defined(CONFIG_SHLOG_SHLOG_LOGD)
        shrlog_kernel_t logd_task_addr;
#endif /* CONFIG_SHLOG_SHLOG_LOGD */

} shrlog_fixed_linux_addresses_t;

typedef struct {
        shrlog_kernel_t info_size;
        union {
                shrlog_fixed_linux_addresses_t st;
                shrlog_kernel_t values[sizeof(shrlog_fixed_linux_addresses_t)/sizeof(shrlog_kernel_t)];
        } adr;
        shrlog_kernel_t size_of_task_struct;
        shrlog_kernel_t size_of_thread_struct;
        shrlog_kernel_t size_of_mm_struct;
        shrlog_kernel_t stack_offset;
        shrlog_kernel_t tasks_offset;
        shrlog_kernel_t pid_offset;
        shrlog_kernel_t thread_group_offset;
        shrlog_kernel_t comm_offset;
        shrlog_kernel_t mm_offset;
        shrlog_kernel_t pgd_offset;
#ifdef CONFIG_ARM64
        shrlog_kernel_t size_of_cpu_context;
        shrlog_kernel_t cpu_context_fp_offset;
        shrlog_kernel_t cpu_context_sp_offset;
        shrlog_kernel_t cpu_context_pc_offset;
#endif /* CONFIG_ARM64 */
#ifdef CONFIG_ARM
        shrlog_kernel_t size_of_thread_info;
        shrlog_kernel_t cpu_context_in_thread_info;
        shrlog_kernel_t size_of_cpu_context_save;
        shrlog_kernel_t cpu_context_save_fp_offset;
        shrlog_kernel_t cpu_context_save_sp_offset;
        shrlog_kernel_t cpu_context_save_pc_offset;
#endif /* CONFIG_ARM */

#if defined(CONFIG_SHLOG_SHLOG_LOGBUF)
        shrlog_kernel_t logbuf_size;
#ifdef CONFIG_OOPS_LOG_BUFFER
        shrlog_kernel_t oops_logbuf_size;
#endif /* CONFIG_OOPS_LOG_BUFFER */
#endif /* CONFIG_SHLOG_SHLOG_LOGBUF */

#if defined(CONFIG_SHLOG_SHLOG_RAMOOPS)
        shrlog_kernel_t oops_addr;
        shrlog_kernel_t oops_size;
        shrlog_kernel_t oops_console_size;
        shrlog_kernel_t oops_record_size;
        shrlog_kernel_t oops_pmsg_size;
#endif /* SHLOG_SHLOG_RAMOOPS */

#if defined(CONFIG_SHLOG_SHLOG_TASKLIST)
#endif /* CONFIG_SHLOG_SHLOG_TASKLIST */

#if defined(CONFIG_SHLOG_SHLOG_LATESTSTACK)
#endif /* CONFIG_SHLOG_SHLOG_LATESTSTACK */

#if defined(CONFIG_SHLOG_SHLOG_LOGGER)
#endif /* CONFIG_SHLOG_SHLOG_LOGGER */

#if defined(CONFIG_SHLOG_SHLOG_LOGD)
        shrlog_kernel_t logd_info;
#endif /* CONFIG_SHLOG_SHLOG_LOGD */

        shrlog_kernel_t info_magic;
} shrlog_fixed_apps_info;

extern int rlog_app_start( struct task_struct *tsk, unsigned long addr,
                           unsigned int esr, unsigned int sig, int code,
                           struct pt_regs *regs );
extern int  rlog_sys_write( const char *attr_name, char *buf, int size );
extern int  rlog_uevent( void );
extern void rlog_fault_init( void );

#endif /* _SHRLOG_H_ */
