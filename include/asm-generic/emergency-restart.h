#ifndef _ASM_GENERIC_EMERGENCY_RESTART_H
#define _ASM_GENERIC_EMERGENCY_RESTART_H

static inline void machine_emergency_restart(void)
{
#ifdef CONFIG_SHLOG_SYSTEM
	machine_restart("emergency");
#else /* ! CONFIG_SHLOG_SYSTEM */
	machine_restart(NULL);
#endif /* ! CONFIG_SHLOG_SYSTEM */
}

#endif /* _ASM_GENERIC_EMERGENCY_RESTART_H */
