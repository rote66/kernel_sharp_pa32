/*
 * MIYABI LSM module
 *
 * based on root_plug.c
 * Copyright (C) 2002 Greg Kroah-Hartman <greg@kroah.com>
 *
 * _xx_is_valid(), _xx_encode(), _xx_realpath_from_path()
 * is ported from security/tomoyo/realpath.c in linux-2.6.32 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/security.h>
#include <linux/moduleparam.h>
#include <linux/mount.h>
#include <linux/mnt_namespace.h>
#include <linux/fs_struct.h>
#include <linux/magic.h>
#include <linux/net.h>
#include <linux/netfilter_ipv4/ip_tables.h>
#include <linux/binfmts.h>
#include <net/sock.h>

//#ifdef CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD
//#define MIYABI_DEBUG_LOG
//#endif

#ifdef MIYABI_DEBUG_LOG
#define MIYABI_LOGE(fmt, args...) { \
    printk("[MIYABI]" fmt, ## args); \
}
#else
#define MIYABI_LOGE(fmt, args...)
#endif

static void record_rooted(void)
{
	printk("!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	printk("!!        rooted        !!\n");
	printk("!!!!!!!!!!!!!!!!!!!!!!!!!!\n");

	return;
}

int miyabi_ptrace_access_check(struct task_struct *child, unsigned int mode)
{
	MIYABI_LOGE("%s\n", __FUNCTION__); 
#ifdef CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD
	return 0;
#else /* ! CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD */
	return -EPERM;
#endif /* CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD */
}

int miyabi_ptrace_traceme(struct task_struct *parent)
{
	MIYABI_LOGE("%s\n", __FUNCTION__); 
#ifdef CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD
	return 0;
#else /* ! CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD */
	return -EPERM;
#endif /* CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD */
}

struct vec
{
	const char *prefix;
};

static struct vec set_creds_wh[] =
{
	{ "/system/bin/run-as" },
#ifdef CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD
	{ "/system/xbin/procrank" },
	{ "/system/xbin/librank" },
	{ "/system/xbin/su" },
#endif /* CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD */
	{ 0 },
};

#ifndef CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD
static struct vec task_fix_setuid_wh[] =
{
	{ 0 },
};
#endif /* CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD */


static int permitted_prog(struct vec *wh, const char *path)
{
	int len;
	struct vec *w = wh;

	for(; w->prefix; w++)
	{
		len = strlen(w->prefix);
		if(strlen(path) == len && !strncmp(w->prefix, path, len))
		{
			return 1;
		}
	}

	return 0;
}

int miyabi_bprm_set_creds(struct linux_binprm *bprm)
{
	struct cred *new = bprm->cred;

	if((!uid_eq(new->uid, GLOBAL_ROOT_UID) &&
	    uid_eq(new->euid, GLOBAL_ROOT_UID)) ||
	   (!gid_eq(new->gid, GLOBAL_ROOT_GID) &&
	    gid_eq(new->egid, GLOBAL_ROOT_GID)))
	{
		if(!permitted_prog(set_creds_wh, bprm->filename))
		{
			record_rooted();
			MIYABI_LOGE("%s filename=%s\n", __FUNCTION__, bprm->filename);

			return -EPERM;
		}
	}

	return 0;
}

static char* _xx_encode(const char *str)
{
	int len = 0;
	const char *p = str;
	char *cp;
	char *cp0;

	if (!p)
		return NULL;
	while (*p) {
		const unsigned char c = *p++;
		if (c == '\\')
			len += 2;
		else if (c > ' ' && c < 127)
			len++;
		else
			len += 4;
	}
	len++;
	/* Reserve space for appending "/". */
	cp = kzalloc(len + 10, GFP_NOFS);
	if (!cp)
		return NULL;
	cp0 = cp;
	p = str;
	while (*p) {
		const unsigned char c = *p++;

		if (c == '\\') {
			*cp++ = '\\';
			*cp++ = '\\';
		} else if (c > ' ' && c < 127) {
			*cp++ = c;
		} else {
			*cp++ = '\\';
			*cp++ = (c >> 6) + '0';
			*cp++ = ((c >> 3) & 7) + '0';
			*cp++ = (c & 7) + '0';
		}
	}
	return cp0;
}

static char* _xx_realpath_from_path(struct path *path)
{
	char *buf = NULL;
	char *name = NULL;
	unsigned int buf_len = PAGE_SIZE / 2;
	struct dentry *dentry = path->dentry;
	bool is_dir;
	if (!dentry)
		return NULL;
	is_dir = dentry->d_inode && S_ISDIR(dentry->d_inode->i_mode);
	while (1) {
		char *pos;
		buf_len <<= 1;
		kfree(buf);
		buf = kmalloc(buf_len, GFP_NOFS);
		if (!buf)
			break;
		/* Get better name for socket. */
		if (dentry->d_sb && dentry->d_sb->s_magic == SOCKFS_MAGIC) {
			struct inode *inode = dentry->d_inode;
			struct socket *sock = inode ? SOCKET_I(inode) : NULL;
			struct sock *sk = sock ? sock->sk : NULL;
			if (sk) {
				snprintf(buf, buf_len - 1, "socket:[family=%u:"
					 "type=%u:protocol=%u]", sk->sk_family,
					 sk->sk_type, sk->sk_protocol);
			} else {
				snprintf(buf, buf_len - 1, "socket:[unknown]");
			}
			name = _xx_encode(buf);
			break;
		}
		/* For "socket:[\$]" and "pipe:[\$]". */
		if (dentry->d_op && dentry->d_op->d_dname) {
			pos = dentry->d_op->d_dname(dentry, buf, buf_len - 1);
			if (IS_ERR(pos))
				continue;
			name = _xx_encode(pos);
			break;
		}
		/* If we don't have a vfsmount, we can't calculate. */
		if (!path->mnt)
			break;
		pos = d_absolute_path(path, buf, buf_len - 1);
		/* If path is disconnected, use "[unknown]" instead. */
		if (pos == ERR_PTR(-EINVAL)) {
			name = _xx_encode("[unknown]");
			break;
		}
		/* Prepend "/proc" prefix if using internal proc vfs mount. */
		if (!IS_ERR(pos) && (path->mnt->mnt_flags & MNT_INTERNAL) &&
		    (path->mnt->mnt_sb->s_magic == PROC_SUPER_MAGIC)) {
			pos -= 5;
			if (pos >= buf)
				memcpy(pos, "/proc", 5);
			else
				pos = ERR_PTR(-ENOMEM);
		}
		if (IS_ERR(pos))
			continue;
		name = _xx_encode(pos);
		break;
	}
	kfree(buf);
	if (is_dir && name != NULL && *name) {
		/* Append trailing '/' if dentry is a directory. */
		char *pos = name + strlen(name) - 1;
		if (*pos != '/')
			/*
			 * This is OK because tomoyo_encode() reserves space
			 * for appending "/".
			 */
			*++pos = '/';
	}
	return name;
}


#ifndef CONFIG_SECURITY_MIYABI_SYSTEM_DIR_PATH
#define CONFIG_SECURITY_MIYABI_SYSTEM_DIR_PATH "/system/"
#endif

#ifndef CONFIG_SECURITY_MIYABI_SYSTEM_DEV_MAJOR
#define CONFIG_SECURITY_MIYABI_SYSTEM_DEV_MAJOR (253)
#endif /* ! CONFIG_SECURITY_MIYABI_SYSTEM_DEV_MAJOR */

#ifndef CONFIG_SECURITY_MIYABI_SYSTEM_DEV_MINOR
#define CONFIG_SECURITY_MIYABI_SYSTEM_DEV_MINOR (0)
#endif /* ! CONFIG_SECURITY_MIYABI_SYSTEM_DEV_MINOR */

int miyabi_sb_mount(const char *dev_name, struct path *path,
			    const char *type, unsigned long flags, void *data)
{
#ifndef CONFIG_ANDROID_ENGINEERING
#ifndef CONFIG_ANDROID_RECOVERY_BUILD
	char* realpath;
	struct block_device* bdev;
	dev_t major, minor;
#endif /* ! CONFIG_ANDROID_RECOVERY_BUILD */
#endif /* ! CONFIG_ANDROID_ENGINEERING */

	if(dev_name != NULL)
	{
		if(strncmp(dev_name, "rootfs", 6) == 0)
		{
			if ((flags & MS_REMOUNT) && (!(flags & MS_RDONLY)))
			{
				printk(KERN_ERR "%s: REJECT dev_name=%s cannot remount as writable\n", __FUNCTION__, dev_name);
				MIYABI_LOGE("%s dev_name=%s type=%s flags=%08lXn", __FUNCTION__, dev_name, type, flags);
				return -EPERM;
			}
		}
	}

#ifndef CONFIG_ANDROID_ENGINEERING
/* enable debugfs for systrace (in compliance with Android 4.2 CDD) */
/*
	if(type != NULL)
	{
		if(strncmp(type, "debugfs", 7) == 0)
		{
			printk(KERN_ERR "%s: REJECT dev_name=%s cannot mount %s\n", __FUNCTION__, dev_name, type);

			return -EPERM;
		}
	}
*/

#ifndef CONFIG_ANDROID_RECOVERY_BUILD
	realpath = _xx_realpath_from_path(path);
	if(realpath == NULL)
	{
		return -ENOMEM;
	}


	if (strncmp(realpath, CONFIG_SECURITY_MIYABI_SYSTEM_DIR_PATH,
		strlen(CONFIG_SECURITY_MIYABI_SYSTEM_DIR_PATH)) == 0)
	{
		if (strcmp(realpath, CONFIG_SECURITY_MIYABI_SYSTEM_DIR_PATH) == 0)
		{

			if ((flags & MS_REMOUNT) && (!(flags & MS_RDONLY)))
			{
				printk(KERN_ERR "%s: REJECT dev_name=%s realpath=%s ro remount\n",
					__FUNCTION__, dev_name, realpath);

				kfree(realpath);
				return -EPERM;
			}

			if ((flags & MS_BIND) && (flags & MS_REMOUNT) && (flags & MS_NOSUID) && (flags & MS_NODEV) && (flags & MS_RDONLY))
			{
				kfree(realpath);
				MIYABI_LOGE("%s dev_name=%s type=%s flags=%08lX permit remount\n", __FUNCTION__, dev_name, type, flags); 

				return 0;
			}

			if (flags & MS_BIND)
			{
				printk(KERN_ERR "%s: REJECT dev_name=%s realpath=%s loopback mount\n",
					__FUNCTION__, dev_name, realpath);

				kfree(realpath);
				return -EPERM;
			}

			bdev = lookup_bdev((const char*)dev_name);

			if( bdev == NULL )
			{
				MIYABI_LOGE("%s cannot lookup %s\n", __FUNCTION__, dev_name);
				printk("cannot lookup\n");

				kfree(realpath);
				return -EPERM;
			}

			major = MAJOR(bdev->bd_dev);
			minor = MINOR(bdev->bd_dev);

			bdput(bdev);

			if((major != CONFIG_SECURITY_MIYABI_SYSTEM_DEV_MAJOR) || (minor != CONFIG_SECURITY_MIYABI_SYSTEM_DEV_MINOR))
			{
				printk(KERN_ERR "%s: REJECT dev_name=%s realpath=%s mismatch major or minor\n",
					__FUNCTION__, dev_name, realpath);

				kfree(realpath);
				return -EPERM;
			}
		}
		else
		{
			printk(KERN_ERR "%s: REJECT realpath=%s\n",
				__FUNCTION__, realpath);

			kfree(realpath);
			return -EPERM;
		}
	}


	kfree(realpath);
#endif /* ! CONFIG_ANDROID_RECOVERY_BUILD */
#endif /* ! CONFIG_ANDROID_ENGINEERING */
	return 0;
}

#ifndef CONFIG_SECURITY_MIYABI_SYSTEM_MOUNT_POINT
#define CONFIG_SECURITY_MIYABI_SYSTEM_MOUNT_POINT "system"
#endif

int miyabi_sb_umount(struct vfsmount *mnt, int flags)
{
#ifndef CONFIG_ANDROID_ENGINEERING
#ifndef CONFIG_ANDROID_RECOVERY_BUILD

	dev_t major, minor;

	(void)flags;

	if(mnt && mnt->mnt_sb && mnt->mnt_sb->s_bdev)
	{
		major = MAJOR(mnt->mnt_sb->s_bdev->bd_dev);
		minor = MINOR(mnt->mnt_sb->s_bdev->bd_dev);

		if((major == CONFIG_SECURITY_MIYABI_SYSTEM_DEV_MAJOR) && (minor == CONFIG_SECURITY_MIYABI_SYSTEM_DEV_MINOR))
		{
			printk(KERN_ERR "%s: REJECT umount=%d,%d\n", __FUNCTION__, major, minor);

			return -EPERM;
		}
	}
#endif /* ! CONFIG_ANDROID_RECOVERY_BUILD */
#endif /* ! CONFIG_ANDROID_ENGINEERING */
	return 0;
}

int miyabi_sb_pivotroot(struct path *old_path, struct path *new_path)
{
	char* old_realpath;
	char* new_realpath;

	old_realpath = _xx_realpath_from_path(old_path);
	if(old_realpath == NULL)
	{
		MIYABI_LOGE("%s old_realpath=NULL\n", __FUNCTION__); 

		return -ENOMEM;
	}

	new_realpath = _xx_realpath_from_path(new_path);
	if(new_realpath == NULL)
	{
		kfree(old_realpath);
		MIYABI_LOGE("%s new_realpath=NULL\n", __FUNCTION__); 
		return -ENOMEM;
	}

	printk(KERN_ERR "%s: REJECT old_path=%s new_path=%s\n",
	       __FUNCTION__, old_realpath, new_realpath);


	kfree(old_realpath);
	kfree(new_realpath);

	return -EPERM;
}

int miyabi_path_symlink(struct path *dir, struct dentry *dentry, const char *old_name)
{
#ifndef CONFIG_ANDROID_RECOVERY_BUILD
	char* realdir;

	realdir = _xx_realpath_from_path(dir);

	if (realdir == NULL) {
		MIYABI_LOGE("%s realdir=NULL\n", __FUNCTION__); 
		return -ENOMEM;
	}

	if(strncmp(realdir, CONFIG_SECURITY_MIYABI_SYSTEM_DIR_PATH,
		    strlen(CONFIG_SECURITY_MIYABI_SYSTEM_DIR_PATH)) == 0)
	{
		printk(KERN_ERR "%s: REJECT dir=%s\n", __FUNCTION__, realdir);

		kfree(realdir);
		return -EPERM;
	}

	if((strcmp(realdir, "/") == 0) && 
	    (strncmp(dentry->d_name.name, CONFIG_SECURITY_MIYABI_SYSTEM_MOUNT_POINT, 
	            strlen(CONFIG_SECURITY_MIYABI_SYSTEM_MOUNT_POINT)) == 0))
	{
		printk(KERN_ERR "%s: REJECT dir + dentry=%s%s\n", __FUNCTION__, realdir, dentry->d_name.name);

		kfree(realdir);
		return -EPERM;
	}

	kfree(realdir);
#endif /* ! CONFIG_ANDROID_RECOVERY_BUILD */

	return 0;
}

int miyabi_path_chmod(struct path* path, umode_t mode)
{
#ifndef CONFIG_ANDROID_ENGINEERING
#ifndef CONFIG_ANDROID_RECOVERY_BUILD
	struct inode* inode;
	const struct cred* cred = current_cred();

	if(path == NULL) return -EFAULT;
	if(path->dentry == NULL) return -EFAULT;
	if(path->dentry->d_inode == NULL) return -EFAULT;

	inode = path->dentry->d_inode;

	if(mode & S_ISUID)
	{
		if(uid_eq(inode->i_uid, GLOBAL_ROOT_UID) && uid_eq(cred->euid, GLOBAL_ROOT_UID))
		{
			record_rooted();
			MIYABI_LOGE("%s uid=euid=0\n", __FUNCTION__);
			return -EPERM;
		}
	}
	if(mode & S_ISGID)
	{
		if(gid_eq(inode->i_gid, GLOBAL_ROOT_GID) && gid_eq(cred->egid, GLOBAL_ROOT_GID))
		{
			record_rooted();
			MIYABI_LOGE("%s gid=egid=0\n", __FUNCTION__);
			return -EPERM;
		}
	}
#endif /* ! CONFIG_ANDROID_RECOVERY_BUILD */
#endif /* ! CONFIG_ANDROID_ENGINEERING */
	return 0;
}

#if 0
#ifndef CONFIG_SECURITY_MIYABI_CHROOT_PATH
#define CONFIG_SECURITY_MIYABI_CHROOT_PATH ""
#endif
#endif

#ifdef CONFIG_SECURITY_PATH
int miyabi_path_chroot(struct path *path)
{
#ifdef CONFIG_SECURITY_MIYABI_CHROOT_PATH
	char* realpath;
	char* tmp;
	char *p, *p2;

	tmp = kmalloc(PATH_MAX + 1, GFP_KERNEL);

	if(tmp == NULL) {
		MIYABI_LOGE("%s mem error\n", __FUNCTION__);
		return -ENOMEM;
	}

	memset(tmp, 0, PATH_MAX + 1);

	realpath = _xx_realpath_from_path(path);

	if (realpath == NULL)
	{
		kfree(tmp);
		MIYABI_LOGE("%s realpath=null\n", __FUNCTION__);
		return -ENOMEM;
	}
	MIYABI_LOGE("%s realpath=%s\n", __FUNCTION__, realpath);

	p = CONFIG_SECURITY_MIYABI_CHROOT_PATH;

	while (*p)
	{
		p2 = strchr(p, ':');
		if (p2)
		{
			strncpy(tmp, p, (p2 - p));
			tmp[p2 - p] = 0;
		}
		else
		{
			strncpy(tmp, p, PATH_MAX);
		}

		if (strcmp(tmp, realpath) == 0)
		{
			kfree(realpath);
			kfree(tmp);

			return 0;
		}

		if (p2)
		{
			p = p2 + 1;
		}
		else
		{
			p += strlen(p);
		}
	}

	MIYABI_LOGE("%s realpath=%s\n", __FUNCTION__, realpath);
	kfree(realpath);
	kfree(tmp);
	return -EPERM;
#else
	return 0;
#endif /* CONFIG_SECURITY_MIYABI_CHROOT_PATH */
}
#endif /* CONFIG_SECURITY_PATH */


#ifndef CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD
static char *miyabi_get_current_process(char *path, int size)
{
	char *res = (char *)-1;
	struct mm_struct *mm;

	if (!path) {
		printk("Invalid parameter path\n");
		return res;
	} 

	mm = current->mm;
	if (mm) {
		down_read(&mm->mmap_sem);
		if (mm->exe_file) {
			/* Get the path name */
			res = d_path(&mm->exe_file->f_path, path, size);
		}
		up_read(&mm->mmap_sem);
	} else {
		printk("mm is NULL!\n");
	}
	return res;
}
#endif /* CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD */

int miyabi_task_fix_setuid(struct cred *new, const struct cred *old, int flags)
{
	if(((uid_eq(new->uid, GLOBAL_ROOT_UID) || uid_eq(new->euid, GLOBAL_ROOT_UID) || uid_eq(new->suid, GLOBAL_ROOT_UID))
	    && !uid_eq(old->uid, GLOBAL_ROOT_UID)) ||
	   ((gid_eq(new->gid, GLOBAL_ROOT_GID) || gid_eq(new->egid, GLOBAL_ROOT_GID) || gid_eq(new->sgid, GLOBAL_ROOT_GID))
	    && !gid_eq(old->gid, GLOBAL_ROOT_GID)))
	{
#ifdef CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD

#else /* ! CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD */
		char *pathbuff = NULL;
		char *path = NULL;

		pathbuff = kmalloc(PATH_MAX + 1, GFP_KERNEL);
		if(pathbuff == NULL) return -ENOMEM;

		memset(pathbuff, 0, PATH_MAX + 1);

		path = miyabi_get_current_process(pathbuff, PATH_MAX);

		if (path == NULL || (long)path == ENAMETOOLONG ) {
			MIYABI_LOGE("%s path=null\n", __FUNCTION__);
			kfree(pathbuff);
			return -EPERM;
		}
//		printk("%s res =%08X, path=%s\n", __FUNCTION__, (unsigned int)path, path);

		if (!permitted_prog(task_fix_setuid_wh, path)) {
			MIYABI_LOGE("%s path=%s denied\n", __FUNCTION__, path);
			kfree(pathbuff);
			return -EPERM;
		}
		kfree(pathbuff);
#endif /* CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD */
	}

	return cap_task_fix_setuid(new, old, flags);
}

#ifdef CONFIG_SECURITY_NETWORK

#define LOCAL_MIYABI_IPTABLE_PATH "/system/bin/iptables"
#define LOCAL_MIYABI_IP6TABLE_PATH "/system/bin/ip6tables"

static struct vec setsockopt_wh[] =
{
	{ "/system/bin/rild" },
	{ "/system/bin/netd" },
	{ "/system/bin/netmgrd" },
	{ "/system/bin/sh" },
#ifdef CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD
	{ "/system/bin/mksh" },
	{ "/sbin/adbd" },
#endif /* CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD */
	{ 0 },
};

int miyabi_socket_setsockopt(struct socket *sock, int level, int optname)
{
	if(optname == IPT_SO_SET_REPLACE || optname == IPT_SO_SET_ADD_COUNTERS)
	{
		char* pathbuff = NULL;
		char* path = NULL;

		pathbuff = kmalloc(PATH_MAX + 1, GFP_KERNEL);

		if(pathbuff == NULL) return -ENOMEM;

		memset(pathbuff, 0, PATH_MAX + 1);

		if(current->mm != NULL && current->mm->exe_file != NULL)
		{
			path = d_path(&current->mm->exe_file->f_path, pathbuff, PATH_MAX);
		}

		if(path == NULL || (long)path == ENAMETOOLONG)
		{
			MIYABI_LOGE("%s path=null\n", __FUNCTION__);

			kfree(pathbuff);
			return -EPERM;
		}

		if(
			!(strlen(path) == strlen(LOCAL_MIYABI_IPTABLE_PATH) && strcmp(path, LOCAL_MIYABI_IPTABLE_PATH) == 0) &&
			!(strlen(path) == strlen(LOCAL_MIYABI_IP6TABLE_PATH) && strcmp(path, LOCAL_MIYABI_IP6TABLE_PATH) == 0)
		)
		{
			MIYABI_LOGE("%s path=%s denied\n", __FUNCTION__, path);

			kfree(pathbuff);
			return -EPERM;
		}
		MIYABI_LOGE("%s path=%s\n", __FUNCTION__, path);

		
		if(current->parent == NULL)
		{
			kfree(pathbuff);
			MIYABI_LOGE("%s parent=null denied\n", __FUNCTION__);
			return -EPERM;
		}

		path = NULL;
		memset(pathbuff, 0, PATH_MAX + 1);

		if(current->parent->mm != NULL && current->parent->mm->exe_file != NULL)
		{
			path = d_path(&current->parent->mm->exe_file->f_path, pathbuff, PATH_MAX);
		}

		if(path == NULL || (long)path == ENAMETOOLONG)
		{
			MIYABI_LOGE("%s parent path=null denied\n", __FUNCTION__);
			kfree(pathbuff);
			return -EPERM;
		}

		if (permitted_prog(setsockopt_wh, path)) {
			MIYABI_LOGE("%s parent path=%s permitted\n", __FUNCTION__, path);

			kfree(pathbuff);
			return 0;
		}
		MIYABI_LOGE("%s parent path=%s denied\n", __FUNCTION__, path);

		kfree(pathbuff);
		return -EPERM;
	}
	else
	{
		return 0;
	}
}
#endif /* CONFIG_SECURITY_NETWORK */

int miyabi_file_open(struct file *file, const struct cred *cred)
{
#if 0
#ifndef CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD
#ifndef LOCAL_MIYABI_FBUILD
	dev_t major;
	dev_t minor;

	if(S_ISBLK(file->f_dentry->d_inode->i_mode))
	{
		major = MAJOR(file->f_dentry->d_inode->i_rdev);
		minor = MINOR(file->f_dentry->d_inode->i_rdev);

		if(	((major == CONFIG_SECURITY_MIYABI_SYSTEM_DEV_MAJOR) && (minor == CONFIG_SECURITY_MIYABI_SYSTEM_DEV_MINOR))
		)
		{
#ifdef MIYABI_DEBUG_LOG
			char *pathbuff = NULL;
			char *path = NULL;

			pathbuff = kmalloc(PATH_MAX + 1, GFP_KERNEL);
			if(pathbuff == NULL) {
				MIYABI_LOGE("%s failed to kmalloc path buffer\n", __FUNCTION__);
				goto NEXT;
			}

			memset(pathbuff, 0, PATH_MAX + 1);

			path = miyabi_get_current_process(pathbuff, PATH_MAX);

			if (path == NULL || (long)path == ENAMETOOLONG ) {
				MIYABI_LOGE("%s error on miyabi_get_current_process.\n", __FUNCTION__);
				kfree(pathbuff);
				goto NEXT;
			}
			MIYABI_LOGE("%s res =%08lX, path=%s\n", __FUNCTION__, (unsigned long)path, path);
			kfree(pathbuff);
NEXT:
#endif /* MIYABI_DEBUG_LOG */
			printk("reject opening device %d / %d\n", major, minor);

			return -EPERM;
		}
	}
#endif /* ! LOCAL_MIYABI_FBUILD */
#endif /* ! CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD */
#endif
	return 0;
}
