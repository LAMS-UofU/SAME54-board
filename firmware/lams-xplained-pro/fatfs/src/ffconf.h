#include "conf_fatfs.h"
#ifndef CONF_FATFS_H__
#define CONF_FATFS_H__

#ifndef _FFCONF
#define _FFCONF 64180	/* Revision ID */
#endif 

/* See http://elm-chan.org/fsw/ff/doc/config.html for config details */

/************************************************************************/
/*		Function Configurations                                         */
/************************************************************************/

#define FF_FS_READONLY		0
#define FF_FS_MINIMIZE		0
#define	FF_USE_STRFUNC		1
#define FF_USE_FIND			1
#define	FF_USE_MKFS			1
#define	FF_USE_FASTSEEK		0
#define FF_USE_EXPAND		0
#define FF_USE_CHMOD		0
#define FF_USE_LABEL		1
#define	FF_USE_FORWARD		1

/************************************************************************/
/*		Namespace and Locale Configurations                             */
/************************************************************************/

#define FF_CODE_PAGE		437
#define	FF_USE_LFN			0	
#define	FF_MAX_LFN			255
#define	FF_LFN_UNICODE		0
#define FF_STRF_ENCODE		3
#define FF_FS_RPATH			2

/************************************************************************/
/*		Volume/Drive Configurations                                     */
/************************************************************************/

#define FF_VOLUMES			1
#define FF_STR_VOLUME_ID	0
#define FF_VOLUME_STRS		"RAM","NAND","CF","SD"
#define	FF_MULTI_PARTITION	0
#define	FF_MIN_SS			512
#define	FF_MAX_SS			512
#define	FF_USE_TRIM			0

/************************************************************************/
/*		System Configurations                                           */
/************************************************************************/

#define	FF_FS_TINY			1
#define FF_FS_NORTC			0
#define FF_NORTC_MON		1
#define FF_NORTC_MDAY		1
#define FF_NORTC_YEAR		2015
#define FF_FS_NOFSINFO		0
#define	FF_FS_LOCK			0
#define FF_FS_REENTRANT		0
#define FF_FS_TIMEOUT		1000
#define	FF_SYNC_t			HANDLE

#endif /* CONF_FATFS_H__ */
