#ifndef CONF_FATFS_H__
#define CONF_FATFS_H__

#ifndef _FFCONF
#define _FFCONF 64180	/* Revision ID */
#endif 

/* See http://elm-chan.org/fsw/ff/doc/config.html for config details */

/************************************************************************/
/*		Function Configurations                                         */
/************************************************************************/

#define _FS_READONLY		0
#define _FS_MINIMIZE		0
#define	_USE_STRFUNC		1
#define _USE_FIND			1
#define	_USE_MKFS			1
#define	_USE_FASTSEEK		0
#define _USE_EXPAND			0
#define _USE_CHMOD			0
#define _USE_LABEL			1
#define	_USE_FORWARD		1

/************************************************************************/
/*		Namespace and Locale Configurations                             */
/************************************************************************/

#define _CODE_PAGE			437
#define	_USE_LFN			0	
#define	_MAX_LFN			255
#define	_LFN_UNICODE		0
#define _STRF_ENCODE		3
#define _FS_RPATH			2

/************************************************************************/
/*		Volume/Drive Configurations                                     */
/************************************************************************/

#define _VOLUMES			1
#define _STR_VOLUME_ID		0
#define _VOLUME_STRS		"RAM","NAND","CF","SD"
#define	_MULTI_PARTITION	0
#define	_MIN_SS				512
#define	_MAX_SS				512
#define	_USE_TRIM			0

/************************************************************************/
/*		System Configurations                                           */
/************************************************************************/

#define	_FS_TINY			1
#define _FS_NORTC			0
#define _NORTC_MON			1
#define _NORTC_MDAY			1
#define _NORTC_YEAR			2015
#define _FS_NOFSINFO		0
#define	_FS_LOCK			0
#define _FS_REENTRANT		0
#define _FS_TIMEOUT			1000
#define	_SYNC_t				HANDLE

#endif /* CONF_FATFS_H__ */
