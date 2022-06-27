/*******************************************************************************
** Copyright (C), 2008-2016, OPPO Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** FILE: - hiamx_inspection.h
** Description : This program is for hiamx driver
** Version: 1.0
** Date : 2018/5/22
**
** -------------------------Revision History:----------------------------------
**  <author>	 <data> 	<version >			<desc>
**
**
*******************************************************************************/
#include "himax_platform.h"
#include "himax_common.h"
#include "himax_ic_core.h"

/*#define HX_GAP_TEST*/
#define HX_INSPECT_LPWUG_TEST
/*#define HX_DOZE_TEST*/
/*define HX_SELF_TEST*/

#define BS_RAWDATANOISE	10
#define BS_OPENSHORT	0

#ifdef HX_INSPECT_LPWUG_TEST
#define	BS_LPWUG	1
#define	BS_LPWUG_dile	1
#endif

#ifdef HX_DOZE_TEST
#define	BS_DOZE	1
#endif

/* skip notch & dummy */
#define SKIP_NOTCH_START	5
#define SKIP_NOTCH_END	10
/* TX+SKIP_NOTCH_START */
#define SKIP_DUMMY_START	23
/* TX+SKIP_NOTCH_END*/
#define SKIP_DUMMY_END	28

#define NOTCH_RX_START	36
#define NOTCH_RX_END		36
#define NOTCH_TX_START	6
#define NOTCH_TX_END		13

#define	NOISEFRAME	(BS_RAWDATANOISE+1)
#define	UNIFMAX	500


/*Himax MP Password*/
#define	PWD_OPEN_START	0x77
#define	PWD_OPEN_END	0x88
#define	PWD_SHORT_START	0x11
#define	PWD_SHORT_END	0x33
#define	PWD_RAWDATA_START	0x00
#define	PWD_RAWDATA_END	0x99
#define	PWD_NOISE_START	0x00
#define	PWD_NOISE_END	0x99
#define	PWD_SORTING_START	0xAA
#define	PWD_SORTING_END	0xCC

#ifdef HX_DOZE_TEST
#define PWD_DOZE_START		0x22
#define PWD_DOZE_END		0x44
#endif

#ifdef HX_INSPECT_LPWUG_TEST
#define PWD_LPWUG_START	0x55
#define PWD_LPWUG_END	0x66

#define PWD_LPWUG_IDLE_START	0x50
#define PWD_LPWUG_IDLE_END	0x60
#endif

/*Himax DataType*/
#define DATA_SORTING	0x0A
#define DATA_OPEN	0x0B
#define DATA_MICRO_OPEN	0x0C
#define DATA_SHORT	0x0A
#define DATA_RAWDATA	0x0A
#define DATA_NOISE	0x0F
#define DATA_BACK_NORMAL	0x00
#define DATA_LPWUG_RAWDATA	0x0C
#define DATA_LPWUG_NOISE	0x0F
#define DATA_DOZE_RAWDATA	0x0A
#define DATA_DOZE_NOISE	0x0F
#define DATA_LPWUG_IDLE_RAWDATA	0x0A
#define DATA_LPWUG_IDLE_NOISE	0x0F

/*Himax Data Ready Password*/
#define	Data_PWD0	0xA5
#define	Data_PWD1	0x5A

/* ASCII format */
#define ASCII_LF	(0x0A)
#define ASCII_CR	(0x0D)
#define ASCII_COMMA	(0x2C)
#define ASCII_ZERO	(0x30)
#define CHAR_EL	'\0'
#define CHAR_NL	'\n'
#define ACSII_SPACE	(0x20)
/* INSOECTION Setting */
#define HX_CRITERIA_ITEM	(24)
#define HX_CRITERIA_SIZE	(HX_CRITERIA_ITEM*2)

typedef enum {
	HIMAX_INSPECTION_OPEN,
	HIMAX_INSPECTION_MICRO_OPEN,
	HIMAX_INSPECTION_SHORT,
	HIMAX_INSPECTION_RAWDATA,
	HIMAX_INSPECTION_NOISE,
	HIMAX_INSPECTION_SORTING,
	HIMAX_INSPECTION_BACK_NORMAL,
	HIMAX_INSPECTION_GAPTEST_RAW,
#ifdef HX_DOZE_TEST
	HIMAX_INSPECTION_DOZE_RAWDATA,
	HIMAX_INSPECTION_DOZE_NOISE,
#endif
#ifdef HX_INSPECT_LPWUG_TEST
	HIMAX_INSPECTION_LPWUG_RAWDATA,
	HIMAX_INSPECTION_LPWUG_NOISE,
	HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA,
	HIMAX_INSPECTION_LPWUG_IDLE_NOISE,
#endif
} THP_INSPECTION_ENUM;

char *g_himax_inspection_mode[] = {
	"HIMAX_INSPECTION_OPEN",
	"HIMAX_INSPECTION_MICRO_OPEN",
	"HIMAX_INSPECTION_SHORT",
	"HIMAX_INSPECTION_RAWDATA",
	"HIMAX_INSPECTION_NOISE",
	"HIMAX_INSPECTION_SORTING",
	"HIMAX_INSPECTION_BACK_NORMAL",
	"HIMAX_INSPECTION_GAPTEST_RAW",
#ifdef HX_DOZE_TEST
	"HIMAX_INSPECTION_DOZE_RAWDATA",
	"HIMAX_INSPECTION_DOZE_NOISE",
#endif
#ifdef HX_INSPECT_LPWUG_TEST
	"HIMAX_INSPECTION_LPWUG_RAWDATA",
	"HIMAX_INSPECTION_LPWUG_NOISE",
	"HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA",
	"HIMAX_INSPECTION_LPWUG_IDLE_NOISE",
#endif
};

/* for criteria */
int *g_inspection_criteria;
typedef enum {
	IDX_RAWMIN		= 0,
	IDX_RAWMAX,
	IDX_SHORTMIN,
	IDX_SHORTMAX,
	IDX_OPENMIN,
	IDX_OPENMAX,
	IDX_M_OPENMIN,
	IDX_M_OPENMAX,
	IDX_NOISEMAX,
	IDX_GAP_RAWMIN,
	IDX_GAP_RAWMAX,
	IDX_SORTMIN,
	IDX_DOZE_NOISE_MIN,
	IDX_DOZE_NOISE_MAX,
	
	IDX_DOZE_RAWDATA_MIN,
	IDX_DOZE_RAWDATA_MAX,
	IDX_LPWUG_NOISE_MIN,
	IDX_LPWUG_NOISE_MAX,
	
	IDX_LPWUG_RAWDATA_MIN,
	IDX_LPWUG_RAWDATA_MAX,
	
	IDX_LPWUG_IDLE_NOISE_MIN,
	IDX_LPWUG_IDLE_NOISE_MAX,
	
	IDX_LPWUG_IDLE_RAWDATA_MIN,
	IDX_LPWUG_IDLE_RAWDATA_MAX,
} HX_CRITERIA_ENUM;

/* Error code of Inspection */
typedef enum {
	HX_INSPECT_OK	= 0,               /* OK */
	HX_INSPECT_ESPI,        /* SPI communication error */
	HX_INSPECT_ERAW,        /* Raw data error */
	HX_INSPECT_ENOISE,        /* Noise error */
	HX_INSPECT_EOPEN,        /* Sensor open error */
	HX_INSPECT_EMOPEN,        /* Sensor micro open error */
	HX_INSPECT_ESHORT,        /* Sensor short error */
	HX_INSPECT_EGAP_RAW,        /* Sensor short error */
#ifdef HX_DOZE_TEST
	HX_INSPECT_EDOZE_RAW,		   /* DOZE RAW ERROR */
	HX_INSPECT_EDOZE_NOISE,		   /* DOZE NOISE ERROR */
#endif
#ifdef HX_INSPECT_LPWUG_TEST
	HX_INSPECT_ELPWUG_RAW,		   /* LPWUG RAW ERROR */
	HX_INSPECT_ELPWUG_NOISE,		   /* LPWUG NOISE ERROR */
	HX_INSPECT_ELPWUG_IDLE_RAW,		   /* LPWUG IDLE RAW ERROR */
	HX_INSPECT_ELPWUG_IDLE_NOISE,		   /* LPWUG IDLE NOISE ERROR */
#endif
	HX_INSPECT_ERC,        /* Sensor RC error */
	HX_INSPECT_EPIN,        /* Errors of TSVD!￠FTSHD!￠FTRCST!￠FTRCRQ and other PINs
				when Report Rate Switching between 60 Hz and 120 Hz*/
	HX_INSPECT_EOTHER,	/* All other errors */
} HX_INSPECT_ERR_ENUM;
