/****************************************************************************/
/*
 * FILE          : cpurttdrv.h
 * DESCRIPTION   : CPU Runtime Test driver for sample code
 * CREATED       : 2021.04.17
 * MODIFIED      : -
 * AUTHOR        : Renesas Electronics Corporation
 * TARGET DEVICE : R-Car V3Hv2
 * TARGET OS     : BareMetal
 * HISTORY       : -
 */
/****************************************************************************/
/*
 * Copyright(C) 2021 Renesas Electronics Corporation. All Rights Reserved.
 * RENESAS ELECTRONICS CONFIDENTIAL AND PROPRIETARY
 * This program must be used solely for the purpose for which
 * it was furnished by Renesas Electronics Corporation.
 * No part of this program may be reproduced or disclosed to
 * others, in any form, without the prior written permission
 * of Renesas Electronics Corporation.
 *
 ****************************************************************************/

#ifndef CPURTTDRV_H
#define CPURTTDRV_H

#include "cpurtt_common.h"

#define UDF_CPURTT_UIO_DRIVER_NAME    "fbc_uio_share"     /* cpurtt driver name for uio */

#define DRV_CPURTTKER_CPUNUM_MAX 4U
#define DRV_CPURTTKER_SMONI_BUF_SIZE 128U

/* FieldBIST related defined values */
/* RTTFINISH1 REG */
#define DRV_RTTKER_RTTFINISH1 0xFF830100U
#define DRV_RTTKER_RTTFINISH2 0xFF830104U

/* RTTEX REG */
#define DRV_RTTKER_IMR0_RTTEX  0xFFF80000U
#define DRV_RTTKER_IMR1_RTTEX  0xFFF81000U
#define DRV_RTTKER_IMS0_RTTEX  0xFFF90000U
#define DRV_RTTKER_IMS1_RTTEX  0xFFF91000U
#define DRV_RTTKER_IMS2_RTTEX  0xFFF92000U
#define DRV_RTTKER_IMP0_RTTEX  0xFF8D0000U
#define DRV_RTTKER_IMP1_RTTEX  0xFFF50000U
#define DRV_RTTKER_IMP2_RTTEX  0xFFF60000U
#define DRV_RTTKER_IMP3_RTTEX  0xFFF70000U
#define DRV_RTTKER_IMP4_RTTEX  0xFF8D1000U
#define DRV_RTTKER_OCV0_RTTEX  0xFFFA0000U
#define DRV_RTTKER_OCV1_RTTEX  0xFFFB0000U
#define DRV_RTTKER_OCV2_RTTEX  0xFF8D2000U
#define DRV_RTTKER_OCV3_RTTEX  0xFF8D3000U
#define DRV_RTTKER_OCV4_RTTEX  0xFF8D4000U
#define DRV_RTTKER_DP0_RTTEX   0xFF8D5000U
#define DRV_RTTKER_DP1_RTTEX   0xFF8D6000U
#define DRV_RTTKER_CNN_RTTEX   0xFF8D7000U
#define DRV_RTTKER_SLIM_RTTEX  0xFF8DC000U
#define DRV_RTTKER_53D_RTTEX   0xFF87C000U
#define DRV_RTTKER_530_RTTEX   0xFF877000U
#define DRV_RTTKER_531_RTTEX   0xFF878000U
#define DRV_RTTKER_532_RTTEX   0xFF879000U
#define DRV_RTTKER_533_RTTEX   0xFF87A000U
#define DRV_RTTKER_DISP_RTTEX  0xE7950000U
#define DRV_RTTKER_UMFL_RTTEX  0xE7930000U
#define DRV_RTTKER_CLE2_RTTEX  0xE7940000U
#define DRV_RTTKER_CLE3_RTTEX  0xE7941000U

#define DRV_CPURTTKER_OFFSET_RTTEX 0x000U

/* Bit number of RTTFINISH1 register */
#define DRV_RTTKER_IMR0_RTTFINISH1_BIT  2U
#define DRV_RTTKER_IMR1_RTTFINISH1_BIT  3U
#define DRV_RTTKER_IMS0_RTTFINISH1_BIT  4U
#define DRV_RTTKER_IMS1_RTTFINISH1_BIT  5U
#define DRV_RTTKER_IMS2_RTTFINISH1_BIT  6U
#define DRV_RTTKER_CA53D_RTTFINISH1_BIT 23U
#define DRV_RTTKER_CA530_RTTFINISH1_BIT 24U
#define DRV_RTTKER_CA531_RTTFINISH1_BIT 25U
#define DRV_RTTKER_CA532_RTTFINISH1_BIT 26U
#define DRV_RTTKER_CA533_RTTFINISH1_BIT 27U

/* Bit number of RTTFINISH2 register */
#define DRV_RTTKER_IMP0_RTTFINISH2_BIT  32U
#define DRV_RTTKER_IMP1_RTTFINISH2_BIT  33U
#define DRV_RTTKER_IMP2_RTTFINISH2_BIT  34U
#define DRV_RTTKER_IMP3_RTTFINISH2_BIT  35U
#define DRV_RTTKER_IMP4_RTTFINISH2_BIT  36U
#define DRV_RTTKER_OCV0_RTTFINISH2_BIT  37U
#define DRV_RTTKER_OCV1_RTTFINISH2_BIT  38U
#define DRV_RTTKER_OCV2_RTTFINISH2_BIT  39U
#define DRV_RTTKER_OCV3_RTTFINISH2_BIT  40U
#define DRV_RTTKER_OCV4_RTTFINISH2_BIT  41U
#define DRV_RTTKER_DP0_RTTFINISH2_BIT   42U
#define DRV_RTTKER_DP1_RTTFINISH2_BIT   43U
#define DRV_RTTKER_CNN_RTTFINISH2_BIT   44U
#define DRV_RTTKER_DISP_RTTFINISH2_BIT  46U
#define DRV_RTTKER_UMFL_RTTFINISH2_BIT  47U
#define DRV_RTTKER_CLE2_RTTFINISH2_BIT  51U
#define DRV_RTTKER_CLE3_RTTFINISH2_BIT  52U
#define DRV_RTTKER_SLIM_RTTFINISH2_BIT  53U

/* Type of HIERARCHY */
#define DRV_RTTKER_HIERARCHY_CPU   0U
#define DRV_RTTKER_HIERARCHY_OTHER 1U

#define DRV_CPURTTKER_SGIR 0xF1010F00U
#define DRV_CPURTTKER_SGI_HIERARCHY530 0x00010004U
#define DRV_CPURTTKER_SGI_HIERARCHY531 0x00020004U
#define DRV_CPURTTKER_SGI_HIERARCHY53D 0x000E0004U
#define DRV_CPURTTKER_SGI_HIERARCHY532 0x00040004U
#define DRV_CPURTTKER_SGI_HIERARCHY533 0x00080004U

#define DRV_CPURTTKER_ITARGETS_11      0xF101082CU
#define DRV_CPURTTKER_ITARGETS_11_MASK 0x00FF0000U
#define DRV_CPURTTKER_ITARGETS_11_CPU0 0x00010000U
#define DRV_CPURTTKER_ITARGETS_11_CPU1 0x00020000U

#define DRV_RTTKER_FIELD_BIST_INT_CPU 0x01U
#define DRV_RTTKER_AFFINITY_MASK_BIT 0x01U

/* return code */
#define FBIST_CB_CLOSE_REQ 1
#define FBIST_BUSCHECK_ERROR 2

/* A2 RuntimeTest Defined value for synchronization management  */
#define A2SYNC_CPU0_BIT 0x0001
#define A2SYNC_CPU1_BIT 0x0002
#define A2SYNC_CPU2_BIT 0x0004
#define A2SYNC_CPU3_BIT 0x0008
#define A2SYNC_ALL      0x000F

#define CB_QUEUE_STATUS_EMPTY   0x00
#define CB_QUEUE_STATUS_ENA     0x01
#define CB_QUEUE_STATUS_FULL    0x02

/* Structure of RTTFINISH1 */
typedef union
{
    struct
    {
        uint32_t Reserved1 :2;
        uint32_t IMR0 :1;
        uint32_t IMR1 :1;
        uint32_t IMS0 :1;
        uint32_t IMS1 :1;
        uint32_t IMS2 :1;
        uint32_t Reserved2 :16;
        uint32_t CA53D :1;
        uint32_t CA530 :1;
        uint32_t CA531 :1;
        uint32_t CA532 :1;
        uint32_t CA533 :1;
        uint32_t Reserved3 :4;
    } BIT;
    uint32_t INT;
} drvRTT_RTTFINISH1_t;

/* Structure of RTTFINISH2 */
typedef union
{
    struct
    {
        uint32_t IMP0 :1;
        uint32_t IMP1 :1;
        uint32_t IMP2 :1;
        uint32_t IMP3 :1;
        uint32_t IMP4 :1;
        uint32_t OCV0 :1;
        uint32_t OCV1 :1;
        uint32_t OCV2 :1;
        uint32_t OCV3 :1;
        uint32_t OCV4 :1;
        uint32_t DP0 :1;
        uint32_t DP1 :1;
        uint32_t CNN :1;
        uint32_t Reserved1 :1;
        uint32_t DISP :1;
        uint32_t UMFL :1;
        uint32_t Reserved2 :3;
        uint32_t CLE2 :1;
        uint32_t CLE3 :1;
        uint32_t SLIM :1;
        uint32_t Reserved3 :10;
    } BIT;
    uint32_t INT;
} drvRTT_RTTFINISH2_t;

/* Structure of RTTEX */
typedef union
{
    struct
    {
        uint32_t EX :1;
        uint32_t STP :1;
        uint32_t TM :1;
        uint32_t WM :1;
        uint32_t TR :1;
        uint32_t SSZ :1;
        uint32_t Reserved1 :2;
        uint32_t STM :3;
        uint32_t Reserved2 :5;
        uint32_t KCD :8;
        uint32_t Reserved3 :4;
        uint32_t Reserved4 :1;
        uint32_t Reserved5 :1;
        uint32_t Reserved6 :2;
    } BIT;
    uint32_t INT;
} drvRTT_RTTEX_t;

typedef enum
{
    DRV_RTTKER_HIERARCHY_IMR0, /* IMR-LX4(ch4) */
    DRV_RTTKER_HIERARCHY_IMR1, /* IMR-LX4(ch5) */
    DRV_RTTKER_HIERARCHY_IMS0, /* IMR-LX4(ch0) */
    DRV_RTTKER_HIERARCHY_IMS1, /* IMR-LX4(ch1) */
    DRV_RTTKER_HIERARCHY_IMS2, /* IMR-LX4(ch2) */
    DRV_RTTKER_HIERARCHY_IMP0, /* Image Recognition Hierarchy (IMP core0) */
    DRV_RTTKER_HIERARCHY_IMP1, /* Image Recognition Hierarchy (IMP core1) */
    DRV_RTTKER_HIERARCHY_IMP2, /* Image Recognition Hierarchy (IMP core2) */
    DRV_RTTKER_HIERARCHY_IMP3, /* Image Recognition Hierarchy (IMP core3) */
    DRV_RTTKER_HIERARCHY_IMP4, /* Image Recognition Hierarchy (IMP core4) */
    DRV_RTTKER_HIERARCHY_OCV0, /* Image Recognition Hierarchy (OCV core0) */
    DRV_RTTKER_HIERARCHY_OCV1, /* Image Recognition Hierarchy (OCV core1) */
    DRV_RTTKER_HIERARCHY_OCV2, /* Image Recognition Hierarchy (OCV core2) */
    DRV_RTTKER_HIERARCHY_OCV3, /* Image Recognition Hierarchy (OCV core3) */
    DRV_RTTKER_HIERARCHY_OCV4, /* Image Recognition Hierarchy (OCV core4) */
    DRV_RTTKER_HIERARCHY_DP0, /* Image Recognition Hierarchy (IMP DMAC0, IMP PSC0) */
    DRV_RTTKER_HIERARCHY_DP1, /* Image Recognition Hierarchy (IMP DMAC1, IMP Slim-IMP DMAC) */
    DRV_RTTKER_HIERARCHY_CNN, /* Image Recognition Hierarchy (IMP CNN) */
    DRV_RTTKER_HIERARCHY_SLIM, /* Image Recognition Hierarchy (Slim-IMP core) */
    DRV_RTTKER_HIERARCHY_53D, /* Cortex-A53 Hierarchy L2 cache */
    DRV_RTTKER_HIERARCHY_530, /* Cortex-A53 Hierarchy cpu0 */
    DRV_RTTKER_HIERARCHY_531, /* Cortex-A53 Hierarchy cpu1 */
    DRV_RTTKER_HIERARCHY_532, /* Cortex-A53 Hierarchy cpu2 */
    DRV_RTTKER_HIERARCHY_533, /* Cortex-A53 Hierarchy cpu3 */
    DRV_RTTKER_HIERARCHY_DISP, /* Vision IP Hierarchy (Disparity) */
    DRV_RTTKER_HIERARCHY_UMFL, /* Vision IP Hierarchy (Optical Flow) */
    DRV_RTTKER_HIERARCHY_CLE2, /* Vision IP Hierarchy (Classifier 0/1) */
    DRV_RTTKER_HIERARCHY_CLE3, /* Vision IP Hierarchy (Classifier 2/3/4) */
    DRV_RTTKER_HIERARCHY_MAX
} drvRTT_hierarchy_t;

typedef struct
{
    uint32_t mHierarchyType;
    drvRTT_hierarchy_t mHierarchy;
    uint32_t mRttfinishBit;
    uint32_t mRttexAdr;
} drvCPURTT_FbistHdr_t;

typedef struct
{
    uint8_t head;
    uint8_t pos;
    uint8_t status;
    drvCPURTT_CallbackInfo_t CbInfo[DRV_RTTKER_HIERARCHY_MAX];
}drvCPURTT_CbInfoQueue_t;

typedef int (*A2ThreadTable)(void*);

struct fbc_uio_share_soc_res {
	char **clk_names;
};

/* */
struct fbc_uio_share_platform_data {
	struct platform_device *pdev;
	struct uio_info *uio_info;
	void __iomem *base_addr[1];
	struct clk *clks[3];
	int clk_count;
    spinlock_t lock;
    unsigned long flags;
};

enum {
    UIO_IRQ_DISABLED = 0,
};

#endif
