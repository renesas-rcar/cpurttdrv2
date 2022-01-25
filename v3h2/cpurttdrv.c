/****************************************************************************/
/*
 * FILE          : cpurttdrv.c
 * DESCRIPTION   : CPU Runtime Test driver for sample code
 * CREATED       : 2021.04.20
 * MODIFIED      : 2022.01.24
 * AUTHOR        : Renesas Electronics Corporation
 * TARGET DEVICE : R-Car V3Hv2
 * TARGET OS     : BareMetal
 * HISTORY       : 
 *                 2021.05.31 Modify so that ioctl arguments can be held correctly when multiple ioctls are executed.
 *                 2021.07.15 Modify prefix of smoni api.
 *                 2021.10.19 Modify the storage method of fail information in FbistInterruptHandler.
 *                            Modify the execution method of A2 Runtime Test.
 *                 2021.11.22 Update software version.
 *                 2022.01.24 Add ioctl command for HWA Runtime Test.
 */
/****************************************************************************/
/*
 * Copyright(C) 2021-2022 Renesas Electronics Corporation. All Rights Reserved.
 * RENESAS ELECTRONICS CONFIDENTIAL AND PROPRIETARY
 * This program must be used solely for the purpose for which
 * it was furnished by Renesas Electronics Corporation.
 * No part of this program may be reproduced or disclosed to
 * others, in any form, without the prior written permission
 * of Renesas Electronics Corporation.
 *
 ****************************************************************************/
/***********************************************************
 Includes <System Includes>, "Project Includes"
 ***********************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/smp.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <linux/ioport.h>
#include <asm/irqflags.h>
#include <linux/completion.h>
#include <linux/cpumask.h>
#include <linux/cpuset.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/semaphore.h>
#include <linux/uio_driver.h>
#include <linux/io.h>
#include <linux/of_irq.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>

#include "cpurttdrv.h"
#include "cpurtt_common.h"
#include "smoni_def.h"
#include "smoni_api_v2.h"

#undef IS_INTERRUPT

#define DRIVER_VERSION "1.0.1"

/***********************************************************
 Macro definitions
 ***********************************************************/

/***********************************************************
 Typedef definitions
 ***********************************************************/

/***********************************************************
 Exported global variables (to be accessed by other files)
 ***********************************************************/

/***********************************************************
 Private global variables and functions
 ***********************************************************/

/* Variables used for device registration of cpurttmod */
static unsigned int cpurtt_major = 0;        /*  0:auto */
static struct class *cpurtt_class = NULL;

/* Variables used unique to cpurttmod */
static uint32_t g_SmoniAddrBuf[DRV_CPURTTKER_CPUNUM_MAX][DRV_CPURTTKER_SMONI_BUF_SIZE]; /* address buffer for parameter of R_SMONI_API_RuntimeTestFbaWrite/R_SMONI_API_RuntimeTestFbaRead */
static uint32_t g_SmoniDataBuf[DRV_CPURTTKER_CPUNUM_MAX][DRV_CPURTTKER_SMONI_BUF_SIZE]; /* data buffer for parameter of R_SMONI_API_RuntimeTestFbaWrite/R_SMONI_API_RuntimeTestFbaRead */
static struct completion g_A2StartSynCompletion;
static struct completion g_A2EndSynCompletion[DRV_CPURTTKER_CPUNUM_MAX];
static struct completion g_A2ThWakeupCompletion[DRV_CPURTTKER_CPUNUM_MAX];

static uint16_t g_A2SmoniResult[DRV_CPURTTKER_CPUNUM_MAX];
static drvCPURTT_A2rttParam_t g_A2Param[DRV_CPURTTKER_CPUNUM_MAX];

static struct task_struct *g_A2Task[DRV_CPURTTKER_CPUNUM_MAX];
static uint32_t g_TaskExitFlg = 0;

static void __iomem *g_RegBaseSgir = NULL;
static void __iomem *g_RegBaseItargets11 = NULL;
static void __iomem *g_RegBaseRttfinish1 = NULL;
static void __iomem *g_RegBaseRttfinish2 = NULL;
static void __iomem *g_RegBaseAddrTable[DRV_RTTKER_HIERARCHY_MAX] = {
    NULL, /* IMR-LX4(ch4) */
    NULL, /* IMR-LX4(ch5) */
    NULL, /* IMR-LX4(ch0) */
    NULL, /* IMR-LX4(ch1) */
    NULL, /* IMR-LX4(ch2) */
    NULL, /* Image Recognition Hierarchy (IMP core0) */
    NULL, /* Image Recognition Hierarchy (IMP core1) */
    NULL, /* Image Recognition Hierarchy (IMP core2) */
    NULL, /* Image Recognition Hierarchy (IMP core3) */
    NULL, /* Image Recognition Hierarchy (IMP core4) */
    NULL, /* Image Recognition Hierarchy (OCV core0) */
    NULL, /* Image Recognition Hierarchy (OCV core1) */
    NULL, /* Image Recognition Hierarchy (OCV core2) */
    NULL, /* Image Recognition Hierarchy (OCV core3) */
    NULL, /* Image Recognition Hierarchy (OCV core4) */
    NULL, /* Image Recognition Hierarchy (IMP DMAC0, IMP PSC0) */
    NULL, /* Image Recognition Hierarchy (IMP DMAC1, IMP PSC1) */
    NULL, /* Image Recognition Hierarchy (IMP CNN) */
    NULL, /* Image Recognition Hierarchy (Slim-IMP core) */
    NULL, /* Cortex-A53 Hierarchy L2 cache */
    NULL, /* Cortex-A53 Hierarchy cpu0 */
    NULL, /* Cortex-A53 Hierarchy cpu1 */
    NULL, /* Cortex-A53 Hierarchy cpu2 */
    NULL, /* Cortex-A53 Hierarchy cpu3 */
    NULL, /* Vision IP Hierarchy (Disparity) */
    NULL, /* Vision IP Hierarchy (Optical Flow) */
    NULL, /* Vision IP Hierarchy (Classifier 0/1) */
    NULL, /* Vision IP Hierarchy (Classifier 2/3/4) */
};

static const phys_addr_t drvRTT_PhyRegAddr[DRV_RTTKER_HIERARCHY_MAX] = {
    DRV_RTTKER_IMR0_RTTEX,
    DRV_RTTKER_IMR1_RTTEX,
    DRV_RTTKER_IMS0_RTTEX,
    DRV_RTTKER_IMS1_RTTEX,
    DRV_RTTKER_IMS2_RTTEX,
    DRV_RTTKER_IMP0_RTTEX,
    DRV_RTTKER_IMP1_RTTEX,
    DRV_RTTKER_IMP2_RTTEX,
    DRV_RTTKER_IMP3_RTTEX,
    DRV_RTTKER_IMP4_RTTEX,
    DRV_RTTKER_OCV0_RTTEX,
    DRV_RTTKER_OCV1_RTTEX,
    DRV_RTTKER_OCV2_RTTEX,
    DRV_RTTKER_OCV3_RTTEX,
    DRV_RTTKER_OCV4_RTTEX,
    DRV_RTTKER_DP0_RTTEX,
    DRV_RTTKER_DP1_RTTEX,
    DRV_RTTKER_CNN_RTTEX,
    DRV_RTTKER_SLIM_RTTEX,
    DRV_RTTKER_53D_RTTEX,
    DRV_RTTKER_530_RTTEX,
    DRV_RTTKER_531_RTTEX,
    DRV_RTTKER_532_RTTEX,
    DRV_RTTKER_533_RTTEX,
    DRV_RTTKER_DISP_RTTEX,
    DRV_RTTKER_UMFL_RTTEX,
    DRV_RTTKER_CLE2_RTTEX,
    DRV_RTTKER_CLE3_RTTEX
};

static const uint32_t drvCPURTTKER_SgirData[5U] = {
    DRV_CPURTTKER_SGI_HIERARCHY53D,
    DRV_CPURTTKER_SGI_HIERARCHY530,
    DRV_CPURTTKER_SGI_HIERARCHY531,
    DRV_CPURTTKER_SGI_HIERARCHY532,
    DRV_CPURTTKER_SGI_HIERARCHY533
};

static drvCPURTT_CbInfoQueue_t g_CallbackInfo;
static struct semaphore CallbackSem;
static bool FbistCloseReq;

static struct platform_device *g_cpurtt_pdev = NULL;

static int CpurttDrv_open(struct inode *inode, struct file *file);
static int CpurttDrv_close(struct inode *inode, struct file *file);
static long CpurttDrv_ioctl(struct file* filp, unsigned int cmd, unsigned long arg );
static int CpurttDrv_init(void);
static void CpurttDrv_exit(void);

static long drvCPURTT_UDF_RuntimeTestInit(void);
static long drvCPURTT_UDF_RuntimeTestDeinit(void);
static long drvCPURTT_UDF_SmoniApiExe(drvCPURTT_SmoniTable_t index, uint32_t aCpuId, uint32_t *aArg, uint32_t *aSmoniret);

static long drvCPURTT_UDF_FbistInit(void);
static long drvCPURTT_UDF_FbistDeInit(void);
static long drvCPURTT_UDF_WaitCbNotice(drvCPURTT_CallbackInfo_t *aParam);
static long drvCPURTT_UDF_HWAExecute(uint32_t aHierarchy, uint32_t aRttex);

static int fbc_uio_share_clk_enable(struct fbc_uio_share_platform_data *pdata)
{
    int ret;
    int i;

    for (i = 0; i < pdata->clk_count; i++) {
        ret = clk_prepare_enable(pdata->clks[i]);
        if (ret) {
            dev_err(&pdata->pdev->dev, "Clock enable failed\n");
            break;
        }
    }

    if (ret)
    {
        for (i--; i >= 0; i--)
            clk_disable_unprepare(pdata->clks[i]);
    }

    return ret;
}

static void fbc_uio_share_clk_disable(struct fbc_uio_share_platform_data *pdata)
{
    int i;

    for (i = 0; i < pdata->clk_count; i++)
        clk_disable_unprepare(pdata->clks[i]);
}

static void FbistInterruptHandler(int irq, struct uio_info *uio_info)
{
    uint32_t RegVal_fin1;
    uint32_t RegVal_fin2;
    drvRTT_hierarchy_t i;
    uint64_t FiniVal;
    uint32_t WriteRegVal;
    uint32_t HierarchyBit;
    uint32_t Address;
    uint32_t Smoniret;
    uint32_t FbistCbRes = 0;
    uint32_t BusCheckCbRes = 0;
    uint32_t RfsoOutputRes = 0;
    drvRTT_RTTEX_t ReadRegVal;

    static const drvCPURTT_FbistHdr_t FbisthdrInfo[DRV_RTTKER_HIERARCHY_MAX] = {
        {DRV_RTTKER_HIERARCHY_OTHER,	DRV_RTTKER_HIERARCHY_IMR0,	DRV_RTTKER_IMR0_RTTFINISH1_BIT,	DRV_RTTKER_IMR0_RTTEX},
        {DRV_RTTKER_HIERARCHY_OTHER,	DRV_RTTKER_HIERARCHY_IMR1,	DRV_RTTKER_IMR1_RTTFINISH1_BIT,	DRV_RTTKER_IMR1_RTTEX},
        {DRV_RTTKER_HIERARCHY_OTHER,	DRV_RTTKER_HIERARCHY_IMS0,	DRV_RTTKER_IMS0_RTTFINISH1_BIT,	DRV_RTTKER_IMS0_RTTEX},
        {DRV_RTTKER_HIERARCHY_OTHER,	DRV_RTTKER_HIERARCHY_IMS1,	DRV_RTTKER_IMS1_RTTFINISH1_BIT,	DRV_RTTKER_IMS1_RTTEX},
        {DRV_RTTKER_HIERARCHY_OTHER,	DRV_RTTKER_HIERARCHY_IMS2,	DRV_RTTKER_IMS2_RTTFINISH1_BIT,	DRV_RTTKER_IMS2_RTTEX},
        {DRV_RTTKER_HIERARCHY_CPU,	    DRV_RTTKER_HIERARCHY_53D,	DRV_RTTKER_CA53D_RTTFINISH1_BIT,	DRV_RTTKER_53D_RTTEX},
        {DRV_RTTKER_HIERARCHY_CPU,	    DRV_RTTKER_HIERARCHY_530,	DRV_RTTKER_CA530_RTTFINISH1_BIT,	DRV_RTTKER_530_RTTEX},
        {DRV_RTTKER_HIERARCHY_CPU,	    DRV_RTTKER_HIERARCHY_531,	DRV_RTTKER_CA531_RTTFINISH1_BIT,	DRV_RTTKER_531_RTTEX},
        {DRV_RTTKER_HIERARCHY_CPU,	    DRV_RTTKER_HIERARCHY_532,	DRV_RTTKER_CA532_RTTFINISH1_BIT,	DRV_RTTKER_532_RTTEX},
        {DRV_RTTKER_HIERARCHY_CPU,	    DRV_RTTKER_HIERARCHY_533,	DRV_RTTKER_CA533_RTTFINISH1_BIT,	DRV_RTTKER_533_RTTEX},
        {DRV_RTTKER_HIERARCHY_OTHER,	DRV_RTTKER_HIERARCHY_IMP0,	DRV_RTTKER_IMP0_RTTFINISH2_BIT,	DRV_RTTKER_IMP0_RTTEX},
        {DRV_RTTKER_HIERARCHY_OTHER,	DRV_RTTKER_HIERARCHY_IMP1,	DRV_RTTKER_IMP1_RTTFINISH2_BIT,	DRV_RTTKER_IMP1_RTTEX},
        {DRV_RTTKER_HIERARCHY_OTHER,	DRV_RTTKER_HIERARCHY_IMP2,	DRV_RTTKER_IMP2_RTTFINISH2_BIT,	DRV_RTTKER_IMP2_RTTEX},
        {DRV_RTTKER_HIERARCHY_OTHER,	DRV_RTTKER_HIERARCHY_IMP3,	DRV_RTTKER_IMP3_RTTFINISH2_BIT,	DRV_RTTKER_IMP3_RTTEX},
        {DRV_RTTKER_HIERARCHY_OTHER,	DRV_RTTKER_HIERARCHY_IMP4,	DRV_RTTKER_IMP4_RTTFINISH2_BIT,	DRV_RTTKER_IMP4_RTTEX},
        {DRV_RTTKER_HIERARCHY_OTHER,	DRV_RTTKER_HIERARCHY_OCV0,	DRV_RTTKER_OCV0_RTTFINISH2_BIT,	DRV_RTTKER_OCV0_RTTEX},
        {DRV_RTTKER_HIERARCHY_OTHER,	DRV_RTTKER_HIERARCHY_OCV1,	DRV_RTTKER_OCV1_RTTFINISH2_BIT,	DRV_RTTKER_OCV1_RTTEX},
        {DRV_RTTKER_HIERARCHY_OTHER,	DRV_RTTKER_HIERARCHY_OCV2,	DRV_RTTKER_OCV2_RTTFINISH2_BIT,	DRV_RTTKER_OCV2_RTTEX},
        {DRV_RTTKER_HIERARCHY_OTHER,	DRV_RTTKER_HIERARCHY_OCV3,	DRV_RTTKER_OCV3_RTTFINISH2_BIT,	DRV_RTTKER_OCV3_RTTEX},
        {DRV_RTTKER_HIERARCHY_OTHER,	DRV_RTTKER_HIERARCHY_OCV4,	DRV_RTTKER_OCV4_RTTFINISH2_BIT,	DRV_RTTKER_OCV4_RTTEX},
        {DRV_RTTKER_HIERARCHY_OTHER,	DRV_RTTKER_HIERARCHY_DP0,	DRV_RTTKER_DP0_RTTFINISH2_BIT,	    DRV_RTTKER_DP0_RTTEX},
        {DRV_RTTKER_HIERARCHY_OTHER,	DRV_RTTKER_HIERARCHY_DP1,	DRV_RTTKER_DP1_RTTFINISH2_BIT,	    DRV_RTTKER_DP1_RTTEX},
        {DRV_RTTKER_HIERARCHY_OTHER,	DRV_RTTKER_HIERARCHY_CNN,	DRV_RTTKER_CNN_RTTFINISH2_BIT,	    DRV_RTTKER_CNN_RTTEX},
        {DRV_RTTKER_HIERARCHY_OTHER,	DRV_RTTKER_HIERARCHY_DISP,	DRV_RTTKER_DISP_RTTFINISH2_BIT,	DRV_RTTKER_DISP_RTTEX},
        {DRV_RTTKER_HIERARCHY_OTHER,	DRV_RTTKER_HIERARCHY_UMFL,	DRV_RTTKER_UMFL_RTTFINISH2_BIT,	DRV_RTTKER_UMFL_RTTEX},
        {DRV_RTTKER_HIERARCHY_OTHER,	DRV_RTTKER_HIERARCHY_CLE2,	DRV_RTTKER_CLE2_RTTFINISH2_BIT,	DRV_RTTKER_CLE2_RTTEX},
        {DRV_RTTKER_HIERARCHY_OTHER,	DRV_RTTKER_HIERARCHY_CLE3,	DRV_RTTKER_CLE3_RTTFINISH2_BIT,	DRV_RTTKER_CLE3_RTTEX},
        {DRV_RTTKER_HIERARCHY_OTHER,	DRV_RTTKER_HIERARCHY_SLIM,	DRV_RTTKER_SLIM_RTTFINISH2_BIT,	DRV_RTTKER_SLIM_RTTEX}
    };

    HierarchyBit = 0;
    Address = 0;
    Smoniret = 0;
    ReadRegVal.INT = 0U;

    RegVal_fin1 = readl(g_RegBaseRttfinish1);
    RegVal_fin2 = readl(g_RegBaseRttfinish2);
    FiniVal = (uint64_t)(((uint64_t) RegVal_fin2 << 32U) | RegVal_fin1);

    for (i=0; i<DRV_RTTKER_HIERARCHY_MAX; i++)
    {
        /* fieldBIST Completion Check for interrupt factors */
        if (0x1U == (((FiniVal >> FbisthdrInfo[i].mRttfinishBit) & 0x1U)))
        {
            WriteRegVal = (uint32_t) 0x0U;
            if (DRV_RTTKER_HIERARCHY_CPU == FbisthdrInfo[i].mHierarchyType)
            {
                /* If the hierarchy is CPU, clear the RTTEX register via the Smoni API */
                HierarchyBit = ((uint32_t)(FbisthdrInfo[i].mRttfinishBit << (uint32_t)16U));
                Address = (HierarchyBit | DRV_CPURTTKER_OFFSET_RTTEX);

                Smoniret = R_SMONI_API_RuntimeTestFbaWrite(&Address, (uint32_t *)&WriteRegVal, 1U);
                if (SMONI_FUSA_OK == Smoniret)
                {
                   /* Set the execution request information of the callback function to the user layer */
                    FbistCbRes |= (uint32_t)1U << FbisthdrInfo[i].mHierarchy; 
                   /* RuntimeTest Requests sgi wakeup from the target CPU */
                    writel(drvCPURTTKER_SgirData[FbisthdrInfo[i].mHierarchy - DRV_RTTKER_HIERARCHY_53D], g_RegBaseSgir);
                }
                else if (SMONI_FUSA_ERROR_EXCLUSIVE == Smoniret)
                {
                   /* If an exclusize error occurs in the Smoni API, set the output terminal output request information to the user layer  */
                    RfsoOutputRes |= (uint32_t)1U << FbisthdrInfo[i].mHierarchy;
                }
                else
                {
                   /* If an error occurs in the Smoni API, Set the execution request information of the bascheck callback function to the user layer */
                    BusCheckCbRes |= (uint32_t)1U << FbisthdrInfo[i].mHierarchy; 
                }
            }
            else
            {

                 ReadRegVal.INT = readl(g_RegBaseAddrTable[FbisthdrInfo[i].mHierarchy]);
                 if (1 == ReadRegVal.BIT.EX)
                 {
                     /* clear RTTEX register*/
                     writel(0U, g_RegBaseAddrTable[FbisthdrInfo[i].mHierarchy]);

                     /* bus check RTTEX registerr*/
                     ReadRegVal.INT = readl(g_RegBaseAddrTable[FbisthdrInfo[i].mHierarchy]);
                     if (0x00000000U != ReadRegVal.INT)
                     {
                         /* If buscheck error occurs, Set the execution request information of the bascheck callback function to the user layer */
                         BusCheckCbRes |= (uint32_t)1U << FbisthdrInfo[i].mHierarchy; 
                     }
                     else
                     {
                         /* Set the execution request information of the callback function to the user layer */
                         FbistCbRes |= (uint32_t)1U << FbisthdrInfo[i].mHierarchy; 

                     }
                }
                else
                {
                   /* If an exclusize error occurs in the Smoni API, set the output terminal output request information to the user layer  */
                    RfsoOutputRes |= (uint32_t)1U << FbisthdrInfo[i].mHierarchy;
                }
            }
        }
    }

    if (g_CallbackInfo.pos < DRV_RTTKER_HIERARCHY_MAX)
    {
        /* Callback request execution information is stored in the queue buffer  */
        g_CallbackInfo.CbInfo[(g_CallbackInfo.head + g_CallbackInfo.pos) % DRV_RTTKER_HIERARCHY_MAX ].FbistCbRequest = FbistCbRes;
        g_CallbackInfo.CbInfo[(g_CallbackInfo.head + g_CallbackInfo.pos) % DRV_RTTKER_HIERARCHY_MAX ].BusCheckCbRequest = BusCheckCbRes;
        g_CallbackInfo.CbInfo[(g_CallbackInfo.head + g_CallbackInfo.pos) % DRV_RTTKER_HIERARCHY_MAX ].RfsoOutputPinRequest = RfsoOutputRes;
        g_CallbackInfo.pos++;
        g_CallbackInfo.status = CB_QUEUE_STATUS_ENA;
    }
    else
    {
        /* If the queue buffer is full Set the status to full  */
        g_CallbackInfo.status = CB_QUEUE_STATUS_FULL;
    }

    /* release semafore for execute callback to the user layer */
    up(&CallbackSem);
}

static irqreturn_t fbc_uio_share_irq_handler(int irq, struct uio_info *uio_info)
{
    FbistInterruptHandler(irq, uio_info);

    return IRQ_HANDLED;
}

static const struct fbc_uio_share_soc_res fbc_uio_share_soc_res = {
    .clk_names = (char*[]){
        "fbc_post",
        "fbc_post2",
        "fbc_post4",
        NULL
    },
};

static const struct of_device_id fbc_uio_share_of_table[] = {
    {
        .compatible = "renesas,uio-fbc" , .data = &fbc_uio_share_soc_res,
    }, {
    },
};
MODULE_DEVICE_TABLE(of, fbc_uio_share_of_table);

static int fbc_uio_share_init_clocks(struct platform_device *pdev, char **clk_names, struct fbc_uio_share_platform_data *pdata)
{
    int i;

    if (!clk_names)
        return 0;

    for (i = 0; clk_names[i]; i++) {
        pdata->clks[i] = devm_clk_get(&pdev->dev, clk_names[i]);
        if (IS_ERR(pdata->clks[i])) {
            int ret = PTR_ERR(pdata->clks[i]);

            if (ret != -EPROBE_DEFER)
                dev_err(&pdev->dev, "Failed to get %s clock\n", clk_names[i]);
            return ret;
        }
    }

    pdata->clk_count = i;

    return i;
}

static int fbc_uio_share_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct uio_info *uio_info;
    struct uio_mem *uio_mem;
    struct resource *res;
    struct fbc_uio_share_platform_data *priv;
    const struct fbc_uio_share_soc_res *soc_res;
    int ret;

    /* check device id */
    soc_res = of_device_get_match_data(dev);
    if (!soc_res)
    {
        dev_err(dev, "failed to not match soc resource\n");
        return -EINVAL;
    }

    priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
    if (!priv)
    {
        dev_err(dev, "failed to alloc fbc_uio_share_platform_data memory\n");
        return -ENOMEM;
    }

    /* Construct the uio_info structure */
    uio_info = devm_kzalloc(dev, sizeof(*uio_info), GFP_KERNEL);
    if (!uio_info)
    {
        dev_err(dev, "failed to alloc uio_info memory\n");
        return -ENOMEM;
    }

    uio_info->version = DRIVER_VERSION;
    uio_info->name = pdev->name;

    /* Set up memory resource */
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res)
    {
        dev_err(dev, "failed to get fbc_uio_share memory\n");
        return -EINVAL;
    }

    uio_mem = &uio_info->mem[0];
    uio_mem->memtype = UIO_MEM_PHYS;
    uio_mem->addr = res->start;
    uio_mem->size = resource_size(res);
    uio_mem->name = res->name;

    ret = fbc_uio_share_init_clocks(pdev, soc_res->clk_names, priv);
    if (ret<0)
        return ret;

    spin_lock_init(&priv->lock);
    priv->flags = 0; /* interrupt is enabled to begin with */

    /* Set up interrupt */
    uio_info->irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
    if (uio_info->irq == -ENXIO) {
        dev_err(dev, "failed to parse fbc_uio_share interrupt\n");
        return -EINVAL;
    }
    uio_info->handler = fbc_uio_share_irq_handler;

    uio_info->priv = priv;

    /* Register the UIO device */
    if ((ret = uio_register_device(dev, uio_info))) {
            dev_err(dev, "failed to register UIO device for fbc_uio_share\n");
            return ret;
        }

    /* Register this pdev and uio_info with the platform data */
    priv->pdev = pdev;
    priv->uio_info = uio_info;
    platform_set_drvdata(pdev, priv);

    g_cpurtt_pdev = pdev;

    /* Register the UIO device with the PM framework */
    pm_runtime_enable(dev);

    /* Print some information */
    dev_info(dev, "probed fbc_uio_share\n");

    return 0;
}

static int fbc_uio_share_remove(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct fbc_uio_share_platform_data *priv = platform_get_drvdata(pdev);
    struct uio_info *uio_info = priv->uio_info;

    pm_runtime_disable(&priv->pdev->dev);

    spin_lock(&priv->lock);
    if (!__test_and_set_bit(UIO_IRQ_DISABLED, &priv->flags))
        disable_irq((unsigned int)uio_info->irq);
    spin_unlock(&priv->lock);

    uio_unregister_device(priv->uio_info);

    platform_set_drvdata(pdev, NULL);

    dev_info(dev, "removed fbc_uio_share\n");

    return 0;
}

static struct platform_driver fbc_uio_share_driver = {
    .probe         = fbc_uio_share_probe,
    .remove        = fbc_uio_share_remove,
    .driver        = {
        .name = UDF_CPURTT_UIO_DRIVER_NAME,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(fbc_uio_share_of_table),
    }
};

static long drvCPURTT_InitRegAddr(void)
{
    int i;
    struct resource *Resource;

    if (NULL == g_RegBaseSgir)
    {
        Resource = request_mem_region(DRV_CPURTTKER_SGIR, 4U, UDF_CPURTT_DRIVER_NAME);
        if (NULL == Resource)
        {
            pr_err("failed to get SGIR resource\n");
            return -ENOSPC;
        }

        g_RegBaseSgir = ioremap_nocache(DRV_CPURTTKER_SGIR, 4U);

        if(NULL == g_RegBaseSgir)
        {
            pr_err("failed to get SGIR address \n");
            return -EFAULT;
        }
    }

    if (NULL == g_RegBaseItargets11)
    {
        Resource = request_mem_region(DRV_CPURTTKER_ITARGETS_11, 4U, UDF_CPURTT_DRIVER_NAME);
        if (NULL == Resource)
        {
            pr_err("failed to get GICD_ITARGETS11 resource\n");
            return -ENOSPC;
        }

        g_RegBaseItargets11 = ioremap_nocache(DRV_CPURTTKER_ITARGETS_11, 4U);

        if(NULL == g_RegBaseItargets11)
        {
            pr_err("failed to get GICD_ITARGETS11 address \n");
            return -EFAULT;
        }
    }

    if (NULL == g_RegBaseRttfinish1)
    {
        Resource = request_mem_region(DRV_RTTKER_RTTFINISH1, 4U, UDF_CPURTT_DRIVER_NAME);
        if (NULL == Resource)
        {
            pr_err("failed to get RTTFINISH1 resource\n");
            return -ENOSPC;
        }

        g_RegBaseRttfinish1 = ioremap_nocache(DRV_RTTKER_RTTFINISH1, 4U);
        if(NULL == g_RegBaseRttfinish1)
        {
            pr_err("failed to get RTTFINISH1 address \n");
            return -EFAULT;
        }
    }

    if (NULL == g_RegBaseRttfinish2)
    {
        Resource = request_mem_region(DRV_RTTKER_RTTFINISH2, 4U, UDF_CPURTT_DRIVER_NAME);
        if (NULL == Resource)
        {
            pr_err("failed to get RTTFINISH2 resource\n");
            return -ENOSPC;
        }

        g_RegBaseRttfinish2 = ioremap_nocache(DRV_RTTKER_RTTFINISH2, 4U);
        if(NULL == g_RegBaseRttfinish2)
        {
            pr_err("failed to get RTTFINISH2 address \n");
            return -EFAULT;
        }
    }

    for (i = 0; i < (uint32_t) DRV_RTTKER_HIERARCHY_MAX; i++)
    {
        if (NULL == g_RegBaseAddrTable[i])
        {
            Resource = request_mem_region(drvRTT_PhyRegAddr[i], 4U, UDF_CPURTT_DRIVER_NAME);
            if (NULL == Resource)
            {
                pr_err("failed to get drvRTT_PhyRegAddr[%d] resource\n", i);
                return -ENOSPC;
            }

            g_RegBaseAddrTable[i] = ioremap_nocache(drvRTT_PhyRegAddr[i], 4U);
            if(NULL == g_RegBaseAddrTable[i])
            {
                pr_err("failed to get drvRTT_PhyRegAddr[%d] address \n", i);
                return -EFAULT;
            }
        }
    }

    return 0;
}

static void drvCPURTT_DeInitRegAddr(void)
{
    uint32_t i;

    /* rerealse io address */
    iounmap(g_RegBaseSgir);
    release_mem_region(DRV_CPURTTKER_SGIR, 4U);
    g_RegBaseSgir = NULL;

    iounmap(g_RegBaseItargets11);
    release_mem_region(DRV_CPURTTKER_ITARGETS_11, 4U);
    g_RegBaseItargets11 = NULL;

    iounmap(g_RegBaseRttfinish1);
    release_mem_region(DRV_RTTKER_RTTFINISH1, 4U);
    g_RegBaseRttfinish1 = NULL;

    iounmap(g_RegBaseRttfinish2);
    release_mem_region(DRV_RTTKER_RTTFINISH2, 4U);
    g_RegBaseRttfinish2 = NULL;

    for (i = 0; i < (uint32_t)DRV_RTTKER_HIERARCHY_MAX; i++)
    {
        iounmap(g_RegBaseAddrTable[i]);
        release_mem_region(drvRTT_PhyRegAddr[i], 4U);
        g_RegBaseAddrTable[i] = NULL;
    }
}

/* enable FbistInterrupt */
static long drvCPURTT_EnableFbistInterrupt(struct fbc_uio_share_platform_data *priv, unsigned int aAffinity)
{
    long ret;
    cpumask_t CpuMask;
    struct uio_info *uio_info = priv->uio_info;
    unsigned long flags;
    volatile unsigned int reg_read_data;
    unsigned int reg_excepted_data;

    spin_lock_irqsave(&priv->lock, flags);
    if (__test_and_clear_bit(UIO_IRQ_DISABLED, &priv->flags))
    {
        enable_irq((unsigned int)uio_info->irq);
    }
    spin_unlock_irqrestore(&priv->lock, flags);

    cpumask_clear(&CpuMask);
    cpumask_set_cpu(aAffinity, &CpuMask);
    ret = (long)irq_set_affinity_hint((unsigned int)uio_info->irq, &CpuMask);
    if (ret < 0)
    {
        pr_err("cannot set irq affinity %d\n", (unsigned int)uio_info->irq);
        return -EINVAL;
    }

    /* internal bus interface check */
    reg_excepted_data = ((unsigned int)((1U<<aAffinity)<<16U) & DRV_CPURTTKER_ITARGETS_11_MASK);
    reg_read_data = readl(g_RegBaseItargets11);
    if (reg_excepted_data != (reg_read_data & DRV_CPURTTKER_ITARGETS_11_MASK))
    {
        pr_err("gicd_targets11 buscheck error read_data = %08x excepted_dat = %08x\n", reg_read_data, reg_excepted_data);
        return FBIST_BUSCHECK_ERROR;
    }

    return ret;
}

static int drvCPURTT_UDF_A2RuntimeThred0(void *aArg)
{
    uint16_t CpuNum;
    long Ret = 0;
    cpumask_t CpuMask;
    unsigned long BackupIrqMask;

    CpuNum = 0;

    /* set cpu affinity to current thread */
    cpumask_clear(&CpuMask);
    cpumask_set_cpu(CpuNum, &CpuMask);
    Ret = sched_setaffinity(current->pid, &CpuMask);

   if(Ret < 0)
    {
        pr_err("KCPUTHREAD[%d] sched_setaffinity fail %ld\n", CpuNum, Ret);
        complete_and_exit(&g_A2EndSynCompletion[CpuNum], Ret);
    }

    g_A2Task[CpuNum]->rt_priority = 1; /* max priority */
    g_A2Task[CpuNum]->policy = SCHED_RR; /* max policy */

    /* Notify the generator that the thread has started */
    complete(&g_A2ThWakeupCompletion[CpuNum]);
    do
    {
        /* Waiting for start request from user  */
        Ret = wait_for_completion_interruptible(&g_A2StartSynCompletion);
        if (0 != Ret)
        {
            pr_err(" CPU%d kthread wait completion error = %ld\n",CpuNum, Ret);
            g_TaskExitFlg = true;
        }

        if (!g_TaskExitFlg)
        {
            /* On CPU0, interrupt mask and execute A2 Runtime Test  */
            BackupIrqMask = arch_local_irq_save();

            g_A2SmoniResult[CpuNum] = R_SMONI_API_RuntimeTestA2Execute(g_A2Param[CpuNum].Rttex, g_A2Param[CpuNum].Sgi);
            arch_local_irq_restore(BackupIrqMask);

           /* Notify that A2RuntimeTest is complete  */
            complete(&g_A2EndSynCompletion[CpuNum]);
        }
        else
        {
            /* Exit if there is a thread termination request */
            complete_and_exit(&g_A2EndSynCompletion[CpuNum], Ret);
        }

    }while(1);

    return 0;
}

static int drvCPURTT_UDF_A2RuntimeThred1(void *aArg)
{
    uint16_t CpuNum;
    long Ret = 0;
    cpumask_t CpuMask;

    CpuNum = 1;

    cpumask_clear(&CpuMask);
    cpumask_set_cpu(CpuNum, &CpuMask);
    Ret = sched_setaffinity(current->pid, &CpuMask);

    if(Ret < 0)
    {
        pr_err("KCPUTHREAD[%d] sched_setaffinity fail %ld\n", CpuNum, Ret);
        complete_and_exit(&g_A2EndSynCompletion[CpuNum], Ret);
    }

    g_A2Task[CpuNum]->rt_priority = 1; /* max priority */
    g_A2Task[CpuNum]->policy = SCHED_RR; /* max policy */

    /* Notify the generator that the thread has started */
    complete(&g_A2ThWakeupCompletion[CpuNum]);
    do
    {
        /* Waiting for start request from user  */
        Ret = wait_for_completion_interruptible(&g_A2StartSynCompletion);
        if (0 != Ret)
        {
            pr_err(" CPU%d kthread wait completion error = %ld\n",CpuNum, Ret);
            g_TaskExitFlg = true;
        }

        if (!g_TaskExitFlg)
        {
            /* On CPU1, interrupt mask and execute A2 Runtime Test.  */
            /* Dummy data is set in the argument except for CPU0.    */
            g_A2SmoniResult[CpuNum] = R_SMONI_API_RuntimeTestA2Execute(DRV_RTTKER_A2_PARAM_RTTEX_DATA, DRV_RTTKER_A2_PARAM_SGI_DATA);

           /* Notify that A2RuntimeTest is complete  */
            complete(&g_A2EndSynCompletion[CpuNum]);
        }
        else
        {
            /* Exit if there is a thread termination request */
            complete_and_exit(&g_A2EndSynCompletion[CpuNum], Ret);
        }

    }while(1);

    return 0;
}

static int drvCPURTT_UDF_A2RuntimeThred2(void *aArg)
{
    uint16_t CpuNum;
    long Ret = 0;
    cpumask_t CpuMask;

    CpuNum = 2;

    cpumask_clear(&CpuMask);
    cpumask_set_cpu(CpuNum, &CpuMask);
    Ret = sched_setaffinity(current->pid, &CpuMask);

    if(Ret < 0)
    {
        pr_err("KCPUTHREAD[%d] sched_setaffinity fail %ld\n", CpuNum, Ret);
        complete_and_exit(&g_A2EndSynCompletion[CpuNum], Ret);
    }

    g_A2Task[CpuNum]->rt_priority = 1; /* max priority */
    g_A2Task[CpuNum]->policy = SCHED_RR; /* max policy */

    /* Notify the generator that the thread has started */
    complete(&g_A2ThWakeupCompletion[CpuNum]);
    do
    {
        /* Waiting for start request from user  */
        Ret = wait_for_completion_interruptible(&g_A2StartSynCompletion);
        if (0 != Ret)
        {
            pr_err(" CPU%d kthread wait completion error = %ld\n",CpuNum, Ret);
            g_TaskExitFlg = true;
        }

        if (!g_TaskExitFlg)
        {
            /* On CPU2, interrupt mask and execute A2 Runtime Test.  */
            /* Dummy data is set in the argument except for CPU0.    */
            g_A2SmoniResult[CpuNum] = R_SMONI_API_RuntimeTestA2Execute(DRV_RTTKER_A2_PARAM_RTTEX_DATA, DRV_RTTKER_A2_PARAM_SGI_DATA);

           /* Notify that A2RuntimeTest is complete  */
            complete(&g_A2EndSynCompletion[CpuNum]);
        }
        else
        {
            /* Exit if there is a thread termination request */
            complete_and_exit(&g_A2EndSynCompletion[CpuNum], Ret);
        }

    }while(1);

    return 0;
}

static int drvCPURTT_UDF_A2RuntimeThred3(void *aArg)
{
    uint16_t CpuNum;
    long Ret = 0;
    cpumask_t CpuMask;

    CpuNum = 3;

    cpumask_clear(&CpuMask);
    cpumask_set_cpu(CpuNum, &CpuMask);
    Ret = sched_setaffinity(current->pid, &CpuMask);

    if(Ret < 0)
    {
        pr_err("KCPUTHREAD[%d] sched_setaffinity fail %ld\n", CpuNum, Ret);
        complete_and_exit(&g_A2EndSynCompletion[CpuNum], Ret);
    }

    g_A2Task[CpuNum]->rt_priority = 1; /* max priority */
    g_A2Task[CpuNum]->policy = SCHED_RR; /* max policy */

    /* Notify the generator that the thread has started */
    complete(&g_A2ThWakeupCompletion[CpuNum]);

    do
    {
        /* Waiting for start request from user  */
        Ret = wait_for_completion_interruptible(&g_A2StartSynCompletion);
        if (0 != Ret)
        {
            pr_err(" CPU%d kthread wait completion error = %ld\n",CpuNum, Ret);
            g_TaskExitFlg = true;
        }

        if (!g_TaskExitFlg)
        {
            /* On CPU3, interrupt mask and execute A2 Runtime Test.  */
            /* Dummy data is set in the argument except for CPU0.    */
            g_A2SmoniResult[CpuNum] = R_SMONI_API_RuntimeTestA2Execute(DRV_RTTKER_A2_PARAM_RTTEX_DATA, DRV_RTTKER_A2_PARAM_SGI_DATA);

           /* Notify that A2RuntimeTest is complete  */
            complete(&g_A2EndSynCompletion[CpuNum]);
        }
        else
        {
            /* Exit if there is a thread termination request */
            complete_and_exit(&g_A2EndSynCompletion[CpuNum], Ret);
        }

    }while(1);

    return 0;
}

/* Perform the preparatory work required to execute cpurtt with this function.  */
static long drvCPURTT_UDF_RuntimeTestInit(void)
{
    long ret = 0;
    uint32_t CpuIndex;
    const A2ThreadTable A2ThreadTable[DRV_CPURTTKER_CPUNUM_MAX] = {
                                      drvCPURTT_UDF_A2RuntimeThred0,
                                      drvCPURTT_UDF_A2RuntimeThred1,
                                      drvCPURTT_UDF_A2RuntimeThred2,
                                      drvCPURTT_UDF_A2RuntimeThred3};
    g_TaskExitFlg = 0;
      /* initalize for A2 RuntimeTest*/
    for(CpuIndex=0; CpuIndex<DRV_CPURTTKER_CPUNUM_MAX; CpuIndex++)
    {
        if(g_A2Task[CpuIndex] == NULL)
        {
            init_completion(&g_A2EndSynCompletion[CpuIndex]);
            init_completion(&g_A2ThWakeupCompletion[CpuIndex]);

            g_A2Task[CpuIndex] = kthread_run(A2ThreadTable[CpuIndex], &CpuIndex, "cpurtt_A2rtt_%d", CpuIndex);
            if (IS_ERR(g_A2Task[CpuIndex]))
            {  
                pr_err("A2_rtt_thread_run[%d] failed", CpuIndex);
                return -EINVAL;
            }

            ret = wait_for_completion_interruptible(&g_A2ThWakeupCompletion[CpuIndex]);
            if (0 != ret)
            {
                pr_err("CPU%d kthread creat err = %ld\n",CpuIndex, ret);
                break;
            }
        }
    }

    return ret;
}

/* This function releases the resources prepared by drvCPURTT_UDF_RuntimeTestInit.  */
static long drvCPURTT_UDF_RuntimeTestDeinit(void)
{
    long ret = 0;

    return ret;
}


/* This function executes the smoni api specified by index. */
static long drvCPURTT_UDF_SmoniApiExe(drvCPURTT_SmoniTable_t index, uint32_t aCpuId, uint32_t *aArg, uint32_t *aSmoniret)
{
    long      ret = -1;
    cpumask_t CpuMask;
    unsigned long InterruptFlag;
    drvCPURTT_A1rttParam_t SmoniArgA1;
    drvCPURTT_A2rttParam_t SmoniArgA2;
    drvCPURTT_FbaWriteParam_t SmoniArgFwrite;
    drvCPURTT_FbaReadParam_t SmoniArgFread;
    drvCPURTT_ConfigRegCheckParam_t SmoniArgCfg;
    drvCPURTT_SetTimeoutParam_t SmoniArgTimeout;
    drvCPURTT_SelfCheckParam_t SmoniArgSelf;
    struct platform_device *pdev = g_cpurtt_pdev;
    struct fbc_uio_share_platform_data *priv = platform_get_drvdata(pdev);
    unsigned int IrqNum = (unsigned int)priv->uio_info->irq;
    unsigned int reg_read_data;
    unsigned int reg_excepted_data;

    if (!(0xFFFFFFF0 & aCpuId))
    {
        cpumask_clear(&CpuMask); 
        cpumask_set_cpu(aCpuId, &CpuMask); 
        ret = sched_setaffinity(current->pid, &CpuMask);
        if(ret < 0)
        {
            pr_err("sched_setaffinity fail %ld\n", ret);
            return ret;
        }
    }
    else
    {
        pr_err("affinity cpu number = %x fail %ld\n", aCpuId, ret);
        return ret;
    }

    switch (index)
    {
        case DRV_CPURTT_SMONIAPI_SETTIMEOUT:

            /* Copy smoni api arguments to kernel memory. */
            ret = copy_from_user(&SmoniArgTimeout, (const void __user *)(aArg), sizeof(drvCPURTT_SetTimeoutParam_t));
            if (ret == 0)
            {
                *aSmoniret = R_SMONI_API_SetTimeout(SmoniArgTimeout.Target, SmoniArgTimeout.MicroSec);
            }
            break;

        case DRV_CPURTT_SMONIAPI_CFGREGCHECK:

            /* Copy smoni api arguments to kernel memory. */
            ret = copy_from_user(&SmoniArgCfg, (const void __user *)(aArg), sizeof(SmoniArgCfg));
            if (ret == 0)
            {
                *aSmoniret = R_SMONI_API_ConfigurationRegisterCheck(SmoniArgCfg.Setting, SmoniArgCfg.TargetReg);
            }
            break;

        case DRV_CPURTT_SMONIAPI_LOCKACQUIRE:

            *aSmoniret = R_SMONI_API_RuntimeTestLockAcquire();
            ret = 0;

            break;

        case DRV_CPURTT_SMONIAPI_LOCKRELEASE:

            *aSmoniret = R_SMONI_API_RuntimeTestLockRelease();
            ret = 0;

            break;

        case DRV_CPURTT_SMONIAPI_A1EXE:

            /* Copy smoni api arguments to kernel memory. */
            ret = copy_from_user(&SmoniArgA1, (const void __user *)(aArg), sizeof(drvCPURTT_A1rttParam_t));

            if (ret == 0U)
            {
                if (aCpuId == 0)
                {
                    reg_excepted_data = DRV_CPURTTKER_ITARGETS_11_CPU1;
                    ret = (long)irq_set_affinity_hint(IrqNum, cpumask_of(1U));
                }
                else
                {
                    reg_excepted_data = DRV_CPURTTKER_ITARGETS_11_CPU0;
                    ret = (long)irq_set_affinity_hint(IrqNum, cpumask_of(0U));
                }
                if (ret != 0)
                {
                    pr_err( "irq_set_affinity_hint failed %ld \n",ret);
                    break;
                }

                 /* internal bus interface check */
                 reg_read_data = readl(g_RegBaseItargets11);
                 if (reg_excepted_data != (reg_read_data & DRV_CPURTTKER_ITARGETS_11_MASK))
                 {
                     pr_err("gicd_targets11 buscheck error read_data = %08x excepted_data = %08x\n", reg_read_data, reg_excepted_data);
                     ret = FBIST_BUSCHECK_ERROR;
                     break;
                 }

                InterruptFlag = arch_local_irq_save();
                *aSmoniret = R_SMONI_API_RuntimeTestA1Execute(SmoniArgA1.Rttex);
                arch_local_irq_restore(InterruptFlag);

            }
            break;

        case DRV_CPURTT_SMONIAPI_A2EXE:
            ret = copy_from_user(&SmoniArgA2, (const void __user *)(aArg), sizeof(drvCPURTT_A2rttParam_t));
            if (ret == 0U)
            {
                if (aCpuId == 0)
                {
                    g_A2Param[aCpuId].Rttex = SmoniArgA2.Rttex;
                    g_A2Param[aCpuId].Sgi = SmoniArgA2.Sgi;

                    ret = (long)irq_set_affinity_hint(IrqNum, cpumask_of(0U));
                    if (ret != 0)
                    {
                        pr_err( "irq_set_affinity_hint failed %ld \n", ret);
                        break;
                    }

                     /* internal bus interface check */
                     reg_excepted_data = DRV_CPURTTKER_ITARGETS_11_CPU0;
                     reg_read_data = readl(g_RegBaseItargets11);
                     if (reg_excepted_data != (reg_read_data & DRV_CPURTTKER_ITARGETS_11_MASK))
                     {
                         pr_err("gicd_targets11 buscheck error read_data = %08x excepted_data = %08x\n", reg_read_data, reg_excepted_data);
                         ret = FBIST_BUSCHECK_ERROR;
                         break;
                     }

                    /* A2 Runtime Test Execution thread start request  */
                    complete(&g_A2StartSynCompletion);
                    complete(&g_A2StartSynCompletion);
                    complete(&g_A2StartSynCompletion);
                    complete(&g_A2StartSynCompletion);

                    /* Wait until A2 Runtime Test completion notification is received from the A2 Runtime Test execution thread all CPUs  */
                    ret = wait_for_completion_interruptible(&g_A2EndSynCompletion[0U]);
                    ret |= wait_for_completion_interruptible(&g_A2EndSynCompletion[1U]);
                    ret |= wait_for_completion_interruptible(&g_A2EndSynCompletion[2U]);
                    ret |= wait_for_completion_interruptible(&g_A2EndSynCompletion[3U]);
                    if (ret != 0)
                    {
                        pr_err(" CPU%d wait completion error = %ld\n",aCpuId, ret);
                        break;
                    }

                    *aSmoniret = g_A2SmoniResult[aCpuId];

                }
                else
                {
                    ret = -EINVAL;
                }
           }
           break;

        case DRV_CPURTT_SMONIAPI_FBAWRITE:

            /* Copy smoni api arguments to kernel memory. */
            ret = copy_from_user(&SmoniArgFwrite, (const void __user *)(aArg), sizeof(drvCPURTT_FbaWriteParam_t));
            if (ret == 0U)
            {
                /* Copy the access destination address data to kernel memory.  */
                ret = copy_from_user(&g_SmoniAddrBuf[aCpuId][0], (const void __user *)SmoniArgFwrite.AddrBuf, (SmoniArgFwrite.RegCount * sizeof(uint32_t)));
                if (ret == 0U)
                {
                    /* Copy the data to write to kernel memory. */
                    ret = copy_from_user(&g_SmoniDataBuf[aCpuId][0], (const void __user *)SmoniArgFwrite.DataBuf, (SmoniArgFwrite.RegCount * sizeof(uint32_t)));
                    if (ret == 0U)
                    {
                        *aSmoniret = R_SMONI_API_RuntimeTestFbaWrite(&g_SmoniAddrBuf[aCpuId][0], &g_SmoniDataBuf[aCpuId][0], SmoniArgFwrite.RegCount);
                    }
                }
            }
            break;

        case DRV_CPURTT_SMONIAPI_FBAREAD:

            /* Copy smoni api arguments to kernel memory. */
            ret = copy_from_user(&SmoniArgFread, (const void __user *)(aArg), sizeof(drvCPURTT_FbaReadParam_t));
            if (ret == 0U)
            {
                /* Copy the access destination address data to kernel memory.  */
                ret = copy_from_user(&g_SmoniAddrBuf[aCpuId][0], (const void __user *)SmoniArgFread.AddrBuf, (SmoniArgFread.RegCount * sizeof(uint32_t)));
                if (ret == 0U)
                {
                    *aSmoniret = R_SMONI_API_RuntimeTestFbaRead(&g_SmoniAddrBuf[aCpuId][0], &g_SmoniDataBuf[aCpuId][0], SmoniArgFread.RegCount);
                    if (SMONI_FUSA_OK == *aSmoniret)
                    {
                        /* Copy the read data to user memory. */
                        ret = copy_to_user((void __user *)SmoniArgFread.DataBuf, (const void *)&g_SmoniDataBuf[aCpuId][0], (SmoniArgFread.RegCount * sizeof(uint32_t)));
                    }
                }
            }
            break;

        case DRV_CPURTT_SMONIAPI_SELFCHECK:

            /* Copy smoni api arguments to kernel memory. */
            ret = copy_from_user(&SmoniArgSelf, (const void __user *)(aArg), sizeof(drvCPURTT_SelfCheckParam_t));
            if (ret == 0U)
            {
                *aSmoniret = R_SMONI_API_SelfCheckExecute(SmoniArgSelf.Rttex, SmoniArgSelf.TargetHierarchy);
            }
            break;

        default:
            ret = -EINVAL;
            break;
    }

    return ret;
}

static long drvCPURTT_UDF_FbistInit(void)
{
    long ret = -1;
    struct platform_device *pdev = g_cpurtt_pdev;
    struct fbc_uio_share_platform_data *priv = platform_get_drvdata(pdev);

    ret = pm_runtime_get_sync(&priv->pdev->dev);
    ret = fbc_uio_share_clk_enable(priv);
    if (ret != 0)
    {
        pm_runtime_put_sync(&priv->pdev->dev);
        return ret;
    }

    /* get resource for fba and fbc regster address */
    ret = drvCPURTT_InitRegAddr();
    if (ret < 0)
    {
        pm_runtime_put_sync(&priv->pdev->dev);
        return ret;
    }

    /* enabeled fbist finish interrupt with set interrupt affinity */
    ret = drvCPURTT_EnableFbistInterrupt(priv, DRV_RTTKER_FIELD_BIST_INT_CPU);
    if (ret != 0)
    {
        pm_runtime_put_sync(&priv->pdev->dev);
        return ret;
    }

    /* iInitialization of parameters related to callback execution request  */
    FbistCloseReq = false;
    g_CallbackInfo.head = 0;
    g_CallbackInfo.pos = 0;
    g_CallbackInfo.status = CB_QUEUE_STATUS_EMPTY;

    pm_runtime_put_sync(&priv->pdev->dev);

    return ret;
}

static long drvCPURTT_UDF_FbistDeInit(void)
{
    long ret = 0;
    struct platform_device *pdev = g_cpurtt_pdev;
    struct fbc_uio_share_platform_data *priv = platform_get_drvdata(pdev);
    struct uio_info *uio_info = priv->uio_info;

    fbc_uio_share_clk_disable(priv);
    pm_runtime_put_sync(&priv->pdev->dev);

    spin_lock(&priv->lock);
    if (!__test_and_set_bit(UIO_IRQ_DISABLED, &priv->flags))
    {
        irq_set_affinity_hint((unsigned int)uio_info->irq, NULL);
        disable_irq((unsigned int)uio_info->irq);
    }
    spin_unlock(&priv->lock);

    drvCPURTT_DeInitRegAddr();

    /* Release of semaphore waiting for callback notification  */
    FbistCloseReq = true;
    up(&CallbackSem);

    return ret;

}

static long drvCPURTT_UDF_WaitCbNotice(drvCPURTT_CallbackInfo_t *aParam)
{
    long      ret = 0;
    struct platform_device *pdev = g_cpurtt_pdev;
    struct fbc_uio_share_platform_data *priv = platform_get_drvdata(pdev);
    struct uio_info *uio_info = priv->uio_info;
    unsigned long flags;
    int      intret = 0;

    intret = down_interruptible(&CallbackSem); /* wait for semaphore release */

    if(!intret)
    {
        spin_lock(&priv->lock);
        if (!__test_and_set_bit(UIO_IRQ_DISABLED, &priv->flags))
        {
            disable_irq((unsigned int)uio_info->irq);
        }
        spin_unlock(&priv->lock);
        if (FbistCloseReq == false)
        {
            if (g_CallbackInfo.pos > 0)
            {
                aParam->FbistCbRequest = g_CallbackInfo.CbInfo[g_CallbackInfo.head].FbistCbRequest;
                aParam->BusCheckCbRequest = g_CallbackInfo.CbInfo[g_CallbackInfo.head].BusCheckCbRequest;
                aParam->RfsoOutputPinRequest = g_CallbackInfo.CbInfo[g_CallbackInfo.head].RfsoOutputPinRequest;
                g_CallbackInfo.head = (uint8_t)((g_CallbackInfo.head + 1U) % DRV_RTTKER_HIERARCHY_MAX);
                g_CallbackInfo.pos--;
                g_CallbackInfo.status = CB_QUEUE_STATUS_ENA;
            }
            else
            {
                g_CallbackInfo.status = CB_QUEUE_STATUS_EMPTY;
            }
        }
        else
        {
            g_CallbackInfo.status = CB_QUEUE_STATUS_EMPTY;
            ret = FBIST_CB_CLOSE_REQ;
        }

        spin_lock_irqsave(&priv->lock, flags);
        if (__test_and_clear_bit(UIO_IRQ_DISABLED, &priv->flags))
        {
            enable_irq((unsigned int)uio_info->irq);
        }
        spin_unlock_irqrestore(&priv->lock, flags);
    }
    else
    {
        pr_err("wait callback err = %d\n",intret);
        ret = (long)intret;
    }

    return ret;
}

static long drvCPURTT_UDF_HWAExecute(uint32_t aHierarchy, uint32_t aRttex)
{
    long      ret = 0;
    struct platform_device *pdev = g_cpurtt_pdev;
    struct fbc_uio_share_platform_data *priv = platform_get_drvdata(pdev);
    struct uio_info *uio_info = priv->uio_info;
    unsigned long flags;
    unsigned int reg_read_data;

    /* check hierarhcy */
    if ( ( (aHierarchy>=(uint32_t)DRV_RTTKER_HIERARCHY_IMR0) && (aHierarchy<=(uint32_t)DRV_RTTKER_HIERARCHY_SLIM) ) ||
         ( (aHierarchy>=(uint32_t)DRV_RTTKER_HIERARCHY_DISP) && (aHierarchy<(uint32_t)DRV_RTTKER_HIERARCHY_MAX ) )  )
    {
        /* Set interrupt disabled so that the field BIST finish interrupt does not occur immediately after writing the RTTEX register.  */
        /* Check in advance if there is a process that disables interrupts before setting. */
        spin_lock(&priv->lock);
        if (!__test_and_set_bit(UIO_IRQ_DISABLED, &priv->flags))
        {
            disable_irq((unsigned int)uio_info->irq);
        }
        spin_unlock(&priv->lock);

        /* write RTTEX register for execute runtime test */
        writel(aRttex, g_RegBaseAddrTable[aHierarchy]);
        /* bus check RTTEX register */
        reg_read_data = readl(g_RegBaseAddrTable[aHierarchy]);
        if (aRttex != reg_read_data)
        {
            /* If the write result and the read result do not match, a check error is returned. */
            pr_err("rttex(%d) buscheck error read_data = %08x excepted_date = %08x\n", aHierarchy, reg_read_data, aRttex);
            ret = FBIST_BUSCHECK_ERROR;
        }
        else
        {
            ret = 0;
        }

        /* If there is no other process that disables interrupts, set interrupt enable. */
        spin_lock_irqsave(&priv->lock, flags);
        if (__test_and_clear_bit(UIO_IRQ_DISABLED, &priv->flags))
        {
            enable_irq((unsigned int)uio_info->irq);
        }
        spin_unlock_irqrestore(&priv->lock, flags);
    }
    else
    {
        ret = -EINVAL;
        pr_err("hierarchy number =%d fail\n", aHierarchy);
    }

    return ret;
}

/* This function executes the purttmod open system call.  */
static int CpurttDrv_open(struct inode *inode, struct file *file)
{
    return 0;
}

/* This function executes the purttmod close system call.  */
static int CpurttDrv_close(struct inode *inode, struct file *file)
{
    return 0;
}

/* This function executes the cpurttmod ioctl system call.  */
static long CpurttDrv_ioctl( struct file* filp, unsigned int cmd, unsigned long arg )
{
    long ret;
    drvCPURTT_CallbackInfo_t CbInfo;
    drvCPURTT_SmoniParam_t smoni_param;
    drvCPURTT_HwaParam_t hwa_param;

    /* Executes the process corresponding to the command specified in the argument "cmd" passed by ioctl execution from the user layer. */
    switch (cmd) {
        case DRV_CPURTT_IOCTL_DEVINIT:
            ret = drvCPURTT_UDF_RuntimeTestInit();
            break;

        case DRV_CPURTT_IOCTL_DEVDEINIT:
            ret = drvCPURTT_UDF_RuntimeTestDeinit();
            break;

        case DRV_CPURTT_IOCTL_SMONI:
            /* Copy smoni api arguments to kernel memory. */            
            ret = copy_from_user(&smoni_param, (void __user *)arg, sizeof(drvCPURTT_SmoniParam_t));
            if (ret != 0) {
                ret = -EFAULT;
            }
            
            if (ret == 0)
            {
                ret = drvCPURTT_UDF_SmoniApiExe((drvCPURTT_SmoniTable_t)smoni_param.Index, smoni_param.CpuId, smoni_param.Arg, &smoni_param.RetArg);
            }

            if (ret == 0)
            {
                /* Copy the execution result of smoni api to user memory.  */
                if (copy_to_user((void __user *)arg, &smoni_param, sizeof(drvCPURTT_SmoniParam_t))) {
                    ret = -EFAULT;
                }
            }
            break;

        case DRV_CPURTT_IOCTL_DEVFBISTINIT:

            ret = drvCPURTT_UDF_FbistInit();
            break;

        case DRV_CPURTT_IOCTL_DEVFBISTDEINIT:

            ret = drvCPURTT_UDF_FbistDeInit();
            break;

        case DRV_CPURTT_IOCTL_WAIT_CALLBACK:

            ret = drvCPURTT_UDF_WaitCbNotice(&CbInfo);
            if (ret == 0)
            {
               if((DRV_CPURTT_CB_REQ_NON != CbInfo.FbistCbRequest) || (DRV_CPURTT_CB_REQ_NON != CbInfo.BusCheckCbRequest) || (DRV_CPURTT_CB_REQ_NON != CbInfo.RfsoOutputPinRequest))
               {
                   /* Copy the execution result of smoni api to user memory.  */
                   if (copy_to_user((void __user *)arg, &CbInfo, sizeof(drvCPURTT_CallbackInfo_t))) {
                       ret = -EFAULT;
                   }
               }
               else
               {
                   ret = -EFAULT;
               }
            }
            break;

        case DRV_CPURTT_IOCTL_HWAEXE:
            /* Copy hwa execute arguments to kernel memory. */
            ret = copy_from_user(&hwa_param, (void __user *)arg, sizeof(drvCPURTT_HwaParam_t));
            if (ret != 0) {
               ret = -EFAULT;
            }

            if (ret == 0)
            {
                ret = drvCPURTT_UDF_HWAExecute(hwa_param.Hierarchy, hwa_param.Rttex);
            }
            break;

        default:
            ret = -EFAULT;
            break;
    }

    return ret;
}

static struct file_operations s_CpurttDrv_fops = {
    .open    = CpurttDrv_open,
    .release = CpurttDrv_close,
    .unlocked_ioctl = CpurttDrv_ioctl,
    .compat_ioctl   = CpurttDrv_ioctl,    /* for 32 bit system */
};

/* This function is executed when cpurttmod is loaded, and cpurttmod initialization processing is performed. */
static int CpurttDrv_init(void)
{
    int ret = 0;
    struct device *dev;
    unsigned int cnt;

    /* register character device for cpurttdrv */
    ret = register_chrdev(cpurtt_major, UDF_CPURTT_DRIVER_NAME, &s_CpurttDrv_fops);
    if (ret < 0)
    {
        pr_err( "register_chrdev = %d\n", cpurtt_major);
        return -EFAULT;
    }
    else
    {
        cpurtt_major = (unsigned int)ret;
        ret = 0;
    }

    /* create class module */
    cpurtt_class = class_create(THIS_MODULE, UDF_CPURTT_CLASS_NAME);
    if (IS_ERR(cpurtt_class))
    {
        pr_err("unable to create cpurttmod class\n");
        ret = PTR_ERR(cpurtt_class);
        unregister_chrdev(cpurtt_major, UDF_CPURTT_DRIVER_NAME);
        pr_err( "class_create = %d\n", ret);
        return ret;
    }

    /* create device file for cpurttdrv */
    dev = device_create(cpurtt_class, NULL, MKDEV(cpurtt_major, 0), NULL, "%s%d", UDF_CPURTT_CLASS_NAME, MINOR(MKDEV(cpurtt_major, 0)));
    if (IS_ERR(dev))
    {
        device_destroy(cpurtt_class, MKDEV(cpurtt_major, 0));
        class_destroy(cpurtt_class);
        unregister_chrdev(cpurtt_major, UDF_CPURTT_DRIVER_NAME);
        pr_err("cpurttmod: unable to create device cpurttmod%d\n", MINOR(MKDEV(cpurtt_major, 0)));
        ret = PTR_ERR(dev);
        return ret;
    }

    platform_driver_register(&fbc_uio_share_driver);
    if (g_cpurtt_pdev == NULL) {
        device_destroy(cpurtt_class, MKDEV(cpurtt_major, 0));
        class_destroy(cpurtt_class);
        unregister_chrdev(cpurtt_major, UDF_CPURTT_DRIVER_NAME);
        platform_driver_unregister(&fbc_uio_share_driver);
        pr_err("failed to cpurtt_driver_register\n");
        return -EINVAL;
    }

    init_completion(&g_A2StartSynCompletion);
    sema_init(&CallbackSem, 0);

    for(cnt=0;cnt<DRV_CPURTTKER_CPUNUM_MAX;cnt++)
    {
        g_A2Task[cnt] = NULL;
    }

    return ret;
}

/* This function is executed when cpurttmod is unloaded and frees cpurttmod resources.  */
static void CpurttDrv_exit(void)
{
    unsigned int cnt;

    /* release cpurtt thread for A2 Runtime Test*/
    g_TaskExitFlg = 1;
    complete_all(&g_A2StartSynCompletion);
    for(cnt=0;cnt<DRV_CPURTTKER_CPUNUM_MAX;cnt++)
    {
        if(g_A2Task[cnt]!=NULL)
        {
            (void)wait_for_completion_interruptible(&g_A2EndSynCompletion[cnt]);
            g_A2Task[cnt] = NULL;
        }
    }

    /* unload fbc uio_chare driver */
    platform_driver_unregister(&fbc_uio_share_driver);

    /* release device clase */
    device_destroy(cpurtt_class, MKDEV(cpurtt_major, 0));
    class_destroy(cpurtt_class);

    /* unload character device */
    unregister_chrdev(cpurtt_major, "UDF_CPURTT_DRIVER_NAME");
}

module_init(CpurttDrv_init);
module_exit(CpurttDrv_exit);

MODULE_AUTHOR("RenesasElectronicsCorp.");
MODULE_DESCRIPTION("CPURTT drive");
MODULE_LICENSE("GPL v2");
