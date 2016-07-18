/*
 * A driver for the ARM PL022 PrimeCell SSP/SPI bus master.
 *
 * Copyright (C) 2008-2009 ST-Ericsson AB
 * Copyright (C) 2006 STMicroelectronics Pvt. Ltd.
 *
 * Author: Linus Walleij <linus.walleij@stericsson.com>
 *
 * Initial version inspired by:
 *	linux-2.6.17-rc3-mm1/drivers/spi/pxa2xx_spi.c
 * Initial adoption to PL022 by:
 *      Sachin Verma <sachin.verma@st.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/amba/bus.h>
#include <linux/amba/pl022.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/pm_runtime.h>
#include <mach/power-gate.h>
#include <mach/pad.h>

/*
********** Caution:
********** SSP is arm core: pl022
********** SPI is infotm self ip core:1/2/4 wire
**********/

/*
 * This macro is used to define some register default values.
 * reg is masked with mask, the OR:ed with an (again masked)
 * val shifted sb steps to the left.
 */
#define SSP_WRITE_BITS(reg, val, mask, sb) \
 ((reg) = (((reg) & ~(mask)) | (((val)<<(sb)) & (mask))))

/*
 * This macro is also used to define some default values.
 * It will just shift val by sb steps to the left and mask
 * the result with mask.
 */
#define GEN_MASK_BITS(val, mask, sb) \
 (((val)<<(sb)) & (mask))

#define DRIVE_TX		0
#define DO_NOT_DRIVE_TX		1

#define DO_NOT_QUEUE_DMA	0
#define QUEUE_DMA		1

#define RX_TRANSFER		1
#define TX_TRANSFER		2

/*
 * Macros to access SSP Registers with their offsets
 */
#define SSP_CR0(r)	(r + 0x000)
#define SSP_CR1(r)	(r + 0x004)
#define SSP_DR(r)	(r + 0x008)
#define SSP_CPSR(r)	(r + 0x010)
#define SSP_IMSC(r)	(r + 0x014)
#define SSP_RIS(r)	(r + 0x018)
#define SSP_MIS(r)	(r + 0x01C)
#define SSP_ICR(r)	(r + 0x020)
#define SSP_DMACR(r)	(r + 0x024)
#define SSP_ITCR(r)	(r + 0x080)
#define SSP_ITIP(r)	(r + 0x084)

//====================imapxQ3F
#define SPI_EN(r)   (r + 0x000)
#define SPI_CTL(r)  (r + 0x004)
#define SPI_CFDF(r) (r + 0x008)
#define SPI_IMSC(r) (r + 0x00C)
#define SPI_RIS(r)	(r + 0x010)
#define SPI_MIS(r)  (r + 0x014)
#define SPI_ICR(r)  (r + 0x018)
#define SPI_DMACR(r) (r+ 0x01C)
#define SPI_FTHD(r)	 (r+ 0x020)
#define SPI_SR(r)    (r+ 0x024)
#define SPI_TTC(r)   (r+ 0x028)
#define SPI_TFDAC(r) (r+ 0x02C)
#define SPI_RFDAC(r) (r+ 0x030)
#define SPI_DR0(r)   (r+ 0x100)
#define SPI_DCTL0(r) (r+ 0x104)
#define SPI_DR1(r)   (r+ 0x110)
#define SPI_DCTL1(r) (r+ 0x114)
#define SPI_DR2(r)   (r+ 0x120)
#define SPI_DCTL2(r) (r+ 0x124)
#define SPI_DR3(r)   (r+ 0x130)
#define SPI_DCTL3(r) (r+ 0x134)
#define SPI_DR4(r)   (r+ 0x140)
#define SPI_DCTL4(r) (r+ 0x144)
#define SPI_FDR(r)   (r+ 0x150)
#define SPI_FDRCTL(r) (r+ 0x154)

#define SPI_MASK_EN		(0x1UL << 0)

/*
 * SSP Control Register 0  - SSP_CR0
 */
#define SPI_CTL_MASK_SPO	(0x1UL << 0)
#define SPI_CTL_MASK_SPH	(0x1UL << 1)
#define SPI_CFDF_MASK_DIV	(0xFFFFUL << 0)

/*
 * The ST version of this block moves som bits
 * in SSP_CR0 and extends it to 32 bits
 */
#define SSP_CR0_MASK_DSS_ST	(0x1FUL << 0)
#define SSP_CR0_MASK_HALFDUP_ST	(0x1UL << 5)
#define SSP_CR0_MASK_FRF_ST	(0x3UL << 21)

/*
 * SSP Control Register 0  - SSP_CR1
 */
#define SPI_CR1_MASK_SSE	(0x1UL << 1)

/*
 * SSP Status Register - SPI_SR
 */
#define SPI_SR_MASK_TNF		(0x1UL << 0) /* Transmit FIFO full*/
#define SPI_SR_MASK_TFE		(0x1UL << 1) /* Transmit FIFO empty */
#define SPI_SR_MASK_RFF		(0x1UL << 2) /* Receive FIFO is full */
#define SPI_SR_MASK_RNE		(0x1UL << 3) /* Receive FIFO is empty*/
#define SPI_SR_MASK_BSY		(0x1UL << 4) /* Busy Flag */

/*
 * SSP Clock Prescale Register  - SSP_CPSR
 */
#define SSP_CPSR_MASK_CPSDVSR	(0xFFUL << 0)

/*
 * SSP Interrupt Mask Set/Clear Register - SSP_IMSC
 */
#define SPI_IMSC_MASK_RORIM (~(0x1UL << 4)) /* Receive Overrun Interrupt mask */
#define SPI_IMSC_MASK_RTIM  (~(0x1UL << 3)) /* Receive timeout Interrupt mask */
#define SPI_IMSC_MASK_RXIM  (~(0x1UL << 2)) /* Receive FIFO Interrupt mask */
#define SPI_IMSC_MASK_TTIM  (~(0x1UL << 1)) /* Transmit timeout Interrupt mask */
#define SPI_IMSC_MASK_TXIM  (~(0x1UL << 0)) /* Transmit FIFO Interrupt mask */

/*
 * SSP Raw Interrupt Status Register - SSP_RIS
 */
/* Receive Overrun Raw Interrupt status */
#define SSP_RIS_MASK_RORRIS		(0x1UL << 4)
/* Receive Timeout Raw Interrupt status */
#define SSP_RIS_MASK_RTRIS		(0x1UL << 3)
/* Receive FIFO Raw Interrupt status */
#define SSP_RIS_MASK_RXRIS		(0x1UL << 2)
/* Transmit FIFO Raw Interrupt status */
#define SSP_RIS_MASK_TXRIS		(0x1UL << 0)

/*
 * SSP Masked Interrupt Status Register - SSP_MIS
 */
/* Receive Overrun Masked Interrupt status */
#define SPI_MIS_MASK_RORMIS		(0x1UL << 4)
/* Receive Timeout Masked Interrupt status */
#define SPI_MIS_MASK_RTMIS		(0x1UL << 3)
/* Receive FIFO Masked Interrupt status */
#define SPI_MIS_MASK_RXMIS		(0x1UL << 2)
/* Transmit Timeout Masked Interrupt status*/
#define SPI_MIS_MASK_TTMIS		(0x1UL << 1)
/* Transmit FIFO Masked Interrupt status */
#define SSP_MIS_MASK_TXMIS		(0x1UL << 0)

/*
 * SSP Interrupt Clear Register - SSP_ICR
 */
 /*Transmit Timeout Clear Interrupt bit*/
#define SSP_ICR_MASK_TTIC		(0x1UL << 0)
/* Receive Overrun Raw Clear Interrupt bit */
#define SSP_ICR_MASK_RORIC		(0x1UL << 2)
/* Receive Timeout Clear Interrupt bit */
#define SSP_ICR_MASK_RTIC		(0x1UL << 1)

/*
 * SSP DMA Control Register - SSP_DMACR
 */
/* Receive DMA Enable bit */
#define SPI_DMACR_MASK_RXDMAE		(0x1UL << 1)
/* Transmit DMA Enable bit */
#define SPI_DMACR_MASK_TXDMAE		(0x1UL << 0)



/*
 * Message State
 * we use the spi_message.state (void *) pointer to
 * hold a single state value, that's why all this
 * (void *) casting is done here.
 */
#define STATE_START			((void *) 0)
#define STATE_RUNNING			((void *) 1)
#define STATE_DONE			((void *) 2)
#define STATE_ERROR			((void *) -1)

/*
 * SSP State - Whether Enabled or Disabled
 */
#define SSP_DISABLED			(0)
#define SSP_ENABLED			(1)

/*
 * SSP DMA State - Whether DMA Enabled or Disabled
 */
#define SSP_DMA_DISABLED		(0)
#define SSP_DMA_ENABLED			(1)

/*
 * SSP Clock Defaults
 */
#define SSP_DEFAULT_CLKRATE 0x2
#define SSP_DEFAULT_PRESCALE 0x40

/*
 * SSP Clock Parameter ranges
 */
#define CPSDVR_MIN 0x02
#define CPSDVR_MAX 0xFE
#define SCR_MIN 0x00
#define SCR_MAX 0xFF

/*
 * SSP Interrupt related Macros
 */
#define DEFAULT_SPI_REG_IMSC  0x0UL
#define DISABLE_ALL_INTERRUPTS (DEFAULT_SPI_REG_IMSC)
#define ENABLE_ALL_INTERRUPTS (~DEFAULT_SPI_REG_IMSC)

#define CLEAR_ALL_INTERRUPTS  0x7

#define SPI_POLLING_TIMEOUT 100000

/*
 * The type of reading going on on this chip
 */
enum ssp_reading {
	READING_NULL,
	READING_U8,
	READING_U16,
	READING_U32
};

/**
 * The type of writing going on on this chip
 */
enum ssp_writing {
	WRITING_NULL,
	WRITING_U8,
	WRITING_U16,
	WRITING_U32
};

/**
 * struct vendor_data - vendor-specific config parameters
 * for PL022 derivates
 * @fifodepth: depth of FIFOs (both)
 * @max_bpw: maximum number of bits per word
 * @unidir: supports unidirection transfers
 * @extended_cr: 32 bit wide control register 0 with extra
 * features and extra features in CR1 as found in the ST variants
 * @pl023: supports a subset of the ST extensions called "PL023"
 */
struct vendor_data {
	int fifodepth;
	int max_bpw;
	bool unidir;
	bool extended_cr;
	bool pl023;
	bool loopback;
};

/**
 * struct pl022 - This is the private SSP driver data structure
 * @adev: AMBA device model hookup
 * @vendor: vendor data for the IP block
 * @phybase: the physical memory where the SSP device resides
 * @virtbase: the virtual memory where the SSP is mapped
 * @clk: outgoing clock "SPICLK" for the SPI bus
 * @master: SPI framework hookup
 * @master_info: controller-specific data from machine setup
 * @kworker: thread struct for message pump
 * @kworker_task: pointer to task for message pump kworker thread
 * @pump_messages: work struct for scheduling work to the message pump
 * @queue_lock: spinlock to syncronise access to message queue
 * @queue: message queue
 * @busy: message pump is busy
 * @running: message pump is running
 * @pump_transfers: Tasklet used in Interrupt Transfer mode
 * @cur_msg: Pointer to current spi_message being processed
 * @cur_transfer: Pointer to current spi_transfer
 * @cur_chip: pointer to current clients chip(assigned from controller_state)
 * @next_msg_cs_active: the next message in the queue has been examined
 *  and it was found that it uses the same chip select as the previous
 *  message, so we left it active after the previous transfer, and it's
 *  active already.
 * @tx: current position in TX buffer to be read
 * @tx_end: end position in TX buffer to be read
 * @rx: current position in RX buffer to be written
 * @rx_end: end position in RX buffer to be written
 * @read: the type of read currently going on
 * @write: the type of write currently going on
 * @exp_fifo_level: expected FIFO level
 * @dma_rx_channel: optional channel for RX DMA
 * @dma_tx_channel: optional channel for TX DMA
 * @sgt_rx: scattertable for the RX transfer
 * @sgt_tx: scattertable for the TX transfer
 * @dummypage: a dummy page used for driving data on the bus with DMA
 */
struct pl022 {
	struct amba_device		*adev;
	struct vendor_data		*vendor;
	resource_size_t			phybase;
	void __iomem			*virtbase;
	struct clk			*clk;
	struct spi_master		*master;
	struct pl022_ssp_controller	*master_info;
	/* Message per-transfer pump */
	struct tasklet_struct		pump_transfers;
	struct spi_message		*cur_msg;
	struct spi_transfer		*cur_transfer;
	struct chip_data		*cur_chip;
	bool				next_msg_cs_active;
	void				*tx;
	void				*tx_end;
	void				*rx;
	void				*rx_end;
	enum ssp_reading		read;
	enum ssp_writing		write;
	u32				exp_fifo_level;
	enum ssp_rx_level_trig		rx_lev_trig;
	enum ssp_tx_level_trig		tx_lev_trig;
	/* DMA settings */
#ifdef CONFIG_DMA_ENGINE
	struct dma_chan			*dma_rx_channel;
	struct dma_chan			*dma_tx_channel;
	struct sg_table			sgt_rx;
	struct sg_table			sgt_tx;
	char				*dummypage;
	bool				dma_running;
#endif
};

/**
 * struct chip_data - To maintain runtime state of SSP for each client chip
 * @cr0: Value of control register CR0 of SSP - on later ST variants this
 *       register is 32 bits wide rather than just 16
 * @cr1: Value of control register CR1 of SSP
 * @dmacr: Value of DMA control Register of SSP
 * @n_bytes: how many bytes(power of 2) reqd for a given data width of client
 * @enable_dma: Whether to enable DMA or not
 * @read: function ptr to be used to read when doing xfer for this chip
 * @write: function ptr to be used to write when doing xfer for this chip
 * @cs_control: chip select callback provided by chip
 * @xfer_type: polling/interrupt/DMA
 *
 * Runtime state of the SSP controller, maintained per chip,
 * This would be set according to the current message that would be served
 */
struct chip_data {
	u32 ctl; //replace cr0 and cr1
	u32 cfdf;//clock frequency radio
	u16 dmacr;
	u8	spi_en;//enable spi
	u8 n_bytes;
	bool enable_dma;
	enum ssp_reading read;
	enum ssp_writing write;
	void (*cs_control) (u32 command);
	int xfer_type;
};
int32_t otf_config_data_register_func(void __iomem  * reg_addr,int32_t  width,              
		uint8_t wires,uint8_t tx_rx,uint8_t reg_en,int32_t data_count );         

/**
 * null_cs_control - Dummy chip select function
 * @command: select/delect the chip
 *
 * If no chip select function is provided by client this is used as dummy
 * chip select
 */
static void null_cs_control(u32 command)
{
	imapx_pad_set_mode(74, "output");
	imapx_pad_set_value(74, !!command);
}

/**
 * giveback - current spi_message is over, schedule next message and call
 * callback of this message. Assumes that caller already
 * set message->status; dma and pio irqs are blocked
 * @pl022: SSP driver private data structure
 */
static void giveback(struct pl022 *pl022)
{
	struct spi_transfer *last_transfer;
	pl022->next_msg_cs_active = false;

	last_transfer = list_entry(pl022->cur_msg->transfers.prev,
					struct spi_transfer,
					transfer_list);

	/* Delay if requested before any change in chip select */
	if (last_transfer->delay_usecs)
		/*
		 * FIXME: This runs in interrupt context.
		 * Is this really smart?
		 */
		udelay(last_transfer->delay_usecs);

	if (!last_transfer->cs_change) {
		struct spi_message *next_msg;

		/*
		 * cs_change was not set. We can keep the chip select
		 * enabled if there is message in the queue and it is
		 * for the same spi device.
		 *
		 * We cannot postpone this until pump_messages, because
		 * after calling msg->complete (below) the driver that
		 * sent the current message could be unloaded, which
		 * could invalidate the cs_control() callback...
		 */
		/* get a pointer to the next message, if any */
		next_msg = spi_get_next_queued_message(pl022->master);

		/*
		 * see if the next and current messages point
		 * to the same spi device.
		 */
		if (next_msg && next_msg->spi != pl022->cur_msg->spi)
			next_msg = NULL;
		if (!next_msg || pl022->cur_msg->state == STATE_ERROR)
			pl022->cur_chip->cs_control(SSP_CHIP_DESELECT);
		else
			pl022->next_msg_cs_active = true;

	}

	pl022->cur_msg = NULL;
	pl022->cur_transfer = NULL;
	pl022->cur_chip = NULL;
	spi_finalize_current_message(pl022->master);
}

/**
 * flush - flush the FIFO to reach a clean state
 * @pl022: SSP driver private data structure
 */
static int flush(struct pl022 *pl022)
{
	unsigned long limit = loops_per_jiffy << 1;

	dev_dbg(&pl022->adev->dev, "flush\n");
	do {
		//not empty,read until empty
		while (! (readw(SPI_SR(pl022->virtbase)) & SPI_SR_MASK_RNE))
			readw(SPI_FDR(pl022->virtbase));
	} while ((readw(SPI_SR(pl022->virtbase)) & SPI_SR_MASK_BSY) && limit--);

	pl022->exp_fifo_level = 0;

	return limit;
}

/**
 * restore_state - Load configuration of current chip
 * @pl022: SSP driver private data structure
 */
static void restore_state(struct pl022 *pl022)
{
	struct chip_data *chip = pl022->cur_chip;

	writew(chip->ctl, SPI_CTL(pl022->virtbase));
	writew(chip->cfdf, SPI_CFDF(pl022->virtbase));
	writew(chip->dmacr, SPI_DMACR(pl022->virtbase));
	//writew(DISABLE_ALL_INTERRUPTS, SPI_IMSC(pl022->virtbase));
	writew(CLEAR_ALL_INTERRUPTS, SPI_ICR(pl022->virtbase));
}

/*
 * Default SSP Register Values
 */
#define DEFAULT_SPI_REG_CTL ( \
	GEN_MASK_BITS(SSP_CLK_POL_IDLE_HIGH, SPI_CTL_MASK_SPO, 0) | \
	GEN_MASK_BITS(SSP_CLK_SECOND_EDGE, SPI_CTL_MASK_SPH, 1)  \
)

#define DEFAULT_SPI_REG_CFDF   ( GEN_MASK_BITS(SSP_DEFAULT_CLKRATE,SPI_CFDF_MASK_DIV,0))

#define DEFAULT_SSP_REG_CR1 ( GEN_MASK_BITS(SSP_DISABLED, SPI_MASK_EN, 1)  )

#define DEFAULT_SPI_REG_DMACR (\
	GEN_MASK_BITS(SSP_DMA_DISABLED, SPI_DMACR_MASK_RXDMAE, 1) | \
	GEN_MASK_BITS(SSP_DMA_DISABLED, SPI_DMACR_MASK_TXDMAE, 0) \
)

static void test_read_controller_register(struct pl022 *pl022)
{
	int32_t		val = 0;
#if 0
	val	= readl(SPI_EN(pl022->virtbase));
	printk("en=0x%x\n",val);
	val	= readl(SPI_CTL(pl022->virtbase));
	printk("ctl=0x%x\n",val);
	val	= readl(SPI_CFDF(pl022->virtbase));
	printk("cfdf=0x%x\n",val);
	val	= readl(SPI_IMSC(pl022->virtbase));
	printk("imsc=0x%x\n",val);
	val	= readl(SPI_RIS(pl022->virtbase));
	printk("ris=0x%x\n",val);
	val	= readl(SPI_MIS(pl022->virtbase));
	printk("mis=0x%x\n",val);
	val	= readl(SPI_ICR(pl022->virtbase));
	printk("icr=0x%x\n",val);
	val	= readl(SPI_DMACR(pl022->virtbase));
	printk("dmactl=0x%x\n",val);
	val	= readl(SPI_FTHD(pl022->virtbase));
	printk("fifothd=0x%x\n",val);
#endif
	val	= readl(SPI_EN(pl022->virtbase));
	printk("en=0x%x\n",val);
	val	= readl(SPI_SR(pl022->virtbase));
	printk("fifostatus=0x%x\n",val);
	val	= readl(SPI_TTC(pl022->virtbase));
	printk("tx-timeout-cnt=0x%x\n",val);
	val	= readl(SPI_TFDAC(pl022->virtbase));
	printk("tx-data-cnt=0x%x\n",val);
	val	= readl(SPI_RFDAC(pl022->virtbase));
	printk("rx-data-cnt=0x%x\n",val);
	val	= readl(SPI_RIS(pl022->virtbase));
	printk("ris=0x%x\n",val);
	val	= readl(SPI_DCTL0(pl022->virtbase));
	printk("dctl0=0x%x\n",val);
	val	= readl(SPI_DCTL1(pl022->virtbase));
	printk("dctl1=0x%x\n",val);
	val	= readl(SPI_DCTL2(pl022->virtbase));
	printk("dctl2=0x%x\n",val);
	val	= readl(SPI_DCTL3(pl022->virtbase));
	printk("dctl3=0x%x\n",val);
	val	= readl(SPI_DCTL4(pl022->virtbase));
	printk("dctl4=0x%x\n",val);
	val	= readl(SPI_FDRCTL(pl022->virtbase));
	printk("fdrctl=0x%x\n",val);
	val	= readl(SPI_DMACR(pl022->virtbase));
	printk("dmactl=0x%x\n",val);
}

#if 0
static void test_modify_controller_register(struct pl022 *pl022)
{
	int32_t		val = 0;
	//writel(0xffffffff,SPI_EN(pl022->virtbase));
	val	= readl(SPI_EN(pl022->virtbase));
	printk("en=0x%x\n",val);
	writel(0xffffffff,SPI_CTL(pl022->virtbase));
	val	= readl(SPI_CTL(pl022->virtbase));
	printk("ctl=0x%x\n",val);
	writel(0xffffffff,SPI_CFDF(pl022->virtbase));
	val	= readl(SPI_CFDF(pl022->virtbase));
	printk("cfdf=0x%x\n",val);
	writel(0xffffffff,SPI_IMSC(pl022->virtbase));
	val	=readl(SPI_IMSC(pl022->virtbase));
	printk("imsc=0x%x\n",val);
	writel(0xffffffff,SPI_RIS(pl022->virtbase));
	val	= readl (SPI_RIS(pl022->virtbase));
	printk("ris=0x%x\n",val);
	writel(0xffffffff,SPI_MIS(pl022->virtbase));
	val	=readl (SPI_MIS(pl022->virtbase));
	printk("mis=0x%x\n",val);
	writel(0xffffffff,SPI_ICR(pl022->virtbase));
	val	=readl (SPI_ICR(pl022->virtbase));
	printk("icr=0x%x\n",val);
	writel(0xffffffff,SPI_DMACR(pl022->virtbase));
	val	=readl (SPI_DMACR(pl022->virtbase));
	printk("dmactl=0x%x\n",val);
	writel(0xffffffff,SPI_FTHD(pl022->virtbase));
	val	=readl (SPI_FTHD(pl022->virtbase));
	printk("fifothd=0x%x\n",val);
	writel(0xffffffff,SPI_SR(pl022->virtbase));
	val	=readl (SPI_SR(pl022->virtbase));
	printk("fifostatus=0x%x\n",val);
	writel(0xffffffff,SPI_TTC(pl022->virtbase));
	val	=readl (SPI_TTC(pl022->virtbase));
	printk("tx-timeout-cnt=0x%x\n",val);
	writel(0xffffffff,SPI_TFDAC(pl022->virtbase));
	val	=readl (SPI_TFDAC(pl022->virtbase));
	printk("tx-data-cnt=0x%x\n",val);
	writel(0xffffffff,SPI_RFDAC(pl022->virtbase));
	val	=readl (SPI_RFDAC(pl022->virtbase));
	printk("rx-data-cnt=0x%x\n",val);
	writel(0xffffffff,SPI_DR0(pl022->virtbase));
	val	=readl (SPI_DR0(pl022->virtbase));
	printk("dr0=0x%x\n",val);
	writel(0xffffffff,SPI_DCTL0(pl022->virtbase));
	val	=readl (SPI_DCTL0(pl022->virtbase));
	printk("dctl0=0x%x\n",val);
	writel(0xffffffff,SPI_DR1(pl022->virtbase));
	val	=readl (SPI_DR1(pl022->virtbase));
	printk("dr1=0x%x\n",val);
	writel(0xffffffff,SPI_DCTL1(pl022->virtbase));
	val	=readl (SPI_DCTL1(pl022->virtbase));
	printk("dctl1=0x%x\n",val);
	writel(0xffffffff,SPI_DR2(pl022->virtbase));
	val	=readl (SPI_DR2(pl022->virtbase));
	printk("dr2=0x%x\n",val);
	writel(0xffffffff,SPI_DCTL2(pl022->virtbase));
	val	=readl (SPI_DCTL2(pl022->virtbase));
	printk("dctl2=0x%x\n",val);
	writel(0xffffffff,SPI_DR3(pl022->virtbase));
	val	=readl (SPI_DR3(pl022->virtbase));
	printk("dr3=0x%x\n",val);
	writel(0xffffffff,SPI_DCTL3(pl022->virtbase));
	val	=readl (SPI_DCTL3(pl022->virtbase));
	printk("dctl3=0x%x\n",val);
	writel(0xffffffff,SPI_DR4(pl022->virtbase));
	val	=readl (SPI_DR4(pl022->virtbase));
	printk("dr4=0x%x\n",val);
	writel(0xffffffff,SPI_DCTL4(pl022->virtbase));
	val	=readl (SPI_DCTL4(pl022->virtbase));
	printk("dctl4=0x%x\n",val);
	writel(0xffffffff,SPI_FDR(pl022->virtbase));
	val	=readl (SPI_FDR(pl022->virtbase));
	printk("fdr=0x%x\n",val);
	writel(0xffffffff,SPI_FDRCTL(pl022->virtbase));
	val	=readl (SPI_FDRCTL(pl022->virtbase));
	printk("fdrctl=0x%x\n",val);

}
#endif


/**
 * load_ssp_default_config - Load default configuration for SSP
 * @pl022: SSP driver private data structure
 */
static void load_ssp_default_config(struct pl022 *pl022)
{
	writel(DEFAULT_SPI_REG_CTL, SPI_CTL(pl022->virtbase));
	writel(DEFAULT_SPI_REG_CFDF, SPI_CFDF(pl022->virtbase));
	writel(DEFAULT_SPI_REG_DMACR, SPI_DMACR(pl022->virtbase));
	//writel(DISABLE_ALL_INTERRUPTS, SPI_IMSC(pl022->virtbase));
	writel(CLEAR_ALL_INTERRUPTS, SPI_ICR(pl022->virtbase));
	//FIFO THRD
	writel(32<<8|32,SPI_FTHD(pl022->virtbase));
	//TX timeout 
	writel(0xffffffff,SPI_TTC(pl022->virtbase));
	//DR disable
	writel(0x0,SPI_DCTL0(pl022->virtbase));
	writel(0x0,SPI_DCTL1(pl022->virtbase));
	writel(0x0,SPI_DCTL2(pl022->virtbase));
	writel(0x0,SPI_DCTL3(pl022->virtbase));
	writel(0x0,SPI_DCTL4(pl022->virtbase));
	writel(0x0,SPI_FDRCTL(pl022->virtbase));
}


int32_t otf_config_data_register_func(void __iomem *reg_addr,int32_t  width,              
		uint8_t wires,uint8_t tx_rx,uint8_t reg_en,int32_t data_count )         
{                                                                                   
	int32_t             val = 0;                                                    
	val |= (width)<<8;                                                          
	switch(wires)                                                                   
	{                                                                               
		case 0x1:                                               
			{                                                                   
				val &= (~(0x3<<4));                                             
				break;                                                              
			}                                                                       
		case 0x2:                                               
			{                                                                       
				val &= (~(0x3<<4));                                             
				val |= 0x1<<4;                                                  
				break;                                                              
			}                                                                       
		case 0x4:                                                  
			{                                                                       
				val &= (~(0x3<<4));                                                 
				val |= 0x2<<4;                                                  
				break;                                                          
			}                                                                   
		default:                                                                
			break;                                                              
	}                                                                           
	val |= (tx_rx) <<1;                                                         
	val |= (reg_en)<<0;                                                         
	if ( data_count > 0 )                                                       
		val |= (data_count-1)<<16;                                              
	//spi_debug(DISPLAY_T4"Will write data_reg[0x%x] value:[0x%x]!\n",reg_addr,val);
	writel( val , reg_addr );                                                   
	return 0;                                                         
}               

//clear controller  
static void clear_controller(struct pl022 *pl022)
{
	//tx timeout 0
	writel(0x0,SPI_TTC(pl022->virtbase));
	//wait until not busy
	while(readl(SPI_SR(pl022->virtbase)) & SPI_SR_MASK_BSY);
	//wait until tx empty
	while(!(readl(SPI_SR(pl022->virtbase)) & SPI_SR_MASK_TFE));

	writel(0x0,SPI_DCTL0(pl022->virtbase));
	writel(0x0,SPI_DCTL1(pl022->virtbase));
	writel(0x0,SPI_DCTL2(pl022->virtbase));
	writel(0x0,SPI_FDRCTL(pl022->virtbase));
	writew(SSP_DISABLED,SPI_EN(pl022->virtbase));

}

//#define spi_debug

/**
 * This will write to TX and read from RX according to the parameters
 * set in pl022.
 */
static void readwriter(struct pl022 *pl022)
{
	unsigned long timeout=0,time=0;
	int		val = 0;
	struct  timeval  begin= {0};
	struct  timeval  end  = {0};
	//test_read_controller_register(pl022);
	/*
	 * The FIFO depth is different between primecell variants.
	 * I believe filling in too much in the FIFO might cause
	 * errons in 8bit wide transfers on ARM variants (just 8 words
	 * FIFO, means only 8x8 = 64 bits in FIFO) at least.
	 *
	 * To prevent this issue, the TX FIFO is only filled to the
	 * unused RX FIFO fill length, regardless of what the TX
	 * FIFO status flag indicates.
	 */

	//printk("===============readwriter\n");
	//pl022->cur_chip->cs_control(SSP_CHIP_SELECT);
	/* Read as much as you can */
	if((pl022->rx < pl022->rx_end) && (pl022->rx)) {
		uint8_t		wires = pl022->cur_transfer->wires;
		int			start = 0x0;
read_start:

#if 0
		printk("%s, rx: %p, rxend: %p, tx: %p, txend: %p\n",
				__func__, pl022->rx, pl022->rx_end, pl022->tx, pl022->tx_end);
		//mdelay(1*1000);
#endif
		if( wires != 1 && wires != 2 && wires != 4)
			wires = 1;
		while(readl(SPI_SR(pl022->virtbase)) & SPI_SR_MASK_BSY);
		//wait until tx empty
		while(!(readl(SPI_SR(pl022->virtbase)) & SPI_SR_MASK_TFE));
		pl022->cur_chip->cs_control(SSP_CHIP_SELECT);
		//CMD
		if ( pl022->cur_transfer->cmd_len > 0)
		{
			int i		=0;
			int cmd_len = 0;
			cmd_len		= pl022->cur_transfer->cmd_len;
			for( i = 0 ; i < cmd_len; i++)
			{
				val |= pl022->cur_transfer->cmd[i]<<(cmd_len-i-1)*8;
			}
			writel(val,SPI_DR0(pl022->virtbase));
			printk("==cmd=0x%x,cmd_len=%d\n",pl022->cur_transfer->cmd[0],cmd_len);
			otf_config_data_register_func(SPI_DCTL0(pl022->virtbase), pl022->cur_transfer->cmd_len*8-1,0x1,0,1,0);
		}
		//ADDR
		if( pl022->cur_transfer->addr_len > 0 ){
			val = 0;
			val |= (pl022->cur_transfer->addr & 0xffffffff);
			writel(val,SPI_DR1(pl022->virtbase));
			otf_config_data_register_func(SPI_DCTL1(pl022->virtbase), pl022->cur_transfer->addr_len*8-1,0x1,0,1,0);
			printk("===addr=0x%x,addr_len=%d\n",val,pl022->cur_transfer->addr_len);
		}
		//dummy 
		if( pl022->cur_transfer->dummy_len > 0 ){
			writel(0x0,SPI_DR2(pl022->virtbase));
			otf_config_data_register_func(SPI_DCTL2(pl022->virtbase), pl022->cur_transfer->dummy_len*8-1,0x1,0,1,0);
			printk("===dummy,len=%d\n",pl022->cur_transfer->dummy_len);
		}
		otf_config_data_register_func(SPI_FDRCTL(pl022->virtbase), pl022->read*8-1, wires,1,1,pl022->rx_end-pl022->rx);
		printk("rx=0x%x,rx_end=0x%x\n",pl022->rx,pl022->rx_end);
		if (pl022->cur_chip->enable_dma) {
			printk("============dma read\n");
			writew(SSP_ENABLED,SPI_EN(pl022->virtbase));
			return;
		}
		start = pl022->rx;
		writew(SSP_ENABLED,SPI_EN(pl022->virtbase));
		//printk("rx=0x%x,rx_end=0x%x\n",pl022->rx,pl022->rx_end);
		do_gettimeofday(&begin);
		while ((pl022->rx < pl022->rx_end)) {
#if 0
			while( readl(SPI_RFDAC(pl022->virtbase)) ){
				*(u8 *) (pl022->rx) = readw(SPI_FDR(pl022->virtbase))&0xFFU;
				pl022->rx += (pl022->cur_chip->n_bytes);
			}
#endif
#if 1
			while( !(readl(SPI_SR(pl022->virtbase)) & SPI_SR_MASK_RNE) )
			{
				*(u8 *) (pl022->rx) = readw(SPI_FDR(pl022->virtbase)) & 0xFFU;
				pl022->rx += (pl022->cur_chip->n_bytes);
				//RX THROD INTR
			}
#endif

			do_gettimeofday(&end);
			if( end.tv_usec - begin.tv_usec > 10*1000)//10ms
			{
				//test_read_controller_register(pl022);
				//printk("======%s,%d,start=%d,end=%d\n",__func__,__LINE__,begin.tv_sec,end.tv_sec);
				//printk("loop:rx=0x%x,rx_end=0x%x,addr=0x%x\n",pl022->rx,pl022->rx_end,pl022->cur_transfer->addr);
				//printk("==cmd=0x%x,addr=0x%x,addr_len=%d\n", pl022->cur_transfer->cmd[0], pl022->cur_transfer->addr,pl022->cur_transfer->addr_len);
				pl022->rx = start;
				//pull gigh
				//tx timeout 0
				writel(0x0,SPI_TTC(pl022->virtbase));
				//wait until not busy
				while(readl(SPI_SR(pl022->virtbase)) & SPI_SR_MASK_BSY);
				//wait until tx empty
				while(!(readl(SPI_SR(pl022->virtbase)) & SPI_SR_MASK_TFE));

				writel(0x0,SPI_DCTL0(pl022->virtbase));
				writel(0x0,SPI_DCTL1(pl022->virtbase));
				writel(0x0,SPI_DCTL2(pl022->virtbase));
				writel(0x0,SPI_FDRCTL(pl022->virtbase));
				writew(SSP_DISABLED,SPI_EN(pl022->virtbase));
				pl022->cur_chip->cs_control(SSP_CHIP_DESELECT);
				goto read_start;
			}
		}
	//printk("======%s,%d,start=%d,end=%d\n",__func__,__LINE__,begin.tv_sec,end.tv_sec);
	}
#ifdef spi_debug
	printk("======%s,%d\n",__func__,__LINE__);
#endif
	if( (pl022->tx < pl022->tx_end) && pl022->tx ) {

		char		cmd_count = pl022->cur_transfer->cmd_len;
		char		i = 0;
		uint8_t		wires = pl022->cur_transfer->wires;
		if( wires != 1 && wires != 2 && wires != 4)
			wires = 1;
		pl022->cur_chip->cs_control(SSP_CHIP_SELECT);
		//printk("debug,write dr0:0x%x,tx=0x%x\n",val,pl022->tx);
		//printk("======%s,%d\n",__func__,__LINE__);
		//printk("tx=0x%x,tx_end=0x%x\n",pl022->tx,pl022->tx_end);
		while(!(readl(SPI_SR(pl022->virtbase))&SPI_SR_MASK_TFE))
		{
			printk("TX fifo not empty!\n");
			dump_stack();
			test_read_controller_register(pl022);
			clear_controller(pl022);
		}
		if(cmd_count > 0){//max is 4
			val = 0;
			for( i = 0 ; i < cmd_count; i ++)
			{
				val |= (*(u8 *)(pl022->tx))<<(cmd_count-i-1)*8;	
				pl022->tx += (pl022->cur_chip->n_bytes);
			}
			writel(val,SPI_DR0(pl022->virtbase));
			otf_config_data_register_func(SPI_DCTL0(pl022->virtbase), cmd_count*8-1,0x1,0,1,0);//one wire
		}else{
			//cmd data:cmd_len > 0;
			//raw data:cmd_len = 0;
			val = 0;
			val = *(u8 *)(pl022->tx);
			writeb(val,SPI_DR0(pl022->virtbase));
			if( pl022->cur_transfer->cmd_len > 0)
				otf_config_data_register_func(SPI_DCTL0(pl022->virtbase), 7,0x1,0,1,0);//one wire
			else 
				otf_config_data_register_func(SPI_DCTL0(pl022->virtbase), 7,wires,0,1,0);//four wire
			pl022->tx += (pl022->cur_chip->n_bytes);

		}

		//has addr,at least is 4
		if( (pl022->tx_end - pl022->tx ) > 2 )
		{
			//ADDR
			val = 0;
			val |= *(u8 *)(pl022->tx)<<16;
			pl022->tx += (pl022->cur_chip->n_bytes);
			val |= *(u8 *)(pl022->tx)<<8;
			pl022->tx += (pl022->cur_chip->n_bytes);
			val |= *(u8 *)(pl022->tx)<<0;
			pl022->tx += (pl022->cur_chip->n_bytes);
			writel(val,SPI_DR1(pl022->virtbase));
			//printk("write dr1:0x%x\n",val);
			if( pl022->cur_transfer->cmd_len > 0)
				otf_config_data_register_func(SPI_DCTL1(pl022->virtbase), 23,0x1,0,1,0);//one wire
			else
				otf_config_data_register_func(SPI_DCTL1(pl022->virtbase), 23,wires,0,1,0);//four wire

		}
		//had 4 byte addr 
		//if( (pl022->tx_end - pl022->tx) > 0)
		if( pl022->cur_transfer->addr_len  > 3)
		{
			val = 0;
			val = *(u8 *)(pl022->tx);
			writeb(val,SPI_DR2(pl022->virtbase));
			//printk("write dr2:0x%x\n",val);
			pl022->tx += (pl022->cur_chip->n_bytes);
			otf_config_data_register_func(SPI_DCTL2(pl022->virtbase), 7,0x1,0,1,0);

		}
		if( pl022->tx < pl022->tx_end){
			if( pl022->cur_transfer->cmd_len >0)
				otf_config_data_register_func(SPI_FDRCTL(pl022->virtbase), pl022->write*8-1, 0x1,0,1,pl022->tx_end-pl022->tx);
			else
				otf_config_data_register_func(SPI_FDRCTL(pl022->virtbase), pl022->write*8-1, wires,0,1,pl022->tx_end-pl022->tx);
		}
		if (pl022->cur_chip->enable_dma) {
			printk("============dma write\n");
			return;
		}
		writew(SSP_ENABLED,SPI_EN(pl022->virtbase));
		while( (pl022->tx < pl022->tx_end) ) {
			if( !(readw(SPI_SR(pl022->virtbase)) & SPI_SR_MASK_TNF )){//TX not full
				writeb(*(u8 *)(pl022->tx), SPI_FDR(pl022->virtbase));
				//printk("===fdr=0x%x,tx=0x%x\n",*(u8 *)(pl022->tx),pl022->tx);
				//test_read_controller_register(pl022);	
				//printk("tx=0x%x,tx_end=0x%x,0x%x,count=%d\n",pl022->tx,pl022->tx_end,readl(SPI_FDRCTL(pl022->virtbase)),i++);
				pl022->tx += (pl022->cur_chip->n_bytes);
				pl022->exp_fifo_level++;
			}
		}

	}
#ifdef spi_debug 
	printk("======%s,%d\n",__func__,__LINE__);
#endif

	//tx timeout 0
	writel(0x0,SPI_TTC(pl022->virtbase));
	//wait until not busy
	while(readl(SPI_SR(pl022->virtbase)) & SPI_SR_MASK_BSY);
	//wait until tx empty
	while(!(readl(SPI_SR(pl022->virtbase)) & SPI_SR_MASK_TFE));

	writel(0x0,SPI_DCTL0(pl022->virtbase));
	writel(0x0,SPI_DCTL1(pl022->virtbase));
	writel(0x0,SPI_DCTL2(pl022->virtbase));
	writel(0x0,SPI_FDRCTL(pl022->virtbase));
	writew(SSP_DISABLED,SPI_EN(pl022->virtbase));
	/*
	 * When we exit here the TX FIFO should be full and the RX FIFO
	 * should be empty
	 */
}

/**
 * next_transfer - Move to the Next transfer in the current spi message
 * @pl022: SSP driver private data structure
 *
 * This function moves though the linked list of spi transfers in the
 * current spi message and returns with the state of current spi
 * message i.e whether its last transfer is done(STATE_DONE) or
 * Next transfer is ready(STATE_RUNNING)
 */
static void *next_transfer(struct pl022 *pl022)
{
	struct spi_message *msg = pl022->cur_msg;
	struct spi_transfer *trans = pl022->cur_transfer;

	/* Move to next transfer */
	if (trans->transfer_list.next != &msg->transfers) {
		pl022->cur_transfer =
		    list_entry(trans->transfer_list.next,
			       struct spi_transfer, transfer_list);
		return STATE_RUNNING;
	}
	return STATE_DONE;
}

/*
 * This DMA functionality is only compiled in if we have
 * access to the generic DMA devices/DMA engine.
 */
#ifdef CONFIG_DMA_ENGINE
static void unmap_free_dma_scatter(struct pl022 *pl022)
{
	/* Unmap and free the SG tables */
	dma_unmap_sg(pl022->dma_tx_channel->device->dev, pl022->sgt_tx.sgl,
		     pl022->sgt_tx.nents, DMA_TO_DEVICE);
	dma_unmap_sg(pl022->dma_rx_channel->device->dev, pl022->sgt_rx.sgl,
		     pl022->sgt_rx.nents, DMA_FROM_DEVICE);
	sg_free_table(&pl022->sgt_rx);
	sg_free_table(&pl022->sgt_tx);
}

static void dma_callback(void *data)
{
	struct pl022 *pl022 = data;
	struct spi_message *msg = pl022->cur_msg;

	BUG_ON(!pl022->sgt_rx.sgl);

	printk("=========================DMA CALL BACK!\n");
	dump_stack();
#ifdef VERBOSE_DEBUG
	/*
	 * Optionally dump out buffers to inspect contents, this is
	 * good if you want to convince yourself that the loopback
	 * read/write contents are the same, when adopting to a new
	 * DMA engine.
	 */
	{
		struct scatterlist *sg;
		unsigned int i;

		dma_sync_sg_for_cpu(&pl022->adev->dev,
				    pl022->sgt_rx.sgl,
				    pl022->sgt_rx.nents,
				    DMA_FROM_DEVICE);

		for_each_sg(pl022->sgt_rx.sgl, sg, pl022->sgt_rx.nents, i) {
			dev_dbg(&pl022->adev->dev, "SPI RX SG ENTRY: %d", i);
			print_hex_dump(KERN_ERR, "SPI RX: ",
				       DUMP_PREFIX_OFFSET,
				       16,
				       1,
				       sg_virt(sg),
				       sg_dma_len(sg),
				       1);
		}
#if 0
		for_each_sg(pl022->sgt_tx.sgl, sg, pl022->sgt_tx.nents, i) {
			dev_dbg(&pl022->adev->dev, "SPI TX SG ENTRY: %d", i);
			print_hex_dump(KERN_ERR, "SPI TX: ",
				       DUMP_PREFIX_OFFSET,
				       16,
				       1,
				       sg_virt(sg),
				       sg_dma_len(sg),
				       1);
		}
#endif
	}
#endif

	unmap_free_dma_scatter(pl022);

	/* Update total bytes transferred */
	msg->actual_length += pl022->cur_transfer->len;
	if (pl022->cur_transfer->cs_change)
		pl022->cur_chip->
			cs_control(SSP_CHIP_DESELECT);

	/* Move to next transfer */
	msg->state = next_transfer(pl022);
	tasklet_schedule(&pl022->pump_transfers);
}

static void setup_dma_scatter(struct pl022 *pl022,
			      void *buffer,
			      unsigned int length,
			      struct sg_table *sgtab)
{
	struct scatterlist *sg;
	int bytesleft = length;
	void *bufp = buffer;
	int mapbytes;
	int i;

	if (buffer) {
		for_each_sg(sgtab->sgl, sg, sgtab->nents, i) {
			/*
			 * If there are less bytes left than what fits
			 * in the current page (plus page alignment offset)
			 * we just feed in this, else we stuff in as much
			 * as we can.
			 */
			if (bytesleft < (PAGE_SIZE - offset_in_page(bufp)))
				mapbytes = bytesleft;
			else
				mapbytes = PAGE_SIZE - offset_in_page(bufp);
			sg_set_page(sg, virt_to_page(bufp),
				    mapbytes, offset_in_page(bufp));
			bufp += mapbytes;
			bytesleft -= mapbytes;
			dev_dbg(&pl022->adev->dev,
				"set RX/TX target page @ %p, %d bytes, %d left\n",
				bufp, mapbytes, bytesleft);
		}
	} else {
		/* Map the dummy buffer on every page */
		for_each_sg(sgtab->sgl, sg, sgtab->nents, i) {
			if (bytesleft < PAGE_SIZE)
				mapbytes = bytesleft;
			else
				mapbytes = PAGE_SIZE;
			sg_set_page(sg, virt_to_page(pl022->dummypage),
				    mapbytes, 0);
			bytesleft -= mapbytes;
			dev_dbg(&pl022->adev->dev,
				"set RX/TX to dummy page %d bytes, %d left\n",
				mapbytes, bytesleft);

		}
	}
	BUG_ON(bytesleft);
}

/**
 * configure_dma - configures the channels for the next transfer
 * @pl022: SSP driver's private data structure
 */
static int configure_dma(struct pl022 *pl022)
{
	struct dma_slave_config rx_conf = {
		.src_addr = SPI_FDR(pl022->phybase),
		.direction = DMA_DEV_TO_MEM,
		.device_fc = false,
	};
	struct dma_slave_config tx_conf = {
		.dst_addr = SPI_FDR(pl022->phybase),
		.direction = DMA_MEM_TO_DEV,
		.device_fc = false,
	};
	unsigned int pages;
	int ret;
	int rx_sglen, tx_sglen;
	struct dma_chan *rxchan = pl022->dma_rx_channel;
	struct dma_chan *txchan = pl022->dma_tx_channel;
	struct dma_async_tx_descriptor *rxdesc;
	struct dma_async_tx_descriptor *txdesc;

	/* Check that the channels are available */
	if (!rxchan || !txchan)
		return -ENODEV;

	/*
	 * If supplied, the DMA burstsize should equal the FIFO trigger level.
	 * Notice that the DMA engine uses one-to-one mapping. Since we can
	 * not trigger on 2 elements this needs explicit mapping rather than
	 * calculation.
	 */
	switch (pl022->rx_lev_trig) {
	case SSP_RX_1_OR_MORE_ELEM:
		rx_conf.src_maxburst = 1;
		break;
	case SSP_RX_4_OR_MORE_ELEM:
		rx_conf.src_maxburst = 4;
		break;
	case SSP_RX_8_OR_MORE_ELEM:
		rx_conf.src_maxburst = 8;
		break;
	case SSP_RX_16_OR_MORE_ELEM:
		rx_conf.src_maxburst = 16;
		break;
	case SSP_RX_32_OR_MORE_ELEM:
		rx_conf.src_maxburst = 32;
		break;
	default:
		rx_conf.src_maxburst = pl022->vendor->fifodepth >> 1;
		break;
	}

	switch (pl022->tx_lev_trig) {
	case SSP_TX_1_OR_MORE_EMPTY_LOC:
		tx_conf.dst_maxburst = 1;
		break;
	case SSP_TX_4_OR_MORE_EMPTY_LOC:
		tx_conf.dst_maxburst = 4;
		break;
	case SSP_TX_8_OR_MORE_EMPTY_LOC:
		tx_conf.dst_maxburst = 8;
		break;
	case SSP_TX_16_OR_MORE_EMPTY_LOC:
		tx_conf.dst_maxburst = 16;
		break;
	case SSP_TX_32_OR_MORE_EMPTY_LOC:
		tx_conf.dst_maxburst = 32;
		break;
	default:
		tx_conf.dst_maxburst = pl022->vendor->fifodepth >> 1;
		break;
	}

	switch (pl022->read) {
	case READING_NULL:
		/* Use the same as for writing */
		rx_conf.src_addr_width = DMA_SLAVE_BUSWIDTH_UNDEFINED;
		break;
	case READING_U8:
		rx_conf.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		break;
	case READING_U16:
		rx_conf.src_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		break;
	case READING_U32:
		rx_conf.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		break;
	}

	switch (pl022->write) {
	case WRITING_NULL:
		/* Use the same as for reading */
		tx_conf.dst_addr_width = DMA_SLAVE_BUSWIDTH_UNDEFINED;
		break;
	case WRITING_U8:
		tx_conf.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		break;
	case WRITING_U16:
		tx_conf.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		break;
	case WRITING_U32:
		tx_conf.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		break;
	}

	/* SPI pecularity: we need to read and write the same width */
	if (rx_conf.src_addr_width == DMA_SLAVE_BUSWIDTH_UNDEFINED)
		rx_conf.src_addr_width = tx_conf.dst_addr_width;
	if (tx_conf.dst_addr_width == DMA_SLAVE_BUSWIDTH_UNDEFINED)
		tx_conf.dst_addr_width = rx_conf.src_addr_width;
	BUG_ON(rx_conf.src_addr_width != tx_conf.dst_addr_width);

	dmaengine_slave_config(rxchan, &rx_conf);
	dmaengine_slave_config(txchan, &tx_conf);

	/* Create sglists for the transfers */
	pages = DIV_ROUND_UP(pl022->cur_transfer->len, PAGE_SIZE);
	dev_dbg(&pl022->adev->dev, "using %d pages for transfer\n", pages);

	ret = sg_alloc_table(&pl022->sgt_rx, pages, GFP_ATOMIC);
	if (ret)
		goto err_alloc_rx_sg;

	ret = sg_alloc_table(&pl022->sgt_tx, pages, GFP_ATOMIC);
	if (ret)
		goto err_alloc_tx_sg;

	/* Fill in the scatterlists for the RX+TX buffers */
	setup_dma_scatter(pl022, pl022->rx,
			  pl022->cur_transfer->len, &pl022->sgt_rx);
	setup_dma_scatter(pl022, pl022->tx,
			  pl022->cur_transfer->len, &pl022->sgt_tx);

	/* Map DMA buffers */
	rx_sglen = dma_map_sg(rxchan->device->dev, pl022->sgt_rx.sgl,
			   pl022->sgt_rx.nents, DMA_FROM_DEVICE);
	if (!rx_sglen)
		goto err_rx_sgmap;

	tx_sglen = dma_map_sg(txchan->device->dev, pl022->sgt_tx.sgl,
			   pl022->sgt_tx.nents, DMA_TO_DEVICE);
	if (!tx_sglen)
		goto err_tx_sgmap;

	/* Send both scatterlists */
	rxdesc = dmaengine_prep_slave_sg(rxchan,
				      pl022->sgt_rx.sgl,
				      rx_sglen,
				      DMA_DEV_TO_MEM,
				      DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!rxdesc)
		goto err_rxdesc;

	txdesc = dmaengine_prep_slave_sg(txchan,
				      pl022->sgt_tx.sgl,
				      tx_sglen,
				      DMA_MEM_TO_DEV,
				      DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!txdesc)
		goto err_txdesc;

	/* Put the callback on the RX transfer only, that should finish last */
	rxdesc->callback = dma_callback;
	rxdesc->callback_param = pl022;

	/* Submit and fire RX and TX with TX last so we're ready to read! */
	dmaengine_submit(rxdesc);
	dmaengine_submit(txdesc);
	dma_async_issue_pending(rxchan);
	dma_async_issue_pending(txchan);
	pl022->dma_running = true;

	return 0;

err_txdesc:
	dmaengine_terminate_all(txchan);
err_rxdesc:
	dmaengine_terminate_all(rxchan);
	dma_unmap_sg(txchan->device->dev, pl022->sgt_tx.sgl,
		     pl022->sgt_tx.nents, DMA_TO_DEVICE);
err_tx_sgmap:
	dma_unmap_sg(rxchan->device->dev, pl022->sgt_rx.sgl,
		     pl022->sgt_tx.nents, DMA_FROM_DEVICE);
err_rx_sgmap:
	sg_free_table(&pl022->sgt_tx);
err_alloc_tx_sg:
	sg_free_table(&pl022->sgt_rx);
err_alloc_rx_sg:
	return -ENOMEM;
}

static int pl022_dma_probe(struct pl022 *pl022)
{
	printk("=========summer,%s,%d\n",__func__,__LINE__);
	dma_cap_mask_t mask;

	/* Try to acquire a generic DMA engine slave channel */
	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	/*
	 * We need both RX and TX channels to do DMA, else do none
	 * of them.
	 */
	pl022->dma_rx_channel = dma_request_channel(mask,
					    pl022->master_info->dma_filter,
					    pl022->master_info->dma_rx_param);
	if (!pl022->dma_rx_channel) {
		dev_dbg(&pl022->adev->dev, "no RX DMA channel!\n");
	printk("==========summer,%s,%d\n",__func__,__LINE__);
		goto err_no_rxchan;
	}

	pl022->dma_tx_channel = dma_request_channel(mask,
					    pl022->master_info->dma_filter,
					    pl022->master_info->dma_tx_param);
	if (!pl022->dma_tx_channel) {
		dev_dbg(&pl022->adev->dev, "no TX DMA channel!\n");
	printk("==========summer,%s,%d\n",__func__,__LINE__);
		goto err_no_txchan;
	}

	pl022->dummypage = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!pl022->dummypage) {
		dev_dbg(&pl022->adev->dev, "no DMA dummypage!\n");
	printk("==========summer,%s,%d\n",__func__,__LINE__);
		goto err_no_dummypage;
	}

	dev_info(&pl022->adev->dev, "setup for DMA on RX %s, TX %s\n",
		 dma_chan_name(pl022->dma_rx_channel),
		 dma_chan_name(pl022->dma_tx_channel));

	return 0;

err_no_dummypage:
	dma_release_channel(pl022->dma_tx_channel);
err_no_txchan:
	dma_release_channel(pl022->dma_rx_channel);
	pl022->dma_rx_channel = NULL;
err_no_rxchan:
	dev_err(&pl022->adev->dev,
			"Failed to work in dma mode, work without dma!\n");
	return -ENODEV;
}

static void terminate_dma(struct pl022 *pl022)
{
	struct dma_chan *rxchan = pl022->dma_rx_channel;
	struct dma_chan *txchan = pl022->dma_tx_channel;

	dmaengine_terminate_all(rxchan);
	dmaengine_terminate_all(txchan);
	unmap_free_dma_scatter(pl022);
	pl022->dma_running = false;
}

static void pl022_dma_remove(struct pl022 *pl022)
{
	if (pl022->dma_running)
		terminate_dma(pl022);
	if (pl022->dma_tx_channel)
		dma_release_channel(pl022->dma_tx_channel);
	if (pl022->dma_rx_channel)
		dma_release_channel(pl022->dma_rx_channel);
	kfree(pl022->dummypage);
}

#else
static inline int configure_dma(struct pl022 *pl022)
{
	return -ENODEV;
}

static inline int pl022_dma_probe(struct pl022 *pl022)
{
	return 0;
}

static inline void pl022_dma_remove(struct pl022 *pl022)
{
}
#endif

/**
 * pl022_interrupt_handler - Interrupt handler for SSP controller
 *
 * This function handles interrupts generated for an interrupt based transfer.
 * If a receive overrun (ROR) interrupt is there then we disable SSP, flag the
 * current message's state as STATE_ERROR and schedule the tasklet
 * pump_transfers which will do the postprocessing of the current message by
 * calling giveback(). Otherwise it reads data from RX FIFO till there is no
 * more data, and writes data in TX FIFO till it is not full. If we complete
 * the transfer we move to the next transfer and schedule the tasklet.
 */
static irqreturn_t pl022_interrupt_handler(int irq, void *dev_id)
{
	struct pl022 *pl022 = dev_id;
	struct spi_message *msg = pl022->cur_msg;
	u16 irq_status = 0;
	u16 flag = 0;

	//test_read_controller_register(pl022);

	if (unlikely(!msg)) {
		//writew(DISABLE_ALL_INTERRUPTS,SPI_IMSC(pl022->virtbase));
		writew(CLEAR_ALL_INTERRUPTS, SPI_ICR(pl022->virtbase));
		dev_err(&pl022->adev->dev,
			"bad message state in interrupt handler");
		/* Never fail */
		return IRQ_HANDLED;
	}

	/* Read the Interrupt Status Register */
	irq_status = readw(SPI_MIS(pl022->virtbase));

	if (unlikely(!irq_status))
		return IRQ_NONE;

	/*
	 * This handles the FIFO interrupts, the timeout
	 * interrupts are flatly ignored, they cannot be
	 * trusted.
	 */
	if (unlikely(irq_status & SPI_MIS_MASK_RORMIS)) {
		/*
		 * Overrun interrupt - bail out since our Data has been
		 * corrupted
		 */
		dev_err(&pl022->adev->dev, "FIFO overrun\n");
		if (readw(SPI_SR(pl022->virtbase)) & SPI_SR_MASK_RFF)
			dev_err(&pl022->adev->dev,
				"RXFIFO is full\n");
		if (readw(SPI_SR(pl022->virtbase)) & SPI_SR_MASK_TNF)
			dev_err(&pl022->adev->dev,
				"TXFIFO is full\n");

		/*
		 * Disable and clear interrupts, disable SSP,
		 * mark message with bad status so it can be
		 * retried.
		 */
		//writew(DISABLE_ALL_INTERRUPTS,SPI_IMSC(pl022->virtbase));
		writew(CLEAR_ALL_INTERRUPTS, SPI_ICR(pl022->virtbase));
		//disable ssp
		writew(SSP_DISABLED,SPI_EN(pl022->virtbase));
		msg->state = STATE_ERROR;

		/* Schedule message queue handler */
		tasklet_schedule(&pl022->pump_transfers);
		return IRQ_HANDLED;
	}

	readwriter(pl022);

	if ((pl022->tx == pl022->tx_end) && (flag == 0)) {
		flag = 1;
		/* Disable Transmit interrupt, enable receive interrupt */
		writew((readw(SPI_IMSC(pl022->virtbase)) & SPI_IMSC_MASK_TXIM), 
				SPI_IMSC(pl022->virtbase));
	}

	/*
	 * Since all transactions must write as much as shall be read,
	 * we can conclude the entire transaction once RX is complete.
	 * At this point, all TX will always be finished.
	 */
	if (pl022->rx >= pl022->rx_end) {
		//writew(DISABLE_ALL_INTERRUPTS, SPI_IMSC(pl022->virtbase));
		writew(CLEAR_ALL_INTERRUPTS, SPI_ICR(pl022->virtbase));
		if (unlikely(pl022->rx > pl022->rx_end)) {
			dev_warn(&pl022->adev->dev, "read %u surplus "
				 "bytes (did you request an odd "
				 "number of bytes on a 16bit bus?)\n",
				 (u32) (pl022->rx - pl022->rx_end));
		}
		/* Update total bytes transferred */
		msg->actual_length += pl022->cur_transfer->len;
		if (pl022->cur_transfer->cs_change)
			pl022->cur_chip->
				cs_control(SSP_CHIP_DESELECT);
		/* Move to next transfer */
		msg->state = next_transfer(pl022);
		tasklet_schedule(&pl022->pump_transfers);
		return IRQ_HANDLED;
	}

	return IRQ_HANDLED;
}

static irqreturn_t pl022_interrupt_handler_or(int irq, void *dev_id)
{
	struct pl022 *pl022 = dev_id;
	printk("===================overrun,status=0x%x\n",readl(SPI_RIS(pl022->virtbase)));
	if( readl(SPI_RIS(pl022->virtbase)) & SSP_RIS_MASK_RORRIS )
		printk("==========overrun!\n");
	writew(CLEAR_ALL_INTERRUPTS, SPI_ICR(pl022->virtbase));
	return IRQ_HANDLED;

}
/**
 * This sets up the pointers to memory for the next message to
 * send out on the SPI bus.
 */
static int set_up_next_transfer(struct pl022 *pl022,
				struct spi_transfer *transfer)
{
	int residue;

	/* Sanity check the message for this bus width */
	residue = pl022->cur_transfer->len % pl022->cur_chip->n_bytes;
	if (unlikely(residue != 0)) {
		dev_err(&pl022->adev->dev,
			"message of %u bytes to transmit but the current "
			"chip bus has a data width of %u bytes!\n",
			pl022->cur_transfer->len,
			pl022->cur_chip->n_bytes);
		dev_err(&pl022->adev->dev, "skipping this message\n");
		return -EIO;
	}
	pl022->tx = (void *)transfer->tx_buf;
	pl022->tx_end = pl022->tx + pl022->cur_transfer->len;
	pl022->rx = (void *)transfer->rx_buf;
	pl022->rx_end = pl022->rx + pl022->cur_transfer->len;
	pl022->write =
	    pl022->tx ? pl022->cur_chip->write : WRITING_NULL;
	pl022->read = pl022->rx ? pl022->cur_chip->read : READING_NULL;
	return 0;
}

/**
 * pump_transfers - Tasklet function which schedules next transfer
 * when running in interrupt or DMA transfer mode.
 * @data: SSP driver private data structure
 *
 */
static void pump_transfers(unsigned long data)
{
	printk("=======summer,%s,%d\n",__func__,__LINE__);
	struct pl022 *pl022 = (struct pl022 *) data;
	struct spi_message *message = NULL;
	struct spi_transfer *transfer = NULL;
	struct spi_transfer *previous = NULL;

	/* Get current state information */
	message = pl022->cur_msg;
	transfer = pl022->cur_transfer;

	/* Handle for abort */
	if (message->state == STATE_ERROR) {
		message->status = -EIO;
		giveback(pl022);
		return;
	}

	/* Handle end of message */
	if (message->state == STATE_DONE) {
		message->status = 0;
		giveback(pl022);
		return;
	}

	/* Delay if requested at end of transfer before CS change */
	if (message->state == STATE_RUNNING) {
		previous = list_entry(transfer->transfer_list.prev,
					struct spi_transfer,
					transfer_list);
		if (previous->delay_usecs)
			/*
			 * FIXME: This runs in interrupt context.
			 * Is this really smart?
			 */
			udelay(previous->delay_usecs);

		/* Reselect chip select only if cs_change was requested */
		if (previous->cs_change)
			pl022->cur_chip->cs_control(SSP_CHIP_SELECT);
	} else {
		/* STATE_START */
		message->state = STATE_RUNNING;
	}

	if (set_up_next_transfer(pl022, transfer)) {
		message->state = STATE_ERROR;
		message->status = -EIO;
		giveback(pl022);
		return;
	}
	printk("=======summer,%s,%d\n",__func__,__LINE__);
	/* Flush the FIFOs and let's go! */
	flush(pl022);
	if (pl022->cur_chip->enable_dma) {
		if (configure_dma(pl022)) {
			dev_dbg(&pl022->adev->dev,
				"configuration of DMA failed, fall back to interrupt mode\n");
			goto err_config_dma;
		}
		//readwriter(pl022);//read command send by it.
		return;
	}

err_config_dma:
	/* enable all interrupts except RX */
	writew(ENABLE_ALL_INTERRUPTS | ~SPI_IMSC_MASK_RXIM, SPI_IMSC(pl022->virtbase));
}

static void do_interrupt_dma_transfer(struct pl022 *pl022)
{
	/*
	 * Default is to enable all interrupts except RX -
	 * this will be enabled once TX is complete
	 */
	u32 irqflags = ENABLE_ALL_INTERRUPTS & SPI_IMSC_MASK_RXIM;

	/* Enable target chip, if not already active */
	if (!pl022->next_msg_cs_active);
		//pl022->cur_chip->cs_control(SSP_CHIP_SELECT);

	printk("=========summer,%s,%d\n",__func__,__LINE__);
	if (set_up_next_transfer(pl022, pl022->cur_transfer)) {
		/* Error path */
		pl022->cur_msg->state = STATE_ERROR;
		pl022->cur_msg->status = -EIO;
		giveback(pl022);
		return;
	}
	/* If we're using DMA, set up DMA here */
	if (pl022->cur_chip->enable_dma) {
		//Enable dma register
		writew(0xff,SPI_DMACR(pl022->virtbase));
		/* Configure DMA transfer */
		if (configure_dma(pl022)) {
			dev_dbg(&pl022->adev->dev,
				"configuration of DMA failed, fall back to interrupt mode\n");
			printk("==============DMA ERROR\n");
			goto err_config_dma;
		}
		/*
		 *(1)CMD+ADDR+DUMMY
		 *(2)FDR configure,data count 
		 * */
		readwriter(pl022);
		//test_read_controller_register(pl022);
		/* Disable interrupts in DMA mode, IRQ from DMA controller */
		irqflags = DISABLE_ALL_INTERRUPTS;
	}
err_config_dma:
	/* Enable SSP, turn on interrupts */
	//writew(SSP_ENABLED,SPI_EN(pl022->virtbase));
	writew(irqflags, SPI_IMSC(pl022->virtbase));
}
//#define debug_polling
static void do_polling_transfer(struct pl022 *pl022)
{
	struct spi_message *message = NULL;
	struct spi_transfer *transfer = NULL;
	struct spi_transfer *previous = NULL;
	struct chip_data *chip;
	int i = 0;

	chip = pl022->cur_chip;
	message = pl022->cur_msg;

#ifdef debug_polling
	printk(KERN_ERR "%s::::::::,%d\n", __func__,__LINE__);
#endif
	while (message->state != STATE_DONE) {
		/* Handle for abort */
		if (message->state == STATE_ERROR)
			break;
		transfer = pl022->cur_transfer;
#ifdef debug_polling
	printk(KERN_ERR "%s::::::::,%d\n", __func__,__LINE__);
#endif

		/* Delay if requested at end of transfer */
		if (message->state == STATE_RUNNING) {
			previous =
			    list_entry(transfer->transfer_list.prev,
				       struct spi_transfer, transfer_list);
			if (previous->delay_usecs)
				udelay(previous->delay_usecs);
			if (previous->cs_change);
				//pl022->cur_chip->cs_control(SSP_CHIP_SELECT);
		} else {
			/* STATE_START */
			message->state = STATE_RUNNING;
			if (!pl022->next_msg_cs_active);
				//pl022->cur_chip->cs_control(SSP_CHIP_SELECT);
		}

		/* Configuration Changing Per Transfer */
		if (set_up_next_transfer(pl022, transfer)) {
			/* Error path */
			message->state = STATE_ERROR;
#ifdef debug_polling
	printk(KERN_ERR "%s::::::::,%d\n", __func__,__LINE__);
#endif
			break;
		}
		/* Flush FIFOs and enable SSP */
		flush(pl022);
#ifdef debug_polling 
	//	printk(&pl022->adev->dev, "polling transfer ongoing ...\n");
	printk(KERN_ERR "%s::::::::,%d\n", __func__,__LINE__);
#endif

		readwriter(pl022);

		/* Update total byte transferred */
		message->actual_length += pl022->cur_transfer->len;
		/* Move to next transfer */
		message->state = next_transfer(pl022);
	}
	/* Handle end of message */
	if (message->state == STATE_DONE)
		message->status = 0;
	else
		message->status = -EIO;

	giveback(pl022);
	return;
}

static int pl022_transfer_one_message(struct spi_master *master,
				      struct spi_message *msg)
{
	struct pl022 *pl022 = spi_master_get_devdata(master);

	/* Initial message state */
	pl022->cur_msg = msg;
	msg->state = STATE_START;

	pl022->cur_transfer = list_entry(msg->transfers.next,
					 struct spi_transfer, transfer_list);

	/* Setup the SPI using the per chip configuration */
	pl022->cur_chip = spi_get_ctldata(msg->spi);

	restore_state(pl022);
	flush(pl022);

	if (pl022->cur_chip->xfer_type == POLLING_TRANSFER)
	{
		do_polling_transfer(pl022);
	}else{
		do_interrupt_dma_transfer(pl022);
	}

	return 0;
}

static int pl022_prepare_transfer_hardware(struct spi_master *master)
{
	struct pl022 *pl022 = spi_master_get_devdata(master);

	/*
	 * Just make sure we have all we need to run the transfer by syncing
	 * with the runtime PM framework.
	 */
	pm_runtime_get_sync(&pl022->adev->dev);
	return 0;
}

static int pl022_unprepare_transfer_hardware(struct spi_master *master)
{
	struct pl022 *pl022 = spi_master_get_devdata(master);

	/* nothing more to do - disable spi/ssp and power off */
	writew(SSP_DISABLED,SPI_EN(pl022->virtbase));

	if (pl022->master_info->autosuspend_delay > 0) {
		pm_runtime_mark_last_busy(&pl022->adev->dev);
		pm_runtime_put_autosuspend(&pl022->adev->dev);
	} else {
		pm_runtime_put(&pl022->adev->dev);
	}

	return 0;
}

static int verify_controller_parameters(struct pl022 *pl022,
				struct pl022_config_chip const *chip_info)
{
	if ((chip_info->iface == SSP_INTERFACE_UNIDIRECTIONAL) &&
	    (!pl022->vendor->unidir)) {
		dev_err(&pl022->adev->dev,
			"unidirectional mode not supported in this "
			"hardware version\n");
		return -EINVAL;
	}
	if ((chip_info->hierarchy != SSP_MASTER)
	    && (chip_info->hierarchy != SSP_SLAVE)) {
		dev_err(&pl022->adev->dev,
			"hierarchy is configured incorrectly\n");
		return -EINVAL;
	}
	if ((chip_info->com_mode != INTERRUPT_TRANSFER)
	    && (chip_info->com_mode != DMA_TRANSFER)
	    && (chip_info->com_mode != POLLING_TRANSFER)) {
		dev_err(&pl022->adev->dev,
			"Communication mode is configured incorrectly\n");
		return -EINVAL;
	}
	switch (chip_info->rx_lev_trig) {
	case SSP_RX_1_OR_MORE_ELEM:
	case SSP_RX_4_OR_MORE_ELEM:
	case SSP_RX_8_OR_MORE_ELEM:
		/* These are always OK, all variants can handle this */
		break;
	case SSP_RX_16_OR_MORE_ELEM:
		if (pl022->vendor->fifodepth < 16) {
			dev_err(&pl022->adev->dev,
			"RX FIFO Trigger Level is configured incorrectly\n");
			return -EINVAL;
		}
		break;
	case SSP_RX_32_OR_MORE_ELEM:
		if (pl022->vendor->fifodepth < 32) {
			dev_err(&pl022->adev->dev,
			"RX FIFO Trigger Level is configured incorrectly\n");
			return -EINVAL;
		}
		break;
	default:
		dev_err(&pl022->adev->dev,
			"RX FIFO Trigger Level is configured incorrectly\n");
		return -EINVAL;
		break;
	}
	switch (chip_info->tx_lev_trig) {
	case SSP_TX_1_OR_MORE_EMPTY_LOC:
	case SSP_TX_4_OR_MORE_EMPTY_LOC:
	case SSP_TX_8_OR_MORE_EMPTY_LOC:
		/* These are always OK, all variants can handle this */
		break;
	case SSP_TX_16_OR_MORE_EMPTY_LOC:
		if (pl022->vendor->fifodepth < 16) {
			dev_err(&pl022->adev->dev,
			"TX FIFO Trigger Level is configured incorrectly\n");
			return -EINVAL;
		}
		break;
	case SSP_TX_32_OR_MORE_EMPTY_LOC:
		if (pl022->vendor->fifodepth < 32) {
			dev_err(&pl022->adev->dev,
			"TX FIFO Trigger Level is configured incorrectly\n");
			return -EINVAL;
		}
		break;
	default:
		dev_err(&pl022->adev->dev,
			"TX FIFO Trigger Level is configured incorrectly\n");
		return -EINVAL;
		break;
	}
	if (chip_info->iface == SSP_INTERFACE_NATIONAL_MICROWIRE) {
		if ((chip_info->ctrl_len < SSP_BITS_4)
		    || (chip_info->ctrl_len > SSP_BITS_32)) {
			dev_err(&pl022->adev->dev,
				"CTRL LEN is configured incorrectly\n");
			return -EINVAL;
		}
		if ((chip_info->wait_state != SSP_MWIRE_WAIT_ZERO)
		    && (chip_info->wait_state != SSP_MWIRE_WAIT_ONE)) {
			dev_err(&pl022->adev->dev,
				"Wait State is configured incorrectly\n");
			return -EINVAL;
		}
		/* Half duplex is only available in the ST Micro version */
		if (pl022->vendor->extended_cr) {
			if ((chip_info->duplex !=
			     SSP_MICROWIRE_CHANNEL_FULL_DUPLEX)
			    && (chip_info->duplex !=
				SSP_MICROWIRE_CHANNEL_HALF_DUPLEX)) {
				dev_err(&pl022->adev->dev,
					"Microwire duplex mode is configured incorrectly\n");
				return -EINVAL;
			}
		} else {
			if (chip_info->duplex != SSP_MICROWIRE_CHANNEL_FULL_DUPLEX)
				dev_err(&pl022->adev->dev,
					"Microwire half duplex mode requested,"
					" but this is only available in the"
					" ST version of PL022\n");
			return -EINVAL;
		}
	}
	return 0;
}

static inline u32 spi_rate(u32 rate, u16 cpsdvsr, u16 scr)
{
	return rate / (cpsdvsr * (1 + scr));
}

static int calculate_effective_freq(struct pl022 *pl022, int freq, struct
				    ssp_clock_params * clk_freq)
{
	/* Lets calculate the frequency parameters */
	u32 rate, max_tclk, min_tclk ;

	rate = clk_get_rate(pl022->clk);
	/* cpsdvscr = 2 & scr 0 */
	max_tclk = spi_rate(rate, CPSDVR_MIN, SCR_MIN);
	/* cpsdvsr = 254 & scr = 255 */
	min_tclk = spi_rate(rate, CPSDVR_MAX, SCR_MAX);

	if (freq > max_tclk)
		dev_warn(&pl022->adev->dev,
			"Max speed that can be programmed is %d Hz, you requested %d\n",
			max_tclk, freq);

	if (freq < min_tclk) {
		dev_warn(&pl022->adev->dev,
			"Requested frequency: %d Hz is less than minimum possible %d Hz\n",
			freq, min_tclk);
		return -EINVAL;
	}

	clk_freq->cfdf = (rate / freq );
	clk_freq->scr  = 0;
	clk_freq->cpsdvsr = 0;
	clk_freq->cfdf = clk_freq->cfdf ? clk_freq->cfdf : 2;
	clk_freq->cfdf = 10;//[ok]40,90,180;[err]20
	printk("spi clock cfdf=%d\n",clk_freq->cfdf);

	return 0;
}

static void imap_ssp_cs_control(u32 control)
{
	return;
	printk(KERN_ERR "ssp:::::::cs--->%d\n", !!control);
	imapx_pad_set_mode(74, "output");
	imapx_pad_set_value(74, !!control);
}

/*
 * A piece of default chip info unless the platform
 * supplies it.
 */
static const struct pl022_config_chip pl022_default_chip_info = {
	//.com_mode = POLLING_TRANSFER,
	.com_mode = DMA_TRANSFER,
	.iface = SSP_INTERFACE_MOTOROLA_SPI,
	.hierarchy = SSP_SLAVE,
	.slave_tx_disable = DO_NOT_DRIVE_TX,
	.rx_lev_trig = SSP_RX_1_OR_MORE_ELEM,
	.tx_lev_trig = SSP_TX_1_OR_MORE_EMPTY_LOC,
	.ctrl_len = SSP_BITS_8,
	.wait_state = SSP_MWIRE_WAIT_ZERO,
	.duplex = SSP_MICROWIRE_CHANNEL_FULL_DUPLEX,
	.cs_control = imap_ssp_cs_control,
};

/**
 * pl022_setup - setup function registered to SPI master framework
 * @spi: spi device which is requesting setup
 *
 * This function is registered to the SPI framework for this SPI master
 * controller. If it is the first time when setup is called by this device,
 * this function will initialize the runtime state for this chip and save
 * the same in the device structure. Else it will update the runtime info
 * with the updated chip info. Nothing is really being written to the
 * controller hardware here, that is not done until the actual transfer
 * commence.
 */
static int pl022_setup(struct spi_device *spi)
{
	struct pl022_config_chip const *chip_info;
	struct chip_data *chip;
	struct ssp_clock_params clk_freq = { .cpsdvsr = 0, .scr = 0};
	int status = 0;
	struct pl022 *pl022 = spi_master_get_devdata(spi->master);
	unsigned int bits = spi->bits_per_word;
	u32 tmp;

	if (!spi->max_speed_hz)
		return -EINVAL;

	/* Get controller_state if one is supplied */
	chip = spi_get_ctldata(spi);

	if (chip == NULL) {
		chip = kzalloc(sizeof(struct chip_data), GFP_KERNEL);
		if (!chip) {
			dev_err(&spi->dev,
				"cannot allocate controller state\n");
			return -ENOMEM;
		}
		dev_dbg(&spi->dev,
			"allocated memory for controller's runtime state\n");
	}

	/* Get controller data if one is supplied */
	chip_info = spi->controller_data;

	if (chip_info == NULL) {
		chip_info = &pl022_default_chip_info;
		printk("================%s,%d\n",__func__,__LINE__);
		/* spi_board_info.controller_data not is supplied */
		dev_dbg(&spi->dev,
			"using default controller_data settings\n");
	} else{
		dev_dbg(&spi->dev,
			"using user supplied controller_data settings\n");
		printk("================%s,%d\n",__func__,__LINE__);
	}

	/*
	 * We can override with custom divisors, else we use the board
	 * frequency setting
	 */
	if ((0 == chip_info->clk_freq.cpsdvsr)
	    && (0 == chip_info->clk_freq.scr)) {
		status = calculate_effective_freq(pl022,
						  spi->max_speed_hz,
						  &clk_freq);
		if (status < 0)
			goto err_config_params;
	} else {
		memcpy(&clk_freq, &chip_info->clk_freq, sizeof(clk_freq));
		if ((clk_freq.cpsdvsr % 2) != 0)
			clk_freq.cpsdvsr =
				clk_freq.cpsdvsr - 1;
	}
	if ((clk_freq.cfdf < CPSDVR_MIN)
	    || (clk_freq.cfdf > CPSDVR_MAX)) {
		status = -EINVAL;
		dev_err(&spi->dev,
			"cfdf is configured incorrectly\n");
		goto err_config_params;
	}

	status = verify_controller_parameters(pl022, chip_info);
	if (status) {
		dev_err(&spi->dev, "controller data is incorrect");
		goto err_config_params;
	}

	pl022->rx_lev_trig = chip_info->rx_lev_trig;
	pl022->tx_lev_trig = chip_info->tx_lev_trig;

	/* Now set controller state based on controller data */
	chip->xfer_type = chip_info->com_mode;
	if (!chip_info->cs_control) {
		chip->cs_control = null_cs_control;
		dev_warn(&spi->dev,
			 "chip select function is NULL for this chip\n");
	} else
		chip->cs_control = chip_info->cs_control;

	/* Check bits per word with vendor specific range */
	if ((bits <= 3) || (bits > pl022->vendor->max_bpw)) {
		status = -ENOTSUPP;
		dev_err(&spi->dev, "illegal data size for this controller!\n");
		dev_err(&spi->dev, "This controller can only handle 4 <= n <= %d bit words\n",
				pl022->vendor->max_bpw);
		goto err_config_params;
	} else if (bits <= 8) {
		dev_dbg(&spi->dev, "4 <= n <=8 bits per word\n");
		dev_err(&spi->dev, "4 <= n <=8 bits per word\n");
		chip->n_bytes = 1;
		chip->read = READING_U8;
		chip->write = WRITING_U8;
	} else if (bits <= 16) {
		dev_dbg(&spi->dev, "9 <= n <= 16 bits per word\n");
		dev_err(&spi->dev, "9 <= n <= 16 bits per word\n");
		chip->n_bytes = 2;
		chip->read = READING_U16;
		chip->write = WRITING_U16;
	} else {
		dev_dbg(&spi->dev, "17 <= n <= 32 bits per word\n");
		dev_err(&spi->dev, "17 <= n <= 32 bits per word\n");
		chip->n_bytes = 4;
		chip->read = READING_U32;
		chip->write = WRITING_U32;
	}

	/* Now Initialize all register settings required for this chip */
	chip->dmacr = 0;
	chip->cfdf = 0;
	(pl022->master_info)->enable_dma = 1;
	if ((chip_info->com_mode == DMA_TRANSFER)
	    && ((pl022->master_info)->enable_dma)) {
		printk("==========%s,%d\n",__func__,__LINE__);
		chip->enable_dma = true;
		dev_dbg(&spi->dev, "DMA mode set in controller state\n");
		SSP_WRITE_BITS(chip->dmacr, SSP_DMA_ENABLED,
			       SPI_DMACR_MASK_RXDMAE, 0);
		SSP_WRITE_BITS(chip->dmacr, SSP_DMA_ENABLED,
			       SPI_DMACR_MASK_TXDMAE, 1);
	} else {
		chip->enable_dma = false;
		printk("==========%s,%d,mode=%d,dma=%d\n",__func__,__LINE__,chip_info->com_mode,(pl022->master_info)->enable_dma);
		dev_dbg(&spi->dev, "DMA mode NOT set in controller state\n");
		SSP_WRITE_BITS(chip->dmacr, SSP_DMA_DISABLED,
			       SPI_DMACR_MASK_RXDMAE, 0);
		SSP_WRITE_BITS(chip->dmacr, SSP_DMA_DISABLED,
			       SPI_DMACR_MASK_TXDMAE, 1);
	}

	chip->cfdf = clk_freq.cfdf;

	/* Stuff that is common for all versions */
	if (spi->mode & SPI_CPOL)
		tmp = SSP_CLK_POL_IDLE_HIGH;
	else
		tmp = SSP_CLK_POL_IDLE_LOW;
	SSP_WRITE_BITS(chip->ctl, tmp, SPI_CTL_MASK_SPO, 0);

	if (spi->mode & SPI_CPHA)
		tmp = SSP_CLK_SECOND_EDGE;
	else
		tmp = SSP_CLK_FIRST_EDGE;
	SSP_WRITE_BITS(chip->ctl, tmp, SPI_CTL_MASK_SPH, 1);

	SSP_WRITE_BITS(chip->cfdf, clk_freq.cfdf, SPI_CFDF_MASK_DIV, 0);
	SSP_WRITE_BITS(chip->spi_en, SSP_DISABLED, SPI_MASK_EN, 0);

	/* Save controller_state */
	spi_set_ctldata(spi, chip);
	return status;
 err_config_params:
	spi_set_ctldata(spi, NULL);
	kfree(chip);
	return status;
}

/**
 * pl022_cleanup - cleanup function registered to SPI master framework
 * @spi: spi device which is requesting cleanup
 *
 * This function is registered to the SPI framework for this SPI master
 * controller. It will free the runtime state of chip.
 */
static void pl022_cleanup(struct spi_device *spi)
{
	struct chip_data *chip = spi_get_ctldata(spi);

	spi_set_ctldata(spi, NULL);
	kfree(chip);
}

static void imap_ssp_module_init(int bus_num)
{
	u32		core_switch = 0;	
	//module enable
	//module_power_on(SYSMGR_OTF_BASE);
	//switch to new ip core
	core_switch = readw(IO_ADDRESS(SYSMGR_OTF_BASE) + 0x20);
	writew( core_switch | 0x1 , IO_ADDRESS(SYSMGR_OTF_BASE) + 0x20);
	core_switch = readw(IO_ADDRESS(SYSMGR_OTF_BASE) + 0x20);
	//enable clock gate
	//writew(readw(IO_ADDRESS(SYSMGR_OTF_BASE) + 0x4)|0x2, IO_ADDRESS(SYSMGR_OTF_BASE)+0x4);
	writew(0xff, IO_ADDRESS(SYSMGR_OTF_BASE)+0x4);
	//choose spi clock source "vpll"
#if 0
	writel(0x3,IO_ADDRESS(SYSMGR_CLKGEN_BASE+0x2A0));
	writel(0x2,IO_ADDRESS(SYSMGR_CLKGEN_BASE+0x2A4));
	writel(0x0,IO_ADDRESS(SYSMGR_CLKGEN_BASE+0x2AC));
#endif
	//io pad switch to spi
	//writew(readw(IO_ADDRESS(SYSMGR_PAD_BASE))|(0x1<<2), IO_ADDRESS(SYSMGR_PAD_BASE));
	//imapx_pad_init("sd0");//iomux with sd0
	imapx_pad_init("ssp0");//iomux with sd0
	imapx_pad_set_mode(74, "output");
	//printk("==========cs pin test low!\n");
	//imapx_pad_set_value(74,0);
	//mdelay(3*1000);
	//printk("==========cs pin test high!\n");
	imapx_pad_set_value(74,1);
}

static int pl022_probe(struct amba_device *adev, const struct amba_id *id)
{
	struct device *dev = &adev->dev;
	struct pl022_ssp_controller *platform_info = adev->dev.platform_data;
	struct spi_master *master;
	struct pl022 *pl022 = NULL;	/*Data for this driver */
	int status = 0;

	dev_err(&adev->dev,
		 "Infotm multi wire spi driver, device ID: 0x%08x\n", adev->periphid);
	printk("================platform_info=0x%x\n",adev->dev.platform_data);
	if (platform_info == NULL) {
		dev_err(&adev->dev, "probe - no platform data supplied\n");
		status = -ENODEV;
		goto err_no_pdata;
	}

	/* Allocate master with space for data */
	master = spi_alloc_master(dev, sizeof(struct pl022));
	if (master == NULL) {
		dev_err(&adev->dev, "probe - cannot alloc SPI master\n");
		status = -ENOMEM;
		goto err_no_master;
	}

	pl022 = spi_master_get_devdata(master);
	pl022->master = master;
	pl022->master_info = platform_info;
	pl022->adev = adev;
	pl022->vendor = id->data;

	/*
	 * Bus Number Which has been Assigned to this SSP controller
	 * on this board
	 */
	master->bus_num = platform_info->bus_id;
	master->num_chipselect = platform_info->num_chipselect;
	master->cleanup = pl022_cleanup;
	master->setup = pl022_setup;
	master->prepare_transfer_hardware = pl022_prepare_transfer_hardware;
	master->transfer_one_message = pl022_transfer_one_message;
	master->unprepare_transfer_hardware = pl022_unprepare_transfer_hardware;
	master->rt = platform_info->rt;

	/*
	 * Supports mode 0-3, loopback, and active low CS. Transfers are
	 * always MS bit first on the original pl022.
	 */
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH ;
	if (pl022->vendor->extended_cr)
		master->mode_bits |= SPI_LSB_FIRST;

	dev_dbg(&adev->dev, "BUSNO: %d\n", master->bus_num);

	imap_ssp_module_init(master->bus_num);
	status = amba_request_regions(adev, NULL);
	if (status)
		goto err_no_ioregion;

	pl022->phybase = adev->res.start;
	pl022->virtbase = ioremap(adev->res.start, resource_size(&adev->res));
	if (pl022->virtbase == NULL) {
		status = -ENOMEM;
		goto err_no_ioremap;
	}
	printk(KERN_INFO "spi-imapxq3f: mapped registers from 0x%x to %p\n",
	       adev->res.start, pl022->virtbase);

	pl022->clk = clk_get_sys(dev_name(&adev->dev), "imap-spi124");
	//pl022->clk = clk_get_sys(dev_name(&adev->dev), "imap-ssp");
	if (IS_ERR(pl022->clk)) {
		status = PTR_ERR(pl022->clk);
		dev_err(&adev->dev, "could not retrieve SSP/SPI bus clock\n");
		goto err_no_clk;
	}

	status = clk_prepare(pl022->clk);
	if (status) {
		dev_err(&adev->dev, "could not prepare SSP/SPI bus clock\n");
		goto  err_clk_prep;
	}

	status = clk_enable(pl022->clk);
	if (status) {
		dev_err(&adev->dev, "could not enable SSP/SPI bus clock\n");
		goto err_no_clk_en;
	}
	/* For spi multi wire,we should enable another register */

	/* Initialize transfer pump */
	tasklet_init(&pl022->pump_transfers, pump_transfers,
		     (unsigned long)pl022);

	/* Disable SSP */
	//writew(SSP_DISABLED,SPI_EN(pl022->virtbase));
	//test_modify_controller_register(pl022);
	load_ssp_default_config(pl022);

#if 1
	status = request_irq(adev->irq[0], pl022_interrupt_handler_or, 0, "pl022",
			     pl022);
	if (status < 0) {
		dev_err(&adev->dev, "probe - cannot get IRQ (%d)\n", status);
		//goto err_no_irq;
	}
	//enable_irq(adev->irq[0]);
#endif
	writew(DISABLE_ALL_INTERRUPTS, SPI_IMSC(pl022->virtbase));
	writew(0x1<<4, SPI_IMSC(pl022->virtbase));

	/* Get DMA channels */
	printk("=========================================dma=%d\n",platform_info->enable_dma);
	if (platform_info->enable_dma) {
		status = pl022_dma_probe(pl022);
		if (status != 0)
			platform_info->enable_dma = 0;
	}

	/* Register with the SPI framework */
	amba_set_drvdata(adev, pl022);
	status = spi_register_master(master);
	if (status != 0) {
		dev_err(&adev->dev,
			"probe - problem registering spi master\n");
		goto err_spi_register;
	}
	dev_dbg(dev, "probe succeeded\n");

	/* let runtime pm put suspend */
	if (platform_info->autosuspend_delay > 0) {
		dev_info(&adev->dev,
			"will use autosuspend for runtime pm, delay %dms\n",
			platform_info->autosuspend_delay);
		pm_runtime_set_autosuspend_delay(dev,
			platform_info->autosuspend_delay);
		pm_runtime_use_autosuspend(dev);
		pm_runtime_put_autosuspend(dev);
	} else {
		pm_runtime_put(dev);
	}
	dev_err(&adev->dev, "!!!!!!!!!!!! %s: probe OK", __func__);
	return 0;

 err_spi_register:
	if (platform_info->enable_dma)
		pl022_dma_remove(pl022);

	free_irq(adev->irq[0], pl022);
 //err_no_irq:
//	clk_disable(pl022->clk);
 err_no_clk_en:
	clk_unprepare(pl022->clk);
 err_clk_prep:
	clk_put(pl022->clk);
 err_no_clk:
	iounmap(pl022->virtbase);
 err_no_ioremap:
	amba_release_regions(adev);
 err_no_ioregion:
	spi_master_put(master);
 err_no_master:
 err_no_pdata:
	return status;
}

static int pl022_remove(struct amba_device *adev)
{
	struct pl022 *pl022 = amba_get_drvdata(adev);

	if (!pl022)
		return 0;

	/*
	 * undo pm_runtime_put() in probe.  I assume that we're not
	 * accessing the primecell here.
	 */
	pm_runtime_get_noresume(&adev->dev);

	load_ssp_default_config(pl022);
	if (pl022->master_info->enable_dma)
		pl022_dma_remove(pl022);

	free_irq(adev->irq[0], pl022);
	clk_disable(pl022->clk);
	clk_unprepare(pl022->clk);
	clk_put(pl022->clk);
	iounmap(pl022->virtbase);
	amba_release_regions(adev);
	tasklet_disable(&pl022->pump_transfers);
	spi_unregister_master(pl022->master);
	spi_master_put(pl022->master);
	amba_set_drvdata(adev, NULL);
	return 0;
}

#ifdef CONFIG_SUSPEND
static int pl022_suspend(struct device *dev)
{
	struct pl022 *pl022 = dev_get_drvdata(dev);
	int ret;

	ret = spi_master_suspend(pl022->master);
	if (ret) {
		dev_warn(dev, "cannot suspend master\n");
		return ret;
	}

	dev_dbg(dev, "suspended\n");
	return 0;
}

static int pl022_resume(struct device *dev)
{
	struct pl022 *pl022 = dev_get_drvdata(dev);
	int ret;

	module_power_on(SYSMGR_OTF_BASE);
	/* Start the queue running */
	ret = spi_master_resume(pl022->master);
	if (ret)
		dev_err(dev, "problem starting queue (%d)\n", ret);
	else
		dev_dbg(dev, "resumed\n");

	return ret;
}
#endif	/* CONFIG_PM */

#ifdef CONFIG_PM_RUNTIME
static int pl022_runtime_suspend(struct device *dev)
{
	struct pl022 *pl022 = dev_get_drvdata(dev);

	clk_disable(pl022->clk);

	return 0;
}

static int pl022_runtime_resume(struct device *dev)
{
	struct pl022 *pl022 = dev_get_drvdata(dev);

	clk_enable(pl022->clk);

	return 0;
}
#endif

static const struct dev_pm_ops pl022_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pl022_suspend, pl022_resume)
	SET_RUNTIME_PM_OPS(pl022_runtime_suspend, pl022_runtime_resume, NULL)
};

static struct vendor_data vendor_imapxq3f = {
	.fifodepth = 64,
	.max_bpw = 32,
	.unidir = false,
	.extended_cr = false,
	.pl023 = false,
	.loopback = false,
};

static struct vendor_data vendor_arm = {
	.fifodepth = 8,
	.max_bpw = 16,
	.unidir = false,
	.extended_cr = false,
	.pl023 = false,
	.loopback = true,
};

static struct vendor_data vendor_st = {
	.fifodepth = 32,
	.max_bpw = 32,
	.unidir = false,
	.extended_cr = true,
	.pl023 = false,
	.loopback = true,
};

static struct vendor_data vendor_st_pl023 = {
	.fifodepth = 32,
	.max_bpw = 32,
	.unidir = false,
	.extended_cr = true,
	.pl023 = true,
	.loopback = false,
};

static struct vendor_data vendor_db5500_pl023 = {
	.fifodepth = 32,
	.max_bpw = 32,
	.unidir = false,
	.extended_cr = true,
	.pl023 = true,
	.loopback = true,
};

static struct amba_id pl022_ids[] = {
	{
		/*
		 * ARM PL022 variant, this has a 16bit wide
		 * and 8 locations deep TX/RX FIFO
		 */
		.id	= 0x00041022,
		.mask	= 0x000fffff,
		.data	= &vendor_arm,
	},
	{
		/*
		 * ARM PL022 variant, this has a 16bit wide
		 * and 8 locations deep TX/RX FIFO
		 */
		.id	= 0x00051007,
		.mask	= 0xffffffff,
		.data	= &vendor_imapxq3f,
	},
	{
		/*
		 * ST Micro derivative, this has 32bit wide
		 * and 32 locations deep TX/RX FIFO
		 */
		.id	= 0x01080022,
		.mask	= 0xffffffff,
		.data	= &vendor_st,
	},
	{
		/*
		 * ST-Ericsson derivative "PL023" (this is not
		 * an official ARM number), this is a PL022 SSP block
		 * stripped to SPI mode only, it has 32bit wide
		 * and 32 locations deep TX/RX FIFO but no extended
		 * CR0/CR1 register
		 */
		.id	= 0x00080023,
		.mask	= 0xffffffff,
		.data	= &vendor_st_pl023,
	},
	{
		.id	= 0x10080023,
		.mask	= 0xffffffff,
		.data	= &vendor_db5500_pl023,
	},
	{ 0, 0 },
};

MODULE_DEVICE_TABLE(amba, pl022_ids);

static struct amba_driver pl022_driver = {
	.drv = {
		.name	= "imap-ssp",
		.pm	= &pl022_dev_pm_ops,
	},
	.id_table	= pl022_ids,
	.probe		= pl022_probe,
	.remove		= pl022_remove,
};

static int __init pl022_init(void)
{
	module_power_on(SYSMGR_OTF_BASE);
	return amba_driver_register(&pl022_driver);
}
subsys_initcall(pl022_init);

static void __exit pl022_exit(void)
{
	amba_driver_unregister(&pl022_driver);
}
module_exit(pl022_exit);

MODULE_AUTHOR("Linus Walleij <linus.walleij@stericsson.com>");
MODULE_DESCRIPTION("PL022 SSP Controller Driver");
MODULE_LICENSE("GPL");
