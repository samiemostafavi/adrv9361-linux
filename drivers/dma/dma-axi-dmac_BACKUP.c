/*
 * Driver for the Analog Devices AXI-DMAC core
 *
 * Copyright 2013-2015 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/time.h>

#include <dt-bindings/dma/axi-dmac.h>

#include "dmaengine.h"
#include "virt-dma.h"

#include <linux/uaccess.h>

/*
 * The AXI-DMAC is a soft IP core that is used in FPGA designs. The core has
 * various instantiation parameters which decided the exact feature set support
 * by the core.
 *
 * Each channel of the core has a source interface and a destination interface.
 * The number of channels and the type of the channel interfaces is selected at
 * configuration time. A interface can either be a connected to a central memory
 * interconnect, which allows access to system memory, or it can be connected to
 * a dedicated bus which is directly connected to a data port on a peripheral.
 * Given that those are configuration options of the core that are selected when
 * it is instantiated this means that they can not be changed by software at
 * runtime. By extension this means that each channel is uni-directional. It can
 * either be device to memory or memory to device, but not both. Also since the
 * device side is a dedicated data bus only connected to a single peripheral
 * there is no address than can or needs to be configured for the device side.
 */

#define AXI_DMAC_REG_VERSION		0x00

#define AXI_DMAC_REG_IRQ_MASK		0x80
#define AXI_DMAC_REG_IRQ_PENDING	0x84
#define AXI_DMAC_REG_IRQ_SOURCE		0x88

#define AXI_DMAC_REG_CTRL		0x400
#define AXI_DMAC_REG_TRANSFER_ID	0x404
#define AXI_DMAC_REG_START_TRANSFER	0x408
#define AXI_DMAC_REG_FLAGS		0x40c
#define AXI_DMAC_REG_DEST_ADDRESS	0x410
#define AXI_DMAC_REG_SRC_ADDRESS	0x414
#define AXI_DMAC_REG_X_LENGTH		0x418
#define AXI_DMAC_REG_Y_LENGTH		0x41c
#define AXI_DMAC_REG_DEST_STRIDE	0x420
#define AXI_DMAC_REG_SRC_STRIDE		0x424
#define AXI_DMAC_REG_TRANSFER_DONE	0x428
#define AXI_DMAC_REG_ACTIVE_TRANSFER_ID 0x42c
#define AXI_DMAC_REG_STATUS		0x430
#define AXI_DMAC_REG_CURRENT_SRC_ADDR	0x434
#define AXI_DMAC_REG_CURRENT_DEST_ADDR	0x438
#define AXI_DMAC_REG_DBG0		0x43c
#define AXI_DMAC_REG_DBG1		0x440 // 110
#define AXI_DMAC_REG_DBG2		0x444 // 111
//#define AXI_DMAC_REG_DBG3		0x448 // up_measured_transfer_length 112
#define AXI_DMAC_REG_PARTIAL_XFER_LEN	0x44c // Len 113
#define AXI_DMAC_REG_PARTIAL_XFER_ID	0x450 // ID 114

#define AXI_DMAC_REG_TIMESTAMPA0_LSB	0x454 // Read Timestamp Register Address 115 in HW
#define AXI_DMAC_REG_TIMESTAMPA0_MSB	0x458 // Read Timestamp Register Address 116 in HW
#define AXI_DMAC_REG_TIMESTAMPA1_LSB	0x45c // Read Timestamp Register Address 117 in HW
#define AXI_DMAC_REG_TIMESTAMPA1_MSB	0x460 // Read Timestamp Register Address 118 in HW
#define AXI_DMAC_REG_TIMESTAMPA2_LSB	0x464 // Read Timestamp Register Address 119 in HW
#define AXI_DMAC_REG_TIMESTAMPA2_MSB	0x468 // Read Timestamp Register Address 11a in HW
#define AXI_DMAC_REG_TIMESTAMPA3_LSB	0x46c // Read Timestamp Register Address 11b in HW
#define AXI_DMAC_REG_TIMESTAMPA3_MSB	0x470 // Read Timestamp Register Address 11c in HW

#define AXI_DMAC_REG_TIMESTAMPB0_LSB	0x474 // Read Timestamp Register Address 11d in HW
#define AXI_DMAC_REG_TIMESTAMPB0_MSB	0x478 // Read Timestamp Register Address 11e in HW
#define AXI_DMAC_REG_TIMESTAMPB1_LSB	0x47c // Read Timestamp Register Address 11f in HW
#define AXI_DMAC_REG_TIMESTAMPB1_MSB	0x480 // Read Timestamp Register Address 120 in HW
#define AXI_DMAC_REG_TIMESTAMPB2_LSB	0x484 // Read Timestamp Register Address 121 in HW
#define AXI_DMAC_REG_TIMESTAMPB2_MSB	0x488 // Read Timestamp Register Address 122 in HW
#define AXI_DMAC_REG_TIMESTAMPB3_LSB	0x48c // Read Timestamp Register Address 123 in HW
#define AXI_DMAC_REG_TIMESTAMPB3_MSB	0x490 // Read Timestamp Register Address 124 in HW


#define AXI_DMAC_CTRL_ENABLE		BIT(0)
#define AXI_DMAC_CTRL_PAUSE		BIT(1)

#define AXI_DMAC_IRQ_SOT		BIT(0)
#define AXI_DMAC_IRQ_EOT		BIT(1)

#define AXI_DMAC_FLAG_CYCLIC		BIT(0)
#define AXI_DMAC_FLAG_LAST		BIT(1)
#define AXI_DMAC_FLAG_PARTIAL_REPORT	BIT(2)

#define AXI_DMAC_FLAG_PARTIAL_XFER_DONE BIT(31)

#undef SPEED_TEST

/* The maximum ID allocated by the hardware is 31 */
#define AXI_DMAC_SG_UNUSED 32U

#define GPIO_BASEADDR   0x43c00000

#define GPIO0_DATA0     0x43c00000      // Enable signals
#define GPIO0_DATA1     0x43c00004      // Valid signals

#define GPIO1_DATA0     0x43c00008      // tx timestamp lsb
#define GPIO1_DATA1     0x43c0000c      // tx timestamp msb

#define GPIO2_DATA0     0x43c00010      // tx timestamp lsb
#define GPIO2_DATA1     0x43c00014      // tx timestamp msb

#define GPIO3_DATA0     0x43c00018      // tx timestamp lsb
#define GPIO3_DATA1     0x43c0001c      // tx timestamp msb

#define GPIO4_DATA0     0x43c00020      // tx timestamp lsb
#define GPIO4_DATA1     0x43c00024      // tx timestamp msb

#define aTIMER_TCR0     0x43c00028      // address of timer 0's counter register
#define aTIMER_TCR1     0x43c0002c      // address of timer 1's counter register

unsigned long* pGPIO0_DATA0; // Enable signal
unsigned long* pGPIO0_DATA1; // Valid signal

unsigned long* pGPIO1_DATA0; // tx timestamp lsb
unsigned long* pGPIO1_DATA1; // tx timestamp msb

unsigned long* pGPIO2_DATA0; // tx timestamp lsb
unsigned long* pGPIO2_DATA1; // tx timestamp msb

unsigned long* pGPIO3_DATA0; // tx timestamp lsb
unsigned long* pGPIO3_DATA1; // tx timestamp msb

unsigned long* pGPIO4_DATA0; // tx timestamp lsb
unsigned long* pGPIO4_DATA1; // tx timestamp msb

unsigned long *paTIMER_TCR0;             // pointer to timer 0 counter register
unsigned long *paTIMER_TCR1;             // pointer to timer 1 counter register

struct axi_dmac_sg {
	dma_addr_t src_addr;
	dma_addr_t dest_addr;
	unsigned int x_len;
	unsigned int y_len;
	unsigned int dest_stride;
	unsigned int src_stride;
	unsigned int id;
	unsigned int partial_len;
	bool last;
	bool schedule_when_free;
};

struct axi_dmac_desc {
	struct virt_dma_desc vdesc;
	bool cyclic;
	bool have_partial_xfer;

	unsigned int num_submitted;
	unsigned int num_completed;
	unsigned int num_sgs;
	struct axi_dmac_sg sg[];
};

struct axi_dmac_chan {
	struct virt_dma_chan vchan;

	struct axi_dmac_desc *next_desc;
	struct list_head active_descs;
	enum dma_transfer_direction direction;

	unsigned int src_width;
	unsigned int dest_width;
	unsigned int src_type;
	unsigned int dest_type;

	unsigned int max_length;
	unsigned int address_align_mask;
	unsigned int length_align_mask;

	bool hw_partial_xfer;
	bool hw_cyclic;
	bool hw_2d;
};

struct axi_dmac {
	void __iomem *base;
	int irq;

	struct clk *clk;

	struct dma_device dma_dev;
	struct axi_dmac_chan chan;

	struct device_dma_parameters dma_parms;

#ifdef SPEED_TEST
	void *test_virt;
	dma_addr_t test_phys;
#endif
};

#define TRANSFER_DELAY_TS 500 	// 5 us
#define LATE_DURATION_TS 3000 	// 30 us
#define TRNSF_DURATION_TS 50000 // 500 us

enum tx_transfer_type {
	notimestamp, 
	late,
	early,
	good
};

struct tx_transfer_cmd {
	uint64_t timestamp;
	uint64_t duration;
	uint16_t driver_id;
        uint16_t size_less;
} tx_cmds[4];
EXPORT_SYMBOL_GPL(tx_cmds);

struct tx_transfer_res {
	enum tx_transfer_type type;
	uint64_t est_finish_time;
} tx_ress[4];

struct tx_transfer_stat {
	uint16_t idcounter;
	uint16_t lates;
	uint16_t earlies;
	uint16_t drops;
	int32_t sot_cmd;
} tx_stat;
EXPORT_SYMBOL_GPL(tx_stat);

uint64_t rx_timestamps[4];
EXPORT_SYMBOL_GPL(rx_timestamps);

uint32_t rxidcounters[4];
EXPORT_SYMBOL_GPL(rxidcounters);

static uint32_t rxidcounter = 0;
static uint16_t txidcounter = 0;
static uint16_t acclates = 0;
static uint16_t accearlies = 0;
static uint16_t accdrops = 0;
uint64_t timehwPrev = 0;


static struct axi_dmac *chan_to_axi_dmac(struct axi_dmac_chan *chan)
{
	return container_of(chan->vchan.chan.device, struct axi_dmac,
		dma_dev);
}

static struct axi_dmac_chan *to_axi_dmac_chan(struct dma_chan *c)
{
	return container_of(c, struct axi_dmac_chan, vchan.chan);
}

static struct axi_dmac_desc *to_axi_dmac_desc(struct virt_dma_desc *vdesc)
{
	return container_of(vdesc, struct axi_dmac_desc, vdesc);
}

static void axi_dmac_write(struct axi_dmac *axi_dmac, unsigned int reg,
	unsigned int val)
{
	writel(val, axi_dmac->base + reg);
}

static int axi_dmac_read(struct axi_dmac *axi_dmac, unsigned int reg)
{
	return readl(axi_dmac->base + reg);
}

static int axi_dmac_src_is_mem(struct axi_dmac_chan *chan)
{
	return chan->src_type == AXI_DMAC_BUS_TYPE_AXI_MM;
}

static int axi_dmac_dest_is_mem(struct axi_dmac_chan *chan)
{
	return chan->dest_type == AXI_DMAC_BUS_TYPE_AXI_MM;
}

static bool axi_dmac_check_len(struct axi_dmac_chan *chan, unsigned int len)
{
	if (len == 0)
		return false;
	if ((len & chan->length_align_mask) != 0) /* Not aligned */
		return false;
	return true;
}

static bool axi_dmac_check_addr(struct axi_dmac_chan *chan, dma_addr_t addr)
{
	if ((addr & chan->address_align_mask) != 0) /* Not aligned */
		return false;
	return true;
}


static uint64_t axi_dmac_axitimer_read(void)
{
        uint32_t upper_val_tmp;
        uint32_t upper_val;
        uint32_t lower_val;
        uint64_t timerval;

        // read timer value
        upper_val_tmp = ioread32(paTIMER_TCR1);                  // Read the upper velue
        lower_val = ioread32(paTIMER_TCR0);                      // Read the lower value

        upper_val = ioread32(paTIMER_TCR1);                      // Read the upper velue again
        if(upper_val != upper_val_tmp)                          // If the value is different, read lower part again
                lower_val = ioread32(paTIMER_TCR0);              // Read the lower velue again

        timerval = (uint64_t) upper_val << 32 | lower_val;      // Combine 2

        return timerval;
}

static int acc_overlapping_samples = 0;

static void axi_dmac_start_transfer(struct axi_dmac_chan *chan)
{
	struct axi_dmac *dmac = chan_to_axi_dmac(chan);
	struct virt_dma_desc *vdesc;
	struct axi_dmac_desc *desc;
	struct axi_dmac_sg *sg;
	unsigned int flags = 0;
	unsigned int val;
	uint32_t read_timestamp_lsb;
	uint32_t read_timestamp_msb;
	//uint64_t read_timestamp;
	bool is_cyclic = false;
	int dev_id;
	uint8_t regVal = 0;
	unsigned int transfer_id_eot = 0;
	int blck_front_id = 0;
	struct timeval tv;
	unsigned long ts_micro = 0;
	char stat[10];
	int repsize = 0;
	int frame_size_samples = 0;
	uint64_t timenow = 0;
	int no_overlapping_samples = 0;
	int sub_front_cur_ts = 0;
	uint64_t one_sample_ts = 0;

	// TX transfer metadata
	uint64_t txtimestamp = 0;
	uint16_t txid = 0;
	uint16_t txsize = 0;
	uint64_t txduration = 0;

	dev_id = dmac->dma_dev.dev_id;

	//printk("[DBG][drivers][dma-axi-dmac][axi_dmac_start_transfer][chan][start_counter: %d][dir: %d][cyclic: %d]\n",start_counter,chan->direction,chan->hw_cyclic);
	//printk("[DBG][drivers][dma-axi-dmac][axi_dmac_start_transfer][chan][hw_partial_xfer: %d][hw_2d: %d]\n",chan->hw_partial_xfer,chan->hw_2d);

	val = axi_dmac_read(dmac, AXI_DMAC_REG_START_TRANSFER);
	if (val) /* Queue is full, wait for the next SOT IRQ */
	{
		//printk("[queue is full]\n");
		return;
	}

	desc = chan->next_desc;

	//printk("[DBG][drivers][dma-axi-dmac][axi_dmac_start_transfer][desc][num_submitted: %d][num_completed: %d][num_sgs: %d]\n",&desc->num_submitted,&desc->num_completed,&desc->num_sgs);

	if (!desc) {
		vdesc = vchan_next_desc(&chan->vchan);
		if (!vdesc)
			return;
		
		list_move_tail(&vdesc->node, &chan->active_descs);
		desc = to_axi_dmac_desc(vdesc);
	}
	sg = &desc->sg[desc->num_submitted];

	/* Already queued in cyclic mode. Wait for it to finish */
	if (sg->id != AXI_DMAC_SG_UNUSED) {
		sg->schedule_when_free = true;
		return;
	}

	desc->num_submitted++;
	if (desc->num_submitted == desc->num_sgs ||
	    desc->have_partial_xfer) {
		if (desc->cyclic) {
			desc->num_submitted = 0; /* Start again */
		} else {
			chan->next_desc = NULL;
		}
		flags |= AXI_DMAC_FLAG_LAST;
	} else {
		chan->next_desc = desc;
	}

	sg->id = axi_dmac_read(dmac, AXI_DMAC_REG_TRANSFER_ID);

	if (axi_dmac_dest_is_mem(chan)) {
		// RX	
		axi_dmac_write(dmac, AXI_DMAC_REG_DEST_ADDRESS, sg->dest_addr);
		axi_dmac_write(dmac, AXI_DMAC_REG_DEST_STRIDE, sg->dest_stride);
	}

	if (axi_dmac_src_is_mem(chan)) {
		// TX
		axi_dmac_write(dmac, AXI_DMAC_REG_SRC_ADDRESS, sg->src_addr);
		axi_dmac_write(dmac, AXI_DMAC_REG_SRC_STRIDE, sg->src_stride);
	}

	/*
	 * If the hardware supports cyclic transfers and there is no callback to
	 * call and only a single segment, enable hw cyclic mode to avoid
	 * unnecessary interrupts.
	 */
	if (chan->hw_cyclic && desc->cyclic && !desc->vdesc.tx.callback &&
		desc->num_sgs == 1)
	{
		is_cyclic = true;
		flags |= AXI_DMAC_FLAG_CYCLIC;
	}

	if (chan->hw_partial_xfer)
	{
		flags |= AXI_DMAC_FLAG_PARTIAL_REPORT;
	}

	//printk("[DBG][drivers][dma-axi-dmac][axi_dmac_start_transfer][sg->x_len: %d][sg->y_len: %d]\n",sg->x_len-1,sg->y_len-1);
	//printk("[DBG][drivers][dma-axi-dmac] TX buffer before transmit timestamp:%llu, size:%d \n", dac_inter_timestamp, (int)sg->x_len);


	// If the device is not HDMI and it is TX:
        if(chan->direction == 1 && (dev_id == 0 || dev_id == 1))
        {
		txtimestamp = tx_cmds[sg->id].timestamp;
		txid = tx_cmds[sg->id].driver_id;
        	txsize = tx_cmds[sg->id].size_less;
		txduration = tx_cmds[sg->id].duration;
		repsize = sg->x_len -1 -(6*4) -(txsize*4);
		frame_size_samples = (repsize+1)>>2;
		one_sample_ts = (int)txduration/(int)frame_size_samples;
		
		// Check packet drops by block ID
		if(txid - txidcounter > 1)
		{
			accdrops += txid-txidcounter-1;
			//printk("Dropped packet in driver: %d - %d = %d, total drops: %d\n",txid,txidcounter,txid-txidcounter-1,accdrops);
		}

		// if variable has reached the end
		if(txid < txidcounter)
		{
			accdrops = 0;
			acclates = 0;
			accearlies = 0;
		}
		txidcounter = txid;
		
		// Check timing TX
		transfer_id_eot = axi_dmac_read(dmac,AXI_DMAC_REG_ACTIVE_TRANSFER_ID); 	// active block id
		blck_front_id = ((((int)sg->id)-1>=0))?(((int)sg->id)-1):(3); // id of the block in front
		// Get the current timestamp
		timenow = axi_dmac_axitimer_read();
		if(txtimestamp == 0) // Set NOTS flag to send the block immediately
		{
			// No Timestamp
			strncpy(stat,"NOTS_NOTS",10);
			tx_ress[sg->id].type = notimestamp;
			// Calculate finish time
			if(sg->id == transfer_id_eot) // There is no blocks in front of us
				tx_ress[sg->id].est_finish_time = timenow + TRANSFER_DELAY_TS + txduration;
			else // There is a block in front of us
				tx_ress[sg->id].est_finish_time = tx_ress[blck_front_id].est_finish_time + TRANSFER_DELAY_TS + txduration;
		}
		else
		{
			// Check LATE, EARLY or GOOD
			// In the case that the clock is 100MHz
			if(txtimestamp > timenow+(400000)) // Set EARLY flag to drop the block. 4 blocks can be tolerated otherwise iio_push will be blocked (4 ms)
			{
				// Check early TX
				strncpy(stat,"EARL_EARL",10);
				accearlies++;
				tx_ress[sg->id].type = early;
				if(sg->id == transfer_id_eot) // There is no blocks in front of us
					tx_ress[sg->id].est_finish_time = timenow + TRANSFER_DELAY_TS;
				else // There is a block in front of us
					tx_ress[sg->id].est_finish_time = tx_ress[blck_front_id].est_finish_time + TRANSFER_DELAY_TS;
			}
			else
			{
				// Check late TX
				if(sg->id == transfer_id_eot) // There is no blocks in front of us
				{
					if(txtimestamp < timenow + TRNSF_DURATION_TS) // According to tests, TRNSF_DURATION_TS 500 us = 50000 ts is needed
					{
						strncpy(stat,"LATE_NOFR",10);
						acclates++;
						tx_ress[sg->id].type = late;
						tx_ress[sg->id].est_finish_time = timenow + LATE_DURATION_TS;
					}
					else
					{
						strncpy(stat,"GOOD_NOFR",10);
						tx_ress[sg->id].type = good;
						tx_ress[sg->id].est_finish_time = txduration + txtimestamp;
					}
				}
				else // There is a block in front of us
				{
					// We allow less than one sample overlapping
					// These lates accumulate overtime and it has to be handled when it gets larger than one sample
					if(txtimestamp < timenow + TRNSF_DURATION_TS) // According to tests, TRNSF_DURATION_TS 500 us = 50000 ts is needed
					{
						strncpy(stat,"LATE_WIFR",10);
                                                acclates++;
                                                tx_ress[sg->id].type = late;
                                                tx_ress[sg->id].est_finish_time = tx_ress[blck_front_id].est_finish_time + LATE_DURATION_TS;
					}
					else if(txtimestamp < tx_ress[blck_front_id].est_finish_time + TRANSFER_DELAY_TS)
					{
						// Calculating overlapping samples
						sub_front_cur_ts = (int64_t)tx_ress[blck_front_id].est_finish_time+TRANSFER_DELAY_TS-(int64_t)txtimestamp;
						no_overlapping_samples = (int)sub_front_cur_ts/(int)one_sample_ts;

						if(no_overlapping_samples >= frame_size_samples) // More than one frame, drop the frame
						{
							strncpy(stat,"LATE_WFOL",10);
							acclates++;
							tx_ress[sg->id].type = late;
							tx_ress[sg->id].est_finish_time = tx_ress[blck_front_id].est_finish_time + LATE_DURATION_TS;
							no_overlapping_samples = 0;
						}
						else // less than one frame, drop the samples from the beginning
						{
							strncpy(stat,"GOOD_WFOL",10);
							tx_ress[sg->id].type = good;
							txduration -= no_overlapping_samples*one_sample_ts; // new duration
							tx_ress[sg->id].est_finish_time = txduration + tx_ress[blck_front_id].est_finish_time;
							acc_overlapping_samples += no_overlapping_samples;
						}
					}
					else
					{
						strncpy(stat,"GOOD_WIFR",10);
						tx_ress[sg->id].type = good;
						tx_ress[sg->id].est_finish_time = txduration + txtimestamp;
					}
				}
				
				if(tx_ress[sg->id].type == good)
				{

			 		// Appropriate Timed TX Transfer
					// Prepare timing block signals
					// TIMING 1) latch low timing controller enable signal, so everything waits
					regVal = ioread32(pGPIO0_DATA0);
					if(sg->id == 0)
					{
						// GPIO_0_DATA_0 = 0, set enable: 11111110 = 254
						regVal = regVal & 254;
					}
					else if(sg->id == 1)
					{
						// GPIO_0_DATA_0 = 0, set enable: 11111101 = 253
						regVal = regVal & 253;
					}
					else if(sg->id == 2)
					{
						// GPIO_0_DATA_0 = 0, set enable: 11111011 = 251
						regVal = regVal & 251;
					}
					else if(sg->id == 3)
					{
						// GPIO_0_DATA_0 = 0, set enable: 11110111 = 247
						regVal = regVal & 247;
					}
					iowrite32(regVal,pGPIO0_DATA0);
				}
			}
		}

		// print info
		/*if(tx_ress[sg->id].type == late)
		{
			do_gettimeofday(&tv);
			ts_micro = 1000000 * tv.tv_sec + tv.tv_usec;
			printk("Start tx %s, id: %d, txtimestamp: %llu, active_id: %d, front_id: %d, txId: %d, timenow:%llu, ts-timenow: %lld, time: %lu, repsize: %d, hw_dif: %llu, est_fin: %llu, dur: %llu, frame_size_samples: %d, no_overlapping_samples: %d, one_sample_ts: %lld, acc_overlapping_samples: %d, sub_front_cur_ts: %d\n",stat,sg->id,txtimestamp,transfer_id_eot,blck_front_id,txid,timenow,(int64_t)(txtimestamp-timenow),ts_micro,repsize, timenow-timehwPrev,tx_ress[sg->id].est_finish_time,txduration,frame_size_samples, no_overlapping_samples,one_sample_ts, acc_overlapping_samples, sub_front_cur_ts);
		}
		timehwPrev = timenow;*/

		// Save drops,lates, earlies, and txidcounter numbers into the global struct
		tx_stat.drops = accdrops;
		tx_stat.lates = acclates;
		tx_stat.earlies = accearlies;
		tx_stat.idcounter = txidcounter;
        }
	
	// If it is not HDMI and it is RX	
        /*if(chan->direction == 2 && (dev_id == 0 || dev_id == 1))
        {
		getnstimeofday(&ts);
		transfer_id_eot = axi_dmac_read(dmac,AXI_DMAC_REG_ACTIVE_TRANSFER_ID);
		printk("rx transfer start, sg-id: %d, transfer_id_eot: %d, time: %ld\n",sg->id,transfer_id_eot,ts.tv_nsec);

        }*/
	
	// If the device is not HDMI, and it is TX
	// request a transfer with 24 bytes less size. Because we already used them for metadata
	// But for TX, request txsize times smaller
	if(chan->direction == 1 && (dev_id == 0 || dev_id == 1)) // if the device is TX
	{
		//printk("[DBG][drivers][dma-axi-dmac][axi_dmac_start_transfer][dma_dev_id: %d][dir: %d][partial_report: %d][cyclic: %d]\n -----[sg->x_len: %d][sg->dst_addr: %d][sg->src_addr: %d]\n",dmac->dma_dev.dev_id,chan->direction,chan->hw_partial_xfer,is_cyclic,sg->x_len,sg->dest_addr,sg->src_addr);
		if(tx_ress[sg->id].type == late || tx_ress[sg->id].type == early)
			axi_dmac_write(dmac, AXI_DMAC_REG_X_LENGTH, 5); // LATE or EARLY block, drop it
		else
			axi_dmac_write(dmac, AXI_DMAC_REG_X_LENGTH, sg->x_len -1 -(6*4) -(txsize*4) -(no_overlapping_samples*4)); // (6 additional items + txsize + overlapping_samples)*4

	}
	else if(chan->direction == 2 && (dev_id == 0 || dev_id == 1)) // if the device is RX
	{
		axi_dmac_write(dmac, AXI_DMAC_REG_X_LENGTH, sg->x_len -1 -(6*4) ); // (6 additional items)*4
	}
	else // for hdmi and others
	{
		axi_dmac_write(dmac, AXI_DMAC_REG_X_LENGTH, sg->x_len - 1);
	}
	axi_dmac_write(dmac, AXI_DMAC_REG_Y_LENGTH, sg->y_len - 1);
	axi_dmac_write(dmac, AXI_DMAC_REG_FLAGS, flags);

	// If the device is not HDMI and it is TX: 
	// TIMING 2) after the descriptor is sent, write the tx counter to the timing controller block and set the valid signal
	if(chan->direction == 1 && (dev_id == 0 || dev_id == 1) && tx_ress[sg->id].type == good)
        {

		// Write the timestamp and free enable
		read_timestamp_lsb = (uint32_t) txtimestamp;
		read_timestamp_msb = txtimestamp >> 32;

		if(sg->id == 0)
		{
			// GPIO_1_DATA_0 = read_timestamp_lsb; 
			// GPIO_1_DATA_1 = read_timestamp_msb;
			iowrite32(read_timestamp_lsb,pGPIO1_DATA0);
			iowrite32(read_timestamp_msb,pGPIO1_DATA1);
			
			// GPIO_0_DATA_1 = 1; set the valid: 00000001
			regVal = ioread32(pGPIO0_DATA1);
			regVal = regVal | 1;
			iowrite32(regVal,pGPIO0_DATA1);
		}
                else if(sg->id ==1)
		{
			// GPIO_2_DATA_0 = read_timestamp_lsb; 
			// GPIO_2_DATA_1 = read_timestamp_msb;
			iowrite32(read_timestamp_lsb,pGPIO2_DATA0);
			iowrite32(read_timestamp_msb,pGPIO2_DATA1);
			
			// GPIO_0_DATA_1 = 1; set the valid: 00000010
			regVal = ioread32(pGPIO0_DATA1);
			regVal = regVal | 2;
			iowrite32(regVal,pGPIO0_DATA1);
		}
                else if(sg->id == 2)
		{
			// GPIO_3_DATA_0 = read_timestamp_lsb; 
			// GPIO_3_DATA_1 = read_timestamp_msb;
			iowrite32(read_timestamp_lsb,pGPIO3_DATA0);
			iowrite32(read_timestamp_msb,pGPIO3_DATA1);
			
			// GPIO_0_DATA_1 = 1; set the valid: 00000100
			regVal = ioread32(pGPIO0_DATA1);
			regVal = regVal | 4;
			iowrite32(regVal,pGPIO0_DATA1);
		}
                else if(sg->id == 3)
		{
			// GPIO_4_DATA_0 = read_timestamp_lsb; 
			// GPIO_4_DATA_1 = read_timestamp_msb;
			iowrite32(read_timestamp_lsb,pGPIO4_DATA0);
			iowrite32(read_timestamp_msb,pGPIO4_DATA1);
			
			// GPIO_0_DATA_1 = 1; set the valid: 00001000
			regVal = ioread32(pGPIO0_DATA1);
			regVal = regVal | 8;
			iowrite32(regVal,pGPIO0_DATA1);
		}
		
		// Now start the good TX transfer
		axi_dmac_write(dmac, AXI_DMAC_REG_START_TRANSFER, 1);
        }
	else
	{
		// Start if it is not a good TX transfer
		axi_dmac_write(dmac, AXI_DMAC_REG_START_TRANSFER, 1);

	}
}

static struct axi_dmac_desc *axi_dmac_active_desc(struct axi_dmac_chan *chan)
{
	return list_first_entry_or_null(&chan->active_descs,
		struct axi_dmac_desc, vdesc.node);
}

static inline unsigned int axi_dmac_total_sg_bytes(struct axi_dmac_chan *chan,
	struct axi_dmac_sg *sg)
{
	if (chan->hw_2d)
		return sg->x_len * sg->y_len;
	else
		return sg->x_len;
}

static void axi_dmac_dequeue_partial_xfers(struct axi_dmac_chan *chan)
{
	struct axi_dmac *dmac = chan_to_axi_dmac(chan);
	struct axi_dmac_desc *desc;
	struct axi_dmac_sg *sg;
	u32 xfer_done, len, id, i;
	bool found_sg;

	do {
		len = axi_dmac_read(dmac, AXI_DMAC_REG_PARTIAL_XFER_LEN);
		id  = axi_dmac_read(dmac, AXI_DMAC_REG_PARTIAL_XFER_ID);

		found_sg = false;
		list_for_each_entry(desc, &chan->active_descs, vdesc.node) {
			for (i = 0; i < desc->num_sgs; i++) {
				sg = &desc->sg[i];
				if (sg->id == AXI_DMAC_SG_UNUSED)
					continue;
				if (sg->id == id) {
					desc->have_partial_xfer = true;
					sg->partial_len = len;
					found_sg = true;
					break;
				}
			}
			if (found_sg)
				break;
		}

		if (found_sg) {
			dev_dbg(dmac->dma_dev.dev,
				"Found partial segment id=%u, len=%u\n",
				id, len);
		} else {
			dev_warn(dmac->dma_dev.dev,
				 "Not found partial segment id=%u, len=%u\n",
				 id, len);
		}

		/* Check if we have any more partial transfers */
		xfer_done = axi_dmac_read(dmac, AXI_DMAC_REG_TRANSFER_DONE);
		xfer_done = !(xfer_done & AXI_DMAC_FLAG_PARTIAL_XFER_DONE);

	} while (!xfer_done);
}

static void axi_dmac_compute_residue(struct axi_dmac_chan *chan,
	struct axi_dmac_desc *active)
{
	struct dmaengine_result *rslt = &active->vdesc.tx_result;
	unsigned int start = active->num_completed - 1;
	struct axi_dmac_sg *sg;
	unsigned int i, total;

	rslt->result = DMA_TRANS_NOERROR;
	rslt->residue = 0;

	/*
	 * We get here if the last completed segment is partial, which
	 * means we can compute the residue from that segment onwards
	 */
	for (i = start; i < active->num_sgs; i++) {
		sg = &active->sg[i];
		total = axi_dmac_total_sg_bytes(chan, sg);
		rslt->residue += (total - sg->partial_len);
	}
}

static uint64_t axi_dmac_read_timestamp(int id,struct axi_dmac *dmac,bool is_tx_eot)
{
	uint32_t timestamp_lsb;
	uint32_t timestamp_msb;
	uint64_t timestamp;
		
	switch(id)
	{
		case 0:
               		timestamp_lsb = axi_dmac_read(dmac, (is_tx_eot)?(AXI_DMAC_REG_TIMESTAMPB0_LSB):(AXI_DMAC_REG_TIMESTAMPA0_LSB));
	                timestamp_msb = axi_dmac_read(dmac, (is_tx_eot)?(AXI_DMAC_REG_TIMESTAMPB0_MSB):(AXI_DMAC_REG_TIMESTAMPA0_MSB));
               		timestamp = (uint64_t) timestamp_msb << 32 | timestamp_lsb;
			break;
		case 1:
               		timestamp_lsb = axi_dmac_read(dmac, (is_tx_eot)?(AXI_DMAC_REG_TIMESTAMPB1_LSB):(AXI_DMAC_REG_TIMESTAMPA1_LSB));
	                timestamp_msb = axi_dmac_read(dmac, (is_tx_eot)?(AXI_DMAC_REG_TIMESTAMPB1_MSB):(AXI_DMAC_REG_TIMESTAMPA1_MSB));
               		timestamp = (uint64_t) timestamp_msb << 32 | timestamp_lsb;
			break;
		case 2:
               		timestamp_lsb = axi_dmac_read(dmac, (is_tx_eot)?(AXI_DMAC_REG_TIMESTAMPB2_LSB):(AXI_DMAC_REG_TIMESTAMPA2_LSB));
	                timestamp_msb = axi_dmac_read(dmac, (is_tx_eot)?(AXI_DMAC_REG_TIMESTAMPB2_MSB):(AXI_DMAC_REG_TIMESTAMPA2_MSB));
               		timestamp = (uint64_t) timestamp_msb << 32 | timestamp_lsb;
			break;
		case 3:
               		timestamp_lsb = axi_dmac_read(dmac, (is_tx_eot)?(AXI_DMAC_REG_TIMESTAMPB3_LSB):(AXI_DMAC_REG_TIMESTAMPA3_LSB));
	                timestamp_msb = axi_dmac_read(dmac, (is_tx_eot)?(AXI_DMAC_REG_TIMESTAMPB3_MSB):(AXI_DMAC_REG_TIMESTAMPA3_MSB));
               		timestamp = (uint64_t) timestamp_msb << 32 | timestamp_lsb;
			break;
		default:
			timestamp = 0;
			break;
	}
	return timestamp;
}

static uint64_t rxtimestamp_tmp = 0;
static uint64_t timenow_tmp = 0;

static bool axi_dmac_transfer_done(struct axi_dmac_chan *chan,
	unsigned int completed_transfers)
{
	struct axi_dmac_desc *active;
	struct axi_dmac *dmac = chan_to_axi_dmac(chan);
	struct axi_dmac_sg *sg;
	bool start_next = false;
	
	uint64_t rxtimestamp_eot;
	uint64_t txtimestamp_sot;
	uint64_t txtimestamp_eot;
	
	int dev_id;
	uint8_t regVal = 0;
	bool itstheone = false;
	
	int transfer_id_eot = 0;
	uint64_t timenow = 0;
	char stat[5];	
	struct timeval tv;
	unsigned long ts_micro=0;

	int32_t sot_minus_cmd = 0;
	int32_t eot_minus_est = 0;

        dev_id = dmac->dma_dev.dev_id;
	
	active = axi_dmac_active_desc(chan);
	if (!active)
		return false;
	
	if (chan->hw_partial_xfer && (completed_transfers & AXI_DMAC_FLAG_PARTIAL_XFER_DONE))
	{
		printk("partial detected! \n");
		axi_dmac_dequeue_partial_xfers(chan);
	}

	
	// Check if this is the right packet to handle
        sg = &active->sg[active->num_completed];
	if ( (sg->id != AXI_DMAC_SG_UNUSED) && (BIT(sg->id) & completed_transfers) )
        	itstheone = true;

        // if it is not hdmi and it is RX: 
	// read the rx counter from DMAC and write it to the global variable
        if(chan->direction == 2 && (dev_id == 0 || dev_id == 1) && itstheone)
        {
		// Increase the rx packets counter id
		rxidcounter++;
		
		// read rx timestamp
		rxtimestamp_eot = axi_dmac_read_timestamp(sg->id,dmac,false);

		// print debug
		/*timenow = axi_dmac_axitimer_read();
		transfer_id_eot = axi_dmac_read(dmac,AXI_DMAC_REG_ACTIVE_TRANSFER_ID);
                //printk("[DBG][drivers][dma-axi-dmac-done] RX timestamp: %llu, sg->id: %d, transfer_id_eot: %d, time: %ld\n", rxtimestamp_eot,sg->id,transfer_id_eot, ts.tv_nsec);
		if(rxtimestamp_tmp != 0)
		{
			if(rxtimestamp_eot - rxtimestamp_tmp > 120000 )
			{
                		printk("RX timestamp: %llu, RX diff: %llu, rxidcounter: %d, sg->id: %d, transfer_id_eot: %d, hw_time: %llu, hw_time_diff: %llu\n", rxtimestamp_eot,rxtimestamp_eot - rxtimestamp_tmp,rxidcounter,sg->id,transfer_id_eot, timenow, timenow-timenow_tmp);
			}
		}
		rxtimestamp_tmp = rxtimestamp_eot;
		timenow_tmp = timenow;*/

		rx_timestamps[sg->id] = rxtimestamp_eot;
		rxidcounters[sg->id] = rxidcounter;
        }

        // if it is not hdmi and it is TX: read diff counter from DMAC and print it
        if(chan->direction == 1 && (dev_id == 0 || dev_id == 1) && itstheone)
        {
		// print info
		sg = &active->sg[active->num_completed];

		// only if the TX transaction was good
		// read tx sot timestamp, update global variable for stat report
		if(tx_ress[sg->id].type == good)
		{
			txtimestamp_sot = axi_dmac_read_timestamp(sg->id,dmac,false);
			sot_minus_cmd = (int64_t)txtimestamp_sot-(int64_t)tx_cmds[sg->id].timestamp;
			tx_stat.sot_cmd = sot_minus_cmd; 
		}

		/*

		// read tx timestamps
		txtimestamp_sot = axi_dmac_read_timestamp(sg->id,dmac,false);
		sot_minus_cmd = (int64_t)txtimestamp_sot-(int64_t)tx_cmds[sg->id].timestamp;
        	txtimestamp_eot = axi_dmac_read_timestamp(sg->id,dmac,true);
		eot_minus_est = (int64_t)txtimestamp_eot-(int64_t)tx_ress[sg->id].est_finish_time;

		timenow = axi_dmac_axitimer_read();
		transfer_id_eot = axi_dmac_read(dmac,AXI_DMAC_REG_ACTIVE_TRANSFER_ID);

		do_gettimeofday(&tv);
                ts_micro = 1000000 * tv.tv_sec + tv.tv_usec;

		if(tx_ress[sg->id].type == late)
			strncpy(stat,"LATE",5);
		else if(tx_ress[sg->id].type == early)
			strncpy(stat,"EARL",5);
		else if(tx_ress[sg->id].type == notimestamp)
			strncpy(stat,"NOTS",5);
		else		
			strncpy(stat,"GOOD",5);

		printk("Done tx %s, id: %d, size: %d, tx sot: %llu, sot-cmd: %lld, tx eot: %llu, eot-est: %lld ,transfer_id_eot: %d, hardware time: %llu, time: %lu, sot-eot: %lld\n",stat, sg->id, sg->x_len,txtimestamp_sot,(int64_t)txtimestamp_sot-(int64_t)tx_cmds[sg->id].timestamp,txtimestamp_eot,(int64_t)txtimestamp_eot-(int64_t)tx_ress[sg->id].est_finish_time,transfer_id_eot,timenow,ts_micro,(int64_t)txtimestamp_eot-(int64_t)txtimestamp_sot);
		
		*/

	}

        //printk("[DBG][drivers][dma-axi-dmac][axi_dmac_transfer_done][end_counter: %d][dir: %d]\n -----[read_timestamp: %llu]\n",start_counter,chan->direction,read_timestamp);

        // if it is not hdmi and it is TX: 
	// TIMING 3) revert enable and valid signals for the next transfer
        if(chan->direction == 1 && (dev_id == 0 || dev_id == 1) && itstheone)
        {
                sg = &active->sg[active->num_completed];

		if(sg->id == 0)
                {
			// puts fifo_en on the output level 1
                        // GPIO_0_DATA_0 = 1, set enable: 00000001
                        regVal = ioread32(pGPIO0_DATA0);
                        regVal = regVal | 1;
                        iowrite32(regVal,pGPIO0_DATA0);

			// puts fifo_en on the end
                	// GPIO_0_DATA_1 = 0, set valid: 11111110 : 254
			regVal = ioread32(pGPIO0_DATA1);
			regVal = regVal & 254;
			iowrite32(regVal,pGPIO0_DATA1);
                }
                else if(sg->id == 1)
                {
			// puts fifo_en on the output level 1
                        // GPIO_0_DATA_0 = 1, set enable: 00000010
                        regVal = ioread32(pGPIO0_DATA0);
                        regVal = regVal | 2;
                        iowrite32(regVal,pGPIO0_DATA0);

			// puts fifo_en on the end
                	// GPIO_0_DATA_1 = 0, set valid: 11111101 : 253
			regVal = ioread32(pGPIO0_DATA1);
			regVal = regVal & 253;
			iowrite32(regVal,pGPIO0_DATA1);
                }
                else if(sg->id == 2)
                {
			// puts fifo_en on the output level 1
                        // GPIO_0_DATA_0 = 1, set enable: 00000100
                        regVal = ioread32(pGPIO0_DATA0);
                        regVal = regVal | 4;
                        iowrite32(regVal,pGPIO0_DATA0);

			// puts fifo_en on the end
                	// GPIO_0_DATA_1 = 0, set valid: 11111011 : 251
			regVal = ioread32(pGPIO0_DATA1);
			regVal = regVal & 251;
			iowrite32(regVal,pGPIO0_DATA1);
                }
                else if(sg->id == 3)
                {
			// puts fifo_en on the output level 1
                        // GPIO_0_DATA_0 = 1, set enable: 00001000
                        regVal = ioread32(pGPIO0_DATA0);
                        regVal = regVal | 8;
                        iowrite32(regVal,pGPIO0_DATA0);

			// puts fifo_en on the end
                	// GPIO_0_DATA_1 = 0, set valid: 11110111 : 247
			regVal = ioread32(pGPIO0_DATA1);
			regVal = regVal & 247;
			iowrite32(regVal,pGPIO0_DATA1);
                }
                
		// announce that the transfer is finished
                //tx_transfer_ongoing = false;
                //printk("----------tx transfer done, tx_transfer_ongoing is false\n");
		
        }


	do {
		sg = &active->sg[active->num_completed];
		if (sg->id == AXI_DMAC_SG_UNUSED) /* Not yet submitted */
			break;
		if (!(BIT(sg->id) & completed_transfers))
			break;
		
		active->num_completed++;
		sg->id = AXI_DMAC_SG_UNUSED;
		if (sg->schedule_when_free) {
			sg->schedule_when_free = false;
			start_next = true;
		}

		if (active->num_completed == active->num_sgs ||
		    sg->partial_len) {
			if (active->cyclic) {
				active->num_completed = 0; /* wrap around */
				if (sg->last)
					vchan_cyclic_callback(&active->vdesc);
			} else {
				list_del(&active->vdesc.node);
				if (sg->partial_len)
					axi_dmac_compute_residue(chan, active);
				vchan_cookie_complete(&active->vdesc);
				active = axi_dmac_active_desc(chan);
			}
		}
	} while (active);

	return start_next;
}

#ifdef SPEED_TEST
static s64 get_time(void)
{
	struct timespec ts;
	ktime_get_real_ts(&ts);

	return timespec_to_ns(&ts);
}

static s64 start;
static unsigned int count;

static irqreturn_t axi_dmac_interrupt_handler(int irq, void *devid)
{
	struct axi_dmac *dmac = devid;
	unsigned int pending;

	pending = axi_dmac_read(dmac, AXI_DMAC_REG_IRQ_PENDING);
	axi_dmac_write(dmac, AXI_DMAC_REG_IRQ_PENDING, pending);

	if (pending & 1) {
		if (count == 0)
			start = get_time();
		if (count < 100) {
			axi_dmac_write(dmac, AXI_DMAC_REG_START_TRANSFER, 1);
			count += 1;
		}
	} else if ((pending & 2) && count == 100) {
		printk("time: %lld %x\n", get_time() - start, pending);
	}

	return IRQ_HANDLED;
}
#else
static irqreturn_t axi_dmac_interrupt_handler(int irq, void *devid)
{
	struct axi_dmac *dmac = devid;
	unsigned int pending;
	bool start_next = false;

	pending = axi_dmac_read(dmac, AXI_DMAC_REG_IRQ_PENDING);
	if (!pending)
		return IRQ_NONE;

	axi_dmac_write(dmac, AXI_DMAC_REG_IRQ_PENDING, pending);

	spin_lock(&dmac->chan.vchan.lock);
	/* One or more transfers have finished */
	if (pending & AXI_DMAC_IRQ_EOT) {
		unsigned int completed;

		completed = axi_dmac_read(dmac, AXI_DMAC_REG_TRANSFER_DONE);
		start_next = axi_dmac_transfer_done(&dmac->chan, completed);
	}
	/* Space has become available in the descriptor queue */
	if ((pending & AXI_DMAC_IRQ_SOT) || start_next)
		axi_dmac_start_transfer(&dmac->chan);
	spin_unlock(&dmac->chan.vchan.lock);

	return IRQ_HANDLED;
}
#endif

static int axi_dmac_terminate_all(struct dma_chan *c)
{
	struct axi_dmac_chan *chan = to_axi_dmac_chan(c);
	struct axi_dmac *dmac = chan_to_axi_dmac(chan);
	unsigned long flags;
	LIST_HEAD(head);

	spin_lock_irqsave(&chan->vchan.lock, flags);
	axi_dmac_write(dmac, AXI_DMAC_REG_CTRL, 0);
	chan->next_desc = NULL;
	vchan_get_all_descriptors(&chan->vchan, &head);
	list_splice_tail_init(&chan->active_descs, &head);
	spin_unlock_irqrestore(&chan->vchan.lock, flags);

	vchan_dma_desc_free_list(&chan->vchan, &head);

	return 0;
}

static void axi_dmac_synchronize(struct dma_chan *c)
{
	struct axi_dmac_chan *chan = to_axi_dmac_chan(c);

	vchan_synchronize(&chan->vchan);
}

static void axi_dmac_issue_pending(struct dma_chan *c)
{
	struct axi_dmac_chan *chan = to_axi_dmac_chan(c);
	struct axi_dmac *dmac = chan_to_axi_dmac(chan);
	unsigned long flags;

	axi_dmac_write(dmac, AXI_DMAC_REG_CTRL, AXI_DMAC_CTRL_ENABLE);

	spin_lock_irqsave(&chan->vchan.lock, flags);
	if (vchan_issue_pending(&chan->vchan))
		axi_dmac_start_transfer(chan);
	spin_unlock_irqrestore(&chan->vchan.lock, flags);
}

static struct axi_dmac_desc *axi_dmac_alloc_desc(unsigned int num_sgs)
{
	struct axi_dmac_desc *desc;
	unsigned int i;

	desc = kzalloc(sizeof(struct axi_dmac_desc) +
		sizeof(struct axi_dmac_sg) * num_sgs, GFP_NOWAIT);
	if (!desc)
		return NULL;

	for (i = 0; i < num_sgs; i++)
		desc->sg[i].id = AXI_DMAC_SG_UNUSED;

	desc->num_sgs = num_sgs;

	return desc;
}

static struct axi_dmac_sg *axi_dmac_fill_linear_sg(struct axi_dmac_chan *chan,
	enum dma_transfer_direction direction, dma_addr_t addr,
	unsigned int num_periods, unsigned int period_len,
	struct axi_dmac_sg *sg)
{
	unsigned int num_segments, i;
	unsigned int segment_size;
	unsigned int len;

	// IMPORTANT: this period_len is 4 times of the buffer size. since we have 4 RX and 4 TX.

	/* Split into multiple equally sized segments if necessary */
	num_segments = DIV_ROUND_UP(period_len, chan->max_length);
	segment_size = DIV_ROUND_UP(period_len, num_segments);
	/* Take care of alignment */
	segment_size = ((segment_size - 1) | chan->length_align_mask) + 1;

	for (i = 0; i < num_periods; i++) {
		len = period_len;

		while (len > segment_size) {
			if (direction == DMA_DEV_TO_MEM)
				sg->dest_addr = addr;
			else
				sg->src_addr = addr;
			sg->x_len = segment_size;
			sg->y_len = 1;
			//printk("[DEBUG][AXI-DMAC][fill_linear_sg] %s, sg->x_len: %d, sg->y_len: %d, sg->last: %d, sg->dest_addr: %d, sg->src_addr: %d \n",(direction==DMA_DEV_TO_MEM)?"DEV_TO_MEM":"MEM_TO_DEV",sg->x_len,sg->y_len,(int)sg->last,(int)sg->dest_addr,(int)sg->src_addr);
			sg++;
			addr += segment_size;
			len -= segment_size;
		}

		if (direction == DMA_DEV_TO_MEM)
			sg->dest_addr = addr;
		else
			sg->src_addr = addr;
		sg->x_len = len;
		sg->y_len = 1;
		sg->last = true;
		//printk("[DEBUG][AXI-DMAC][fill_linear_sg] %s, sg->x_len: %d, sg->y_len: %d, sg->last: %d, sg->dest_addr: %d, sg->src_addr: %d \n",(direction==DMA_DEV_TO_MEM)?"DEV_TO_MEM":"MEM_TO_DEV",sg->x_len,sg->y_len,(int)sg->last,(int)sg->dest_addr,(int)sg->src_addr);
		sg++;
		addr += len;
	}

	return sg;
}

static struct dma_async_tx_descriptor *axi_dmac_prep_slave_sg(
	struct dma_chan *c, struct scatterlist *sgl,
	unsigned int sg_len, enum dma_transfer_direction direction,
	unsigned long flags, void *context)
{
	struct axi_dmac_chan *chan = to_axi_dmac_chan(c);
	struct axi_dmac_desc *desc;
	struct axi_dmac_sg *dsg;
	struct scatterlist *sg;
	unsigned int num_sgs;
	unsigned int i;

	if (direction != chan->direction)
		return NULL;

	num_sgs = sg_nents_for_dma(sgl, sg_len, chan->max_length);
	desc = axi_dmac_alloc_desc(num_sgs);
	if (!desc)
		return NULL;

	dsg = desc->sg;

	for_each_sg(sgl, sg, sg_len, i) {
		if (!axi_dmac_check_addr(chan, sg_dma_address(sg)) ||
		    !axi_dmac_check_len(chan, sg_dma_len(sg))) {
			kfree(desc);
			return NULL;
		}

		dsg = axi_dmac_fill_linear_sg(chan, direction, sg_dma_address(sg), 1,
			sg_dma_len(sg), dsg);
	}

	desc->cyclic = false;

	return vchan_tx_prep(&chan->vchan, &desc->vdesc, flags);
}

static struct dma_async_tx_descriptor *axi_dmac_prep_dma_cyclic(
	struct dma_chan *c, dma_addr_t buf_addr, size_t buf_len,
	size_t period_len, enum dma_transfer_direction direction,
	unsigned long flags)
{
	struct axi_dmac_chan *chan = to_axi_dmac_chan(c);
	struct axi_dmac_desc *desc;
	unsigned int num_periods, num_segments;

	if (direction != chan->direction)
		return NULL;

	if (!axi_dmac_check_len(chan, buf_len) ||
	    !axi_dmac_check_addr(chan, buf_addr))
		return NULL;

	if (period_len == 0 || buf_len % period_len)
		return NULL;

	num_periods = buf_len / period_len;
	num_segments = DIV_ROUND_UP(period_len, chan->max_length);

	desc = axi_dmac_alloc_desc(num_periods * num_segments);
	if (!desc)
		return NULL;

	axi_dmac_fill_linear_sg(chan, direction, buf_addr, num_periods,
		buf_len, desc->sg);

	desc->cyclic = true;

	return vchan_tx_prep(&chan->vchan, &desc->vdesc, flags);
}

static struct dma_async_tx_descriptor *axi_dmac_prep_interleaved(
	struct dma_chan *c, struct dma_interleaved_template *xt,
	unsigned long flags)
{
	struct axi_dmac_chan *chan = to_axi_dmac_chan(c);
	struct axi_dmac_desc *desc;
	size_t dst_icg, src_icg;

	if (xt->frame_size != 1)
		return NULL;

	if (xt->dir != chan->direction)
		return NULL;

	if (axi_dmac_src_is_mem(chan)) {
		if (!xt->src_inc || !axi_dmac_check_addr(chan, xt->src_start))
			return NULL;
	}

	if (axi_dmac_dest_is_mem(chan)) {
		if (!xt->dst_inc || !axi_dmac_check_addr(chan, xt->dst_start))
			return NULL;
	}

	dst_icg = dmaengine_get_dst_icg(xt, &xt->sgl[0]);
	src_icg = dmaengine_get_src_icg(xt, &xt->sgl[0]);

	if (chan->hw_2d) {
		if (!axi_dmac_check_len(chan, xt->sgl[0].size) ||
		    xt->numf == 0)
			return NULL;
		if (xt->sgl[0].size + dst_icg > chan->max_length ||
		    xt->sgl[0].size + src_icg > chan->max_length)
			return NULL;
	} else {
		if (dst_icg != 0 || src_icg != 0)
			return NULL;
		if (chan->max_length / xt->sgl[0].size < xt->numf)
			return NULL;
		if (!axi_dmac_check_len(chan, xt->sgl[0].size * xt->numf))
			return NULL;
	}

	desc = axi_dmac_alloc_desc(1);
	if (!desc)
		return NULL;

	if (axi_dmac_src_is_mem(chan)) {
		desc->sg[0].src_addr = xt->src_start;
		desc->sg[0].src_stride = xt->sgl[0].size + src_icg;
	}

	if (axi_dmac_dest_is_mem(chan)) {
		desc->sg[0].dest_addr = xt->dst_start;
		desc->sg[0].dest_stride = xt->sgl[0].size + dst_icg;
	}

	if (chan->hw_2d) {
		desc->sg[0].x_len = xt->sgl[0].size;
		desc->sg[0].y_len = xt->numf;
	} else {
		desc->sg[0].x_len = xt->sgl[0].size * xt->numf;
		desc->sg[0].y_len = 1;
	}

	if (flags & DMA_CYCLIC)
		desc->cyclic = true;

	return vchan_tx_prep(&chan->vchan, &desc->vdesc, flags);
}

static void axi_dmac_free_chan_resources(struct dma_chan *c)
{
	vchan_free_chan_resources(to_virt_chan(c));
}

static void axi_dmac_desc_free(struct virt_dma_desc *vdesc)
{
	kfree(container_of(vdesc, struct axi_dmac_desc, vdesc));
}

static bool axi_dmac_regmap_rdwr(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case AXI_DMAC_REG_IRQ_MASK:
	case AXI_DMAC_REG_IRQ_SOURCE:
	case AXI_DMAC_REG_IRQ_PENDING:
	case AXI_DMAC_REG_CTRL:
	case AXI_DMAC_REG_TRANSFER_ID:
	case AXI_DMAC_REG_START_TRANSFER:
	case AXI_DMAC_REG_FLAGS:
	case AXI_DMAC_REG_DEST_ADDRESS:
	case AXI_DMAC_REG_SRC_ADDRESS:
	case AXI_DMAC_REG_X_LENGTH:
	case AXI_DMAC_REG_Y_LENGTH:
	case AXI_DMAC_REG_DEST_STRIDE:
	case AXI_DMAC_REG_SRC_STRIDE:
	case AXI_DMAC_REG_TRANSFER_DONE:
	case AXI_DMAC_REG_ACTIVE_TRANSFER_ID :
	case AXI_DMAC_REG_STATUS:
	case AXI_DMAC_REG_CURRENT_SRC_ADDR:
	case AXI_DMAC_REG_CURRENT_DEST_ADDR:
	case AXI_DMAC_REG_DBG0:
	case AXI_DMAC_REG_DBG1:
	case AXI_DMAC_REG_DBG2:
	case AXI_DMAC_REG_TIMESTAMPA0_LSB:
	case AXI_DMAC_REG_TIMESTAMPA0_MSB:
	case AXI_DMAC_REG_TIMESTAMPA1_LSB:
	case AXI_DMAC_REG_TIMESTAMPA1_MSB:
	case AXI_DMAC_REG_TIMESTAMPA2_LSB:
	case AXI_DMAC_REG_TIMESTAMPA2_MSB:
	case AXI_DMAC_REG_TIMESTAMPA3_LSB:
	case AXI_DMAC_REG_TIMESTAMPA3_MSB:
	case AXI_DMAC_REG_TIMESTAMPB0_LSB:
	case AXI_DMAC_REG_TIMESTAMPB0_MSB:
	case AXI_DMAC_REG_TIMESTAMPB1_LSB:
	case AXI_DMAC_REG_TIMESTAMPB1_MSB:
	case AXI_DMAC_REG_TIMESTAMPB2_LSB:
	case AXI_DMAC_REG_TIMESTAMPB2_MSB:
	case AXI_DMAC_REG_TIMESTAMPB3_LSB:
	case AXI_DMAC_REG_TIMESTAMPB3_MSB:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config axi_dmac_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = AXI_DMAC_REG_TIMESTAMPB3_MSB,
	.readable_reg = axi_dmac_regmap_rdwr,
	.writeable_reg = axi_dmac_regmap_rdwr,
};

/*
 * The configuration stored in the devicetree matches the configuration
 * parameters of the peripheral instance and allows the driver to know which
 * features are implemented and how it should behave.
 */
static int axi_dmac_parse_chan_dt(struct device_node *of_chan,
	struct axi_dmac_chan *chan)
{
	u32 val;
	int ret;

	ret = of_property_read_u32(of_chan, "reg", &val);
	if (ret)
		return ret;

	/* We only support 1 channel for now */
	if (val != 0)
		return -EINVAL;

	ret = of_property_read_u32(of_chan, "adi,source-bus-type", &val);
	if (ret)
		return ret;
	if (val > AXI_DMAC_BUS_TYPE_FIFO)
		return -EINVAL;
	chan->src_type = val;

	ret = of_property_read_u32(of_chan, "adi,destination-bus-type", &val);
	if (ret)
		return ret;
	if (val > AXI_DMAC_BUS_TYPE_FIFO)
		return -EINVAL;
	chan->dest_type = val;

	ret = of_property_read_u32(of_chan, "adi,source-bus-width", &val);
	if (ret)
		return ret;
	chan->src_width = val / 8;

	ret = of_property_read_u32(of_chan, "adi,destination-bus-width", &val);
	if (ret)
		return ret;
	chan->dest_width = val / 8;

	chan->address_align_mask = max(chan->dest_width, chan->src_width) - 1;

	if (axi_dmac_dest_is_mem(chan) && axi_dmac_src_is_mem(chan))
		chan->direction = DMA_MEM_TO_MEM;
	else if (!axi_dmac_dest_is_mem(chan) && axi_dmac_src_is_mem(chan))
		chan->direction = DMA_MEM_TO_DEV;
	else if (axi_dmac_dest_is_mem(chan) && !axi_dmac_src_is_mem(chan))
		chan->direction = DMA_DEV_TO_MEM;
	else
		chan->direction = DMA_DEV_TO_DEV;

	return 0;
}

/* Support old binding */
static int axi_dmac_parse_chan_dt_compat(struct device_node *of_node,
	struct axi_dmac_chan *chan)
{
	struct device_node *of_chan;
	u32 tmp;

	of_chan = of_get_child_by_name(of_node, "dma-channel");
	if (of_chan == NULL)
		return -ENODEV;

	tmp = 0;
	of_property_read_u32(of_chan, "adi,type", &tmp);

	switch (tmp) {
	case 0:
		chan->direction = DMA_DEV_TO_MEM;
		chan->src_type = AXI_DMAC_BUS_TYPE_AXI_STREAM;
		chan->dest_type = AXI_DMAC_BUS_TYPE_AXI_MM;
		break;
	case 1:
		chan->direction = DMA_MEM_TO_DEV;
		chan->src_type = AXI_DMAC_BUS_TYPE_AXI_MM;
		chan->dest_type = AXI_DMAC_BUS_TYPE_AXI_STREAM;
		break;
	case 2:
		chan->direction = DMA_MEM_TO_MEM;
		chan->src_type = AXI_DMAC_BUS_TYPE_AXI_MM;
		chan->dest_type = AXI_DMAC_BUS_TYPE_AXI_MM;
		break;
	case 3:
		chan->direction = DMA_DEV_TO_DEV;
		chan->src_type = AXI_DMAC_BUS_TYPE_AXI_STREAM;
		chan->dest_type = AXI_DMAC_BUS_TYPE_AXI_STREAM;
		break;
	default:
		return -EINVAL;
	}

	tmp = 64;
	of_property_read_u32(of_chan, "adi,source-bus-width", &tmp);
	chan->src_width = tmp / 8;

	tmp = 64;
	of_property_read_u32(of_chan, "adi,destination-bus-width", &tmp);
	chan->dest_width = tmp / 8;

	chan->address_align_mask = max(chan->dest_width, chan->src_width) - 1;

	return 0;
}

static int axi_dmac_detect_caps(struct axi_dmac *dmac)
{
	struct axi_dmac_chan *chan = &dmac->chan;
	unsigned int version, version_minor;

	version = axi_dmac_read(dmac, AXI_DMAC_REG_VERSION);
	version_minor = version & 0xff00;

	axi_dmac_write(dmac, AXI_DMAC_REG_FLAGS, AXI_DMAC_FLAG_CYCLIC);
	if (axi_dmac_read(dmac, AXI_DMAC_REG_FLAGS) == AXI_DMAC_FLAG_CYCLIC)
		chan->hw_cyclic = true;

	axi_dmac_write(dmac, AXI_DMAC_REG_Y_LENGTH, 1);
	if (axi_dmac_read(dmac, AXI_DMAC_REG_Y_LENGTH) == 1)
		chan->hw_2d = true;

	axi_dmac_write(dmac, AXI_DMAC_REG_X_LENGTH, 0xffffffff);
	chan->max_length = axi_dmac_read(dmac, AXI_DMAC_REG_X_LENGTH);
	if (chan->max_length != UINT_MAX)
		chan->max_length++;

	axi_dmac_write(dmac, AXI_DMAC_REG_DEST_ADDRESS, 0xffffffff);
	if (axi_dmac_read(dmac, AXI_DMAC_REG_DEST_ADDRESS) == 0 &&
	    chan->dest_type == AXI_DMAC_BUS_TYPE_AXI_MM) {
		dev_err(dmac->dma_dev.dev,
			"Destination memory-mapped interface not supported.");
		return -ENODEV;
	}

	axi_dmac_write(dmac, AXI_DMAC_REG_SRC_ADDRESS, 0xffffffff);
	if (axi_dmac_read(dmac, AXI_DMAC_REG_SRC_ADDRESS) == 0 &&
	    chan->src_type == AXI_DMAC_BUS_TYPE_AXI_MM) {
		dev_err(dmac->dma_dev.dev,
			"Source memory-mapped interface not supported.");
		return -ENODEV;
	}

	if (version_minor >= 0x0200)
		chan->hw_partial_xfer = true;

	if (version_minor >= 0x0100) {
		axi_dmac_write(dmac, AXI_DMAC_REG_X_LENGTH, 0x00);
		chan->length_align_mask = axi_dmac_read(dmac, AXI_DMAC_REG_X_LENGTH);
	} else {
		chan->length_align_mask = chan->address_align_mask;
	}
	
	return 0;
}

static int axi_dmac_probe(struct platform_device *pdev)
{
	struct device_node *of_channels, *of_chan;
	struct dma_device *dma_dev;
	struct axi_dmac *dmac;
	struct resource *res;
	int ret;

	dmac = devm_kzalloc(&pdev->dev, sizeof(*dmac), GFP_KERNEL);
	if (!dmac)
		return -ENOMEM;

	dmac->irq = platform_get_irq(pdev, 0);
	if (dmac->irq < 0)
		return dmac->irq;
	if (dmac->irq == 0)
		return -EINVAL;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dmac->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dmac->base))
		return PTR_ERR(dmac->base);

	dmac->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(dmac->clk))
		return PTR_ERR(dmac->clk);

	INIT_LIST_HEAD(&dmac->chan.active_descs);

	of_channels = of_get_child_by_name(pdev->dev.of_node, "adi,channels");
	if (of_channels == NULL) {
		ret = axi_dmac_parse_chan_dt_compat(pdev->dev.of_node, &dmac->chan);
		if (ret)
			return ret;
	} else {
		for_each_child_of_node(of_channels, of_chan) {
			ret = axi_dmac_parse_chan_dt(of_chan, &dmac->chan);
			if (ret) {
				of_node_put(of_chan);
				of_node_put(of_channels);
				return -EINVAL;
			}
		}
		of_node_put(of_channels);
	}

	pdev->dev.dma_parms = &dmac->dma_parms;
	dma_set_max_seg_size(&pdev->dev, UINT_MAX);

	dma_dev = &dmac->dma_dev;
	dma_cap_set(DMA_SLAVE, dma_dev->cap_mask);
	dma_cap_set(DMA_CYCLIC, dma_dev->cap_mask);
	dma_cap_set(DMA_INTERLEAVE, dma_dev->cap_mask);
	dma_dev->device_free_chan_resources = axi_dmac_free_chan_resources;
	dma_dev->device_tx_status = dma_cookie_status;
	dma_dev->device_issue_pending = axi_dmac_issue_pending;
	dma_dev->device_prep_slave_sg = axi_dmac_prep_slave_sg;
	dma_dev->device_prep_dma_cyclic = axi_dmac_prep_dma_cyclic;
	dma_dev->device_prep_interleaved_dma = axi_dmac_prep_interleaved;
	dma_dev->device_terminate_all = axi_dmac_terminate_all;
	dma_dev->device_synchronize = axi_dmac_synchronize;
	dma_dev->dev = &pdev->dev;
	dma_dev->chancnt = 1;
	dma_dev->src_addr_widths = BIT(dmac->chan.src_width);
	dma_dev->dst_addr_widths = BIT(dmac->chan.dest_width);
	dma_dev->directions = BIT(dmac->chan.direction);
	dma_dev->residue_granularity = DMA_RESIDUE_GRANULARITY_DESCRIPTOR;
	INIT_LIST_HEAD(&dma_dev->channels);

	dmac->chan.vchan.desc_free = axi_dmac_desc_free;
	vchan_init(&dmac->chan.vchan, dma_dev);

	ret = clk_prepare_enable(dmac->clk);
	if (ret < 0)
		return ret;

	ret = axi_dmac_detect_caps(dmac);
	if (ret)
		goto err_clk_disable;

	dma_dev->copy_align = (dmac->chan.address_align_mask + 1);

	axi_dmac_write(dmac, AXI_DMAC_REG_IRQ_MASK, 0x00);

	ret = dma_async_device_register(dma_dev);
	if (ret)
		goto err_clk_disable;

	ret = of_dma_controller_register(pdev->dev.of_node,
		of_dma_xlate_by_chan_id, dma_dev);
	if (ret)
		goto err_unregister_device;

	ret = request_irq(dmac->irq, axi_dmac_interrupt_handler, 0,
		dev_name(&pdev->dev), dmac);
	if (ret)
		goto err_unregister_of;

	platform_set_drvdata(pdev, dmac);

	devm_regmap_init_mmio(&pdev->dev, dmac->base, &axi_dmac_regmap_config);


	//ECL
	printk("[DBG][drivers][dma-axi-dmac][axi_dmac_probe]\n");

	// Timing IP GPIO AXI signals
	pGPIO0_DATA0 = ioremap_nocache(GPIO0_DATA0,0x4); // Enable signal
	pGPIO0_DATA1 = ioremap_nocache(GPIO0_DATA1,0x4); // Valid signal

	pGPIO1_DATA0 = ioremap_nocache(GPIO1_DATA0,0x4); // timestamp lsb
	pGPIO1_DATA1 = ioremap_nocache(GPIO1_DATA1,0x4); // timestamp msb

	pGPIO2_DATA0 = ioremap_nocache(GPIO2_DATA0,0x4); // timestamp lsb
	pGPIO2_DATA1 = ioremap_nocache(GPIO2_DATA1,0x4); // timestamp msb

	pGPIO3_DATA0 = ioremap_nocache(GPIO3_DATA0,0x4); // timestamp lsb
	pGPIO3_DATA1 = ioremap_nocache(GPIO3_DATA1,0x4); // timestamp msb

	pGPIO4_DATA0 = ioremap_nocache(GPIO4_DATA0,0x4); // timestamp lsb
	pGPIO4_DATA1 = ioremap_nocache(GPIO4_DATA1,0x4); // timestamp msb

	paTIMER_TCR0 = ioremap_nocache(aTIMER_TCR0,0x4); // map timer 0 count register
        paTIMER_TCR1 = ioremap_nocache(aTIMER_TCR1,0x4); // map timer 1 count register


#ifdef SPEED_TEST
	for (i = 0; i < 0x30; i += 4)
		printk("reg %x: %x\n", i, axi_dmac_read(dmac, i));
	dmac->test_virt = dma_alloc_coherent(&pdev->dev, SZ_8M,
			&dmac->test_phys, GFP_KERNEL);

	axi_dmac_write(dmac, AXI_DMAC_REG_CTRL, AXI_DMAC_CTRL_ENABLE);
	axi_dmac_write(dmac, AXI_DMAC_REG_DMA_ADDRESS, dmac->test_phys);
	axi_dmac_write(dmac, AXI_DMAC_REG_DMA_COUNT, SZ_8M);

	printk("Check registers\n");
	printk("CTRL: %x %x\n", AXI_DMAC_CTRL_ENABLE, axi_dmac_read(dmac, AXI_DMAC_REG_CTRL));
	printk("ADDR: %x %x\n", dmac->test_phys, axi_dmac_read(dmac, AXI_DMAC_REG_DMA_ADDRESS));
	printk("COUNT: %x %x\n", PAGE_SIZE, axi_dmac_read(dmac, AXI_DMAC_REG_DMA_COUNT));
	printk("MASK: %x %x\n", 0, axi_dmac_read(dmac, AXI_DMAC_REG_IRQ_MASK));

	printk("Start transfer\n");
	axi_dmac_write(dmac, AXI_DMAC_REG_START_TRANSFER, 1);
	printk("START: %x %x\n", 1, axi_dmac_read(dmac, AXI_DMAC_REG_START_TRANSFER));

	for (i = 0; i < 0x100; i++)
		printk("%.8x%c", ((unsigned long *)dmac->test_virt)[i],
			i % 16 == 15 ? '\n' : ' ');
	printk("Last: %x\n", ((unsigned long *)dmac->test_virt)[PAGE_SIZE/4-1]);
	printk("PROGRESS: %x %x\n", 1, axi_dmac_read(dmac, AXI_DMAC_REG_DMA_COUNT_PROGRESS));
#endif

	return 0;

err_unregister_of:
	of_dma_controller_free(pdev->dev.of_node);
err_unregister_device:
	dma_async_device_unregister(&dmac->dma_dev);
err_clk_disable:
	clk_disable_unprepare(dmac->clk);

	return ret;
}

static int axi_dmac_remove(struct platform_device *pdev)
{
	struct axi_dmac *dmac = platform_get_drvdata(pdev);

	of_dma_controller_free(pdev->dev.of_node);
	free_irq(dmac->irq, dmac);
	tasklet_kill(&dmac->chan.vchan.task);
	dma_async_device_unregister(&dmac->dma_dev);
	clk_disable_unprepare(dmac->clk);

	return 0;
}

static const struct of_device_id axi_dmac_of_match_table[] = {
	{ .compatible = "adi,axi-dmac-1.00.a" },
	{ },
};
MODULE_DEVICE_TABLE(of, axi_dmac_of_match_table);

static struct platform_driver axi_dmac_driver = {
	.driver = {
		.name = "dma-axi-dmac",
		.of_match_table = axi_dmac_of_match_table,
	},
	.probe = axi_dmac_probe,
	.remove = axi_dmac_remove,
};
module_platform_driver(axi_dmac_driver);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("DMA controller driver for the AXI-DMAC controller");
MODULE_LICENSE("GPL v2");
