#include <common.h>
#include <malloc.h>
#include <dma-ops.h>
#include <dma.h>

struct dma_device *current = NULL;

int dmadev_get_position(struct dma_chan *chan, dma_addr_t *src, dma_addr_t *dst)
{
	if (!current)
		return -1;

	return current->device_get_position(chan, src, dst);	
}

struct dma_chan *dmadev_request(struct dma_info *info)
{
	if (!current)
		return NULL;
	
	return current->device_alloc_chan_resources(info);
}

void dmadev_release(struct dma_chan *chan)
{
	if (!current)
		return;
	return current->device_free_chan_resources(chan);
}

int dmadev_prepare(struct dma_chan *chan, struct dma_prep_info *info)
{
	if (!current)
		return -1;
	
	if (info->mode == DMA_SLAVE)
		return current->device_prep_dma_slave(chan, info);
	else
		return current->device_prep_dma_cyclic(chan, info);
}

void dmadev_set_mode(struct dma_chan *chan, unsigned int  mode)
{
	if (!current)
		return;

	current->device_cfg_ctrl(chan, CFG_MODE, (unsigned int)mode);
}

void dmadev_set_burst(struct dma_chan *chan, int width, int depth)
{
	struct dma_burst burst;

	if (!current)
		return;
	
	burst.width = width;
	burst.depth = depth;
	current->device_cfg_ctrl(chan, CFG_BURST, (unsigned int)(&burst));
}

void dmadev_set_swap(struct dma_chan *chan, enum pl330_byteswap swap_mode)
{
	struct dma_burst burst;

	if (!current)
		return;
	
	if( swap_mode < SWAP_NO || swap_mode > SWAP_16 )
		burst.swap_mode = SWAP_NO;
	else
		burst.swap_mode = swap_mode;
	current->device_cfg_ctrl(chan, CFG_SWAP, (unsigned int)(&burst));
}
int dmadev_start(struct dma_chan *chan)
{
	if (!current)
		return -1;

	return current->device_cmd_ctrl(chan, DMA_START);
}

int dmadev_stop(struct dma_chan *chan)
{
	if (!current)
		return -1;

	return current->device_cmd_ctrl(chan, DMA_STOP);
}

int dmadev_get_status(struct dma_chan *chan)
{
	if (!current)
		return -1;

	return current->device_get_status(chan);
}

void dma_register(struct dma_device *dev)
{
	current = dev;
}
