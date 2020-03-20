/*
 * Copyright 2013-2015 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/dma-mapping.h>
#include <linux/sizes.h>
#include <linux/delay.h>

/*
 * For DMA buffers the storage is sub-divided into so called blocks. Each block
 * has its own memory buffer. The size of the block is the granularity at which
 * memory is exchanged between the hardware and the application. Increasing the
 * basic unit of data exchange from one sample to one block decreases the
 * management overhead that is associated with each sample. E.g. if we say the
 * management overhead for one exchange is x and the unit of exchange is one
 * sample the overhead will be x for each sample. Whereas when using a block
 * which contains n samples the overhead per sample is reduced to x/n. This
 * allows to achieve much higher samplerates than what can be sustained with
 * the one sample approach.
 *
 * Blocks are exchanged between the DMA controller and the application via the
 * means of two queues. The incoming queue and the outgoing queue. Blocks on the
 * incoming queue are waiting for the DMA controller to pick them up and fill
 * them with data. Block on the outgoing queue have been filled with data and
 * are waiting for the application to dequeue them and read the data.
 *
 * A block can be in one of the following states:
 *  * Owned by the application. In this state the application can read data from
 *    the block.
 *  * On the incoming list: Blocks on the incoming list are queued up to be
 *    processed by the DMA controller.
 *  * Owned by the DMA controller: The DMA controller is processing the block
 *    and filling it with data.
 *  * On the outgoing list: Blocks on the outgoing list have been successfully
 *    processed by the DMA controller and contain data. They can be dequeued by
 *    the application.
 *  * Dead: A block that is dead has been marked as to be freed. It might still
 *    be owned by either the application or the DMA controller at the moment.
 *    But once they are done processing it instead of going to either the
 *    incoming or outgoing queue the block will be freed.
 *
 * In addition to this blocks are reference counted and the memory associated
 * with both the block structure as well as the storage memory for the block
 * will be freed when the last reference to the block is dropped. This means a
 * block must not be accessed without holding a reference.
 *
 * The iio_dma_buffer implementation provides a generic infrastructure for
 * managing the blocks.
 *
 * A driver for a specific piece of hardware that has DMA capabilities need to
 * implement the submit() callback from the iio_dma_buffer_ops structure. This
 * callback is supposed to initiate the DMA transfer copying data from the
 * converter to the memory region of the block. Once the DMA transfer has been
 * completed the driver must call iio_dma_buffer_block_done() for the completed
 * block.
 *
 * Prior to this it must set the bytes_used field of the block contains
 * the actual number of bytes in the buffer. Typically this will be equal to the
 * size of the block, but if the DMA hardware has certain alignment requirements
 * for the transfer length it might choose to use less than the full size. In
 * either case it is expected that bytes_used is a multiple of the bytes per
 * datum, i.e. the block must not contain partial samples.
 *
 * The driver must call iio_dma_buffer_block_done() for each block it has
 * received through its submit_block() callback, even if it does not actually
 * perform a DMA transfer for the block, e.g. because the buffer was disabled
 * before the block transfer was started. In this case it should set bytes_used
 * to 0.
 *
 * In addition it is recommended that a driver implements the abort() callback.
 * It will be called when the buffer is disabled and can be used to cancel
 * pending and stop active transfers.
 *
 * The specific driver implementation should use the default callback
 * implementations provided by this module for the iio_buffer_access_funcs
 * struct. It may overload some callbacks with custom variants if the hardware
 * has special requirements that are not handled by the generic functions. If a
 * driver chooses to overload a callback it has to ensure that the generic
 * callback is called from within the custom callback.
 */

static uint64_t rfrx_timestamp0 = 0;
static uint64_t rfrx_timestamp1 = 0;
static uint64_t rfrx_timestamp2 = 0;
static uint64_t rfrx_timestamp3 = 0;

static unsigned int iio_dma_buffer_max_block_size = SZ_16M;
module_param_named(max_block_size, iio_dma_buffer_max_block_size, uint, 0644);

static void iio_buffer_block_release(struct kref *kref)
{
	struct iio_dma_buffer_block *block = container_of(kref,
		struct iio_dma_buffer_block, kref);

	WARN_ON(block->state != IIO_BLOCK_STATE_DEAD);

	dma_free_coherent(block->queue->dev, PAGE_ALIGN(block->block.size),
					block->vaddr, block->phys_addr);

	iio_buffer_put(&block->queue->buffer);
	kfree(block);
}

static void iio_buffer_block_get(struct iio_dma_buffer_block *block)
{
	kref_get(&block->kref);
}

static void iio_buffer_block_put(struct iio_dma_buffer_block *block)
{
	kref_put(&block->kref, iio_buffer_block_release);
}

/*
 * dma_free_coherent can sleep, hence we need to take some special care to be
 * able to drop a reference from an atomic context.
 */
static LIST_HEAD(iio_dma_buffer_dead_blocks);
static DEFINE_SPINLOCK(iio_dma_buffer_dead_blocks_lock);

static void iio_dma_buffer_cleanup_worker(struct work_struct *work)
{
	struct iio_dma_buffer_block *block, *_block;
	LIST_HEAD(block_list);

	spin_lock_irq(&iio_dma_buffer_dead_blocks_lock);
	list_splice_tail_init(&iio_dma_buffer_dead_blocks, &block_list);
	spin_unlock_irq(&iio_dma_buffer_dead_blocks_lock);

	list_for_each_entry_safe(block, _block, &block_list, head)
		iio_buffer_block_release(&block->kref);
}
static DECLARE_WORK(iio_dma_buffer_cleanup_work, iio_dma_buffer_cleanup_worker);

static void iio_buffer_block_release_atomic(struct kref *kref)
{
	struct iio_dma_buffer_block *block;
	unsigned long flags;

	block = container_of(kref, struct iio_dma_buffer_block, kref);

	spin_lock_irqsave(&iio_dma_buffer_dead_blocks_lock, flags);
	list_add_tail(&block->head, &iio_dma_buffer_dead_blocks);
	spin_unlock_irqrestore(&iio_dma_buffer_dead_blocks_lock, flags);

	schedule_work(&iio_dma_buffer_cleanup_work);
}

/*
 * Version of iio_buffer_block_put() that can be called from atomic context
 */
static void iio_buffer_block_put_atomic(struct iio_dma_buffer_block *block)
{
	kref_put(&block->kref, iio_buffer_block_release_atomic);
}

static struct iio_dma_buffer_queue *iio_buffer_to_queue(struct iio_buffer *buf)
{
	return container_of(buf, struct iio_dma_buffer_queue, buffer);
}

static struct iio_dma_buffer_block *iio_dma_buffer_alloc_block(
	struct iio_dma_buffer_queue *queue, size_t size)
{
	struct iio_dma_buffer_block *block;

	block = kzalloc(sizeof(*block), GFP_KERNEL);
	if (!block)
		return NULL;

	block->vaddr = dma_alloc_coherent(queue->dev, PAGE_ALIGN(size),
		&block->phys_addr, GFP_KERNEL);
	if (!block->vaddr) {
		kfree(block);
		return NULL;
	}

	block->block.size = size;
	block->state = IIO_BLOCK_STATE_DEQUEUED;
	block->queue = queue;
	INIT_LIST_HEAD(&block->head);
	kref_init(&block->kref);

	iio_buffer_get(&queue->buffer);

	return block;
}

static void _iio_dma_buffer_block_done(struct iio_dma_buffer_block *block)
{
	struct iio_dma_buffer_queue *queue = block->queue;

	/*
	 * The buffer has already been freed by the application, just drop the
	 * reference.
	 */
	if (block->state != IIO_BLOCK_STATE_DEAD) {
		block->state = IIO_BLOCK_STATE_DONE;
		list_add_tail(&block->head, &queue->outgoing);
	}
}

/**
 * iio_dma_buffer_block_done() - Indicate that a block has been completed
 * @block: The completed block
 *
 * Should be called when the DMA controller has finished handling the block to
 * pass back ownership of the block to the queue.
 */
void iio_dma_buffer_block_done(struct iio_dma_buffer_block *block)
{
	struct iio_dma_buffer_queue *queue = block->queue;
	unsigned long flags;

	spin_lock_irqsave(&queue->list_lock, flags);
	_iio_dma_buffer_block_done(block);
	spin_unlock_irqrestore(&queue->list_lock, flags);

	iio_buffer_block_put_atomic(block);
	wake_up_interruptible_poll(&queue->buffer.pollq, (uintptr_t)queue->poll_wakup_flags);
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_block_done);

/**
 * iio_dma_buffer_block_list_abort() - Indicate that a list block has been
 *   aborted
 * @queue: Queue for which to complete blocks.
 * @list: List of aborted blocks. All blocks in this list must be from @queue.
 *
 * Typically called from the abort() callback after the DMA controller has been
 * stopped. This will set bytes_used to 0 for each block in the list and then
 * hand the blocks back to the queue.
 */
void iio_dma_buffer_block_list_abort(struct iio_dma_buffer_queue *queue,
	struct list_head *list)
{
	struct iio_dma_buffer_block *block, *_block;
	unsigned long flags;

	spin_lock_irqsave(&queue->list_lock, flags);
	list_for_each_entry_safe(block, _block, list, head) {
		list_del(&block->head);
		block->block.bytes_used = 0;
		_iio_dma_buffer_block_done(block);
		iio_buffer_block_put_atomic(block);
	}
	spin_unlock_irqrestore(&queue->list_lock, flags);

	wake_up_interruptible_poll(&queue->buffer.pollq, POLLIN | POLLRDNORM);
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_block_list_abort);

static int iio_dma_buffer_fileio_alloc(struct iio_dma_buffer_queue *queue,
	struct iio_dev *indio_dev)
{
	size_t size = queue->buffer.bytes_per_datum * queue->buffer.length;
	struct iio_dma_buffer_block *block;

	block = iio_dma_buffer_alloc_block(queue, size);
	if (!block)
		return -ENOMEM;

	queue->fileio.active_block = block;
	queue->fileio.pos = 0;

	if (indio_dev->direction == IIO_DEVICE_DIRECTION_IN) {
		list_add_tail(&block->head, &queue->incoming);
		queue->poll_wakup_flags = POLLIN | POLLRDNORM;
	} else {
		queue->poll_wakup_flags = POLLOUT | POLLWRNORM;
	}

	return 0;
}

static void iio_dma_buffer_fileio_free(struct iio_dma_buffer_queue *queue)
{
	spin_lock_irq(&queue->list_lock);
	queue->fileio.active_block->state = IIO_BLOCK_STATE_DEAD;
	INIT_LIST_HEAD(&queue->incoming);
	INIT_LIST_HEAD(&queue->outgoing);
	spin_unlock_irq(&queue->list_lock);
	iio_buffer_block_put(queue->fileio.active_block);
	queue->fileio.active_block = NULL;
}



static void iio_dma_buffer_submit_block(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block *block)
{
	int ret;
        
	// Timestamp variabes
        uint64_t dac_inter_timestamp0;
        uint64_t dac_inter_timestamp1;
        uint64_t dac_inter_timestamp2;
        uint64_t dac_inter_timestamp3;

        uint64_t flag_val;
        uint64_t dac_to_nano_s = 10; // Counter clock frequency = 1e8, Seconds to Nanoseconds = 1e9
        uint64_t timestamp_val;
        uint32_t virt_addr;
        uint32_t virt_addr2;
        uint32_t virt_addr3;
        uint64_t* volatile pVal;        // writing in the virtual address
        uint64_t* volatile pVal2;        // writing in the virtual address
        uint64_t* volatile pVal3;        // writing in the virtual address
        uint64_t* p_var;

	struct iio_dma_buffer_block *dma_block = block;

	/*
	 * If the hardware has already been removed we put the block into
	 * limbo. It will neither be on the incoming nor outgoing list, nor will
	 * it ever complete. It will just wait to be freed eventually.
	 */
	if (!queue->ops)
		return;

	// Timestamp management: if the block.bytes_used > 0 then this function
	// sends a block to the DMA driver for the transmission or reception. ECL Samie
	// We have to take the timestamp value from the block and save it to
	// the inter_dac_timestamp variable
	//if(block->block.bytes_used > 0) // it is TX or RX
	if(block->block.size > 0) // it is TX or RX
	{
		// printk("^^^^^ iio_dma_buffer_submit_block, bytes_used: %d, before TX buffer to DMA, does not wait in the queue ^^^^^\n",block->block.bytes_used);
		// Writing the timestamp into the dma buffer TX
		// Find if it is TX. I insert a flag in [-12 to -8]
        	virt_addr3 = ((unsigned int)block->vaddr)+((unsigned int)(block->block.size)-24);
	        pVal3 = (uint64_t*) virt_addr3;
                flag_val = *pVal3;
		if(flag_val == 1234512345) // it is TX because of the flag
		{
			// Writing the timestamp into the dma buffer TX
			// copy user_buffer[last-4_to_last] to timestamp_val
			virt_addr = ((unsigned int)dma_block->vaddr)+((unsigned int)(dma_block->block.bytes_used)-8);
			pVal = (uint64_t*) virt_addr;
			timestamp_val = *pVal;

			// write timestamp_val to the inter driver register
			if(dma_block->block.id == 0)
				p_var = symbol_get(dac_inter_timestamp0);
			else if(dma_block->block.id == 1)
				p_var = symbol_get(dac_inter_timestamp1);
			else if(dma_block->block.id == 2)
				p_var = symbol_get(dac_inter_timestamp2);
			else if(dma_block->block.id == 3)
				p_var = symbol_get(dac_inter_timestamp3);
			else
				p_var = symbol_get(dac_inter_timestamp0);


			if(p_var)
			{
				// convert nanoseconds to clock cycles
				do_div(timestamp_val,dac_to_nano_s);
				*p_var = timestamp_val;

				// put back the p_var
				if(dma_block->block.id == 0)
					symbol_put(dac_inter_timestamp0);
				else if(dma_block->block.id == 1)
					symbol_put(dac_inter_timestamp1);
				else if(dma_block->block.id == 2)
					symbol_put(dac_inter_timestamp2);
				else if(dma_block->block.id == 3)
					symbol_put(dac_inter_timestamp3);
				else
					symbol_put(dac_inter_timestamp0);

			}
			else
				printk(KERN_INFO "[IIO/industrialio-dma-buf][write_buf] inter_module_get failed");

			//printk("[DEBUG][iio_dma_block] phys_addr: %u, virtual_addr: %u, bytes_used: %d, timestamp: %llu\n",(unsigned int)dma_block->phys_addr,(unsigned int)dma_block->vaddr,(int)dma_block->block.bytes_used,timestamp_val);
			// Writing the timestamp is done
				
			// copy user_buffer[last-8_to_last-4] to rfrx_timestamp variables
			timestamp_val = 0;
			virt_addr2 = ((unsigned int)dma_block->vaddr)+((unsigned int)(dma_block->block.bytes_used)-16);
			pVal2 = (uint64_t*) virt_addr2;
			timestamp_val = *pVal2;

			// write timestamp_val to the inter driver register
			if(dma_block->block.id == 0)
				rfrx_timestamp0 = timestamp_val;
			else if(dma_block->block.id == 1)
				rfrx_timestamp1 = timestamp_val;
			else if(dma_block->block.id == 2)
				rfrx_timestamp2 = timestamp_val;
			else if(dma_block->block.id == 3)
				rfrx_timestamp3 = timestamp_val;
			else
				rfrx_timestamp0 = timestamp_val;

			//printk("submitting a TX block, TX id: %d, rfrx timestamp: %llu\n",dma_block->block.id, timestamp_val);
			//printk("[DBG][iio_dma_buffer_submit_block][TX] id: %d, bytes_used: %d, timestamp: %llu \n",block->block.id,block->block.bytes_used,timestamp_val);
			
		}
	}

	block->state = IIO_BLOCK_STATE_ACTIVE;
	iio_buffer_block_get(block);
	ret = queue->ops->submit(queue, block);
	if (ret) {
		/*
		 * This is a bit of a problem and there is not much we can do
		 * other then wait for the buffer to be disabled and re-enabled
		 * and try again. But it should not really happen unless we run
		 * out of memory or something similar.
		 *
		 * TODO: Implement support in the IIO core to allow buffers to
		 * notify consumers that something went wrong and the buffer
		 * should be disabled.
		 */
		iio_buffer_block_put(block);
	}
}

/**
 * iio_dma_buffer_enable() - Enable DMA buffer
 * @buffer: IIO buffer to enable
 * @indio_dev: IIO device the buffer is attached to
 *
 * Needs to be called when the device that the buffer is attached to starts
 * sampling. Typically should be the iio_buffer_access_ops enable callback.
 *
 * This will allocate the DMA buffers and start the DMA transfers.
 */
int iio_dma_buffer_enable(struct iio_buffer *buffer,
	struct iio_dev *indio_dev)
{
	struct iio_dma_buffer_queue *queue = iio_buffer_to_queue(buffer);
	struct iio_dma_buffer_block *block, *_block;

	mutex_lock(&queue->lock);
	queue->active = true;

	/**
	 * If no buffer blocks are allocated when we start streaming go into
	 * fileio mode.
	 */
	if (!queue->num_blocks)
		iio_dma_buffer_fileio_alloc(queue, indio_dev);

	list_for_each_entry_safe(block, _block, &queue->incoming, head) {
		list_del(&block->head);
		iio_dma_buffer_submit_block(queue, block);
	}
	mutex_unlock(&queue->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_enable);

/**
 * iio_dma_buffer_disable() - Disable DMA buffer
 * @buffer: IIO DMA buffer to disable
 * @indio_dev: IIO device the buffer is attached to
 *
 * Needs to be called when the device that the buffer is attached to stops
 * sampling. Typically should be the iio_buffer_access_ops disable callback.
 */
int iio_dma_buffer_disable(struct iio_buffer *buffer,
	struct iio_dev *indio_dev)
{
	struct iio_dma_buffer_queue *queue = iio_buffer_to_queue(buffer);

	mutex_lock(&queue->lock);

	if (queue->fileio.active_block)
		iio_dma_buffer_fileio_free(queue);

	queue->active = false;

	if (queue->ops && queue->ops->abort)
		queue->ops->abort(queue);
	mutex_unlock(&queue->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_disable);

static void iio_dma_buffer_enqueue(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block *block)
{
	if (block->state == IIO_BLOCK_STATE_DEAD) {
		iio_buffer_block_put(block);
	} else if (queue->active) {
		iio_dma_buffer_submit_block(queue, block);
	} else {
		block->state = IIO_BLOCK_STATE_QUEUED;
		list_add_tail(&block->head, &queue->incoming);
	}
}

// This function gets called for every big block
static struct iio_dma_buffer_block *iio_dma_buffer_dequeue(
	struct iio_dma_buffer_queue *queue)
{
	struct iio_dma_buffer_block *block;

	spin_lock_irq(&queue->list_lock);
	block = list_first_entry_or_null(&queue->outgoing, struct
		iio_dma_buffer_block, head);
	if (block != NULL) {
		list_del(&block->head);
		block->state = IIO_BLOCK_STATE_DEQUEUED;
	}
	spin_unlock_irq(&queue->list_lock);

	return block;
}

/**
 * iio_dma_buffer_read() - DMA buffer read callback
 * @buffer: Buffer to read form
 * @n: Number of bytes to read
 * @user_buffer: Userspace buffer to copy the data to
 *
 * Should be used as the read_first_n callback for iio_buffer_access_ops
 * struct for DMA buffers.
 */

int iio_dma_buffer_read(struct iio_buffer *buffer, size_t n,
	char __user *user_buffer)
{
	struct iio_dma_buffer_queue *queue = iio_buffer_to_queue(buffer);
	struct iio_dma_buffer_block *block;
	int ret;

	if (n < buffer->bytes_per_datum)
		return -EINVAL;

	mutex_lock(&queue->lock);

	if (!queue->fileio.active_block) {
		ret = -EBUSY;
		goto out_unlock;
	}

	if (queue->fileio.active_block->state != IIO_BLOCK_STATE_DEQUEUED) {
		block = iio_dma_buffer_dequeue(queue);
		if (block == NULL) {
			ret = 0;
			goto out_unlock;
		}
		queue->fileio.pos = 0;
	} else {
		block = queue->fileio.active_block;
	}

	block = queue->fileio.active_block;
        
	n = rounddown(n, buffer->bytes_per_datum);
	if (n > block->block.bytes_used - queue->fileio.pos)
		n = block->block.bytes_used - queue->fileio.pos;

	if (copy_to_user(user_buffer, block->vaddr + queue->fileio.pos, n)) {
		ret = -EFAULT;
		goto out_unlock;
	}

	queue->fileio.pos += n;

	if (queue->fileio.pos == block->block.bytes_used)
		iio_dma_buffer_enqueue(queue, block);

	ret = n;

out_unlock:
	mutex_unlock(&queue->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_read);


int iio_dma_buffer_write(struct iio_buffer *buf, size_t n,
	const char __user *user_buffer)
{
	struct iio_dma_buffer_queue *queue = iio_buffer_to_queue(buf);
	struct iio_dma_buffer_block *block;
	int ret;

	if (n < buf->bytes_per_datum)
		return -EINVAL;

	mutex_lock(&queue->lock);

	if (!queue->fileio.active_block) {
		ret = -EBUSY;
		goto out_unlock;
	}

	if (queue->fileio.active_block->state != IIO_BLOCK_STATE_DEQUEUED) {
		block = iio_dma_buffer_dequeue(queue);
		if (block == NULL) {
			ret = 0;
			goto out_unlock;
		}
		queue->fileio.pos = 0;
	} else {
		block = queue->fileio.active_block;
	}

	block = queue->fileio.active_block;
	
	n = ALIGN(n, buf->bytes_per_datum);
	if (n > block->block.size - queue->fileio.pos)
		n = block->block.size - queue->fileio.pos;

	if (copy_from_user(block->vaddr + queue->fileio.pos, user_buffer, n)) {
		ret = -EFAULT;
		goto out_unlock;
	}


	queue->fileio.pos += n;

	if (queue->fileio.pos == block->block.size) {
		block->block.bytes_used = block->block.size;
		iio_dma_buffer_enqueue(queue, block);
	}

	ret = n;

out_unlock:
	mutex_unlock(&queue->lock);

	return ret;

}
EXPORT_SYMBOL_GPL(iio_dma_buffer_write);

/**
 * iio_dma_buffer_data_available() - DMA buffer data_available callback
 * @buf: Buffer to check for data availability
 *
 * Should be used as the data_available callback for iio_buffer_access_ops
 * struct for DMA buffers.
 */
size_t iio_dma_buffer_data_available(struct iio_buffer *buf)
{
	struct iio_dma_buffer_queue *queue = iio_buffer_to_queue(buf);
	struct iio_dma_buffer_block *block;
	size_t data_available = 0;

	/*
	 * For counting the available bytes we'll use the size of the block not
	 * the number of actual bytes available in the block. Otherwise it is
	 * possible that we end up with a value that is lower than the watermark
	 * but won't increase since all blocks are in use.
	 */

	mutex_lock(&queue->lock);
	if (queue->fileio.active_block)
		data_available += queue->fileio.active_block->block.size;

	spin_lock_irq(&queue->list_lock);
	list_for_each_entry(block, &queue->outgoing, head)
		data_available += block->block.size;
	spin_unlock_irq(&queue->list_lock);
	mutex_unlock(&queue->lock);

	return data_available;
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_data_available);

bool iio_dma_buffer_space_available(struct iio_buffer *buf)
{
	struct iio_dma_buffer_queue *queue = iio_buffer_to_queue(buf);
	bool space_available = false;

	mutex_lock(&queue->lock);
	if (queue->fileio.active_block &&
		queue->fileio.active_block->state == IIO_BLOCK_STATE_DEQUEUED)
		space_available = true;
	spin_lock_irq(&queue->list_lock);
	space_available |= !list_empty(&queue->outgoing);
	spin_unlock_irq(&queue->list_lock);
	mutex_unlock(&queue->lock);

	return space_available;
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_space_available);

int iio_dma_buffer_alloc_blocks(struct iio_buffer *buffer,
	struct iio_buffer_block_alloc_req *req)
{
	struct iio_dma_buffer_queue *queue = iio_buffer_to_queue(buffer);
	struct iio_dma_buffer_block **blocks;
	unsigned int num_blocks;
	unsigned int i;
	int ret = 0;

	mutex_lock(&queue->lock);

	if (queue->fileio.active_block) {
		ret = -EBUSY;
		goto err_unlock;
	}

	/* 64 blocks ought to be enough for anybody ;) */
	if (req->count > 64 - queue->num_blocks)
		req->count = 64 - queue->num_blocks;
	if (req->size > iio_dma_buffer_max_block_size)
		req->size = iio_dma_buffer_max_block_size;

	req->id = queue->num_blocks;

	if (req->count == 0 || req->size == 0) {
		ret = 0;
		goto err_unlock;
	}

	num_blocks = req->count + queue->num_blocks;

	blocks = krealloc(queue->blocks, sizeof(*blocks) * num_blocks,
			GFP_KERNEL);
	if (!blocks) {
		ret = -ENOMEM;
		goto err_unlock;
	}

	for (i = queue->num_blocks; i < num_blocks; i++) {
		blocks[i] = iio_dma_buffer_alloc_block(queue, req->size);
		if (!blocks[i])
			break;
		blocks[i]->block.id = i;
		blocks[i]->block.data.offset = queue->max_offset;
		queue->max_offset += PAGE_ALIGN(req->size);
	}

	req->count = i - queue->num_blocks;
	queue->num_blocks = i;
	queue->blocks = blocks;

err_unlock:
	mutex_unlock(&queue->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_alloc_blocks);

int iio_dma_buffer_free_blocks(struct iio_buffer *buffer)
{
	struct iio_dma_buffer_queue *queue = iio_buffer_to_queue(buffer);
	unsigned int i;

	mutex_lock(&queue->lock);

	spin_lock_irq(&queue->list_lock);
	INIT_LIST_HEAD(&queue->incoming);
	INIT_LIST_HEAD(&queue->outgoing);

	for (i = 0; i < queue->num_blocks; i++)
		queue->blocks[i]->state = IIO_BLOCK_STATE_DEAD;
	spin_unlock_irq(&queue->list_lock);

	for (i = 0; i < queue->num_blocks; i++)
		iio_buffer_block_put(queue->blocks[i]);

	kfree(queue->blocks);
	queue->blocks = NULL;
	queue->num_blocks = 0;
	queue->max_offset = 0;

	mutex_unlock(&queue->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_free_blocks);


int iio_dma_buffer_query_block(struct iio_buffer *buffer,
	struct iio_buffer_block *block)
{
	struct iio_dma_buffer_queue *queue = iio_buffer_to_queue(buffer);
	int ret = 0;

	mutex_lock(&queue->lock);

	if (block->id >= queue->num_blocks) {
		ret = -EINVAL;
		goto out_unlock;
	}

	*block = queue->blocks[block->id]->block;

out_unlock:
	mutex_unlock(&queue->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_query_block);


// Samie - Edge Computing Lab @ KTH
// Here the data is being copied from the user space buffer to the dma queue
// I add my timestamp which is already read from the dma-axi-dmac.c driver and inserted in the
// adc_inter_timestamp and dac_inter_timestamp shared variable.
// In order to do that, 4 last bytes of the buffer is considered to be the timestamp
// either in the read or write.
int iio_dma_buffer_enqueue_block(struct iio_buffer *buffer,
	struct iio_buffer_block *block)
{
	struct iio_dma_buffer_queue *queue = iio_buffer_to_queue(buffer);
	struct iio_dma_buffer_block *dma_block;
	int ret = 0;

	mutex_lock(&queue->lock);

	if (block->id >= queue->num_blocks) {
		ret = -EINVAL;
		goto out_unlock;
	}

	dma_block = queue->blocks[block->id];
	dma_block->block.bytes_used = block->bytes_used;
	dma_block->block.flags = block->flags;

	switch (dma_block->state) {
	case IIO_BLOCK_STATE_DONE:
		list_del_init(&dma_block->head);
		break;
	case IIO_BLOCK_STATE_QUEUED:
		/* Nothing to do */
		goto out_unlock;
	case IIO_BLOCK_STATE_DEQUEUED:
		break;
	default:
		ret = -EBUSY;
		goto out_unlock;
	}

	iio_dma_buffer_enqueue(queue, dma_block);

out_unlock:
	mutex_unlock(&queue->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_enqueue_block);

// Samie - Edge Computing Lab @ KTH
// Here the data is being copied from the dma queue to the user space buffer
// I add my timestamp which is alread read from the dma-axi-dmac.c driver and inserted in the
// adc_inter_timestamp and dac_inter_timestamp shared variable.
// In order to do that, 4 last bytes of the buffer is considered to be the timestamp
// either in the read or write.
int iio_dma_buffer_dequeue_block(struct iio_buffer *buffer,
	struct iio_buffer_block *block)
{
	// Timestamp variables
	uint64_t adc_inter_timestamp0;
	uint64_t adc_inter_timestamp1;
	uint64_t adc_inter_timestamp2;
	uint64_t adc_inter_timestamp3;
	// Timestamp variables
	uint64_t dif_inter_timestamp0;
	uint64_t dif_inter_timestamp1;
	uint64_t dif_inter_timestamp2;
	uint64_t dif_inter_timestamp3;
	int dif_inter_num;

        uint64_t adc_to_nano_s = 10;      // Counter clock frequency = 1e8, Seconds to Nanoseconds = 1e9
        uint64_t* p_var;
        uint64_t timestamp_val;
        uint32_t virt_addr;
        uint32_t virt_addr2;
        uint32_t virt_addr3;
        uint64_t* volatile pVal;        // writing in the virtual address
        uint64_t* volatile pVal3;        // writing in the virtual address
        uint64_t old_val;

	uint64_t* p_vard;
        uint64_t timestamp_vald;
        uint64_t* volatile pVald;        // writing in the virtual address
        uint64_t* volatile pVald2;        // writing in the virtual address
        uint64_t old_vald;
	
	int* p_vari;
	int id_vali;
        
	struct iio_dma_buffer_queue *queue = iio_buffer_to_queue(buffer);
	struct iio_dma_buffer_block *dma_block;
	int ret = 0;

	mutex_lock(&queue->lock);

	dma_block = iio_dma_buffer_dequeue(queue);
	if (!dma_block) {
		ret = -EAGAIN;
		goto out_unlock;
	}

	// Check if it is really RX (the TX flag is not there)
        virt_addr3 = ((unsigned int)dma_block->vaddr)+((unsigned int)(dma_block->block.bytes_used)-24);
        pVal3 = (uint64_t*) virt_addr3;

        // Reading the rx_timestamp to insert into the user buffer RX
	if(((int)dma_block->block.bytes_used > 0) && (*pVal3 != (uint64_t)1234512345)) // This is RX
        {
                timestamp_val = 0;
                
		// Reading the timestamp to insert into the user buffer
		if(dma_block->block.id == 0)
                	p_var = symbol_get(adc_inter_timestamp0);
                else if(dma_block->block.id == 1)
                	p_var = symbol_get(adc_inter_timestamp1);
                else if(dma_block->block.id == 2)
                	p_var = symbol_get(adc_inter_timestamp2);
                else if(dma_block->block.id == 3)
	                p_var = symbol_get(adc_inter_timestamp3);
		else
	                p_var = symbol_get(adc_inter_timestamp0);


                if(p_var)
                {
                        timestamp_val = *p_var;
                        // convert from cycles to nanoseconds
                        timestamp_val = timestamp_val*adc_to_nano_s;

			// put back the p_var
			if(dma_block->block.id == 0)
                		symbol_put(adc_inter_timestamp0);
	                else if(dma_block->block.id == 1)
                		symbol_put(adc_inter_timestamp1);
                	else if(dma_block->block.id == 2)
                		symbol_put(adc_inter_timestamp2);
        	        else if(dma_block->block.id == 3)
                		symbol_put(adc_inter_timestamp3);
			else
                		symbol_put(adc_inter_timestamp0);
	        }
                else
                        printk(KERN_INFO "[IIO/industrialio-dma-buf][read_buf] inter_module_get timestamp failed");

		// copy timestamp_val to user_buffer[last-4_to_last]
                virt_addr = ((unsigned int)dma_block->vaddr)+((unsigned int)(dma_block->block.bytes_used)-8);
                pVal = (uint64_t*) virt_addr;
                old_val = *pVal;
                *pVal = timestamp_val;

                // Reading the timestamp is done

		///////// Reporting the previous TX info
		// Get the ready tx block id
		p_vari = symbol_get(dif_inter_num);
		if(p_vari)
		{
			id_vali = *p_vari;
			symbol_put(dif_inter_num);
		}
		else
		{
			id_vali = 0;
        		printk(KERN_INFO "[IIO/industrialio-dma-buf][read_buf] inter_module_get id failed");
		}
		
		// Reading the timestamp to insert into the user buffer
                timestamp_vald = 0;
		if(id_vali == 0)
                	p_vard = symbol_get(dif_inter_timestamp0);
		else if(id_vali == 1)
			p_vard = symbol_get(dif_inter_timestamp1);
		else if(id_vali == 2)
			p_vard = symbol_get(dif_inter_timestamp2);
		else if(id_vali == 3)
			p_vard = symbol_get(dif_inter_timestamp3);
		else
			p_vard = symbol_get(dif_inter_timestamp0);


                if(p_vard)
                {
                        timestamp_vald = *p_vard;
                        // convert from cycles to nanoseconds
                        timestamp_vald = timestamp_vald*adc_to_nano_s;
			
			// put back the p_var
			if(id_vali == 0)
                		symbol_put(dif_inter_timestamp0);
			else if(id_vali == 1)
                		symbol_put(dif_inter_timestamp1);
			else if(id_vali == 2)
				symbol_put(dif_inter_timestamp2);
			else if(id_vali == 3)
				symbol_put(dif_inter_timestamp3);
			else
				symbol_put(dif_inter_timestamp0);
                }
                else
                        printk(KERN_INFO "[IIO/industrialio-dma-buf][read_buf] inter_module_get timestamp failed");

		// copy timestamp_val to user_buffer[last-8_to_last-4]
                virt_addr = ((unsigned int)dma_block->vaddr)+((unsigned int)(dma_block->block.bytes_used)-16);
                pVald = (uint64_t*) virt_addr;
                old_vald = *pVald;
                *pVald = timestamp_vald;
		
		// copy timestamp_val to user_buffer[last-12_to_last-8]
                virt_addr2 = ((unsigned int)dma_block->vaddr)+((unsigned int)(dma_block->block.bytes_used)-24);
                pVald2 = (uint64_t*) virt_addr2;
                old_vald = *pVald2;
		if(id_vali == 0)
                	*pVald2 = rfrx_timestamp0;
		else if(id_vali == 1)
                	*pVald2 = rfrx_timestamp1;
		else if(id_vali == 2)
                	*pVald2 = rfrx_timestamp2;
		else if(id_vali == 3)
                	*pVald2 = rfrx_timestamp3;
		else
                	*pVald2 = rfrx_timestamp0;

		//printk("dequeing an RX block, RX id: %d, TX report id: %d, rfrx timestamp: %llu\n",dma_block->block.id,id_vali,*pVald2);
                // Reading the timestamp is done
                //printk("[DEBUG][iio_dma_block] phys_addr: %u, virtual_addr: %u, bytes_used: %d, old_val_dif: %llu, tx_timestamp_dif: %llu\n",(unsigned int)dma_block->phys_addr,(unsigned int)dma_block->vaddr,(int)dma_block->block.bytes_used,old_vald,timestamp_vald);
        }
        // tx_timestamp_dif is inserted


	*block = dma_block->block;

out_unlock:
	mutex_unlock(&queue->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_dequeue_block);


static void iio_dma_buffer_mmap_open(struct vm_area_struct *area)
{
	struct iio_dma_buffer_block *block = area->vm_private_data;
	iio_buffer_block_get(block);
}

static void iio_dma_buffer_mmap_close(struct vm_area_struct *area)
{
	struct iio_dma_buffer_block *block = area->vm_private_data;
	iio_buffer_block_put(block);
}

static const struct vm_operations_struct iio_dma_buffer_vm_ops = {
	.open = iio_dma_buffer_mmap_open,
	.close = iio_dma_buffer_mmap_close,
};

int iio_dma_buffer_mmap(struct iio_buffer *buffer,
	struct vm_area_struct *vma)
{
	struct iio_dma_buffer_queue *queue = iio_buffer_to_queue(buffer);
	struct iio_dma_buffer_block *block = NULL;
	size_t vm_offset;
	unsigned int i;

	vm_offset = vma->vm_pgoff << PAGE_SHIFT;

	for (i = 0; i < queue->num_blocks; i++) {
		if (queue->blocks[i]->block.data.offset == vm_offset) {
			block = queue->blocks[i];
			break;
		}
	}

	if (block == NULL)
		return -EINVAL;

	if (PAGE_ALIGN(block->block.size) < vma->vm_end - vma->vm_start)
		return -EINVAL;

	vma->vm_pgoff = 0;
	
	vma->vm_flags |= VM_DONTEXPAND | VM_DONTDUMP;
	vma->vm_ops = &iio_dma_buffer_vm_ops;
	vma->vm_private_data = block;

	vma->vm_ops->open(vma);

	return dma_mmap_coherent(queue->dev, vma, block->vaddr,
		block->phys_addr, vma->vm_end - vma->vm_start);
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_mmap);

/**
 * iio_dma_buffer_set_bytes_per_datum() - DMA buffer set_bytes_per_datum callback
 * @buffer: Buffer to set the bytes-per-datum for
 * @bpd: The new bytes-per-datum value
 *
 * Should be used as the set_bytes_per_datum callback for iio_buffer_access_ops
 * struct for DMA buffers.
 */
int iio_dma_buffer_set_bytes_per_datum(struct iio_buffer *buffer, size_t bpd)
{
	buffer->bytes_per_datum = bpd;

	return 0;
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_set_bytes_per_datum);

/**
 * iio_dma_buffer_set_length - DMA buffer set_length callback
 * @buffer: Buffer to set the length for
 * @length: The new buffer length
 *
 * Should be used as the set_length callback for iio_buffer_access_ops
 * struct for DMA buffers.
 */
int iio_dma_buffer_set_length(struct iio_buffer *buffer, int length)
{
	/* Avoid an invalid state */
	if (length < 2)
		length = 2;
	buffer->length = length;

	return 0;
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_set_length);

static u64 dmamask = DMA_BIT_MASK(64);

/**
 * iio_dma_buffer_init() - Initialize DMA buffer queue
 * @queue: Buffer to initialize
 * @dev: DMA device
 * @ops: DMA buffer queue callback operations
 *
 * The DMA device will be used by the queue to do DMA memory allocations. So it
 * should refer to the device that will perform the DMA to ensure that
 * allocations are done from a memory region that can be accessed by the device.
 */
int iio_dma_buffer_init(struct iio_dma_buffer_queue *queue,
	struct device *dev, const struct iio_dma_buffer_ops *ops,
	void *driver_data)
{
	iio_buffer_init(&queue->buffer);
	queue->buffer.length = PAGE_SIZE;
	queue->buffer.watermark = queue->buffer.length / 2;
	queue->dev = dev;
	queue->ops = ops;
	queue->driver_data = driver_data;

	INIT_LIST_HEAD(&queue->incoming);
	INIT_LIST_HEAD(&queue->outgoing);

	mutex_init(&queue->lock);
	spin_lock_init(&queue->list_lock);

	if (!queue->dev->dma_mask)
		queue->dev->dma_mask = &dmamask;
	if (!queue->dev->coherent_dma_mask)
		queue->dev->coherent_dma_mask = DMA_BIT_MASK(64);

	return 0;
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_init);

/**
 * iio_dma_buffer_exit() - Cleanup DMA buffer queue
 * @queue: Buffer to cleanup
 *
 * After this function has completed it is safe to free any resources that are
 * associated with the buffer and are accessed inside the callback operations.
 */
void iio_dma_buffer_exit(struct iio_dma_buffer_queue *queue)
{
	mutex_lock(&queue->lock);
	queue->ops = NULL;
	mutex_unlock(&queue->lock);
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_exit);

/**
 * iio_dma_buffer_release() - Release final buffer resources
 * @queue: Buffer to release
 *
 * Frees resources that can't yet be freed in iio_dma_buffer_exit(). Should be
 * called in the buffers release callback implementation right before freeing
 * the memory associated with the buffer.
 */
void iio_dma_buffer_release(struct iio_dma_buffer_queue *queue)
{
	mutex_destroy(&queue->lock);
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_release);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("DMA buffer for the IIO framework");
MODULE_LICENSE("GPL v2");
