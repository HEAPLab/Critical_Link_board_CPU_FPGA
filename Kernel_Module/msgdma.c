#include "msgdma.h"

/* Utilitary functions */
static void setbit_reg32(volatile void __iomem *reg, u32 mask) {
    u32 val = ioread32(reg);
    iowrite32(val | mask, reg);
}

static void clearbit_reg32(volatile void __iomem *reg, u32 mask) {
    u32 val = ioread32(reg);

    iowrite32((val & (~mask)), reg);
}

static void msgdma_reset(struct msgdma_reg *reg) {
    setbit_reg32(&reg->csr_ctrl, RESET_DISPATCHER);
    while(ioread32(&reg->csr_status) & RESETTING);
}

static void
msgdma_push_descr(
        struct msgdma_reg *reg,
        dma_addr_t rd_addr,
        dma_addr_t wr_addr,
        u32 len,
        u32 ctrl) {
    iowrite32(rd_addr, &reg->desc_read_addr);
    iowrite32(wr_addr, &reg->desc_write_addr);
    iowrite32(len, &reg->desc_len);
    iowrite32(ctrl | GO, &reg->desc_ctrl);
}

static inline int send_dma_write(struct msgdma_data* data, int blocking, uint16_t constant_lines_per_buffer, dma_addr_t wr_dma_addr, size_t write_size) {
    uint16_t previous_write_offset;
    uint16_t newLines;
    int dec = 0;
    uint16_t curr_line_write_offset = data->curr_line_write_offset;
    uint16_t curr_line_in = data->curr_line_in;
    uint16_t curr_line_out = data->curr_line_out;
    atomic_t* msgdma_expected_remaining_read = &data->msgdma_expected_remaining_read;
    dma_addr_t dma_addr_rd;
    void* buf_wr;
    size_t i;
    uint16_t newLinesOut;
    uint16_t entriesOut;

    previous_write_offset = curr_line_write_offset;
    curr_line_write_offset = (curr_line_write_offset + write_size) % INPUT_FRAME_WIDTH;
    newLines = constant_lines_per_buffer + (previous_write_offset > curr_line_write_offset  ? 1 : 0);
    if((curr_line_in <= 1 && curr_line_in + newLines > 1) || (curr_line_in + newLines) > INPUT_FRAME_HEIGHT + 1) {
        dec = 1;
    }
    if(curr_line_in == 0 || (curr_line_in + newLines) > INPUT_FRAME_HEIGHT) {
        dec++;
    }

    newLinesOut = (newLines - dec);
    entriesOut = newLinesOut/2;

    if(curr_line_out % 2 != 0 && newLinesOut % 2 != 0)
        entriesOut++;

    if(!blocking && rbuffer_consume_available_count(&data->dma_buf_rd_handle_full_start) < entriesOut)
        return 0;

    for(i = 0; i < entriesOut; i++) {
        buf_wr = rbuffer_consume(&data->dma_buf_rd_handle_full_start, &dma_addr_rd, 1);
        if(!buf_wr) {
            printk(KERN_ERR "send_dma_write: Fatal error");
            return -1;
        }

        dma_sync_single_for_device(data->dev, dma_addr_rd, data->dma_buf_rd_handle_full_start.size, DMA_FROM_DEVICE);

        msgdma_push_descr(
                data->msgdma_write_to_ram_for_read_reg,
                0,
                dma_addr_rd,
                data->dma_buf_rd_handle_full_start.size,
                TX_COMPLETE_IRQ_EN
        );
    }

    dma_sync_single_for_device(data->dev, data->pending_write_dma_addr, write_size, DMA_TO_DEVICE);

    if (!test_and_set_bit(0, &data->writer_timer_running)) {
        data->writer_timer.expires = jiffies + MSGDMA_TIMEOUT;
        add_timer(&data->writer_timer);
    }

    if (!test_and_set_bit(0, &data->reader_timer_running)) {
        data->reader_timer.expires = jiffies + MSGDMA_TIMEOUT;
        add_timer(&data->reader_timer);
    }

    spin_lock_bh(&data->writer_push_descriptor_lock);
    atomic_add(newLines - dec, msgdma_expected_remaining_read);
    data->curr_line_in = (curr_line_in + newLines) % INPUT_FRAME_HEIGHT;
    data->curr_line_write_offset = curr_line_write_offset;
    data->curr_line_out = (curr_line_out + newLines - dec) % OUTPUT_FRAME_HEIGHT;

    barrier();

    rbuffer_consume(&data->dma_buf_wr_handle, NULL, 1);
    msgdma_push_descr(
            data->msgdma_read_from_ram_for_write_reg,
            wr_dma_addr,
            0,
            write_size,
            TX_COMPLETE_IRQ_EN
    );

    spin_unlock_bh(&data->writer_push_descriptor_lock);

    return 1;
}

static int send_pending_write(struct msgdma_data* data) {
    uint16_t size;
    int ret = 0;
    mutex_lock(&data->write_mutex);

    printk(KERN_DEBUG "send_pending_write start\n");

    if(data->pending_write_buffer) {
        size = data->dma_buf_wr_handle.size - data->pending_write_size; /* pending_write_size is the space left in the buffer to write. */

        if(send_dma_write(data, 1, size / INPUT_FRAME_WIDTH, data->pending_write_dma_addr, size)) {
            data->pending_write_buffer = NULL;
            printk(KERN_DEBUG "send_pending_write done %d\n", size);
        } else {
            ret = 1;
            printk(KERN_ERR "send_pending_write FAILED\n");
        }
    }

    mutex_unlock(&data->write_mutex);

    return ret;
}
static int msgdma_open(struct inode *node, struct file *f) {
    struct msgdma_data *data = container_of(node->i_cdev, struct msgdma_data, cdev);
    f->private_data = data;

    return 0;
}

static int msgdma_release(struct inode *node, struct file *f) {
    if((f->f_mode & FMODE_WRITE) != 0)
        send_pending_write((struct msgdma_data*) f->private_data);

    return 0;
}

static ssize_t msgdma_write(struct file *f, const char __user *ubuf, size_t len, loff_t *off) {
    struct msgdma_data *data;
    dma_addr_t dma_addr_wr;
    size_t to_write;
    size_t index = 0;
    rbuffer_handle_t* handle;
    size_t curr_amount;
    int err = 0;
    size_t buf_size;
    int full;
    void* buf_wr;
    void* pending_write_buffer;
    dma_addr_t pending_write_dma_addr;
    size_t pending_write_size;

    data = (struct msgdma_data*)f->private_data;

    if(f->f_flags & O_NONBLOCK) {
        if(!mutex_trylock(&data->write_mutex)) {
            err = -EAGAIN;
            goto _mutex_lock_failed;;
        }
    } else {
        mutex_lock(&data->write_mutex);
    }

    pending_write_buffer = data->pending_write_buffer;
    pending_write_dma_addr = data->pending_write_dma_addr;
    pending_write_size = data->pending_write_size;

    to_write = len;
    /* Make transfer to DMA, in a pipeline fashion */
    handle = &data->dma_buf_wr_handle;

    while(to_write > 0) {
        if(pending_write_buffer) {
            buf_wr = pending_write_buffer;
            dma_addr_wr = pending_write_dma_addr;
            buf_size = pending_write_size;

            pending_write_buffer = NULL;
        } else {
            buf_wr = rbuffer_get_tail(handle, &dma_addr_wr, f->f_flags & O_NONBLOCK || index != 0 ? 0 : 1);
            if(!buf_wr) {
                if(index == 0)
                    err = f->f_flags & O_NONBLOCK ? -EAGAIN : -ERESTARTSYS;
                goto _sgdma_write_fail;
            }

            buf_size = handle->size;

            dma_sync_single_for_cpu(data->dev, dma_addr_wr, buf_size, DMA_TO_DEVICE);
        }

        full = to_write >= buf_size;
        curr_amount = full ? buf_size : to_write;

        if(curr_amount > 0) {
            if (copy_from_user(buf_wr, ubuf + index, curr_amount) != 0) {
                err = -EFAULT;
                goto _sgdma_write_fail;
            }
        }

        to_write -= curr_amount;
        index += curr_amount;

        if(full) {
            if(!send_dma_write(data, f->f_flags & O_NONBLOCK || index != 0 ? 0 : 1, CONSTANT_LINES_PER_BUFFER, dma_addr_wr,handle->size)) {
                pending_write_buffer = buf_wr;
                pending_write_dma_addr = dma_addr_wr;
                pending_write_size = 0;

                if(index == 0)
                    err = f->f_flags & O_NONBLOCK ? -EAGAIN : -ERESTARTSYS;

                goto _sgdma_write_fail;
            }
        } else {
            pending_write_buffer = buf_wr + curr_amount;
            pending_write_dma_addr = dma_addr_wr;
            pending_write_size = buf_size - curr_amount;
        }
    }

_sgdma_write_fail:
    data->pending_write_buffer = pending_write_buffer;
    data->pending_write_dma_addr = pending_write_dma_addr;
    data->pending_write_size = pending_write_size;

    mutex_unlock(&data->write_mutex);
_mutex_lock_failed:

    if(err != 0)
        return err;
    return index;
}

static ssize_t msgdma_read(struct file *f, char __user *ubuf, size_t len, loff_t *off) {
    struct msgdma_data *data;
    void* rd_addr;
    dma_addr_t dma_addr_read;
    size_t to_read;
    size_t buf_size;
    size_t index = 0;
    size_t cur_len;
    int err = 0;
    int full;
    void* pending_read_buffer;
    dma_addr_t pending_read_dma_addr;
    size_t pending_read_size;

    data = (struct msgdma_data*) f->private_data;

    if(f->f_flags & O_NONBLOCK) {
        if(!mutex_trylock(&data->read_mutex)) {
            err = -EAGAIN;
            goto _read_mutex_lock_failed;
        }
    } else {
        mutex_lock(&data->read_mutex);
    }

    pending_read_buffer = data->pending_read_buffer;
    pending_read_dma_addr = data->pending_read_dma_addr;
    pending_read_size = data->pending_read_size;

    to_read = len;
    /* Initiate transfer */
    while(to_read > 0) {
        if(pending_read_buffer) {
            rd_addr = pending_read_buffer;
            dma_addr_read = pending_read_dma_addr;
            buf_size = pending_read_size;

            pending_read_buffer = NULL;
        } else {
            rd_addr = rbuffer_get_tail(&data->dma_buf_rd_handle_empty_start, &dma_addr_read, f->f_flags & O_NONBLOCK || index != 0  ? 0 : 1);
            buf_size = data->dma_buf_rd_handle_empty_start.size;

            if(!rd_addr) {
                if(index != 0) break;

                err = f->f_flags & O_NONBLOCK ? -EAGAIN : -ERESTARTSYS;
                goto _sgdma_read_fail;
            }

            dma_sync_single_for_cpu(data->dev, dma_addr_read, buf_size, DMA_FROM_DEVICE);
        }

        full = to_read >= buf_size;
        cur_len = full ? buf_size : to_read;

        if(copy_to_user(ubuf + index, rd_addr, cur_len) != 0) {
            err = -EFAULT;
            goto _sgdma_read_fail;
        }

        if(full) {
            rbuffer_consume(&data->dma_buf_rd_handle_empty_start, NULL, 1);
            rbuffer_produce(&data->dma_buf_rd_handle_full_start);
        } else {
            pending_read_buffer = rd_addr + cur_len;
            pending_read_dma_addr = dma_addr_read;
            pending_read_size = buf_size - cur_len;
        }

        to_read -= cur_len;
        index += cur_len;
    }

_sgdma_read_fail:
    data->pending_read_buffer = pending_read_buffer;
    data->pending_read_dma_addr = pending_read_dma_addr;
    data->pending_read_size = pending_read_size;

    mutex_unlock(&data->read_mutex);
_read_mutex_lock_failed:
    if(err != 0) return err;
    return index;
}

static unsigned int msgdma_poll(struct file *f, struct poll_table_struct *wait) {
    struct msgdma_data *dev = (struct msgdma_data*) f->private_data;
    unsigned int mask = 0;

    poll_wait(f, &dev->dma_buf_rd_handle_empty_start.consumer, wait);
    poll_wait(f, &dev->dma_buf_wr_handle.consumer, wait);
    poll_wait(f, &dev->dma_buf_rd_handle_full_start.consumer, wait);

    if(rbuffer_consume_available(&dev->dma_buf_rd_handle_empty_start))
        mask |= POLLIN | POLLRDNORM;

    if(rbuffer_consume_available(&dev->dma_buf_wr_handle) && rbuffer_consume_available_count(&dev->dma_buf_rd_handle_full_start) > CONSTANT_LINES_PER_BUFFER)
        mask |= POLLOUT | POLLWRNORM;

    return mask;
}

void msgdma_do_tasklet(unsigned long ptr) {
    int msgdma_rd_count;
    int msgdma_wr_count;
    size_t produceCount;
    struct msgdma_data* data = (struct msgdma_data *) ptr;
    int remaining_read;

    msgdma_wr_count = atomic_xchg(&data->msgdma_wr_count, 0);
    msgdma_rd_count = atomic_xchg(&data->msgdma_rd_count, 0);

    //printk(KERN_DEBUG "MSGDMA tasklet: called rd=%d wr=%d\n", msgdma_rd_count, msgdma_wr_count);

    if(msgdma_wr_count > 0) {
        rbuffer_produce_n(&data->dma_buf_wr_handle, msgdma_wr_count);

        if(test_bit(0, &data->writer_timer_running)) {
            mod_timer_pending(&data->writer_timer, jiffies + MSGDMA_TIMEOUT);
        }
    }

    if(msgdma_rd_count > 0) {
        spin_lock(&data->tasklet_update_remaining_read_lock); //both this and the read_timer_expired modify msgdma_expected_remaining_read non atomically.

        remaining_read = atomic_read(&data->msgdma_expected_remaining_read) / LINES_PER_OUTPUT_BUFFER;
        msgdma_rd_count = msgdma_rd_count > remaining_read ? remaining_read : msgdma_rd_count;
        produceCount = rbuffer_produce_n(&data->dma_buf_rd_handle_empty_start, msgdma_rd_count);
        atomic_sub(produceCount*LINES_PER_OUTPUT_BUFFER, &data->msgdma_expected_remaining_read);

        spin_unlock(&data->tasklet_update_remaining_read_lock);

        if(test_bit(0, &data->reader_timer_running)) {
            mod_timer_pending(&data->reader_timer, jiffies + MSGDMA_TIMEOUT);
        }
    }
}

static irqreturn_t msgdma_irq_handler_0(int irq, void *dev_id) {
    struct msgdma_data *data = (struct msgdma_data*) dev_id;
    struct msgdma_reg *msgdma0_reg = data->msgdma_read_from_ram_for_write_reg;

    /* Acknowledge corresponding DMA, and wake up whoever is waiting */
    if(ioread32(&msgdma0_reg->csr_status) & IRQ) {
        atomic_inc(&data->msgdma_wr_count);
        tasklet_schedule(&data->tasklet);
        barrier();
        setbit_reg32(&msgdma0_reg->csr_status, IRQ);
    }

    return IRQ_HANDLED;
}

static irqreturn_t msgdma_irq_handler_1(int irq, void *dev_id) {
    struct msgdma_data *data = (struct msgdma_data*) dev_id;
    struct msgdma_reg *msgdma1_reg = data->msgdma_write_to_ram_for_read_reg;

    if(ioread32(&msgdma1_reg->csr_status) & IRQ) {
        atomic_inc(&data->msgdma_rd_count);
        tasklet_schedule(&data->tasklet);
        barrier();
        setbit_reg32(&msgdma1_reg->csr_status, IRQ);
    }

    return IRQ_HANDLED;
}

static int msgdma_register_chrdev(struct msgdma_data *data) {
    int ret = 0;

    ret = alloc_chrdev_region(&data->dev_id, 0, 1, DEV_NAME);
    if(ret < 0) {
        pr_err("Character device region allocation failed\n");
        goto _ret;
    }

    /* Actual registering of the device. At this point it must be
     * fully initialized */
    cdev_init(&(data->cdev), &msgdma_fops);
    ret = cdev_add(&(data->cdev), data->dev_id, 1);
    if(ret < 0) {
        pr_err("Character device initialization failed\n");
        goto _cdev_add_err;
    }

    return 0;

    _cdev_add_err:
    unregister_chrdev_region(data->dev_id, 1);

    _ret:
    return ret;
}

static void msgdma_unregister_chrdev(struct msgdma_data *data) {
    cdev_del(&data->cdev);
    unregister_chrdev_region(data->dev_id, 1);
}

static void reader_timer_expired(ulong l) {
    struct msgdma_data *data = (struct msgdma_data*) l;
    u32 csr_0;
    int rem;
    size_t produceCount;

    if(atomic_read(&data->msgdma_rd_count) > 0) {
        msgdma_do_tasklet(l);
        return;
    }

    clear_bit(0, &data->reader_timer_running);

    spin_lock(&data->writer_push_descriptor_lock); //it prevents the char device write to issue more write descriptors during this period.
    spin_lock(&data->tasklet_update_remaining_read_lock); //both the tasklet and this function modify the msgdma_expected_remaining_read non atomically.

    rem = atomic_read(&data->msgdma_expected_remaining_read) / LINES_PER_OUTPUT_BUFFER;
    csr_0 = ioread32(&data->msgdma_read_from_ram_for_write_reg->csr_status);

    if((csr_0 & BUSY) == 0) { //if the the msgdma has finished reading the image -> everything is ready.
        if(rem > 0) {
            produceCount = rbuffer_produce_n(&data->dma_buf_rd_handle_empty_start, rem);
            atomic_sub(produceCount*LINES_PER_OUTPUT_BUFFER, &data->msgdma_expected_remaining_read);
            printk(KERN_DEBUG "reader_timer_expired: +%d lines\n", produceCount*LINES_PER_OUTPUT_BUFFER);
        }
    } else {
        if(!test_and_set_bit(0, &data->reader_timer_running)) {
            data->reader_timer.expires = jiffies + MSGDMA_TIMEOUT;
            add_timer(&data->reader_timer);
        }
    }
    spin_unlock(&data->tasklet_update_remaining_read_lock);
    spin_unlock(&data->writer_push_descriptor_lock);
}

static void writer_timer_expired(ulong l) {
    struct msgdma_data *data = (struct msgdma_data*) l;
    u32 csr_0;
    size_t pending;

    if(atomic_read(&data->msgdma_wr_count) > 0) {
        msgdma_do_tasklet(l);
        return;
    }

    clear_bit(0, &data->writer_timer_running);

    spin_lock(&data->writer_push_descriptor_lock); //it prevents the char device write to issue more write descriptors during this period.
    pending = rbuffer_produce_available_count(&data->dma_buf_wr_handle);
    csr_0 = ioread32(&data->msgdma_read_from_ram_for_write_reg->csr_status);

    if((csr_0 & BUSY) == 0) {
        if(pending > 0) {
            printk(KERN_DEBUG "writer_timer_expired +%d produce\n", pending);
            rbuffer_produce_n(&data->dma_buf_wr_handle, pending);
        }
    } else {
        if (!test_and_set_bit(0, &data->writer_timer_running)) {
            data->writer_timer.expires = jiffies + MSGDMA_TIMEOUT;
            add_timer(&data->writer_timer);
        }
    }

    spin_unlock(&data->writer_push_descriptor_lock);
}

static int msgdma_probe(struct platform_device *pdev) {
    struct msgdma_data *data;
    struct resource *res;
    struct resource *region;
    struct device *dev;
    int ret = 0;
    int err = 0;
    dev = &pdev->dev;

    printk(KERN_INFO "MSGDMA probe: init.\n");

    data = (struct msgdma_data*)devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
    if(data == NULL)
        return -ENOMEM;

    data->pending_write_buffer = NULL;
    data->pending_read_buffer = NULL;
    data->dev = dev;

    data->writer_timer_running = 0;
    init_timer(&data->writer_timer);
    data->writer_timer.data = (long) data;
    data->writer_timer.function = &writer_timer_expired;

    data->reader_timer_running = 0;
    init_timer(&data->reader_timer);
    data->reader_timer.data = (long) data;
    data->reader_timer.function = &reader_timer_expired;

    data->curr_line_in = 0;
    data->curr_line_out = 0;
    data->curr_line_write_offset = 0;
    atomic_set(&data->msgdma_expected_remaining_read, 0);

    atomic_set(&data->msgdma_rd_count, 0);
    atomic_set(&data->msgdma_wr_count, 0);

    spin_lock_init(&data->writer_push_descriptor_lock);
    spin_lock_init(&data->tasklet_update_remaining_read_lock);
    tasklet_init(&data->tasklet, msgdma_do_tasklet, (unsigned long) data);

    mutex_init(&data->read_mutex);
    mutex_init(&data->write_mutex);

    platform_set_drvdata(pdev, (void*)data);

    /* Prepare DMA buffers */
    dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));

    err = two_way_rbuffer_init(&data->dma_buf_rd_handle_full_start, &data->dma_buf_rd_handle_empty_start, dev, OUTPUT_BUFFER_SIZE);
    if(err) {
        printk(KERN_ERR "MSGDMA probe: dma_buf_rd_handle FAILED.\n");
        ret = -ENOMEM;
        goto fail;
    }

    printk(KERN_INFO "MSGDMA probe: dma_buf_rd_handle initialized.\n");

    err = rbuffer_init(&data->dma_buf_wr_handle, dev, INPUT_BUFFER_SIZE);
    if(err) {
        printk(KERN_ERR "MSGDMA probe: dma_buf_wr_handle FAILED.\n");
        ret = -ENOMEM;
        goto fail;
    }

    printk(KERN_INFO "MSGDMA probe: dma_buf_wr_handle initialized.\n");

    /* Remap IO region of the device */
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if(res == NULL) {
        printk(KERN_ERR "MSGDMA probe: platform_get_resource FAILED.\n");
        dev_err(&pdev->dev, "io region resource not defined");
        return -ENODEV;
    }

    region = devm_request_mem_region(
            dev,
            res->start,
            resource_size(res),
            dev_name(dev)
    );

    if(region == NULL) {
        printk(KERN_ERR "MSGDMA probe: devm_request_mem_region FAILED.\n");
        dev_err(dev, "mem region not requested");
        return -EBUSY;
    }

    printk(KERN_ERR "ADDR START=0x%x . IRQ1=%d IRQ2=%d\n", region->start, platform_get_irq(pdev, 0), platform_get_irq(pdev, 1));

    data->msgdma_read_from_ram_for_write_reg = devm_ioremap_nocache(dev, region->start, MSGDMA_MAP_SIZE);
    if(data->msgdma_read_from_ram_for_write_reg <= 0) {
        printk(KERN_ERR "MSGDMA probe: could not remap io region msgdma_read_from_ram_for_write_reg.\n");
        dev_err(dev, "could not remap io region");
        return -EFAULT;
    }

    data->msgdma_write_to_ram_for_read_reg = devm_ioremap_nocache(dev, region->start + MSGDMA1_OFFSET, MSGDMA_MAP_SIZE);
    if(data->msgdma_write_to_ram_for_read_reg <= 0) {
        printk(KERN_ERR "MSGDMA probe: could not remap io region msgdma_write_to_ram_for_read_reg.\n");
        dev_err(dev, "could not remap io region");
        return -EFAULT;
    }

    /* Initialize the device itself */
    msgdma_reset(data->msgdma_read_from_ram_for_write_reg);
    msgdma_reset(data->msgdma_write_to_ram_for_read_reg);

    setbit_reg32(&data->msgdma_read_from_ram_for_write_reg->csr_ctrl,
                 STOP_ON_EARLY_TERM | STOP_ON_ERROR | GLOBAL_INT_EN_MASK);
    setbit_reg32(&data->msgdma_write_to_ram_for_read_reg->csr_ctrl,
                 STOP_ON_EARLY_TERM | STOP_ON_ERROR | GLOBAL_INT_EN_MASK);

    printk(KERN_INFO "MSGDMA probe: msgdma reset\n");

    /* Get device's irq number(s) */
    data->msgdma0_irq = platform_get_irq(pdev, 0);
    if(data->msgdma0_irq < 0) {
        printk(KERN_ERR "MSGDMA probe: could not get irq number 0\n");
        return -ENXIO;
    }

    ret = devm_request_irq(dev, data->msgdma0_irq, msgdma_irq_handler_0, IRQF_SHARED, "msgdma0", data);
    if(ret < 0) {
        dev_err(dev, "Could not request irq %d", data->msgdma0_irq);
        return ret;
    }

    data->msgdma1_irq = platform_get_irq(pdev, 1);
    if(data->msgdma1_irq < 0) {
        printk(KERN_ERR "MSGDMA probe: could not get irq number 1\n");
        return -ENXIO;
    }

    ret = devm_request_irq(dev, data->msgdma1_irq, msgdma_irq_handler_1, IRQF_SHARED, "msgdma1", data);
    if(ret < 0) {
        dev_err(dev, "Could not request irq %d", data->msgdma1_irq);
        return ret;
    }

    ret = msgdma_register_chrdev(data);
    if(ret < 0)
        return ret;

    data->proc_dir = proc_mkdir(pdev->name, NULL);
    if(!data->proc_dir) {
        printk(KERN_ERR "Could not create proc dir for %s\n", pdev->name);
        return ret;
    } else {
        data->proc_file = proc_create_data("msgdma_csr_rx", 0, data->proc_dir, &proc_fops, data);
        if(!data->proc_file) {
            printk(KERN_ERR "Could not create proc file for %s\n", pdev->name);
        } else {
            printk(KERN_INFO "Proc file created successfully %s!", pdev->name);
        }
    }

    printk(KERN_INFO "MSGDMA probe: DONE!\n");

    return 0;

    fail:
    msgdma_remove(pdev);

    return ret;
}

static int msgdma_remove(struct platform_device *pdev) {
    struct msgdma_data *data = (struct msgdma_data *) platform_get_drvdata(pdev);

    del_timer_sync(&data->reader_timer);
    del_timer_sync(&data->writer_timer);

    msgdma_unregister_chrdev(data);

    two_way_rbuffer_free(&data->dma_buf_rd_handle_full_start, &data->dma_buf_rd_handle_empty_start, &pdev->dev);
    rbuffer_free(&data->dma_buf_wr_handle, &pdev->dev);

    if (data->proc_file) proc_remove(data->proc_file);
    if (data->proc_dir) proc_remove(data->proc_dir);

    return 0;
}

static int msgdma_proc_open(struct inode *inode, struct file *file) {
    return single_open(file, msgdma_proc_show, PDE_DATA(inode));
}

void print_csr(void* m, char* title, u32 csr) {
    seq_printf(m, title);
    seq_printf(m, "   irq:                %d\n", (csr & IRQ) != 0);
    seq_printf(m, "   stopped_early_term: %d\n", (csr & STOPPED_EARLY_TERM) != 0);
    seq_printf(m, "   stopped_on_err:     %d\n", (csr & STOPPED_ON_ERR) != 0);
    seq_printf(m, "   resetting:          %d\n", (csr & RESETTING) != 0);
    seq_printf(m, "   stopped:            %d\n", (csr & STOPPED) != 0);
    seq_printf(m, "   resp_buf_full:      %d\n", (csr & RESP_BUF_FULL) != 0);
    seq_printf(m, "   resp_buf_full:      %d\n", (csr & RESP_BUF_EMPTY) != 0);
    seq_printf(m, "   descr_buf_full:     %d\n", (csr & DESCR_BUF_FULL) != 0);
    seq_printf(m, "   descr_buf_empty:    %d\n", (csr & DESCR_BUF_EMPTY) != 0);
    seq_printf(m, "   busy:               %d\n", (csr & BUSY) != 0);
}

static int msgdma_fsync(struct file * f, loff_t, loff_t, int datasync) {
    return send_pending_write((struct msgdma_data*) f->private_data);
}

static int msgdma_proc_show(struct seq_file *m, void *v) {
    struct msgdma_data *data = m->private;

    u32 msgdma_read_from_ram_for_write_reg = ioread32(&data->msgdma_read_from_ram_for_write_reg->csr_status);
    u32 msgdma_write_to_ram_for_read_reg = ioread32(&data->msgdma_write_to_ram_for_read_reg->csr_status);

    seq_printf(m, "pending read size:     %d\n", data->pending_read_buffer ? data->pending_read_size : 0);
    seq_printf(m, "pending write size:    %d\n", data->pending_write_buffer ? data->dma_buf_wr_handle.size - data->pending_write_size : 0);
    seq_printf(m, "msgdma_rd_count:       %d\n", atomic_read(&data->msgdma_rd_count));
    seq_printf(m, "msgdma_wr_count:       %d\n", atomic_read(&data->msgdma_wr_count));
    seq_printf(m, "rd_rbuf_consume_availe:%d\n", rbuffer_consume_available_count(&data->dma_buf_rd_handle_empty_start));
    seq_printf(m, "rd_rbuf_consume_availf:%d\n", rbuffer_consume_available_count(&data->dma_buf_rd_handle_full_start));
    seq_printf(m, "wr_rbuf_consume_avail: %d\n", rbuffer_consume_available_count(&data->dma_buf_wr_handle));
    seq_printf(m, "curr_line_in:          %d\n", data->curr_line_in);
    seq_printf(m, "curr_line_write_offset:%d\n", data->curr_line_write_offset);
    seq_printf(m, "dma_expected_rem_read: %d\n", atomic_read(&data->msgdma_expected_remaining_read));
    seq_printf(m, "reader_timer_running:  %d\n", test_bit(1, &data->reader_timer_running));
    seq_printf(m, "writer_timer_running:  %d\n", test_bit(1, &data->writer_timer_running));

    print_csr(m, "msgdma_read_from_ram_for_write_reg:\n", msgdma_read_from_ram_for_write_reg);
    print_csr(m, "msgdma_write_to_ram_for_read_reg:\n", msgdma_write_to_ram_for_read_reg);
    return 0;
}

static int __init msgdma_init(void) {
    return platform_driver_register(&msgdma_driver);
}

static void __exit msgdma_exit(void) {
    platform_driver_unregister(&msgdma_driver);
}

subsys_initcall(msgdma_init);
module_exit(msgdma_exit);

MODULE_DESCRIPTION("MSGDMA driver");
MODULE_AUTHOR("Matteo Carrara");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");