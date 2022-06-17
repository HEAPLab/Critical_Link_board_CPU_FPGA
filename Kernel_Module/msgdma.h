#ifndef MSGDMA_H
#define MSGDMA_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <asm/io.h>
#include "circular_buffer.h"
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "two_way_circular_buffer.h"

#define DEV_NAME            "msgdma"

#define MSGDMA_MAP_SIZE         0x30
#define MSGDMA1_OFFSET          0x40

#define INPUT_BUFFER_SIZE 4096
#define INPUT_FRAME_WIDTH 1920
#define INPUT_FRAME_HEIGHT 1080
#define OUTPUT_FRAME_WIDTH (INPUT_FRAME_WIDTH - 2)
#define OUTPUT_FRAME_HEIGHT (INPUT_FRAME_HEIGHT - 2)
#define LINES_PER_OUTPUT_BUFFER 2
#define OUTPUT_BUFFER_SIZE (LINES_PER_OUTPUT_BUFFER * OUTPUT_FRAME_WIDTH)
#define CONSTANT_LINES_PER_BUFFER (INPUT_BUFFER_SIZE/INPUT_FRAME_WIDTH)
#define MSGDMA_TIMEOUT (HZ)

typedef u32 volatile reg_t;

#pragma pack(1)
struct msgdma_reg {
    /* CSR port Registers */
    reg_t csr_status;
    reg_t csr_ctrl;
    reg_t csr_fill_lvl;
    reg_t csr_resp_fill_lvl;
    reg_t csr_seq_num;
    reg_t csr_comp_config1;
    reg_t csr_comp_config2;
    reg_t csr_comp_info;

    /* Descriptor port registers (standard format) */
    reg_t desc_read_addr;
    reg_t desc_write_addr;
    reg_t desc_len;
    reg_t desc_ctrl;

    /* Response port registers */
    reg_t resp_bytes_transferred;
    reg_t resp_status; 
};
#pragma pack()

/* MSGDMA Register bit fields */
enum STATUS {
    IRQ                 = (1 << 9),
    STOPPED_EARLY_TERM  = (1 << 8),
    STOPPED_ON_ERR      = (1 << 7),
    RESETTING           = (1 << 6),
    STOPPED             = (1 << 5),
    RESP_BUF_FULL       = (1 << 4),
    RESP_BUF_EMPTY      = (1 << 3),
    DESCR_BUF_FULL      = (1 << 2),
    DESCR_BUF_EMPTY     = (1 << 1),
    BUSY                = (1 << 0),
};

enum CONTROL {
    STOP_DESCR          = (1 << 5),
    GLOBAL_INT_EN_MASK  = (1 << 4),
    STOP_ON_EARLY_TERM  = (1 << 3),
    STOP_ON_ERROR       = (1 << 2),
    RESET_DISPATCHER    = (1 << 1),
    STOP_DISPATCHER     = (1 << 0),
};

enum DESC_CTRL {
    GO                  = (1 << 31),
    EARLY_DONE_EN       = (1 << 24),
    TX_ERR_IRQ_EN       = (1 << 16),
    EARLY_TERM_IRQ_EN   = (1 << 15),
    TX_COMPLETE_IRQ_EN  = (1 << 14),
    END_ON_EOP          = (1 << 12),
    PARK_WR             = (1 << 11),
    PARK_RD             = (1 << 10),
    GEN_EOP             = (1 << 9),
    GEN_SOP             = (1 << 8),
    // TODO : TX CHAN
};

struct dma_read_data_t {
	dma_addr_t dma_handle;
	void* buf;
	int last;
};

/* Driver private data */
struct msgdma_data {
    dev_t dev_id;
    struct device* dev;
    struct cdev cdev;

    struct msgdma_reg *msgdma_read_from_ram_for_write_reg;
    struct msgdma_reg *msgdma_write_to_ram_for_read_reg;

    int msgdma0_irq;
    int msgdma1_irq;

    rbuffer_handle_t dma_buf_wr_handle;

    rbuffer_handle_t dma_buf_rd_handle_full_start;
    rbuffer_handle_t dma_buf_rd_handle_empty_start;

    void* pending_read_buffer;
    dma_addr_t pending_read_dma_addr;
    size_t pending_read_size;

    void* pending_write_buffer;
    dma_addr_t pending_write_dma_addr;
    size_t pending_write_size;

    ulong writer_timer_running;
    struct timer_list writer_timer;

    ulong reader_timer_running;
    struct timer_list reader_timer;

    spinlock_t tasklet_update_remaining_read_lock;
    uint16_t curr_line_in;
    uint16_t curr_line_out;
    uint16_t curr_line_write_offset;
    atomic_t msgdma_expected_remaining_read;

    atomic_t msgdma_rd_count;

    atomic_t msgdma_wr_count;

    spinlock_t writer_push_descriptor_lock;
    struct tasklet_struct tasklet;
    struct proc_dir_entry *proc_dir;
    struct proc_dir_entry *proc_file;

    struct mutex write_mutex;
    struct mutex read_mutex;

    unsigned long notBusyJiffy;
};

/* Function declarations */
static int msgdma_open(struct inode *node, struct file *f);
static int msgdma_release(struct inode *node, struct file *f);
static ssize_t msgdma_read(struct file *f, char __user *ubuf, size_t len, loff_t *off);
static ssize_t msgdma_write(struct file *f, const char __user *ubuf, size_t len, loff_t *off);
static unsigned int msgdma_poll(struct file *f, struct poll_table_struct *wait);
static int msgdma_probe(struct platform_device *pdev);
static int msgdma_remove(struct platform_device *pdev);
static int msgdma_proc_open(struct inode *inode, struct file *file);
static int msgdma_proc_show(struct seq_file *m, void *v);
static int msgdma_fsync(struct file *, loff_t, loff_t, int datasync);

static const struct file_operations msgdma_fops = {
    .owner      = THIS_MODULE,
    .open       = msgdma_open,
    .release    = msgdma_release,
    .read       = msgdma_read,
    .write      = msgdma_write,
    .poll       = msgdma_poll,
    .fsync      = msgdma_fsync
};

struct file_operations proc_fops = {
        .owner = THIS_MODULE,
        .open = msgdma_proc_open,
        .read =    seq_read,
        .llseek = seq_lseek,
        .release = single_release
};

static const struct of_device_id msgdma_of_match[] = {
    {.compatible = "msgdma_test" },
    {}
};

static struct platform_driver msgdma_driver = {
    .probe = msgdma_probe,
    .remove = msgdma_remove,
    .driver = {
        .name = DEV_NAME,
        .of_match_table = msgdma_of_match,
    },
};
#endif