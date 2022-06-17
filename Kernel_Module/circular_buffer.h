#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include "linux/jiffies.h"
#include "linux/spinlock_types.h"
#include "linux/workqueue.h"
#include <asm/io.h>
#include <linux/circ_buf.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/gfp.h>

/* The following circular buffer follows the implementation suggested by Howells
 * and Mc.Kenney in
 *
 * https://www.kernel.org/doc/html/latest/core-api/circular-buffers.html
 *
 */

//THIS IS MEM TO STORAGE

#define RING_BUFFER_SIZE 256 //Must be power of 2

typedef struct rbuffer_entry {
    void* buf;
    dma_addr_t dmas;
} rbuffer_entry;

typedef struct rbuffer_handle_t {
    rbuffer_entry entry[RING_BUFFER_SIZE];
    int head;
    int tail;
    wait_queue_head_t consumer;
    spinlock_t cons_lock;
    spinlock_t prod_lock;
    size_t size;
} rbuffer_handle_t;

inline int rbuffer_init(rbuffer_handle_t* handle, struct device *dev, size_t size) {
    int i;
    for(i = 0; i < RING_BUFFER_SIZE; i++) {
        handle->entry[i].buf = dma_alloc_noncoherent(dev, size, &handle->entry[i].dmas, GFP_KERNEL);
        if(!handle->entry[i].buf)
            return 1;
    }
    handle->head = RING_BUFFER_SIZE - 1;
    handle->tail = 0;
    handle->size = size;
    init_waitqueue_head(&handle->consumer);
    spin_lock_init(&handle->cons_lock);
    spin_lock_init(&handle->prod_lock);
    return 0;
}

inline void rbuffer_free(rbuffer_handle_t* handle, struct device *dev) {
    int i;
    for(i = 0; i < RING_BUFFER_SIZE; i++) {
        dma_free_noncoherent(dev, handle->size, handle->entry[i].buf, handle->entry[i].dmas);
    }
}

inline int rbuffer_consume_available(rbuffer_handle_t* buffer) {
    unsigned long head, tail;
    head = READ_ONCE(buffer->head);
    tail = READ_ONCE(buffer->tail);
    return CIRC_CNT(head, tail, RING_BUFFER_SIZE) >= 1;
}

inline size_t rbuffer_consume_available_count(rbuffer_handle_t* buffer) {
    unsigned long head, tail;
    head = READ_ONCE(buffer->head);
    tail = READ_ONCE(buffer->tail);
    return CIRC_CNT(head, tail, RING_BUFFER_SIZE);
}

inline size_t rbuffer_produce_available_count(rbuffer_handle_t* buffer) {
    unsigned long head, tail;
    head = READ_ONCE(buffer->head);
    tail = READ_ONCE(buffer->tail);
    return CIRC_SPACE(head, tail, RING_BUFFER_SIZE);
}

inline int rbuffer_is_consume_available(rbuffer_handle_t* buffer, unsigned long* head_ptr, unsigned long* tail_ptr) {
    *head_ptr = smp_load_acquire(&buffer->head);
    *tail_ptr = buffer->tail;

    return CIRC_CNT(*head_ptr, *tail_ptr, RING_BUFFER_SIZE) >= 1;
}

inline void* rbuffer_get_tail(rbuffer_handle_t* buffer, dma_addr_t* dma_addr, int blocking) {
    void* addr = NULL;
    unsigned long head, tail;
    rbuffer_entry entry;
    spin_lock(&buffer->cons_lock);

    while(!rbuffer_is_consume_available(buffer, &head, &tail)) {
        spin_unlock(&buffer->cons_lock);
        if(!blocking)
            return NULL;

        if(wait_event_interruptible(buffer->consumer, rbuffer_is_consume_available(buffer, &head, &tail)))
            return NULL;

        spin_lock(&buffer->cons_lock);
    }
    /* extract one item from the buffer */
    entry = buffer->entry[tail];
    addr = entry.buf;
    if(dma_addr) *dma_addr = entry.dmas;

    spin_unlock(&buffer->cons_lock);
    return addr;
}

/*
 * Blocking method
 */
inline void* rbuffer_consume(rbuffer_handle_t* buffer, dma_addr_t* dma_addr, int blocking) {
    void* addr = NULL;
    unsigned long head, tail;
    rbuffer_entry entry;
    spin_lock(&buffer->cons_lock);

    while(!rbuffer_is_consume_available(buffer, &head, &tail)) {
        spin_unlock(&buffer->cons_lock);
        if(!blocking)
            return NULL;

        if(wait_event_interruptible(buffer->consumer, rbuffer_is_consume_available(buffer, &head, &tail)))
            return NULL;

        spin_lock(&buffer->cons_lock);
    }
    /* extract one item from the buffer */
    entry = buffer->entry[tail];
    addr = entry.buf;
    if(dma_addr) *dma_addr = entry.dmas;

    /* Finish reading descriptor before incrementing tail. */
    smp_store_release(&buffer->tail, (tail + 1) & (RING_BUFFER_SIZE - 1));

    spin_unlock(&buffer->cons_lock);
    return addr;
}

inline void* rbuffer_consume_blocking(rbuffer_handle_t* buffer, dma_addr_t* dma_addr) {
    return rbuffer_consume(buffer, dma_addr, 1);
}

inline void* rbuffer_consume_nonblocking(rbuffer_handle_t* buffer, dma_addr_t* dma_addr) {
    return rbuffer_consume(buffer, dma_addr, 0);
}

inline size_t rbuffer_produce_n(rbuffer_handle_t* buffer, size_t n) {
    unsigned long head, tail;
    unsigned space;
    unsigned ret = 0;
    spin_lock(&buffer->prod_lock);

    head = buffer->head;
    /* The spin_unlock() and next spin_lock() provide needed ordering. */
    tail = READ_ONCE(buffer->tail);

    space = CIRC_SPACE(head, tail, RING_BUFFER_SIZE);
    ret = space > n ? n : space;
    if (ret > 0) {
        /* insert n items into the buffer */
        smp_store_release(&buffer->head, (head + ret) & (RING_BUFFER_SIZE - 1));

        /* wake_up() will make sure that the head is committed before
         * waking anyone up */
        wake_up_interruptible(&buffer->consumer);
    }

    spin_unlock(&buffer->prod_lock);

    if(ret < n){
        printk(KERN_ERR "rbuffer_produce_n: tried to increment %d, but it had space for just %d\n", n, ret);
    }

    return ret;
}

inline int rbuffer_produce(rbuffer_handle_t* buffer) {
    return rbuffer_produce_n(buffer, 1);
}
#endif
