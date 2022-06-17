#ifndef TWO_WAY_CIRCULAR_BUFFER_H
#define TWO_WAY_CIRCULAR_BUFFER_H

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
#include "circular_buffer.h"

/* The following circular buffer follows the implementation suggested by Howells
 * and Mc.Kenney in
 *
 * https://www.kernel.org/doc/html/latest/core-api/circular-buffers.html
 *
 */

//THIS IS MEM TO STORAGE

inline int two_way_rbuffer_init(rbuffer_handle_t* handle1, rbuffer_handle_t* handle2, struct device *dev, size_t size) {
    int i;
    if(rbuffer_init(handle1, dev, size)) {
        return 1;
    }

    for(i = 0; i < RING_BUFFER_SIZE; i++) {
        handle2->entry[i].buf = handle1->entry[i].buf;
    }
    handle2->head = 0;
    handle2->tail = 0;
    handle2->size = size;
    init_waitqueue_head(&handle2->consumer);
    spin_lock_init(&handle2->cons_lock);
    spin_lock_init(&handle2->prod_lock);
    return 0;
}

inline void two_way_rbuffer_free(rbuffer_handle_t* handle1, rbuffer_handle_t* handle2, struct device *dev) {
    rbuffer_free(handle1, dev);
}
#endif
