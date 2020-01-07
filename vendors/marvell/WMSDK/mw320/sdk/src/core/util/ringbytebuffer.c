#include <wmstdio.h>
#include <wm_os.h>
#include <stdbool.h>
#include <stdint.h>
#include "ringbytebuffer.h"

ringbytebuffer_t* ringbytebuffer_create(int size)
{
    ringbytebuffer_t *rbuf = NULL;

    rbuf = (ringbytebuffer_t *)os_mem_calloc(sizeof(ringbytebuffer_t));
    if (rbuf) {
        rbuf->capacity = size;
        ringbytebuffer_reset(rbuf);
        rbuf->buffer = os_mem_calloc(size);
    }
    return rbuf;
}

void ringbytebuffer_destroy(ringbytebuffer_t *rbuf)
{
    ringbytebuffer_reset(rbuf);
    if (rbuf->buffer) {
        os_mem_free(rbuf->buffer);
        rbuf->buffer = NULL;
        rbuf->capacity = 0;
    }
    os_mem_free(rbuf);
    rbuf = NULL;
}

void ringbytebuffer_reset(ringbytebuffer_t *rbuf)
{
    rbuf->wr_idx = rbuf->rd_idx = rbuf->available = 0;
}

int ringbytebuffer_capacity(ringbytebuffer_t *rbuf)
{
    return rbuf->capacity;
}

int ringbytebuffer_available(ringbytebuffer_t *rbuf)
{
    return rbuf->available;
}

bool ringbytebuffer_is_empty(ringbytebuffer_t *rbuf)
{
    return (rbuf->available == 0);
}

bool ringbytebuffer_is_full(ringbytebuffer_t *rbuf)
{
    return (rbuf->available == rbuf->available);
}

int ringbytebuffer_get(ringbytebuffer_t *rbuf, uint8_t *data)
{
    uint8_t value = 0;

    if (rbuf->available == 0) {
        return 0;
    }
    value = rbuf->buffer[rbuf->rd_idx];
    rbuf->rd_idx = (rbuf->rd_idx + 1) % rbuf->capacity;
    rbuf->available--;
    *data = value;
    return 1;
}

int ringbytebuffer_gets(ringbytebuffer_t *rbuf, uint8_t *data, int len)
{
    int count = 0;

    if (rbuf->available == 0) {
        return 0;
    }
    len = (len > rbuf->capacity)? rbuf->capacity : len;

    if (rbuf->rd_idx < rbuf->wr_idx) {
        memcpy(data, &rbuf->buffer[rbuf->rd_idx], len);
    } else {
        count = rbuf->capacity - rbuf->rd_idx;
        memcpy(data, &rbuf->buffer[rbuf->rd_idx], count);
        if ((rbuf->rd_idx + len) > rbuf->capacity) {
            memcpy(data + count, rbuf->buffer, len - count);
        }
    }
    rbuf->rd_idx = (rbuf->rd_idx + len) % rbuf->capacity;
    rbuf->available -= len;
    return len;
}

int ringbytebuffer_put(ringbytebuffer_t *rbuf, uint8_t data)
{
    if (rbuf->available == rbuf->capacity) {
        return 0;
    }
    rbuf->buffer[rbuf->wr_idx] = data;
    rbuf->wr_idx = (rbuf->wr_idx + 1) % rbuf->capacity;
    rbuf->available++;
    return 1;
}

int ringbytebuffer_puts(ringbytebuffer_t *rbuf, uint8_t *data, int len)
{
    int count = 0;

    if (rbuf->available == rbuf->capacity) {
        return 0;
    }
    if (len > (rbuf->capacity - rbuf->available)) {
        len = rbuf->capacity - rbuf->available;
    }

    if (rbuf->wr_idx < rbuf->rd_idx) {
        memcpy(&rbuf->buffer[rbuf->wr_idx], data, len);
    } else {
        count = rbuf->capacity - rbuf->wr_idx;
        memcpy(&rbuf->buffer[rbuf->wr_idx], data, count);
        if ((rbuf->wr_idx + len) > rbuf->capacity) {
            memcpy(rbuf->buffer, data + count, len - count);
        }
    }
    rbuf->wr_idx = (rbuf->wr_idx + len) % rbuf->capacity;
    rbuf->available += len;
    return len;
}

int ringbytebuffer_peek(ringbytebuffer_t *rbuf)
{
    uint8_t value = 0;

    if (rbuf->available == 0) {
        return -WM_FAIL;
    }
    value = rbuf->buffer[rbuf->rd_idx];
    return value;
}

int ringbytebuffer_skip(ringbytebuffer_t *rbuf, int count)
{
    if (rbuf->available == 0) {
        return -WM_FAIL;
    }
    count = (count > rbuf->capacity)? rbuf->capacity : count;
    rbuf->rd_idx = (rbuf->rd_idx + count) % rbuf->capacity;
    rbuf->available -= count;
    return count;
}
