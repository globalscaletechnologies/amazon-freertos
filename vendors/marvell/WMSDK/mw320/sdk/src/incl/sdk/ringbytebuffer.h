#ifndef _RINGBYTEBUFFER_H_
#define _RINGBYTEBUFFER_H_

typedef struct _ringbytebuffer {
    int wr_idx;             /* write pointer */
    int rd_idx;             /* read pointer */
    int capacity;           /* number of bytes in buffer *buf */
    int available;          /* number of bytes in buffer *buf */
    uint8_t *buffer;        /* pointer to buffer storage */
} ringbytebuffer_t;

ringbytebuffer_t* ringbytebuffer_create(int size);
void ringbytebuffer_destroy(ringbytebuffer_t *rbuf);
void ringbytebuffer_reset(ringbytebuffer_t *rbuf);
int ringbytebuffer_capacity(ringbytebuffer_t *rbuf);
int ringbytebuffer_available(ringbytebuffer_t *rbuf);
int ringbytebuffer_get(ringbytebuffer_t *rbuf, uint8_t *data);
int ringbytebuffer_gets(ringbytebuffer_t *rbuf, uint8_t *data, int len);
int ringbytebuffer_put(ringbytebuffer_t *rbuf, uint8_t data);
int ringbytebuffer_puts(ringbytebuffer_t *rbuf, uint8_t *data, int len);
int ringbytebuffer_peek(ringbytebuffer_t *rbuf);
int ringbytebuffer_skip(ringbytebuffer_t *rbuf, int count);
bool ringbytebuffer_is_empty(ringbytebuffer_t *rbuf);
bool ringbytebuffer_is_full(ringbytebuffer_t *rbuf);
#endif /* _RINGBYTEBUFFER_H_ */
