// Source: https://embedjournal.com/implementing-circular-buffer-embedded-c/ (public domain)

#define BUFF_OP_ERR -1
#define BUFF_OP_OK 0

typedef struct {
    uint8_t * const buffer;
    int head;
    int tail;
    const int maxLen;
    int overflow;
} circBuf_t;


#define CIRCBUF_DEF(x,y)          \
    uint8_t x##_dataSpace[y];     \
    circBuf_t x = {               \
        .buffer = x##_dataSpace,  \
        .head = 0,                \
        .tail = 0,                \
        .maxLen = y               \
    }

int circBufPush(circBuf_t *c, uint8_t data);
int circBufPop(circBuf_t *c, uint8_t *data);
int circBufLen(circBuf_t *c);
void circBufReset(circBuf_t *c);
int circBufCopy(circBuf_t *c, uint8_t *out, int maxLen);
