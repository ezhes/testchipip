#include <unistd.h>
#define UART_BASE 0x10020000
#define UART_TXDATA (UART_BASE + 0x00)
#define UART_RXDATA (UART_BASE + 0x04)
#define UART_TXCTRL (UART_BASE + 0x08)
#define UART_RXCTRL (UART_BASE + 0x0c)
#define UART_DIV    (UART_BASE + 0x14)

static volatile uint32_t * uart_txdata = (volatile uint32_t *)(UART_TXDATA);
static volatile uint32_t * uart_rxdata = (volatile uint32_t *)(UART_RXDATA);
static volatile uint32_t * uart_txctrl = (volatile uint32_t *)(UART_TXCTRL);
static volatile uint32_t * uart_rxctrl = (volatile uint32_t *)(UART_RXCTRL);
static volatile uint32_t * uart_div    = (volatile uint32_t *)(UART_DIV);

#define BEBE_NOCK_REQ   'A' /* DUT -> HOST command advertising that BEBE is alive */
#define BEBE_NOCK_MAGIC (0x474f424541525321ULL) /* "GOBEARS!", big endian */

#define BEBE_CMD_READV  'R'
#define BEBE_CMD_WRITEV 'W'
#define BEBE_CMD_JUMP   'J'
#define BEBE_CMD_NOCK   'G' /* first letter of the nock, not a real command but
                                used to allow re-nocking */

#define BEBE_RSP_ACK    'Y'
#define BEBE_RSP_NACK   'N'

static void
uart_init(void) {
    /* Enable TX and RX */
    *uart_txctrl = 0x1;
    *uart_rxctrl = 0x1;
    /* 
    Force config modifications to complete complete before attempting to 
    use UART
    */
    asm volatile ("fence rw, rw");
}

static void
uart_tx(uint8_t b) {
    while (*uart_txdata) { /* spin while FIFO full */ }
    *uart_txdata = b;
}

static uint8_t
uart_rx(void) {
    uint32_t result;
    do {
        result = *uart_rxdata;
    } while (result >> 31); /* spin till not empty */

    return (uint8_t)result;
}

static int
uart_rx_no_wait(uint8_t *rx) {
    uint32_t result = *uart_rxdata;
    *rx = result;
    if (result >> 31) {
        /* empty */
        return 0;
    } else {
        return 1;
    }
}

static uint32_t
uart_rx32(void) {
    uint32_t result = 0;
    for (int i = 0; i < 4; i++) {
        result = (result << 8 ) | uart_rx(); 
    }

    return result;
}

static uint64_t
uart_rx64(void) {
    uint64_t result = 0;
    for (int i = 0; i < 8; i++) {
        result = (result << 8 ) | uart_rx(); 
    }

    return result;
}

static void
uart_txv(uint8_t *buf, size_t sz) {
    for (size_t i = 0; i < sz; i++) {
        uart_tx(buf[i]);
    }
}

typedef void func_t(void);

void 
bebe_main(void) {
    uart_init();

    /* wait for nock with sync-free tape search */
    uint64_t shift = 0;
    do {
        uart_tx(BEBE_NOCK_REQ);
        uint8_t rx;
        if (uart_rx_no_wait(&rx)) {
            shift = (shift << 8) | rx;
            /* 
            Echo back any characters we see to give an indication for 
            RX liveliness. Note that we also still send NOCK REQ so that even if
            we are just reading garbage we still send an externally visible probe 
            */
           uart_tx(rx);
        }
    } while (shift != BEBE_NOCK_MAGIC);
    uart_tx(BEBE_RSP_ACK);

    /* BEBE unlocked, enter command loop! */
    while (1) {
        uint8_t cmd = uart_rx();
        switch (cmd) {
            case BEBE_CMD_WRITEV: {
                uint32_t len = uart_rx32();
                uintptr_t dst = uart_rx64();
                if (len == 1) {
                    *(volatile uint8_t *)(dst) = uart_rx();
                } else if (len == 4) {
                    *(volatile uint32_t *)(dst) = uart_rx32();
                } else if (len == 8) {
                    *(volatile uint64_t *)(dst) = uart_rx64();
                } else {
                    for (uint32_t i = 0; i < len; i++) {
                        *(volatile uint8_t *)(dst + i) = uart_rx();
                    }
                }

                uart_tx(BEBE_RSP_ACK);
                break;
            }
            
            case BEBE_CMD_READV: {
                uint32_t len = uart_rx32();
                uintptr_t dst = uart_rx64();
                if (len == 1) {
                    uint8_t result = *(volatile uint8_t *)(dst);
                    uart_txv(&result, len);
                } else if (len == 4) {
                    uint32_t result = *(volatile uint32_t *)(dst);
                    uart_txv((uint8_t *)&result, len);
                } else if (len == 8) {
                    uint64_t result = *(volatile uint64_t *)(dst);
                    uart_txv((uint8_t *)&result, len);
                } else {
                    uart_txv((uint8_t *)dst, len);
                }

                /* no ack, the response is the ack */
                break;
            }
            
            case BEBE_CMD_JUMP: {
                uintptr_t dst = uart_rx64();
                uart_tx(BEBE_RSP_ACK);
                asm volatile ("fence.i");
                ((func_t *)(dst))();
                break;
            }

            case BEBE_CMD_NOCK: {
                for (int i = 0; i < 7; i++) {
                    (void)uart_rx();
                }

                uart_tx(BEBE_RSP_ACK);
                break;
            }
            
            default:
                uart_tx(BEBE_RSP_NACK);
                break;
        }
    }
}