/*
 * Copyright (C) 2016 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_ethos
 * @{
 *
 * @file
 * @brief       Implementation of a simple ethernet-over-serial driver
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 *
 * @}
 */

#include <assert.h>
#include <errno.h>
#include <string.h>

#include "random.h"
#include "ethos.h"
#include "periph/uart.h"
#include "tsrb.h"
#include "irq.h"

#include "net/netdev.h"
#include "net/netdev/eth.h"
#include "net/eui64.h"
#include "net/ethernet.h"
#include "st7580.h"

#ifdef USE_ETHOS_FOR_STDIO
#error USE_ETHOS_FOR_STDIO is deprecated, use USEMODULE+=stdio_ethos instead
#endif

#ifdef MODULE_STDIO_ETHOS
#include "stdio_uart.h"
#include "isrpipe.h"
extern isrpipe_t stdio_uart_isrpipe;
#endif

#define ENABLE_DEBUG (0)
#include "debug.h"

static void _get_mac_addr(netdev_t *dev, uint8_t* buf);
static void ethos_isr(void *arg, uint8_t c);
static const netdev_driver_t netdev_driver_ethos;

static const uint8_t _esc_esc[] = {ETHOS_ESC_CHAR, (ETHOS_ESC_CHAR ^ 0x20)};
static const uint8_t _esc_delim[] = {ETHOS_ESC_CHAR, (ETHOS_FRAME_DELIMITER ^ 0x20)};

#define USE_CPL

#ifdef USE_CPL
kernel_pid_t _rcv_pid;
char _rcv_stack[THREAD_STACKSIZE_SMALL];

void *_ethos_recv_chk(void *arg)
{   
    ST7580Frame* RxFrame;
    uint8_t cRxLen;
	uint8_t lastIDRcv = 0;
    uint8_t i;
    
    uint8_t aRcvBuffer[200];
    
    while(1)
	{
        do
		{
			RxFrame = ST7580NextIndicationFrame();
			
			if (RxFrame != NULL)
			{
				/* Check if a duplicated indication frame with STX = 03 is received */
				if ((RxFrame->stx == ST7580_STX_03)&&(lastIDRcv == RxFrame->data[3 + sizeof(aRcvBuffer)]))
				{
                    RxFrame = NULL;
				}
				else
				{
                    lastIDRcv = RxFrame->data[3 + sizeof(aRcvBuffer)];
					break;
				}
			}
			xtimer_usleep(200000);
		} while(RxFrame==NULL);
        
        cRxLen = (RxFrame->length - 4);
		memcpy(aRcvBuffer, &(RxFrame->data[4]), cRxLen);

        //printf("PAYLOAD: %s", aRcvBuffer);
        for (i = 0; i < cRxLen; i++)
        {
            ethos_isr(arg, aRcvBuffer[i]);
        }
    }
}
#endif

void ethos_setup(ethos_t *dev, const ethos_params_t *params)
{
    dev->netdev.driver = &netdev_driver_ethos;
    dev->uart = params->uart;
    dev->state = WAIT_FRAMESTART;
    dev->framesize = 0;
    dev->frametype = 0;
    dev->last_framesize = 0;
    dev->accept_new = true;

    tsrb_init(&dev->inbuf, params->buf, params->bufsize);
    mutex_init(&dev->out_mutex);

    uint32_t a = random_uint32();
    memcpy(dev->mac_addr, (char*)&a, 4);
    a = random_uint32();
    memcpy(dev->mac_addr+4, (char*)&a, 2);

    dev->mac_addr[0] &= (0x2);      /* unset globally unique bit */
    dev->mac_addr[0] &= ~(0x1);     /* set unicast bit*/

    #ifdef USE_CPL
    uint8_t frame_delim = ETHOS_FRAME_DELIMITER;

    ST7580InterfaceInit(UART_DEV(1), GPIO_PIN(PORT_A, 8), GPIO_PIN(PORT_A, 5));
    _rcv_pid = thread_create(_rcv_stack, sizeof(_rcv_stack), THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST, _ethos_recv_chk, (void*)dev, "_ethos_recv_chk");
    
    ST7580DlData(DATA_OPT, &frame_delim, 1, NULL);
    ethos_send_frame(dev, dev->mac_addr, 6, ETHOS_FRAME_TYPE_HELLO);
    #else
    uart_init(params->uart, params->baudrate, ethos_isr, (void*)dev);

    uint8_t frame_delim = ETHOS_FRAME_DELIMITER;
    uart_write(dev->uart, &frame_delim, 1);
    ethos_send_frame(dev, dev->mac_addr, 6, ETHOS_FRAME_TYPE_HELLO);
    #endif
}

static void _reset_state(ethos_t *dev)
{
    dev->state = WAIT_FRAMESTART;
    dev->frametype = 0;
    dev->framesize = 0;
    dev->accept_new = true;
}

static void _handle_char(ethos_t *dev, char c)
{
    switch (dev->frametype) {
        case ETHOS_FRAME_TYPE_DATA:
        case ETHOS_FRAME_TYPE_HELLO:
        case ETHOS_FRAME_TYPE_HELLO_REPLY:
            if (dev->accept_new) {
                if (tsrb_add_one(&dev->inbuf, c) == 0) {
                    dev->framesize++;
                }
            }
            else {
                //puts("lost frame");
                dev->inbuf.reads = 0;
                dev->inbuf.writes = 0;
                _reset_state(dev);
            }
            break;
#ifdef MODULE_STDIO_ETHOS
        case ETHOS_FRAME_TYPE_TEXT:
            dev->framesize++;
            isrpipe_write_one(&stdio_uart_isrpipe, c);
#endif
    }
}

static void _end_of_frame(ethos_t *dev)
{
    switch(dev->frametype) {
        case ETHOS_FRAME_TYPE_DATA:
            if (dev->framesize) {
                assert(dev->last_framesize == 0);
                dev->last_framesize = dev->framesize;
                dev->netdev.event_callback((netdev_t*) dev, NETDEV_EVENT_ISR);
            }
            break;
        case ETHOS_FRAME_TYPE_HELLO:
            ethos_send_frame(dev, dev->mac_addr, 6, ETHOS_FRAME_TYPE_HELLO_REPLY);
            /* fall through */
        case ETHOS_FRAME_TYPE_HELLO_REPLY:
            if (dev->framesize == 6) {
                tsrb_get(&dev->inbuf, dev->remote_mac_addr, 6);
            }
            break;
    }

    _reset_state(dev);
}

static void ethos_isr(void *arg, uint8_t c)
{
    ethos_t *dev = (ethos_t *) arg;

    switch (dev->state) {
        case WAIT_FRAMESTART:
            if (c == ETHOS_FRAME_DELIMITER) {
                _reset_state(dev);
                if (dev->last_framesize) {
                    dev->accept_new = false;
                }
                dev->state = IN_FRAME;
            }
            break;
        case IN_FRAME:
            if (c == ETHOS_ESC_CHAR) {
                dev->state = IN_ESCAPE;
            }
            else if (c == ETHOS_FRAME_DELIMITER) {
                if (dev->framesize) {
                    _end_of_frame(dev);
                }
            }
            else {
                _handle_char(dev, c);
            }
            break;
        case IN_ESCAPE:
            switch (c) {
                case (ETHOS_FRAME_DELIMITER ^ 0x20):
                    _handle_char(dev, ETHOS_FRAME_DELIMITER);
                    break;
                case (ETHOS_ESC_CHAR ^ 0x20):
                    _handle_char(dev, ETHOS_ESC_CHAR);
                    break;
                case (ETHOS_FRAME_TYPE_TEXT ^ 0x20):
                    dev->frametype = ETHOS_FRAME_TYPE_TEXT;
                    break;
                case (ETHOS_FRAME_TYPE_HELLO ^ 0x20):
                    dev->frametype = ETHOS_FRAME_TYPE_HELLO;
                    break;
                case (ETHOS_FRAME_TYPE_HELLO_REPLY ^ 0x20):
                    dev->frametype = ETHOS_FRAME_TYPE_HELLO_REPLY;
                    break;
            }
            dev->state = IN_FRAME;
            break;
    }
}

static void _isr(netdev_t *netdev)
{
    ethos_t *dev = (ethos_t *) netdev;

    dev->netdev.event_callback((netdev_t*) dev, NETDEV_EVENT_RX_COMPLETE);
}

static int _init(netdev_t *encdev)
{
    ethos_t *dev = (ethos_t *) encdev;
    (void)dev;
    
    return 0;
}

static size_t iolist_count_total(const iolist_t *iolist)
{
    size_t result = 0;
    for (const iolist_t *iol = iolist; iol; iol = iol->iol_next) {
        result += iol->iol_len;
    }
    return result;
}

#ifndef USE_CPL
static void _write_escaped(uart_t uart, uint8_t c)
{
    const uint8_t *out;
    int n;

    switch(c) {
        case ETHOS_FRAME_DELIMITER:
            out = _esc_delim;
            n = 2;
            break;
        case ETHOS_ESC_CHAR:
            out = _esc_esc;
            n = 2;
            break;
        default:
            out = &c;
            n = 1;
    }

    uart_write(uart, out, n);
}
#endif

void ethos_send_frame(ethos_t *dev, const uint8_t *data, size_t len, unsigned frame_type)
{
    uint8_t frame_delim = ETHOS_FRAME_DELIMITER;
    #ifdef USE_CPL
    uint8_t pkt_out[255];
    uint8_t pkt_len = 0;
    #endif

    if (!irq_is_in()) {
        mutex_lock(&dev->out_mutex);
    }
    else {
        /* Send frame delimiter. This cancels the current frame,
         * but enables in-ISR writes.  */
        #ifdef USE_CPL
        pkt_out[pkt_len++] = frame_delim;
        #else
        uart_write(dev->uart, &frame_delim, 1);
        #endif
    }

    /* send frame delimiter */
    #ifdef USE_CPL
    pkt_out[pkt_len++] = frame_delim;
    #else
    uart_write(dev->uart, &frame_delim, 1);
    #endif

    /* set frame type */
    if (frame_type) {
        uint8_t out[2] = { ETHOS_ESC_CHAR, (frame_type ^ 0x20) };

        #ifdef USE_CPL
        memcpy(&pkt_out[pkt_len], out, sizeof(out));
        pkt_len = pkt_len + sizeof(out);
        #else
        uart_write(dev->uart, out, 2);
        #endif
    }

    /* send frame content */
    while(len--) {
        #ifdef USE_CPL
        switch(*data) {
            case ETHOS_FRAME_DELIMITER: 
                memcpy(&pkt_out[pkt_len], _esc_delim, sizeof(_esc_delim));
                pkt_len = pkt_len + sizeof(_esc_delim);
                break;
            case ETHOS_ESC_CHAR:
                memcpy(&pkt_out[pkt_len], _esc_esc, sizeof(_esc_esc));
                pkt_len = pkt_len + sizeof(_esc_esc);
                break;
            default:
                pkt_out[pkt_len++] = *data;
        }
        data++;
        #else
        _write_escaped(dev->uart, *data++);
        #endif
    }

    /* end of frame */
    #ifdef USE_CPL
    pkt_out[pkt_len++] = frame_delim;
    ST7580DlData(DATA_OPT, pkt_out, pkt_len, NULL);
    #else
    uart_write(dev->uart, &frame_delim, 1);
    #endif

    if (!irq_is_in()) {
        mutex_unlock(&dev->out_mutex);
    }
}

static int _send(netdev_t *netdev, const iolist_t *iolist)
{
    ethos_t *dev = (ethos_t *) netdev;
    (void)dev;
    #ifdef USE_CPL
    uint8_t pkt_out[255];
    uint8_t pkt_len = 0;
    #endif

    /* count total packet length */
    size_t pktlen = iolist_count_total(iolist);

    /* lock line in order to prevent multiple writes */
    mutex_lock(&dev->out_mutex);

    /* send start-frame-delimiter */
    uint8_t frame_delim = ETHOS_FRAME_DELIMITER;
    #ifdef USE_CPL
    pkt_out[pkt_len++] = frame_delim;
    #else
    uart_write(dev->uart, &frame_delim, 1);
    #endif

    /* send iolist */
    for (const iolist_t *iol = iolist; iol; iol = iol->iol_next) {
        size_t n = iol->iol_len;
        uint8_t *ptr = iol->iol_base;
        while(n--) {
            #ifdef USE_CPL
            switch(*ptr) {
                case ETHOS_FRAME_DELIMITER: 
                    pkt_len = pkt_len + sizeof(_esc_delim);
                    memcpy(&pkt_out[pkt_len], _esc_delim, sizeof(_esc_delim));
                    break;
                case ETHOS_ESC_CHAR:
                    pkt_len = pkt_len + sizeof(_esc_esc);
                    memcpy(&pkt_out[pkt_len], _esc_esc, sizeof(_esc_esc));
                    break;
                default:
                    pkt_out[pkt_len++] = *ptr;
            }
            ptr++;
            #else
            _write_escaped(dev->uart, *ptr++);
            #endif
        }
    }

    #ifdef USE_CPL
    pkt_out[pkt_len++] = frame_delim;
    ST7580DlData(DATA_OPT, pkt_out, pkt_len, NULL);
    #else
    uart_write(dev->uart, &frame_delim, 1);
    #endif

    mutex_unlock(&dev->out_mutex);

    return pktlen;
}

static void _get_mac_addr(netdev_t *encdev, uint8_t* buf)
{
    ethos_t * dev = (ethos_t *) encdev;
    memcpy(buf, dev->mac_addr, 6);
}

static int _recv(netdev_t *netdev, void *buf, size_t len, void* info)
{
    (void) info;
    ethos_t * dev = (ethos_t *) netdev;

    if (buf) {
        if (len < dev->last_framesize) {
            DEBUG("ethos _recv(): receive buffer too small.\n");
            return -1;
        }

        len = dev->last_framesize;

        if ((tsrb_get(&dev->inbuf, buf, len) != (int)len)) {
            DEBUG("ethos _recv(): inbuf doesn't contain enough bytes.\n");
            dev->last_framesize = 0;
            return -1;
        }
        dev->last_framesize = 0;
        return (int)len;
    }
    else {
        if (len) {
            int dropsize = dev->last_framesize;
            dev->last_framesize = 0;
            return tsrb_drop(&dev->inbuf, dropsize);
        }
        else {
            return dev->last_framesize;
        }
    }
}

static int _get(netdev_t *dev, netopt_t opt, void *value, size_t max_len)
{
    int res = 0;

    switch (opt) {
        case NETOPT_ADDRESS:
            if (max_len < ETHERNET_ADDR_LEN) {
                res = -EINVAL;
            }
            else {
                _get_mac_addr(dev, (uint8_t*)value);
                res = ETHERNET_ADDR_LEN;
            }
            break;
        default:
            res = netdev_eth_get(dev, opt, value, max_len);
            break;
    }

    return res;
}

/* netdev interface */
static const netdev_driver_t netdev_driver_ethos = {
    .send = _send,
    .recv = _recv,
    .init = _init,
    .isr = _isr,
    .get = _get,
    .set = netdev_eth_set
};
