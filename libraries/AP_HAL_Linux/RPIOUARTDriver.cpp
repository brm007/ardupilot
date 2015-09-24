#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <stdlib.h>
#include <cstdio>
#include "RPIOUARTDriver.h"
#include "../AP_HAL/utility/RingBuffer.h"

#include "px4io_protocol.h"

#define RPIOUART_POLL_TIME_INTERVAL 10000

extern const AP_HAL::HAL& hal;

#define RPIOUART_DEBUG 0

#include <cassert>

#if RPIOUART_DEBUG
#define debug(fmt, args ...)  do {hal.console->printf("[RPIOUARTDriver]: %s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#define error(fmt, args ...)  do {fprintf(stderr,"%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#define error(fmt, args ...)
#endif

using namespace Linux;

LinuxRPIOUARTDriver::LinuxRPIOUARTDriver() :
    LinuxUARTDriver(false),
    _last_update_timestamp(0),
    _external(false),
    _baudrate(0)
{
    _readbuf = NULL;
    _writebuf = NULL;
}

bool LinuxRPIOUARTDriver::isExternal()
{
    return _external;
}

void LinuxRPIOUARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    //hal.console->printf("[RPIOUARTDriver]: begin \n");

    if (device_path != NULL) {
        LinuxUARTDriver::begin(b,rxS,txS);
        if ( is_initialized()) {
            _external = true;
            return;
        }
    }

   if (rxS < 1024) {
       rxS = 2048;
   }
   if (txS < 1024) {
       txS = 2048;
   }

    _initialised = false;
    while (_in_timer) hal.scheduler->delay(1);

   /*
     allocate the read buffer
   */
   if (rxS != 0 && rxS != _readbuf_size) {
       _readbuf_size = rxS;
       if (_readbuf != NULL) {
           free(_readbuf);
       }
       _readbuf = (uint8_t *)malloc(_readbuf_size);
       _readbuf_head = 0;
       _readbuf_tail = 0;
   }

   /*
     allocate the write buffer
   */
   if (txS != 0 && txS != _writebuf_size) {
       _writebuf_size = txS;
       if (_writebuf != NULL) {
           free(_writebuf);
       }
       _writebuf = (uint8_t *)malloc(_writebuf_size);
       _writebuf_head = 0;
       _writebuf_tail = 0;
   }

    /* set baudrate */
    _baudrate = b;

    hal.scheduler->suspend_timer_procs();
    uint16_t tx_buf[2] = { (uint16_t)(b & 0xffff), (uint16_t)(b >> 16) };
    hal.iomcu->write(PX4IO_PAGE_UART_BUFFER, 0, 2, tx_buf);
    hal.scheduler->delay(1);
    hal.scheduler->resume_timer_procs();

    if (_writebuf_size != 0 && _readbuf_size != 0) {
        _initialised = true;
    }

}

int LinuxRPIOUARTDriver::_write_fd(const uint8_t *buf, uint16_t n)
{
    if (_external) {
        return LinuxUARTDriver::_write_fd(buf, n);
    }

    return -1;
}

int LinuxRPIOUARTDriver::_read_fd(uint8_t *buf, uint16_t n)
{
    if (_external) {
        return LinuxUARTDriver::_read_fd(buf, n);
    }

    return -1;
}

void LinuxRPIOUARTDriver::_timer_tick(void)
{
    if (_external) {
        LinuxUARTDriver::_timer_tick();
        return;
    }

    if (!_initialised) return;

    /* lower the update rate */
    if (hal.scheduler->micros() - _last_update_timestamp < RPIOUART_POLL_TIME_INTERVAL) {
        return;
    }

    _in_timer = true;

    uint8_t tx_buf[PKT_MAX_REGS * 2] = {0};
    uint8_t rx_buf[PKT_MAX_REGS * 2] = {0};

    /* get write_buf bytes */
    uint16_t _tail;
    uint16_t n = BUF_AVAILABLE(_writebuf);

    if (n > PKT_MAX_REGS * 2) {
        n = PKT_MAX_REGS * 2;
    }

    uint16_t _max_size = _baudrate / 10 / (1000000 / RPIOUART_POLL_TIME_INTERVAL);
    if (n > _max_size) {
        n = _max_size;
    }

    if (n > 0) {
        uint16_t n1 = _writebuf_size - _writebuf_head;
        if (n1 >= n) {
            // do as a single write
            memcpy( tx_buf, &_writebuf[_writebuf_head], n );
        } else {
            // split into two writes
            memcpy( tx_buf, &_writebuf[_writebuf_head], n1 );
            memcpy( &tx_buf[n1], &_writebuf[0], n-n1 );
        }

        BUF_ADVANCEHEAD(_writebuf, n);
    }

    int ret = hal.iomcu->spiuart(n, tx_buf, PKT_MAX_REGS*2, rx_buf);

    /* add bytes to read buf */
    uint16_t _head;
    n = BUF_SPACE(_readbuf);

    if (ret) {

        if (n > ret) {
            n = ret;
        }

        if (n > PKT_MAX_REGS * 2) {
            n = PKT_MAX_REGS * 2;
        }

        if (n > 0) {
            uint16_t n1 = _readbuf_size - _readbuf_tail;
            if (n1 >= n) {
                // one read will do
                memcpy( &_readbuf[_readbuf_tail], rx_buf, n );
            } else {
                memcpy( &_readbuf[_readbuf_tail], rx_buf, n1 );
                memcpy( &_readbuf[0], &rx_buf[n1], n-n1 );
            }

            BUF_ADVANCETAIL(_readbuf, n);
        }

    }

    _in_timer = false;

    _last_update_timestamp = hal.scheduler->micros();
}

#endif
