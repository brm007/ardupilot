#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT

#include "RPIODriver.h"

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>

static const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

using namespace Linux;

LinuxRPIODriver::LinuxRPIODriver()
{

}

void LinuxRPIODriver::init()
{
    _spi = hal.spi->device(AP_HAL::SPIDevice_RASPIO);

    if (_spi == NULL) {
        hal.scheduler->panic("Cannot get SPIDevice_RASPIO!");
        return;
    }

    _spi_sem = _spi->get_semaphore();

    if (_spi_sem == NULL) {
        hal.scheduler->panic(PSTR("PANIC: RCIutput_Raspilot did not get "
                                  "valid SPI semaphore!"));
        return; // never reached
    }

    // start the timer process to read samples
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&LinuxRPIODriver::_poll_data, void));
}

bool LinuxRPIODriver::read(uint8_t page, uint8_t offset, uint8_t len, uint16_t *data)
{
    struct IOPacket _dma_packet_tx, _dma_packet_rx;

    if (!_spi_sem->take_nonblocking()) {
        return false;
    }

    if (len > PKT_MAX_REGS) len = PKT_MAX_REGS;

    _dma_packet_tx.count_code = len | PKT_CODE_READ;
    _dma_packet_tx.page = page;
    _dma_packet_tx.offset = offset;
    _dma_packet_tx.crc = 0;
    _dma_packet_tx.crc = crc_packet(&_dma_packet_tx);

    _spi->transaction((uint8_t *)&_dma_packet_tx, (uint8_t *)&_dma_packet_rx, sizeof(_dma_packet_tx));

    if (page == PX4IO_PAGE_RAW_ADC_INPUT) hal.scheduler->delay_microseconds(200); //wait for adc sampling

    _dma_packet_tx.count_code = 0 | PKT_CODE_READ;
    _dma_packet_tx.page = 0;
    _dma_packet_tx.offset = 0;
    _dma_packet_tx.crc = 0;
    _dma_packet_tx.crc = crc_packet(&_dma_packet_tx);

    _spi->transaction((uint8_t *)&_dma_packet_tx, (uint8_t *)&_dma_packet_rx, sizeof(_dma_packet_tx));

    _spi_sem->give();

    uint8_t rx_crc = _dma_packet_rx.crc;
    _dma_packet_rx.crc = 0;

    if ( PKT_CODE(_dma_packet_rx) == PKT_CODE_SUCCESS && rx_crc == crc_packet(&_dma_packet_rx) ) {
        memcpy( data, &_dma_packet_rx.regs[0], len * sizeof(uint16_t) );
        return true;
    }

    return false;
}

bool LinuxRPIODriver::write(uint8_t page, uint8_t offset, uint8_t len, const uint16_t *data)
{
    struct IOPacket _dma_packet_tx, _dma_packet_rx;

    if (!_spi_sem->take_nonblocking()) {
        return false;
    }

    if (len > PKT_MAX_REGS) len = PKT_MAX_REGS;

    _dma_packet_tx.count_code = len | PKT_CODE_WRITE;
    _dma_packet_tx.page = page;
    _dma_packet_tx.offset = offset;
    memcpy( &_dma_packet_tx.regs[0], data, len * sizeof(uint16_t) );
    _dma_packet_tx.crc = 0;
    _dma_packet_tx.crc = crc_packet(&_dma_packet_tx);

    _spi->transaction((uint8_t *)&_dma_packet_tx, (uint8_t *)&_dma_packet_rx, sizeof(_dma_packet_tx));

    _spi_sem->give();

    return true;
}

int  LinuxRPIODriver::spiuart(uint8_t tx_len, const uint8_t *tx_data, uint8_t rx_len, uint8_t *rx_data)
{
    struct IOPacket _dma_packet_tx, _dma_packet_rx;

    if (!_spi_sem->take_nonblocking()) {
        return -1;
    }

    if (tx_len > PKT_MAX_REGS*2) tx_len = PKT_MAX_REGS*2;

    _dma_packet_tx.count_code = PKT_MAX_REGS | PKT_CODE_SPIUART;
    _dma_packet_tx.page = PX4IO_PAGE_UART_BUFFER;
    _dma_packet_tx.offset = tx_len;
    memcpy( &_dma_packet_tx.regs[0], tx_data, tx_len );
    _dma_packet_tx.crc = 0;
    _dma_packet_tx.crc = crc_packet(&_dma_packet_tx);
    /* set raspilotio to read uart data */
    _spi->transaction((uint8_t *)&_dma_packet_tx, (uint8_t *)&_dma_packet_rx, sizeof(_dma_packet_tx));

    hal.scheduler->delay_microseconds(100);

    /* get uart data from raspilotio */
    _dma_packet_tx.count_code = 0 | PKT_CODE_READ;
    _dma_packet_tx.page = 0;
    _dma_packet_tx.offset = 0;
    memset( &_dma_packet_tx.regs[0], 0, PKT_MAX_REGS*sizeof(uint16_t) );
    _dma_packet_tx.crc = 0;
    _dma_packet_tx.crc = crc_packet(&_dma_packet_tx);

    _spi->transaction((uint8_t *)&_dma_packet_tx, (uint8_t *)&_dma_packet_rx, sizeof(_dma_packet_tx));

    _spi_sem->give();

    uint8_t rx_crc = _dma_packet_rx.crc;
    _dma_packet_rx.crc = 0;

    if ( PKT_CODE(_dma_packet_rx) == PKT_CODE_SUCCESS && rx_crc == crc_packet(&_dma_packet_rx) && _dma_packet_rx.page == PX4IO_PAGE_UART_BUFFER )
    {
        if (rx_len > _dma_packet_rx.offset) rx_len = _dma_packet_rx.offset;
        if (rx_len > PKT_MAX_REGS*2) rx_len = PKT_MAX_REGS*2;

        memcpy( rx_data, &_dma_packet_rx.regs[0], rx_len );
        return rx_len;
    }

    return -1;
}

void LinuxRPIODriver::_poll_data()
{
    if (hal.scheduler->micros() - _last_update_timestamp < 10000) {
        return;
    }

    _last_update_timestamp = hal.scheduler->micros();
}

#endif // CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
