#ifndef __AP_HAL_LINUX_RPIODRIVER_H__
#define __AP_HAL_LINUX_RPIODRIVER_H__

#include "AP_HAL_Linux.h"
#include "px4io_protocol.h"

class Linux::LinuxRPIODriver : public AP_HAL::IOMCUDriver {
public:
    LinuxRPIODriver();
    void init();
    bool read(uint8_t page, uint8_t offset, uint8_t len, uint16_t *data);
    bool write(uint8_t page, uint8_t offset, uint8_t len, const uint16_t *data);
    int  spiuart(uint8_t tx_len, const uint8_t *tx_data, uint8_t rx_len, uint8_t *rx_data);

private:
    void _poll_data();

    AP_HAL::SPIDeviceDriver *_spi = nullptr;
    AP_HAL::Semaphore *_spi_sem = nullptr;

    uint32_t _last_update_timestamp = 0;
};

#endif //__AP_HAL_LINUX_RPIODRIVER_H__
