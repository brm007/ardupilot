#ifndef __AP_HAL_IOMCU_DRIVER_H__
#define __AP_HAL_IOMCU_DRIVER_H__

#include <stdint.h>

#include "AP_HAL_Namespace.h"

class AP_HAL::IOMCUDriver {
public:
    virtual void init() = 0;
    virtual bool read(uint8_t page, uint8_t offset, uint8_t len, uint16_t *data) = 0;
    virtual bool write(uint8_t page, uint8_t offset, uint8_t len, const uint16_t *data) = 0;
    virtual int  spiuart(uint8_t tx_len, const uint8_t *tx_data, uint8_t rx_len, uint8_t *rx_data) = 0;
    virtual int  get_safety_state() = 0;
};

#endif //__AP_HAL_IOMCU_DRIVER_H__
