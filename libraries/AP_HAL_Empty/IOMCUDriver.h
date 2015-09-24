#ifndef __AP_HAL_EMPTY_IOMCU_DRIVER_H__
#define __AP_HAL_EMPTY_IOMCU_DRIVER_H__

#include <stdint.h>

#include "AP_HAL_Empty.h"

class Empty::EmptyIOMCUDriver : public AP_HAL::IOMCUDriver {
public:
    void init();
    bool read(uint8_t page, uint8_t offset, uint8_t len, uint16_t *data);
    bool write(uint8_t page, uint8_t offset, uint8_t len, const uint16_t *data);
    int  spiuart(uint8_t tx_len, const uint8_t *tx_data, uint8_t rx_len, uint8_t *rx_data);
};

#endif //__AP_HAL_EMPTY_IOMCU_DRIVER_H__
