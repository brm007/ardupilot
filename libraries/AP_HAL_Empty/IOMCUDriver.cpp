#include <AP_HAL/AP_HAL.h>
#include "IOMCUDriver.h"

using namespace Empty;

void EmptyIOMCUDriver::init() {}

bool EmptyIOMCUDriver::read(uint8_t page, uint8_t offset, uint8_t len, uint16_t *data)
{
    return false;
}

bool EmptyIOMCUDriver::write(uint8_t page, uint8_t offset, uint8_t len, const uint16_t *data)
{
    return true;
}

int  EmptyIOMCUDriver::spiuart(uint8_t tx_len, const uint8_t *tx_data, uint8_t rx_len, uint8_t *rx_data)
{
    return 0;
}

int  EmptyIOMCUDriver::get_safety_state()
{
    return 0;
}
