#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "AnalogIn_Raspilot.h"
#include "px4io_protocol.h"

#define RASPILOT_ANALOGIN_DEBUG 0
#if RASPILOT_ANALOGIN_DEBUG
#include <cstdio>
#define debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#define error(fmt, args ...)  do {fprintf(stderr,"%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#define error(fmt, args ...)
#endif

RaspilotAnalogSource::RaspilotAnalogSource(int16_t pin):
    _pin(pin),
    _value(0.0f)
{
}

void RaspilotAnalogSource::set_pin(uint8_t pin)
{
    if (_pin == pin) {
        return;
    }
    _pin = pin;
}

float RaspilotAnalogSource::read_average()
{
    return read_latest();
}

float RaspilotAnalogSource::read_latest()
{
    return _value;
}

float RaspilotAnalogSource::voltage_average()
{
    return _value;
}

float RaspilotAnalogSource::voltage_latest()
{
    return _value;
}

float RaspilotAnalogSource::voltage_average_ratiometric()
{
    return _value;
}

extern const AP_HAL::HAL& hal;

RaspilotAnalogIn::RaspilotAnalogIn()
{
    _channels_number = RASPILOT_ADC_MAX_CHANNELS;
}

float RaspilotAnalogIn::board_voltage(void)
{
    _vcc_pin_analog_source->set_pin(4);

    return 5.0;
    //return _vcc_pin_analog_source->voltage_average() * 2.0;
}

AP_HAL::AnalogSource* RaspilotAnalogIn::channel(int16_t pin)
{
    for (uint8_t j = 0; j < _channels_number; j++) {
        if (_channels[j] == NULL) {
            _channels[j] = new RaspilotAnalogSource(pin);
            return _channels[j];
        }
    }

    hal.console->println("Out of analog channels");
    return NULL;
}

void RaspilotAnalogIn::init(void* implspecific)
{
    _vcc_pin_analog_source = channel(4);

    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&RaspilotAnalogIn::_update, void));
}

void RaspilotAnalogIn::_update()
{
    if (hal.scheduler->micros() - _last_update_timestamp < 100000) {
        return;
    }

    uint16_t adc_raw[RASPILOT_ADC_MAX_CHANNELS] = {0};

    if ( !hal.iomcu->read(PX4IO_PAGE_RAW_ADC_INPUT, 0, RASPILOT_ADC_MAX_CHANNELS, adc_raw) )
    {
        _last_update_timestamp = hal.scheduler->micros();
        return;
    }

    for (int16_t i = 0; i < RASPILOT_ADC_MAX_CHANNELS; i++) {
        for (int16_t j=0; j < RASPILOT_ADC_MAX_CHANNELS; j++) {
            RaspilotAnalogSource *source = _channels[j];

            if (source != NULL && i == source->_pin) {
                source->_value = adc_raw[i] * 3.3 / 4096.0;
            }
        }
    }

    _last_update_timestamp = hal.scheduler->micros();
}

#endif
