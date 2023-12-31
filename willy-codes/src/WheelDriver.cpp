#include <math.h>
#include <Wire.h>
#include "utils.h"
#include "WheelDriver.h"
#include "driver/pcnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"


/*
*****************************************************************************************
* CONSTANTS
*****************************************************************************************
*/


/*
*****************************************************************************************
* MACROS & STRUCTURES
*****************************************************************************************
*/


/*
*****************************************************************************************
* VARIABLES
*****************************************************************************************
*/
uint8_t WheelDriver::_num = 0;
Adafruit_PWMServoDriver *WheelDriver::_pExtPwm = NULL;
static const int16_t  _k_ctr_limit = 16384;

/*
*****************************************************************************************
* FUNCTIONS
*****************************************************************************************
*/

/*
*****************************************************************************************
* ISR
*****************************************************************************************
*/
void IRAM_ATTR isrHandlerPCNT(void* arg) {
    struct WheelDriver *pDriver = reinterpret_cast<WheelDriver*>(arg);
    uint32_t            status;

    pcnt_get_event_status((pcnt_unit_t)pDriver->_unit, &status);
    if (status & PCNT_EVT_L_LIM) {
        pDriver->_ticks_mult--;
    } else if (status & PCNT_EVT_H_LIM) {
        pDriver->_ticks_mult++;
    }
}

void initPCNT(pcnt_unit_t unit, int gpio_pulse, int gpio_ctrl = PCNT_PIN_NOT_USED,
        pcnt_channel_t channel = PCNT_CHANNEL_0, int16_t h_lim = 16384, int16_t l_lim = -16384) {

    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = gpio_pulse,       // set gpio for pulse input gpio
        .ctrl_gpio_num = gpio_ctrl,         // set gpio for control
        .lctrl_mode = PCNT_MODE_KEEP ,      // Rising A on LOW B  = CW Step
        .hctrl_mode = PCNT_MODE_REVERSE,    // Rising A on HIGH B = CCW Step
        .pos_mode = PCNT_COUNT_INC,         // increment the counter on positive edge
        .neg_mode = PCNT_COUNT_DIS,         // do nothing on falling edge
        .counter_h_lim = h_lim,
        .counter_l_lim = l_lim,
        .unit = unit,                       // PCNT unit number
        .channel = channel
    };
    pcnt_unit_config(&pcnt_config);
    pcnt_set_filter_value(unit, 500);
    pcnt_filter_enable(unit);

    /* Set threshold 0 and 1 values and enable events to watch */
    // pcnt_set_event_value(unit, PCNT_EVT_THRES_1, thres1);
    // pcnt_event_enable(unit, PCNT_EVT_THRES_1);
    // pcnt_set_event_value(unit, PCNT_EVT_THRES_0, thres0);
    // pcnt_event_enable(unit, PCNT_EVT_THRES_0);
    /* Enable events on zero, maximum and minimum limit values */
    pcnt_event_enable(unit, PCNT_EVT_L_LIM);
    pcnt_event_enable(unit, PCNT_EVT_H_LIM);

    pcnt_counter_pause(unit);
    pcnt_intr_enable(unit);
    pcnt_counter_resume(unit);
    pcnt_counter_clear(unit);
}


/*
*****************************************************************************************
* WheelDriver
*****************************************************************************************
*/
void WheelDriver::init(motor_drv_t drv_type, int8_t pin_pwm, int8_t pin_in1, int8_t pin_in2, int8_t pin_ctr, int8_t pin_ctr_dir,
                  bool reverse, uint16_t radius, uint16_t tpr, uint8_t deadzone, uint8_t addr) {
    _drv_type = drv_type;
    _pin_pwm = pin_pwm;
    _pin_in1 = pin_in1;
    _pin_in2 = pin_in2;
    _pin_ctr = pin_ctr;
    _pin_ctr_dir = pin_ctr_dir;
    _reverse = reverse;
    _radius  = radius;
    _tpr     = tpr;
    _ext_pwm_addr = addr;
    _tgtTick = 0;
    _pid_hz = 50;
    _p = 1.0;
    _i = 0.5;
    _d = 0.0;
    _deadzone = deadzone;

    _pid.configure(_p, _i, _d, _pid_hz, 16, true);
    _pid.setOutputRange(-128, 128);
    _pid_period = 1000 / _pid_hz;
}

void WheelDriver::setPID(float p, float i, float d) {
    _p = p;
    _i = i;
    _d = d;
    _pid.setCoefficients(_p, _i, _d, _pid_hz);
}

WheelDriver::WheelDriver(int8_t pin_esc, int8_t pin_ctr, int8_t pin_ctr_dir, bool reverse,
                         uint16_t radius, uint16_t tpr, uint8_t deadzone) {
    init(DRV_ESC, pin_esc, PIN_NONE, PIN_NONE, pin_ctr, pin_ctr_dir, reverse, radius, tpr, deadzone, 0);
}

WheelDriver::WheelDriver(int8_t pin_in1, int8_t pin_in2, int8_t pin_ctr, int8_t pin_ctr_dir, bool reverse,
                         uint16_t radius, uint16_t tpr, uint8_t deadzone, uint8_t addr) {

    init(DRV_8833, PIN_NONE, pin_in1, pin_in2, pin_ctr, pin_ctr_dir, reverse, radius, tpr, deadzone, addr);
}

WheelDriver::WheelDriver(int8_t pin_in1, int8_t pin_in2, int8_t pin_pwm, int8_t pin_ctr, uint8_t pin_ctr_dir, bool reverse,
                         uint16_t radius, uint16_t tpr, uint8_t deadzone) {
    init(DRV_TB6612FNG, pin_pwm, pin_in1, pin_in2, pin_ctr, pin_ctr_dir, reverse, radius, tpr, deadzone, 0);
}

void WheelDriver::setup() {
    if (_ext_pwm_addr != 0) {
        _pwm_freq = 1600;
        _pwm_res  = 8;
        if (_pExtPwm == NULL) {
            _pExtPwm = new Adafruit_PWMServoDriver(_ext_pwm_addr, Wire);
            _pExtPwm->begin();
            _pExtPwm->setOscillatorFrequency(27000000);
            _pExtPwm->setPWMFreq(_pwm_freq);
        }
    }

    if (_drv_type == DRV_ESC) {
        _pESC = new Servo();
        _pESC->attach(_pin_pwm, 1000, 2000);
        _pESC->setPeriodHertz(50);
        _pESC->write(1500);
    } else if (_ext_pwm_addr == 0) {
            pinMode(_pin_in1, OUTPUT);
            pinMode(_pin_in2, OUTPUT);
            digitalWrite(_pin_in1, LOW);
            digitalWrite(_pin_in2, LOW);

        if (_drv_type == DRV_TB6612FNG) {
            _pwm_freq = 10000;
            _pwm_res  = 8;
            _pPwm[0] = new ESP32PWM();
            _pPwm[0]->attachPin(_pin_pwm, _pwm_freq, _pwm_res);
            LOG("TB6612FNG PWM Ch:%d\n", _pPwm[0]->getChannel());
        } else if (_drv_type == DRV_8833) {
            _pwm_freq = 50000;
            _pwm_res  = 8;
            _pPwm[0] = new ESP32PWM();
            _pPwm[0]->attachPin(_pin_in1, _pwm_freq, _pwm_res);
            LOG("DRV_8833 PWM Ch:%d\n", _pPwm[0]->getChannel());

            _pPwm[1] = new ESP32PWM();
            _pPwm[1]->attachPin(_pin_in2, _pwm_freq, _pwm_res);
            LOG("DRV_8833 PWM Ch:%d\n", _pPwm[1]->getChannel());
        }
    }

    if (_pin_ctr_dir != PIN_NONE)
        pinMode(_pin_ctr_dir, INPUT_PULLDOWN);

    if (_pin_ctr != PIN_NONE) {
        _unit = _num++;
        pinMode(_pin_ctr, INPUT_PULLDOWN);
        initPCNT((pcnt_unit_t)_unit, _pin_ctr, _pin_ctr_dir, PCNT_CHANNEL_0, _k_ctr_limit, -_k_ctr_limit);
        pcnt_isr_service_install(0);
        pcnt_isr_handler_add((pcnt_unit_t)_unit, isrHandlerPCNT, (void*)this);
        reset();
    }
    _speed = -1000;
    setSpeed(0);
}

void WheelDriver::setSpeed(int speed) {
    int spd;
    int abs_spd;

    if (speed == _speed)
        return;

    speed = constrain(speed, -255, 255);
    if (speed != 0 && abs(speed) < _deadzone) {
        speed = (speed >= 0) ? _deadzone : -_deadzone;
    }

    if (_pin_ctr_dir == PIN_NONE) {
        int8_t mode = (speed > 0) ? PCNT_COUNT_INC : ((speed < 0) ? PCNT_COUNT_DEC : -1);
        if (mode != -1)
            pcnt_set_mode((pcnt_unit_t)_unit, PCNT_CHANNEL_0, (pcnt_count_mode_t)mode, PCNT_COUNT_DIS, PCNT_MODE_KEEP, PCNT_MODE_KEEP);
    }

    switch (_drv_type) {
        case DRV_ESC:
             spd = _reverse ? map(speed, -255, 255, 2000, 1000) : map(speed, -255, 255, 1000, 2000);
            _pESC->writeMicroseconds(spd);
            break;

        case DRV_8833:
            spd = _reverse ? -speed : speed;

			// slow decay mode, pwm value reversed
            abs_spd = 255 - abs(spd);

            // brake
            if (spd == 0) {
                if (_ext_pwm_addr != 0) {
                    _pExtPwm->setPin(_pin_in1, 4095);
                    _pExtPwm->setPin(_pin_in2, 4095);
                } else {
                    _pPwm[0]->detachPin(_pin_in1);
                    pinMode(_pin_in1, OUTPUT);
                    digitalWrite(_pin_in1, 1);

                    _pPwm[1]->detachPin(_pin_in2);
                    pinMode(_pin_in2, OUTPUT);
                    digitalWrite(_pin_in2, 1);
                }
            } else if (spd < 0) {
                if (_ext_pwm_addr != 0) {
                    _pExtPwm->setPin(_pin_in1, map(abs_spd, 0, 255, 0, 4095));
                    _pExtPwm->setPin(_pin_in2, 4095);
                } else {
                    _pPwm[0]->attachPin(_pin_in1, _pwm_freq, _pwm_res);
                    _pPwm[0]->write(abs_spd);

                    _pPwm[1]->detachPin(_pin_in2);
                    pinMode(_pin_in2, OUTPUT);
                    digitalWrite(_pin_in2, 1);
                }
            } else {
                if (_ext_pwm_addr != 0) {
                    _pExtPwm->setPin(_pin_in1, 4095);
                    _pExtPwm->setPin(_pin_in2, map(abs_spd, 0, 255, 0, 4095));
                } else {
                    _pPwm[0]->detachPin(_pin_in1);
                    pinMode(_pin_in1, OUTPUT);
                    digitalWrite(_pin_in1, 1);

                    _pPwm[1]->attachPin(_pin_in2, _pwm_freq, _pwm_res);
                    _pPwm[1]->write(abs_spd);
                }
            }
            break;

        case DRV_TB6612FNG:
            spd = _reverse ? -speed : speed;
            // short brake mode
            if (spd == 0) {
                digitalWrite(_pin_in1, HIGH);
                digitalWrite(_pin_in2, HIGH);

                _pPwm[0]->detachPin(_pin_pwm);
                pinMode(_pin_pwm, OUTPUT);
                digitalWrite(_pin_pwm, 1);
            } else if (spd < 0) {
                digitalWrite(_pin_in1, LOW);
                digitalWrite(_pin_in2, HIGH);
                _pPwm[0]->attachPin(_pin_pwm, _pwm_freq, _pwm_res);
                _pPwm[0]->write(abs_spd);
            } else {
                digitalWrite(_pin_in1, HIGH);
                digitalWrite(_pin_in2, LOW);
                _pPwm[0]->attachPin(_pin_pwm, _pwm_freq, _pwm_res);
                _pPwm[0]->write(abs_spd);
            }
            break;
    }
    _speed = speed;
}

long WheelDriver::getTicks() {
    int16_t cnt;

    pcnt_get_counter_value((pcnt_unit_t)_unit, &cnt);
    return _ticks_mult * long(_k_ctr_limit) + cnt;
}

void WheelDriver::reset() {
    _ticks_mult = 0;
    _tgtTick = 0;
    if (_pin_ctr != PIN_NONE) {
        pcnt_counter_pause((pcnt_unit_t)_unit);
        pcnt_counter_clear((pcnt_unit_t)_unit);
        pcnt_counter_resume((pcnt_unit_t)_unit);
    }
}

void WheelDriver::loop(bool debug) {
    if (_pin_ctr == PIN_NONE)
        return;

    unsigned long ts = millis();
    if (ts - _last_ms >= _pid_period) {
        long cur = getTicks();
        int  spd = _pid.step(_tgtTick, cur);

        setSpeed(spd);
        _last_ms = ts;

        if (debug)
            LOGI("pos : %8ld / %8ld => spd:%3d\n", cur, _tgtTick, spd);
    }
}
