#include <math.h>
#include "utils.h"
#include "driver/pcnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "SwerveWheel.h"

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


/*
*****************************************************************************************
* FUNCTIONS
*****************************************************************************************
*/
SwerveWheel::SwerveWheel(WheelDriver *up, WheelDriver *down) :
    _pDriver { up, down }
{
    reset(false);
}

void SwerveWheel::setup() {
    for (int i = 0; i < ARRAY_SIZE(_pDriver); i++) {
        if (_pDriver[i])
            _pDriver[i]->setup();
    }
}

void SwerveWheel::setMotor(int speedU, int speedR) {
    if (_pDriver[IDX_UWHEEL])
        _pDriver[IDX_UWHEEL]->setSpeed(speedU);
    if (_pDriver[IDX_DWHEEL])
        _pDriver[IDX_DWHEEL]->setSpeed(speedR);
}

void SwerveWheel::drive(int angle, int speed) {
    int speedU;
    int speedR;

    if (angle >= 0) {
        float rad = radians(angle);
        speedU = speed;
        speedR = speed * cos(rad);
    } else {
        float rad = radians(-angle);
        speedU = speed * cos(rad);
        speedR = speed;
    }
    //LOG("ang:%6d,  spd:%6d, %6d\n", angle, speedU, speedR);
    setMotor(speedU, speedR);
}

bool SwerveWheel::getDelta(Ticks &a, Ticks &b, float *dtheta, float *ddist) {
    Ticks dlt = a - b;
    float len_l  = 2 * M_PI * _pDriver[IDX_UWHEEL]->getRadius();
    float dist_l = len_l * dlt.get().up  / _pDriver[IDX_UWHEEL]->getTPR();

    float len_r  = (2 * M_PI * _pDriver[IDX_DWHEEL]->getRadius());
    float dist_r = (len_r * dlt.get().down / _pDriver[IDX_DWHEEL]->getTPR());
    *dtheta = (dist_l - dist_r);  // CW
    *ddist  = (dist_r + dist_l) / 2;

    return (dlt.get().up != 0 || dlt.get().down != 0);
}

Ticks SwerveWheel::getTicks(Ticks *ticks) {
    Ticks t;

    t.set(millis(), _pDriver[IDX_UWHEEL]->getTicks(), _pDriver[IDX_DWHEEL]->getTicks());
    if (ticks)
        *ticks = t;
    return t;
}

void SwerveWheel::reset(bool driver) {
    if (driver) {
        for (int i = 0; i < ARRAY_SIZE(_pDriver); i++) {
            _pDriver[i]->reset();
        }
    }
    _ticks.reset();
    _last_ticks.reset();
}

void SwerveWheel::debug(int key) {
    float   rot;
    Ticks   delta;

    switch (key) {
        case '[':
            _pDebug = _pDriver[0];
            LOG("drv up\n");
            break;
        case ']':
            _pDebug = _pDriver[1];
            LOG("drv down\n");
            break;

        case ',':
            _pDebug->setSpeed(_pDebug->getSpeed() + 5);
            LOG("drv:%d, speed:%4d\n", _idx, _pDebug->getSpeed());
            break;

        case '.':
            _pDebug->setSpeed(_pDebug->getSpeed() - 5);
            LOG("drv:%d, speed:%4d\n", _idx, _pDebug->getSpeed());
            break;

        case 'r':
            reset();
            // no break

        case '/':
            _last_ticks = _ticks;
            _ticks.set(millis(), _pDriver[IDX_UWHEEL]->getTicks(), _pDriver[IDX_DWHEEL]->getTicks());

            delta = _ticks - _last_ticks;
            rot  = delta.get().up / _pDriver[IDX_UWHEEL]->getTPR();
            rot  = rot * 60000 / delta.get().millis;
            LOG("drv_u ctr:%8ld, rpm:%6.1f\n", _ticks.get().up,  rot);

            rot  = delta.get().down / _pDriver[IDX_DWHEEL]->getTPR();
            rot  = rot * 60000 / delta.get().millis;
            LOG("drv_d ctr:%8ld, rpm:%6.1f\n", _ticks.get().down, rot);
            break;

        case 's':
            for (int i = 0; i < ARRAY_SIZE(_pDriver); i++) {
                if (_pDriver[i])
                    _pDriver[i]->setSpeed(0);
            }
            LOG("stop !!!\n");
            break;

        case 'P':
            _pDebug->setPID(_pDebug->getP() + 0.1, _pDebug->getI(), _pDebug->getD());
            LOGI("PID: %4.1f, %4.1f, %4.1f\n", _pDebug->getP(), _pDebug->getI(), _pDebug->getD());
            break;

        case 'p':
            _pDebug->setPID(_pDebug->getP() - 0.1, _pDebug->getI(), _pDebug->getD());
            LOGI("PID: %4.1f, %4.1f, %4.1f\n", _pDebug->getP(), _pDebug->getI(), _pDebug->getD());
            break;

        case 'I':
            _pDebug->setPID(_pDebug->getP(), _pDebug->getI() + 0.1, _pDebug->getD());
            LOGI("PID: %4.1f, %4.1f, %4.1f\n", _pDebug->getP(), _pDebug->getI(), _pDebug->getD());
            break;

        case 'i':
            _pDebug->setPID(_pDebug->getP(), _pDebug->getI() - 0.1, _pDebug->getD());
            LOGI("PID: %4.1f, %4.1f, %4.1f\n", _pDebug->getP(), _pDebug->getI(), _pDebug->getD());
            break;

        case 'D':
            _pDebug->setPID(_pDebug->getP(), _pDebug->getI(), _pDebug->getD() + 0.1);
            LOGI("PID: %4.1f, %4.1f, %4.1f\n", _pDebug->getP(), _pDebug->getI(), _pDebug->getD());
            break;

        case 'd':
            _pDebug->setPID(_pDebug->getP(), _pDebug->getI(), _pDebug->getD() - 0.1);
            LOGI("PID: %4.1f, %4.1f, %4.1f\n", _pDebug->getP(), _pDebug->getI(), _pDebug->getD());
            break;

        case 'T':
            _pDebug->moveTo(_pDebug->getTPR());
            LOGI("target : %ld\n", _pDebug->getTarget());
            break;

        case 't':
            _pDebug->moveTo(-_pDebug->getTPR());
            LOGI("target : %ld\n", _pDebug->getTarget());
            break;
    }
}

void SwerveWheel::loop() {
    for (int i = 0; i < ARRAY_SIZE(_pDriver); i++) {
        if (_pDriver[i])
            _pDriver[i]->loop();
    }
}
