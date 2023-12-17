#ifndef _SWERVE_WHEEL_H_
#define _SWERVE_WHEEL_H_
#include <ESP32Servo.h>
#include "config.h"
#include "WheelDriver.h"


/*
*****************************************************************************************
* CONSTANTS
*****************************************************************************************
*/
enum {
    IDX_UWHEEL = 0,
    IDX_DWHEEL
};

/*
*****************************************************************************************
* MACROS & STRUCTURES
*****************************************************************************************
*/


/*
*****************************************************************************************
* CLASS
*****************************************************************************************
*/
class Ticks {
public:
    typedef struct {
        unsigned long   millis;     // ms
        long            up;         // ticks
        long            down;       // ticks
    } __attribute__((packed)) ticks_t;

    Ticks() {
        set(0, 0, 0);
    }

    Ticks(unsigned long ts, long u, long d) {
        set(ts, u, d);
    }

    void set(unsigned long ts, long u, long d) {
        _t.millis = ts;
        _t.up     = u;
        _t.down   = d;
    }

    ticks_t get() {
        return _t;
    }

    void reset() {
        set(0, 0, 0);
    }

	Ticks operator-(Ticks &ref) {
		return Ticks(_t.millis - ref._t.millis, _t.up - ref._t.up, _t.down - ref._t.down);
	}

private:
    ticks_t  _t;
};

class SwerveWheel {
public:
    SwerveWheel(WheelDriver *up, WheelDriver *down);

    void    setup();
    void    setMotor(int speedU, int speedD);
    void    drive(int angle, int speed);
    void    reset(bool driver=true);
    void    calibrate(int key);
    Ticks   getTicks(Ticks *ticks=NULL);

private:
    bool    getDelta(Ticks &a, Ticks &b, float *dtheta, float *ddist);

    WheelDriver     *_pDriver[2];

    Ticks           _ticks;
    Ticks           _last_ticks;
};

#endif
