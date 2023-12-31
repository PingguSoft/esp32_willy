#ifndef _WHEEL_DRIVER_H_
#define _WHEEL_DRIVER_H_
#include <ESP32Servo.h>
#include <Adafruit_PWMServoDriver.h>
#include <FastPID.h>
#include "PIDController.h"
#include "config.h"



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
* CLASS
*****************************************************************************************
*/
class WheelDriver {
public:
    typedef enum {
        DRV_ESC = 0,
        DRV_8833,
        DRV_TB6612FNG
    } motor_drv_t;

    WheelDriver(int8_t pin_esc, int8_t pin_ctr, int8_t pin_ctr_dir, bool reverse,                                   // DRV_ESC
                uint16_t radius, uint16_t tpr, uint8_t deadzone);
    WheelDriver(int8_t pin_in1, int8_t pin_in2, int8_t pin_ctr, int8_t pin_ctr_dir, bool reverse,                   // DRV_8833
                uint16_t radius, uint16_t tpr, uint8_t deadzone, uint8_t addr=0);
    WheelDriver(int8_t pin_in1, int8_t pin_in2, int8_t pin_pwm, int8_t pin_ctr, uint8_t pin_ctr_dir, bool reverse,  // DRV_TB6612FNG
                uint16_t radius, uint16_t tpr, uint8_t deadzone);

    void     setup();
    void     reset();
    uint16_t getTPR()               { return _tpr;       }
    uint16_t getRadius()            { return _radius;    }

    void     setSpeed(int speed);
    int      getSpeed()             { return _speed;     }
    long     getTicks();
    int      getDegreePerTick()     { return 360 / _tpr; }
    int      getDegree()            { return (getTicks() % _tpr) * getDegreePerTick(); }

    long     getTarget()            { return _tgtTick;    }
    void     move(long ticks)       { _tgtTick  = ticks;  }
    void     moveTo(long ticks)     { _tgtTick += ticks;  }
    void     loop(bool debug);

    float    getP()                 { return _p;          }
    float    getI()                 { return _i;          }
    float    getD()                 { return _d;          }
    void     setPID(float p, float i, float d);
    FastPID *getPID()               { return &_pid;       }

    friend void isrHandlerPCNT(void *arg);

private:
    void     init(motor_drv_t drv_type, int8_t pin_pwm, int8_t pin_in1, int8_t pin_in2, int8_t pin_ctr, int8_t pin_ctr_dir,
                  bool reverse, uint16_t radius, uint16_t tpr, uint8_t deadzone, uint8_t addr);

    // for tracking odometry
    uint8_t  _unit;
    long     _ticks_mult;

    motor_drv_t _drv_type;
    uint8_t  _ext_pwm_addr;

    bool     _reverse;
    int8_t   _pin_in1;
    int8_t   _pin_in2;
    int8_t   _pin_pwm;
    int8_t   _pin_ctr;
    int8_t   _pin_ctr_dir;
    int      _speed;
    uint8_t  _deadzone;

    double   _pwm_freq;
    uint8_t  _pwm_res;
    ESP32PWM *_pPwm[2];
    Servo    *_pESC;

    uint16_t  _radius;
    uint16_t  _tpr;

    float     _p, _i, _d;
    FastPID   _pid;
    uint16_t  _pid_hz;
    uint16_t  _pid_period;
    long      _tgtTick;
    unsigned long _last_ms;

    // for counting instances
    static uint8_t _num;
    static Adafruit_PWMServoDriver *_pExtPwm;
};
#endif
