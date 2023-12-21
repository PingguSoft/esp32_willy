#ifndef _PID_CONTROLLER_H_
#define _PID_CONTROLLER_H_

#include <Arduino.h>

/*
*****************************************************************************************
* CONSTANTS
*****************************************************************************************
*/

class PIDController {
private:
    String  _strName;
    float   _p;
    float   _i;
    float   _d;
    unsigned long _lLastTS;
    float   _fTarget;
    float   _fLastInput;
    float   _fLastError;
    float   _fIntegral;
    float   _fIntegralLimit;
    bool    _isCircular;
    bool    _isStableCheck;
    float   _fIntegStableCheck;
    float   _fDurationStableCheck;

public:
    PIDController();
    PIDController(String name, bool circular, float p, float i, float d);
    PIDController(String name, float p, float i, float d);
    PIDController(String name, float p, float i, float d, float iLimit);
    PIDController(String name, bool circular, float p, float i, float d, float iLimit);

    void   reset();
    String getName()    { return _strName; }
    float  getP()       { return _p; }
    float  getI()       { return _i; }
    float  getD()       { return _d; }
    float  getTarget()  { return _fTarget; }
    void   setTarget(float target) { reset(); _fTarget = target; }

    void  set(float p, float i, float d);
    float compute(unsigned long now, float input);
    float compute(unsigned long now, float input, float limit);
    bool  isStable(float duration, float v);
private:
    float getLastInput() { return _fLastInput; }
    float calcError(float a, float b);
    float computeWithDelta(float input, float delta);
    float computeWithDelta(float input, float limit, float delta);
};

#endif
