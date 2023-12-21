#include "INA226.h"
#include "config.h"
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>
#include <Wire.h>
#include "utils.h"
#include "TimeEvent.h"
#include "WheelDriver.h"
#include "FastPID.h"
#include "SwerveWheel.h"

INA226      _ina(0x40);
bool        _is_go = false;

SwerveWheel *_pSwerves[] = {
    // center
    new SwerveWheel(new WheelDriver(PIN_W3_M1_PCA_IN1, PIN_W3_M1_PCA_IN2, PIN_W3_M1_C1, PIN_W3_M1_C2, false, WHEEL_RADIUS_MM, TICKS_PER_CYCLE, 20, 0x47),
                    new WheelDriver(PIN_W3_M2_PCA_IN1, PIN_W3_M2_PCA_IN2, PIN_W3_M2_C1, PIN_W3_M2_C2, false, WHEEL_RADIUS_MM, TICKS_PER_CYCLE, 20, 0x47)),

    // left
    new SwerveWheel(new WheelDriver(PIN_W2_M1_PCA_IN1, PIN_W2_M1_PCA_IN2, PIN_W2_M1_C1, PIN_W2_M1_C2, false, WHEEL_RADIUS_MM, TICKS_PER_CYCLE, 20, 0x47),
                    new WheelDriver(PIN_W2_M2_PCA_IN1, PIN_W2_M2_PCA_IN2, PIN_W2_M2_C1, PIN_W2_M2_C2, false, WHEEL_RADIUS_MM, TICKS_PER_CYCLE, 20, 0x47)),

    // right
    new SwerveWheel(new WheelDriver(PIN_W1_M1_PCA_IN1, PIN_W1_M1_PCA_IN2, PIN_W1_M1_C1, PIN_W1_M1_C2, false, WHEEL_RADIUS_MM, TICKS_PER_CYCLE, 20, 0x47),
                    new WheelDriver(PIN_W1_M2_PCA_IN1, PIN_W1_M2_PCA_IN2, PIN_W1_M2_C1, PIN_W1_M2_C2, false, WHEEL_RADIUS_MM, TICKS_PER_CYCLE, 20, 0x47))
};

SwerveWheel *_pDebugSwerve = _pSwerves[0];

void setup() {
    Serial.begin(115200);
    Wire.begin(PIN_SDA0, PIN_SCL0, 400000);

    if (!_ina.begin()) {
        LOGI("could not connect. Fix and Reboot\n");
    }
    _ina.setMaxCurrentShunt(5, 0.005);
    _ina.setAverage(4);

    for (int i = 0; i < ARRAY_SIZE(_pSwerves); i++) {
        _pSwerves[i]->setup();
    }

    pinMode(PIN_MOTOR_EN, OUTPUT);
    digitalWrite(PIN_MOTOR_EN, HIGH);
}

void debug() {
    if (Serial.available()) {
        int key = Serial.read();
        int val;

        switch (key) {
            case '1':
                _pDebugSwerve = _pSwerves[0];
                LOGI("center swerve selected\n");
                break;

            case '2':
                _pDebugSwerve = _pSwerves[1];
                LOGI("left   swerve selected\n");
                break;

            case '3':
                _pDebugSwerve = _pSwerves[2];
                LOGI("right swerve selected\n");
                break;

            case ' ':
                _is_go = !_is_go;
                LOGI("go:%d\n", _is_go);
                break;

            case 'm':
                val = digitalRead(PIN_MOTOR_EN);
                val = !val;
                digitalWrite(PIN_MOTOR_EN, val);
                LOGI("motor en:%d\n", val);
                break;

            case '\\':
                LOGI("bus:%4.1fV, shunt:%4.1fmV, current:%5.0fmA, power:%5.0fmW\n", _ina.getBusVoltage(), _ina.getShuntVoltage_mV(), _ina.getCurrent_mA(), _ina.getPower_mW());
                break;
        }
        _pDebugSwerve->debug(key);
    }
}

void loop() {
    debug();
    if (_is_go)
        _pDebugSwerve->loop();
}
