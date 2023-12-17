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
float       _p = 0.7, _i = 0.3, _d = 0.0;
FastPID     _pid(_p, _i, _d, 50, 16, true);

long _tgtTick = 0;
bool _is_go = false;


SwerveWheel _wc = SwerveWheel(new WheelDriver(PIN_W1_M1_PCA_IN1, PIN_W1_M1_PCA_IN2, PIN_W1_M1_C1, PIN_W1_M1_C2, false, WHEEL_RADIUS_MM, TICKS_PER_CYCLE, 0x47),
                              new WheelDriver(PIN_W1_M2_PCA_IN1, PIN_W1_M2_PCA_IN2, PIN_W1_M2_C1, PIN_W1_M2_C2, false, WHEEL_RADIUS_MM, TICKS_PER_CYCLE, 0x47));

void setup() {
    Serial.begin(115200);
    Wire.begin(PIN_SDA0, PIN_SCL0, 400000);

    if (!_ina.begin()) {
        LOGI("could not connect. Fix and Reboot\n");
    }
    _ina.setMaxCurrentShunt(5, 0.005);
    _ina.setAverage(4);

    _wc.setup();

    pinMode(PIN_MOTOR_EN, OUTPUT);
    digitalWrite(PIN_MOTOR_EN, HIGH);

    _pid.setOutputRange(-128, 128);
}

void calibrate(int key) {
    static int idx = 0;
    int      speed;
    int      val;
    float    rot;

    switch (key) {
        case 'P':
            _p += 0.1;
            _pid.setCoefficients(_p, _i, _d, 100);
            LOGI("PID: %4.1f, %4.1f, %4.1f\n", _p, _i, _d);
            break;

        case 'p':
            _p -= 0.1;
            _pid.setCoefficients(_p, _i, _d, 100);
            LOGI("PID: %4.1f, %4.1f, %4.1f\n", _p, _i, _d);
            break;

        case 'I':
            _i += 0.1;
            _pid.setCoefficients(_p, _i, _d, 100);
            LOGI("PID: %4.1f, %4.1f, %4.1f\n", _p, _i, _d);
            break;

        case 'i':
            _i -= 0.1;
            _pid.setCoefficients(_p, _i, _d, 100);
            LOGI("PID: %4.1f, %4.1f, %4.1f\n", _p, _i, _d);
            break;

        case 'D':
            _d += 0.1;
            _pid.setCoefficients(_p, _i, _d, 100);
            LOGI("PID: %4.1f, %4.1f, %4.1f\n", _p, _i, _d);
            break;

        case 'd':
            _d -= 0.1;
            _pid.setCoefficients(_p, _i, _d, 100);
            LOGI("PID: %4.1f, %4.1f, %4.1f\n", _p, _i, _d);
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

        case 't':
            _tgtTick -= 350;
            LOGI("target : %ld\n", _tgtTick);
            _is_go = true;
            break;

        case 'T':
            _tgtTick += 350;
            LOGI("target : %ld\n", _tgtTick);
            _is_go = true;
            break;

        case '\\':
            LOGI("bus:%4.1fV, shunt:%4.1fmV, current:%5.0fmA, power:%5.0fmW\n", _ina.getBusVoltage(), _ina.getShuntVoltage_mV(), _ina.getCurrent_mA(), _ina.getPower_mW());
            break;
    }
}

TimeEvent _evt(5);

void loop() {
    if (Serial.available()) {
        int ch = Serial.read();
        calibrate(ch);
        _wc.calibrate(ch);
    }

    if (_is_go && _evt.every(20)) {
        Ticks t = _wc.getTicks();

        long cur = t.get().down;
        int  spd = _pid.step(_tgtTick, cur);
        LOGI("pos : %8ld / %8ld => spd:%3d\n", cur, _tgtTick, spd);

        _wc.setMotor(spd, spd);
    }

    _evt.tick();
}

#if TEST_I2C
#    include <Wire.h>

void setup() {
    // pinMode(PIN_SCL0, INPUT_PULLUP);
    // pinMode(PIN_SDA0, INPUT_PULLUP);
    pinMode(PIN_INA226_ALE, INPUT_PULLUP);
    Wire.begin(PIN_SDA0, PIN_SCL0, 100000);
    pinMode(PIN_SCL0, INPUT);
    pinMode(PIN_SDA0, INPUT);

    Serial.begin(115200);
    while (!Serial)
        ;  // Leonardo: wait for serial monitor
    LOGI("\nI2C Scanner");
}

void i2cdetect(uint8_t first, uint8_t last) {
    uint8_t i, address, error;
    char buff[10];

    // table header
    Serial.print("   ");
    for (i = 0; i < 16; i++) {
        // Serial.printf("%3x", i);
        sprintf(buff, "%3x", i);
        Serial.print(buff);
    }

    // table body
    // addresses 0x00 through 0x77
    for (address = 0; address <= 119; address++) {
        if (address % 16 == 0) {
            // Serial.printf("\n%#02x:", address & 0xF0);a
            sprintf(buff, "\n%02x:", address & 0xF0);
            Serial.print(buff);
        }
        if (address >= first && address <= last) {
            Wire.beginTransmission(address);
            error = Wire.endTransmission();
            if (error == 0) {
                // device found
                // Serial.printf(" %02x", address);
                sprintf(buff, " %02x", address);
                Serial.print(buff);
            } else if (error == 4) {
                // other error
                Serial.print(" XX");
            } else {
                // error = 2: received NACK on transmit of address
                // error = 3: received NACK on transmit of data
                Serial.print(" --");
            }
        } else {
            // address not scanned
            Serial.print("   ");
        }
    }
    LOGI("\n");
}

// void i2cdetect() {
//   i2cdetect(0x03, 0x77);  // default range
// }

void loop() {
    i2cdetect(0x03, 0x77);  // default range

    delay(1000);  // wait 5 seconds for next scan
}
#endif
