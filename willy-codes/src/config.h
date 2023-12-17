#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <Arduino.h>

/*
*****************************************************************************************
* FEATURES
*****************************************************************************************
*/
#define MOTOR_PWM_LIMIT         120

/*
*****************************************************************************************
* CONSTANTS
*****************************************************************************************
*/
#define WIFI_SSID       "TJ's House"
#define WIFI_PASSWORD   "cafebabe12"

#define WHEEL_RADIUS_MM     (75 / 2)
#define TICKS_PER_CYCLE     350

#define AXLE_WIDTH_MM       150

/*
*****************************************************************************************
* H/W CONSTANTS (PINS)
*****************************************************************************************
*/
#define PIN_NONE            -1

#define PIN_SDA0            21
#define PIN_SCL0            22
#define PIN_INA226_ALE      16

#define PIN_MOTOR_EN        2

#define PIN_W1_SENSE        15
#define PIN_W1_M1_C1        19
#define PIN_W1_M1_C2        23
#define PIN_W1_M1_PCA_IN1   0
#define PIN_W1_M1_PCA_IN2   1
#define PIN_W1_M2_C1        18
#define PIN_W1_M2_C2        17
#define PIN_W1_M2_PCA_IN1   2
#define PIN_W1_M2_PCA_IN2   3

#define PIN_W2_SENSE        4
#define PIN_W2_M1_C1        32
#define PIN_W2_M1_C2        33
#define PIN_W2_M1_PCA_IN1   4
#define PIN_W2_M1_PCA_IN2   5
#define PIN_W2_M2_C1        34
#define PIN_W2_M2_C2        35
#define PIN_W2_M2_PCA_IN1   6
#define PIN_W2_M2_PCA_IN2   7

#define PIN_W3_SENSE        5
#define PIN_W3_M1_C1        26
#define PIN_W3_M1_C2        27
#define PIN_W3_M1_PCA_IN1   8
#define PIN_W3_M1_PCA_IN2   9
#define PIN_W3_M2_C1        14
#define PIN_W3_M2_C2        13
#define PIN_W3_M2_PCA_IN1   10
#define PIN_W3_M2_PCA_IN2   11

#define PIN_LED         5
#define PIN_DRV_EN      2
#define PIN_L_CTR       4
#define PIN_L_DRV_IN1   23
#define PIN_L_DRV_IN2   19
#define PIN_R_CTR       18
#define PIN_R_DRV_IN1   16
#define PIN_R_DRV_IN2   17

#define PIN_LIDAR_RX    13
#define PIN_LIDAR_PWM   14


/*
*****************************************************************************************
* MACROS & STRUCTURES
*****************************************************************************************
*/

#endif