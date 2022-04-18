
#include "grbl.h"

#include <Wire.h>
#include "PCA9685.h"

// Library using default Wire and default linear phase balancing scheme
PCA9685 pwmController;                  


// Linearly interpolates between standard 2.5%/12.5% phase length (102/512) for -90°/+90°
PCA9685_ServoEvaluator pwmServo1;

// Testing our second servo has found that -90° sits at 128, 0° at 324, and +90° at 526.
// Since 324 isn't precisely in the middle, a cubic spline will be used to smoothly
// interpolate PWM values, which will account for said discrepancy. Additionally, since
// 324 is closer to 128 than 526, there is less resolution in the -90° to 0° range, and
// more in the 0° to +90° range.
// PCA9685_ServoEvaluator pwmServo2(128,324,526);

const float PEN_HORIZONTAL = 90.0;
const float PEN_UP = PEN_HORIZONTAL - 30.0;
const float PEN_DOWN = PEN_HORIZONTAL + 1.0;

#define MIN_PEN_CHANNEL 12
#define MIN_LED_CHANNEL 0

void pwm_init()
{
    Wire.begin();                       // Wire must be started first
    Wire.setClock(400000);              // Supported baud rates are 100kHz, 400kHz, and 1000kHz

    pwmController.resetDevices();       // Software resets all PCA9685 devices on Wire line

    pwmController.init(B000000);        // Address pins A5-A0 set to B010101, default mode settings
    pwmController.setPWMFrequency(50);  // Default is 200Hz, supports 24Hz to 1526Hz

    pwmController.setChannelPWM(0, pwmServo1.pwmForAngle(PEN_UP));

    digitalWrite(1, 1);
}

void pen_move(byte tool, boolean up)
{
    pwmController.setChannelPWM(MIN_PEN_CHANNEL+tool, pwmServo1.pwmForAngle(up ? PEN_UP : PEN_DOWN));
}

void led(byte led, boolean on)
{
    pwmController.setChannelPWM(MIN_LED_CHANNEL+led, on ? 4096 : 0);
}
