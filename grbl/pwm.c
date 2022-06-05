
#include <Arduino.h>

#include "grbl.h"

// #include <Wire.h>
// #include "PCA9685.h"

// Library using default Wire and default linear phase balancing scheme
// PCA9685 pwmController;                  


// Linearly interpolates between standard 2.5%/12.5% phase length (102/512) for -90°/+90°
// PCA9685_ServoEvaluator pwmServo1;

// Testing our second servo has found that -90° sits at 128, 0° at 324, and +90° at 526.
// Since 324 isn't precisely in the middle, a cubic spline will be used to smoothly
// interpolate PWM values, which will account for said discrepancy. Additionally, since
// 324 is closer to 128 than 526, there is less resolution in the -90° to 0° range, and
// more in the 0° to +90° range.
// PCA9685_ServoEvaluator pwmServo2(128,324,526);

//default angle mapping (it is not linear)
//angle pwm
//  -90 102
//  -30 239
//  -15 273
//    0 307
//   15 341
//   30 375
//   90 512

const float PEN_HORIZONTAL = 307;
const float PEN_UP = 341;
const float PEN_DOWN = 273;

#define MAX_PEN_CHANNEL 15
#define MIN_PEN_CHANNEL 12
#define MAX_LED_CHANNEL 11
#define MIN_LED_CHANNEL 0

//#define LED_POWER 4096
#define LED_POWER 512

#define I2C_TIMEOUT 1000
#define I2C_PULLUP 1

// #ifdef __AVR_ATmega328P__
/* Corresponds to A4/A5 - the hardware I2C pins on Arduinos */
#define SDA_PORT PORTC
#define SDA_PIN 4
#define SCL_PORT PORTC
#define SCL_PIN 5
#define I2C_FASTMODE 1
// #define I2C_SLOWMODE 1
// #else
// #define SDA_PORT PORTB
// #define SDA_PIN 0
// #define SCL_PORT PORTB
// #define SCL_PIN 2
// #define I2C_FASTMODE 1
// #endif

#define I2C_HARDWARE 1
#define I2C_PULLUP 1

#include <SoftI2CMaster.h>

#define PCA9685_I2C_BASE_ADDRESS    (byte)0x40

// I2C 7-bit address is B 1 A5 A4 A3 A2 A1 A0
// RW bit added by Arduino core TWI library
#define PCA9685_PIN_ADRES B00000000
#define I2C_7BITADDR (PCA9685_I2C_BASE_ADDRESS | (PCA9685_PIN_ADRES & 0x3F))

#define PCA9685_SW_RESET            (byte)0x06 // Sent to address 0x00 to reset all devices on Wire line
#define PCA9685_LED0_REG            (byte)0x06 // Start of LEDx regs, 4B per reg, 2B on phase, 2B off phase, little-endian
#define PCA9685_ALLLED_REG          (byte)0xFA
#define PCA9685_PWM_FULL            (uint16_t)0x01000 // Special value for full on/full off LEDx modes
 // Register addresses from data sheet
#define PCA9685_MODE1_REG           (byte)0x00
#define PCA9685_MODE2_REG           (byte)0x01
#define PCA9685_SUBADR1_REG         (byte)0x02
#define PCA9685_SUBADR2_REG         (byte)0x03
#define PCA9685_SUBADR3_REG         (byte)0x04
#define PCA9685_ALLCALL_REG         (byte)0x05
#define PCA9685_LED0_REG            (byte)0x06 // Start of LEDx regs, 4B per reg, 2B on phase, 2B off phase, little-endian
#define PCA9685_PRESCALE_REG        (byte)0xFE
#define PCA9685_ALLLED_REG          (byte)0xFA

// Mode1 register pin layout
#define PCA9685_MODE_RESTART        (byte)0x80
#define PCA9685_MODE_EXTCLK         (byte)0x40
#define PCA9685_MODE_AUTOINC        (byte)0x20
#define PCA9685_MODE_SLEEP          (byte)0x10
#define PCA9685_MODE_SUBADR1        (byte)0x08
#define PCA9685_MODE_SUBADR2        (byte)0x04
#define PCA9685_MODE_SUBADR3        (byte)0x02
#define PCA9685_MODE_ALLCALL        (byte)0x01
 
// Output-enabled/active-low-OE-pin=LOW driver control modes (see datasheet Table 12 and Fig 13, 14, and 15 concerning correct usage of INVRT and OUTDRV):
#define PCA9685_MODE_INVRT          (byte)0x10  // Enables channel output polarity inversion (applicable only when active-low-OE-pin=LOW)
#define PCA9685_MODE_OUTDRV_TPOLE   (byte)0x04  // Enables totem-pole (instead of open-drain) style structure to be used for driving channel output, allowing use of an external output driver

// The try requests number before pen move is reported as done by pin PEN_SENSOR
uint16_t penRetryCounter = 0;

    void resetDevices() {
        if (!i2c_start((I2C_7BITADDR << 1) | I2C_WRITE)) {
            printPgmString(PSTR("I2C device busy"));
            // delay(1000);
            // return;
        }
        i2c_write(PCA9685_SW_RESET);
        i2c_stop();
    }

    void getPhaseCycle(int channel, uint16_t pwmAmount, uint16_t *phaseBegin, uint16_t *phaseEnd) {
        // Set delay
        if (channel < 0) {
            // All channels
            *phaseBegin = 0;
        }
        else  {
            // Distribute high phase area over entire phase range to balance load.
            *phaseBegin = channel * (4096 / 16);
        }
        
        // See datasheet section 7.3.3
        if (pwmAmount == 0) {
            // Full OFF => time_off[12] = 1;
            *phaseEnd = PCA9685_PWM_FULL;
        }
        else if (pwmAmount >= PCA9685_PWM_FULL) {
            // Full ON => time_on[12] = 1; time_off = ignored;
            *phaseBegin |= PCA9685_PWM_FULL;
            *phaseEnd = 0;
        }
        else {
            *phaseEnd = *phaseBegin + pwmAmount;
            if (*phaseEnd >= PCA9685_PWM_FULL)
                *phaseEnd -= PCA9685_PWM_FULL;
        }
    }

    void setChannelPWM(int channel, uint16_t pwmAmount) {

        // writeChannelBegin(channel);
        byte regAddress;
        if (channel != -1) {
            regAddress = PCA9685_LED0_REG + (channel * 0x04);
        }
        else {
            regAddress = PCA9685_ALLLED_REG;
        }
        if (!i2c_start((I2C_7BITADDR << 1) | I2C_WRITE)) {
            printPgmString(PSTR("I2C device busy"));
            // delay(1000);
            // return;
        }
        i2c_write(regAddress);

        uint16_t phaseBegin, phaseEnd;
        getPhaseCycle(channel, pwmAmount, &phaseBegin, &phaseEnd);
        i2c_write(lowByte(phaseBegin));
        i2c_write(highByte(phaseBegin));
        i2c_write(lowByte(phaseEnd));
        i2c_write(highByte(phaseEnd));
        i2c_stop();  
    }

    byte readRegister(byte regAddress) {
        if (!i2c_start((I2C_7BITADDR << 1) | I2C_WRITE)) {
            printPgmString(PSTR("I2C device busy"));
        }
        i2c_write(regAddress);
        i2c_rep_start((I2C_7BITADDR << 1) | I2C_READ);
        byte val = i2c_read(true);
        i2c_stop();
        return val;
    }

    void setPWMFrequency(float pwmFrequency) {
     if (pwmFrequency < 0) return;
        // This equation comes from section 7.3.5 of the datasheet, but the rounding has been
        // removed because it isn't needed. Lowest freq is 23.84, highest is 1525.88.
        int preScalerVal = (25000000 / (4096 * pwmFrequency)) - 1;
        if (preScalerVal > 255) preScalerVal = 255;
        if (preScalerVal < 3) preScalerVal = 3;
        // The PRE_SCALE register can only be set when the SLEEP bit of MODE1 register is set to logic 1.
        byte mode1Reg = readRegister(PCA9685_MODE1_REG);
        writeRegister(PCA9685_MODE1_REG, (mode1Reg = (mode1Reg & ~PCA9685_MODE_RESTART) | PCA9685_MODE_SLEEP));
        writeRegister(PCA9685_PRESCALE_REG, (byte)preScalerVal);
        // It takes 500us max for the oscillator to be up and running once SLEEP bit has been set to logic 0.
        writeRegister(PCA9685_MODE1_REG, (mode1Reg = (mode1Reg & ~PCA9685_MODE_SLEEP) | PCA9685_MODE_RESTART));
        delayMicroseconds(500);
    }
  
    void writeRegister(byte regAddress, byte value) {
        if (!i2c_start((I2C_7BITADDR << 1) | I2C_WRITE)) {
            printPgmString(PSTR("I2C device busy"));
            // delay(1000);
            // return;
        }
        i2c_write(regAddress);
        i2c_write(value);
        i2c_stop();  
    } 

    pwmControlerInit() {
        // pwmController.init(B000000);        // Address pins A5-A0 set to B010101, default mode settings
        writeRegister(PCA9685_MODE1_REG, PCA9685_MODE_RESTART | PCA9685_MODE_AUTOINC);
        writeRegister(PCA9685_MODE2_REG, PCA9685_MODE_OUTDRV_TPOLE);
    }


    void pwm_init()
    {
        if (!i2c_init()) {
            printPgmString(PSTR("I2C init failed"));
        }

        // pwmController.resetDevices();       // Software resets all PCA9685 devices on Wire line
        resetDevices();

        pwmControlerInit();
        
        // pwmController.setPWMFrequency(50);  // Default is 200Hz, supports 24Hz to 1526Hz
        setPWMFrequency(50);

        // pinMode(A6, INPUT);
        //    pinMode(LED_BUILTIN, OUTPUT);
        // led(5, digitalRead(12)==HIGH);

    }

    void pwm_reset()
    {
        for(int i=MIN_PEN_CHANNEL; i<=MAX_PEN_CHANNEL; i++)
        {
        //     pwmController.setChannelPWM(i, PEN_UP);
            setChannelPWM(i, PEN_UP);
        }

        for(int i=MIN_LED_CHANNEL; i<=MAX_LED_CHANNEL; i++)
        {
        //     pwmController.setChannelPWM(i, 0);
            setChannelPWM(i, 0);
        }
        penRetryCounter = 0;
        sys_rt_pen_motion = 0;
        sys_position[Z_AXIS] = settings.steps_per_mm[Z_AXIS];
        gc_sync_position();
        sys_tool = 0;
        gc_state.tool = 0;
    }

    void pen_move(uint8_t tool, boolean up)
    {
    //    pwmController.setChannelPWM(MIN_PEN_CHANNEL+tool, up ? PEN_UP : PEN_DOWN);
       setChannelPWM(MIN_PEN_CHANNEL+tool, up ? PEN_UP : PEN_DOWN);
    }

    void led(uint8_t led, boolean on)
    {
        // pwmController.setChannelPWM(MIN_LED_CHANNEL+led, on ? 4096 : 0);
        setChannelPWM(MIN_LED_CHANNEL+led, on ? LED_POWER : 0);
    }

    void pen_rt_move() {
        if (sys.state == STATE_ALARM) {
            sys_rt_pen_motion = EXEC_PEN_REQUEST_UP | EXEC_PEN_MOVE_FORCE;
        }
        if ((sys_rt_pen_motion & EXEC_PEN_MOVE_FORCE) && (++penRetryCounter>30000)) {
            system_set_exec_alarm(EXEC_ALARM_PEN_MOVE_FAIL);
            return;
        }
        if (sys_rt_pen_motion & EXEC_PEN_REQUEST_UP) {
            pen_move(sys_tool, true);
            if (sys_rt_pen_motion & EXEC_PEN_MOVE_FORCE) {
                if (!PEN_IS_DOWN) {
                    sys_rt_pen_motion = 0;
                    penRetryCounter = 0;
                }
            }
        }
        else if(sys_rt_pen_motion & EXEC_PEN_REQUEST_DOWN) {
            pen_move(sys_tool, false);
            if (sys_rt_pen_motion & EXEC_PEN_MOVE_FORCE) {
                if (PEN_IS_DOWN) {
                    sys_rt_pen_motion = 0;
                    penRetryCounter = 0;
                }
            }
        }
        // no request, clear EXEC_PEN_MOVE_FORCE and set pen state
        else {
            sys_rt_pen_motion = 0;
            penRetryCounter = 0;
        }
    }

    