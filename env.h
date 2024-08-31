#pragma once

#ifndef serial
#define serial
#endif

#ifndef BAUDRATE
#define BAUDRATE 19200
#endif

#ifndef NUM_FLAPS
#define NUM_FLAPS 45
#endif

// Pins of I2C adress switch
// https://europe1.discourse-cdn.com/arduino/original/4X/7/b/2/7b223c07c6657f87fe374782fdd86f353b475f1a.jpeg
#define ADDR_BIT0 2  // PD 2
#define ADDR_BIT1 3  // PD 3
#define ADDR_BIT2 4  // PD 4
#define ADDR_BIT3 5  // PD 5
#define ADDR_BIT4 6  // PD 6
#define ADDR_BIT5 A0 // PC 0, 14
#define ADDR_BIT6 A1 // PC 1, 15
#define ADDR_BIT7 A2 // PC 2, 16

// constants stepper
#define STEPPERPIN1 11
#define STEPPERPIN2 10
#define STEPPERPIN3 9
#define STEPPERPIN4 8
#define STEPS 2038 // 28BYJ-48 stepper, number of steps
#define HALLPIN 7  // Pin of hall sensor

// constants others
#define ROTATIONDIRECTION 1     //-1 for reverse direction
#define OVERHEATINGTIMEOUT 2    // timeout in seconds to avoid overheating of stepper. After starting rotation, the counter will start. Stepper won't move again until timeout is passed
#define COMMAND_UPDATE_OFFSET 0 // command to update offset
#define COMMAND_SHOW_LETTER 1   // command to show letter

// EEPROM addresses
#define EEPROM_ADDR_OFFSET_HIGHER_BYTE 1 // offset in EEPROM
#define EEPROM_ADDR_OFFSET_LOWER_BYTE 2  // offset in EEPROM

// 2038 steps / 45 letters = 45.28888 steps / letter
