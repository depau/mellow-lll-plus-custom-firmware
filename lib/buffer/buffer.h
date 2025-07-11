/**
  ***************************************************************************************
  * @file    buffer.cpp
  * @author  lijihu
  * @version V1.0.0
  * @date    2025/05/10
 * @brief   Implementation of the buffer functionality
        *Buffer description
        Optical sensor: 1 when blocked, 0 when unblocked
        Filament switch: 0 when filament present, 1 when absent
        Button: 0 when pressed, 1 when released

        Pins:
        HALL1 --> PB2 (optical sensor 3)
        HALL2 --> PB3 (optical sensor 2)
        HALL3 --> PB4 (optical sensor 1)
        ENDSTOP_3 --> PB7 (filament switch)
        KEY1 --> PB13 (backward)
        KEY2 --> PB12 (forward)
  *
  * @note
  ***************************************************************************************
 * Copyright 2024 xxx@126.com
  ***************************************************************************************
**/

#ifndef LIB_BUFFER_BUFFER_H
#define LIB_BUFFER_BUFFER_H

#include <Arduino.h>
#include <TMCStepper.h>

#define HALL1 PB2 // optical sensor 3
#define HALL2 PB3 // optical sensor 2
#define HALL3 PB4 // optical sensor 1

#define ENDSTOP_3 PB7 // filament switch

#define KEY1 PB13 // backward
#define KEY2 PB12 // forward

#define EN_PIN PA6 // enable
#define DIR_PIN PA7 // direction
#define STEP_PIN PC13 // step
#define UART PB1 // software serial

#define FILAMENT_BREAK_INDICATOR PB15 // filament break indicator
#define ERR_LED PA15 // error LED
#define START_LED PA8 // start LED

#define DRIVER_ADDRESS 0b00 // TMC Driver address according to MS1 and MS2
#define R_SENSE 0.11f // Match to your driver

inline constexpr double SPEED_MM_S = 30.0;
inline constexpr double SPEED_MULTIPLIER = 9.1463414634;
inline constexpr double SPEED_RPM = SPEED_MM_S * SPEED_MULTIPLIER; // speed in r/min
inline constexpr int32_t MOVE_DIVIDE_NUM = 16;

#define STOP 0 // stop
#define I_CURRENT (600) // motor current
#define WRITE_EN_PIN(x) digitalWrite(EN_PIN, x) // toggle EN pin
#define FORWARD 1 // filament direction forward
#define BACK 0

#define DEBUG 0

// Structure storing the state of each sensor in the buffer
typedef struct Buffer {
  // buffer1
  bool buffer1_pos1_sensor_state;
  bool buffer1_pos2_sensor_state;
  bool buffer1_pos3_sensor_state;
  bool buffer1_material_swtich_state;
  bool key1;
  bool key2;

} Buffer;

// Motor state enumeration
typedef enum {
  Forward = 0, // forward
  Stop, // stop
  Back // backward
} Motor_State;

extern void buffer_sensor_init();
extern void buffer_motor_init();

extern void read_sensor_state();
extern void motor_control();

extern void buffer_init();
[[noreturn]] extern void buffer_loop();
extern void timer_it_callback();
extern void buffer_debug();

#endif // LIB_BUFFER_BUFFER_H
