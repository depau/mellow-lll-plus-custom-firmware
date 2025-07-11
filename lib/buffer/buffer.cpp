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
 * Copyright 2025 xxx@126.com
  ***************************************************************************************
**/

#include <USBSerial.h>
#include <array>
#include <cstdio>

#include "buffer.h"

namespace {
TMC2209Stepper driver(UART, UART, R_SENSE, DRIVER_ADDRESS);
Buffer buffer{}; // stores sensor states
Motor_State motor_state = Stop;

bool is_front = false; // forward flag
uint32_t front_time = 0; // forward time
constexpr uint32_t DEFAULT_TIMEOUT = 90000U;
constexpr uint32_t TIMER_PRESCALE = 48U;
constexpr uint32_t TIMER_OVERFLOW = 1000U;
constexpr uint32_t ONE_SECOND_MS = 1000U;
constexpr int MAX_INDEX = 0x1ff;
constexpr uint32_t TOGGLE_INTERVAL_MS = 500U;
constexpr uint32_t SERIAL_BAUDRATE = 9600U;
constexpr uint8_t DRIVER_TOFF = 5U;
constexpr uint32_t MAX_TIMEOUT = 0xFFFFFFFFU;

// duration (ms) for a press to be considered "short"
constexpr uint32_t SHORT_PRESS_MAX_DURATION_MS = 150U;

// number of presses required to enable continuous movement
constexpr uint8_t DEFAULT_MULTI_PRESS_COUNT = 2U;
uint8_t multi_press_count = DEFAULT_MULTI_PRESS_COUNT;
// minimum time between presses to be considered part of the same sequence (ms)
constexpr uint32_t MULTI_PRESS_MIN_INTERVAL_MS = 50U;
// maximum time between presses to be considered part of the same sequence (ms)
constexpr uint32_t MULTI_PRESS_MAX_INTERVAL_MS = 500U;

constexpr size_t CMD_BUF_MAX_LEN = 64U;
constexpr uint32_t CMD_TIMEOUT_MS = 3000U;

uint32_t last_char_time = 0;
String command_buffer;

enum class MoveReport {
  F,
  B,
  FPLUS,
  BPLUS,
  S,
  T,
  O,
  NONE
};
MoveReport last_move_report = MoveReport::NONE;
bool filament_present = true;
bool last_filament_present = true;
bool motor_on = true;

float speed_mm_s = SPEED_MM_S;
uint32_t current_vactual = 0;

bool move_active = false;
Motor_State move_direction = Stop;
uint32_t move_end_time = 0;

uint32_t timeout = DEFAULT_TIMEOUT; // timeout in ms
bool is_timeout = false; // error flag, set if pushing filament for 30s without stopping
bool continuous_run = false; // flag for continuous movement
Motor_State continuous_direction = Stop; // direction for continuous movement

void sendMessage(const String &msg) {
  SerialUSB.println(msg);
  Serial2.println(msg);
}

void sendMovement(MoveReport r) {
  if (r == last_move_report) {
    return;
  }
  last_move_report = r;
  switch (r) {
  case MoveReport::F:
    sendMessage("f");
    break;
  case MoveReport::B:
    sendMessage("b");
    break;
  case MoveReport::FPLUS:
    sendMessage("f+");
    break;
  case MoveReport::BPLUS:
    sendMessage("b+");
    break;
  case MoveReport::S:
    sendMessage("s");
    break;
  case MoveReport::T:
    sendMessage("t");
    break;
  case MoveReport::O:
    sendMessage("o");
    break;
  case MoveReport::NONE:
    break;
  }
}

uint32_t mmpsToVactual(float mm_s) {
  const float rpm = mm_s * SPEED_MULTIPLIER;
  return static_cast<uint32_t>(rpm * MOVE_DIVIDE_NUM * 200.0f / 60.0f / 0.715f);
}

inline float absFloat(float v) {
  return v < 0.0f ? -v : v;
}

} // namespace

void buffer_init() {
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-cstyle-cast, performance-no-int-to-ptr)
  static HardwareTimer timer(TIM6); // timer for timeout handling

  buffer_sensor_init();
  buffer_motor_init();
  current_vactual = mmpsToVactual(speed_mm_s);
  delay(ONE_SECOND_MS);

  timer.pause();
  timer.setPrescaleFactor(TIMER_PRESCALE); // divide by 48 -> 48 MHz / 48 = 1 MHz
  timer.setOverflow(TIMER_OVERFLOW); // 1ms
  timer.attachInterrupt(&timer_it_callback);
  timer.resume();
}

MoveReport currentMoveReport();
void reportAllStatus();
void handleCommand(const String &cmd);
void processSerialInput();
void updateStatus();

[[noreturn]] void buffer_loop() {
  uint32_t lastToggleTime = millis(); // remember the last toggle time

  sendMessage(String("timeout ") + timeout);
  sendMessage(String("multi_press_count ") + multi_press_count);
  sendMessage(String("speed ") + String(speed_mm_s, 2));
  last_filament_present = digitalRead(ENDSTOP_3) == 0;
  sendMessage(last_filament_present ? "p1" : "p0");

  while (true) {
    if (millis() - lastToggleTime >= TOGGLE_INTERVAL_MS) // toggle every 500ms
    {
      lastToggleTime = millis(); // update current time
      digitalToggle(ERR_LED);
    }
    // 1. read values from all sensors
    read_sensor_state();

#if DEBUG
    static String serial_buf;
    buffer_debug();
    while (SerialUSB.available() > 0) {
      const char incoming_char = SerialUSB.read();
      serial_buf += incoming_char;
      int pos_enter = -1;
      pos_enter = serial_buf.indexOf("\n");
      if (pos_enter != -1) {
        String str = serial_buf.substring(0, pos_enter);
        serial_buf = serial_buf.substring(pos_enter + 1);
        if (strstr(str.c_str(), "gconf") != nullptr) {
          TMC2208_n::CHOPCONF_t gconf{ 0 };

          // Extract the hexadecimal string after "gconf"
          int pos = str.indexOf("gconf");
          if (pos != -1) {
            String hexPart = str.substring(pos + 5); // skip "gconf"
            hexPart.trim(); // remove whitespace

            // Convert the string to a 32-bit unsigned integer
            uint32_t hexValue = strtoul(hexPart.c_str(), nullptr, 16);

            // Assign it to the struct (according to your definition)
            gconf.sr = hexValue; // assume sr is the raw register field
          }
          driver.GCONF(gconf.sr);
          SerialUSB.print("write GCONF:0x");
          SerialUSB.println(gconf.sr, HEX);
          SerialUSB.print("read GCONF: 0x");
          SerialUSB.println(driver.GCONF(), HEX);
        }
      }
    }
#else
    motor_control();
    processSerialInput();
    updateStatus();
#endif
  }
}

void buffer_sensor_init() {
  // Initialize sensors
  pinMode(HALL1, INPUT);
  pinMode(HALL2, INPUT);
  pinMode(HALL3, INPUT);
  pinMode(ENDSTOP_3, INPUT);
  pinMode(KEY1, INPUT);
  pinMode(KEY2, INPUT);

  // Initialize filament indicator LEDs
  pinMode(FILAMENT_BREAK_INDICATOR, OUTPUT);
  pinMode(ERR_LED, OUTPUT);
  pinMode(START_LED, OUTPUT);
}

void buffer_motor_init() {
  // Initialize motor driver pins
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // Enable driver in hardware

  // Initialize motor driver
  driver.begin(); // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.beginSerial(SERIAL_BAUDRATE);
  driver.I_scale_analog(false);
  driver.toff(DRIVER_TOFF); // Enables driver in software
  driver.rms_current(I_CURRENT); // Set motor RMS current
  driver.microsteps(MOVE_DIVIDE_NUM); // Set microsteps to 1/16th
  driver.VACTUAL(STOP); // Set velocity
  driver.en_spreadCycle(true);
  driver.pwm_autoscale(true);
}

/**
 * @brief  Read the state of all sensors
 * @retval nullptr
 **/
void read_sensor_state() {
  buffer.buffer1_pos1_sensor_state = digitalRead(HALL3) != 0;
  buffer.buffer1_pos2_sensor_state = digitalRead(HALL2) != 0;
  buffer.buffer1_pos3_sensor_state = digitalRead(HALL1) != 0;
  buffer.buffer1_material_swtich_state = digitalRead(ENDSTOP_3) != 0;
  buffer.key1 = digitalRead(KEY1) != 0;
  buffer.key2 = digitalRead(KEY2) != 0;
}

namespace {
void runMotor(Motor_State dir, Motor_State last_state) {
  WRITE_EN_PIN(0);
  if (dir != last_state) {
    driver.VACTUAL(STOP);
  }
  driver.shaft(dir == Forward ? FORWARD : BACK);
  driver.VACTUAL(current_vactual);
  motor_on = true;
}

void holdMotor() {
  WRITE_EN_PIN(0);
  driver.VACTUAL(STOP);
  motor_on = true;
}

void disableMotor() {
  WRITE_EN_PIN(1);
  driver.VACTUAL(STOP);
  motor_on = false;
}

bool handleMove(Motor_State &last_motor_state) {
  if (!move_active) {
    return false;
  }
  if (digitalRead(ENDSTOP_3) != 0) {
    disableMotor();
    motor_state = Stop;
    sendMovement(MoveReport::O);
    move_active = false;
    return true;
  }
  if (millis() >= move_end_time) {
    holdMotor();
    motor_state = Stop;
    sendMovement(MoveReport::S);
    move_active = false;
    return true;
  }
  runMotor(move_direction, last_motor_state);
  is_front = move_direction == Forward;
  last_motor_state = move_direction;
  motor_state = move_direction;
  sendMovement(move_direction == Forward ? MoveReport::F : MoveReport::B);
  return true;
}

bool handleContinuousRun(Motor_State &last_motor_state) {
  if (!continuous_run) {
    return false;
  }
  if (digitalRead(KEY1) == LOW || digitalRead(KEY2) == LOW || is_timeout) {
    holdMotor();
    motor_state = Stop;
    continuous_run = false;
    is_front = false;
    front_time = 0;
    is_timeout = false;
    while (digitalRead(KEY1) == LOW || digitalRead(KEY2) == LOW) {
    }
    return true;
  }
  runMotor(continuous_direction, last_motor_state);
  is_front = continuous_direction == Forward;
  last_motor_state = continuous_direction;
  return true;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
bool handleButton(uint8_t pin, Motor_State dir, uint32_t &last_time, uint8_t &count, Motor_State &last_motor_state) {
  if (digitalRead(pin) != LOW) {
    return false;
  }
  const uint32_t now = millis();
  if (now - last_time >= MULTI_PRESS_MIN_INTERVAL_MS && now - last_time <= MULTI_PRESS_MAX_INTERVAL_MS) {
    count++;
  } else {
    count = 1;
  }
  last_time = now;

  runMotor(dir, last_motor_state);
  last_motor_state = dir;
  last_move_report = MoveReport::NONE;
  sendMovement(dir == Forward ? MoveReport::FPLUS : MoveReport::BPLUS);
  const uint32_t press_start = millis();
  while (digitalRead(pin) == LOW) {
  }
  const uint32_t duration = millis() - press_start;
  if (digitalRead(ENDSTOP_3) == 0) {
    holdMotor();
  } else {
    disableMotor();
  }
  motor_state = Stop;
  is_front = false;
  front_time = 0;

  const bool enable_continuous = count >= multi_press_count;

  if (duration <= SHORT_PRESS_MAX_DURATION_MS && !enable_continuous) {
    is_timeout = !is_timeout; // toggle timeout state
  } else {
    is_timeout = false;
  }

  if (enable_continuous) {
    continuous_run = true;
    continuous_direction = dir;
    count = 0;
  }
  return true;
}
} // namespace

/**
 * @brief  Motor control
 * @retval nullptr
 **/
void motor_control() {
  static Motor_State last_motor_state = Stop;
  static uint32_t last_key1_time = 0;
  static uint8_t key1_count = 0;
  static uint32_t last_key2_time = 0;
  static uint8_t key2_count = 0;

  if (handleMove(last_motor_state)) {
    return;
  }

  if (handleContinuousRun(last_motor_state)) {
    return;
  }

  if (handleButton(KEY1, Back, last_key1_time, key1_count, last_motor_state)) {
    return;
  }
  if (handleButton(KEY2, Forward, last_key2_time, key2_count, last_motor_state)) {
    return;
  }

  // Check filament presence
  if (digitalRead(ENDSTOP_3) != 0) {
    // No filament, stop motor
    driver.VACTUAL(STOP); // stop
    motor_state = Stop;

    // Pull filament-break pin low
    digitalWrite(FILAMENT_BREAK_INDICATOR, LOW);

    // Turn off indicator LED
    digitalWrite(START_LED, LOW);

    is_front = false;
    front_time = 0;
    is_timeout = false;
    continuous_run = false;
    move_active = false;
    disableMotor();

    return; // no filament, exit
  }

  // Filament present, set filament-break pin high
  digitalWrite(FILAMENT_BREAK_INDICATOR, HIGH);

  // Turn on indicator LED
  digitalWrite(START_LED, HIGH);

  // Check for error condition
  if (is_timeout) {
    // Stop motor
    driver.VACTUAL(STOP); // stop
    motor_state = Stop;
    holdMotor();
    return;
  }

  // Buffer position handling
  if (buffer.buffer1_pos1_sensor_state) // buffer position 1 -> push filament forward
  {
    last_motor_state = motor_state; // remember previous state
    motor_state = Forward;
    is_front = true;

  } else if (buffer.buffer1_pos2_sensor_state) // buffer position 2 -> stop motor
  {
    last_motor_state = motor_state; // remember previous state
    motor_state = Stop;
    is_front = false;
    front_time = 0;
  } else if (buffer.buffer1_pos3_sensor_state) // buffer position 3 -> retract filament
  {
    last_motor_state = motor_state; // remember previous state
    motor_state = Back;
    is_front = false;
    front_time = 0;
  }

  if (motor_state == last_motor_state) { // same as last state, no need to send command
    return;
  }

  // Motor control
  switch (motor_state) {
  case Forward: // forward
  {
    runMotor(Forward, last_motor_state);

  } break;
  case Stop: // stop
  {
    holdMotor();

  } break;
  case Back: // backward
  {
    runMotor(Back, last_motor_state);
  } break;
  }
}

void timer_it_callback() {
  if (is_front) { // when pushing forward
    front_time++;
    if (front_time > timeout) { // timeout reached
      is_timeout = true;
    }
  }
}

MoveReport currentMoveReport() {
  if (!motor_on) {
    return MoveReport::O;
  }
  if (is_timeout) {
    return MoveReport::T;
  }
  if (continuous_run) {
    return continuous_direction == Forward ? MoveReport::FPLUS : MoveReport::BPLUS;
  }
  switch (motor_state) {
  case Forward:
    return MoveReport::F;
  case Back:
    return MoveReport::B;
  case Stop:
  default:
    return MoveReport::S;
  }
}

void reportAllStatus() {
  sendMovement(currentMoveReport());
  sendMessage(filament_present ? "p1" : "p0");
  sendMessage(String("timeout ") + timeout);
  sendMessage(String("multi_press_count ") + multi_press_count);
  sendMessage(String("speed ") + String(speed_mm_s, 2));
}

void handleCommand(const String &cmd) {
  if (cmd == "f") {
    move_active = false;
    continuous_run = true;
    continuous_direction = Forward;
    runMotor(Forward, motor_state);
    motor_state = Forward;
    sendMovement(MoveReport::FPLUS);
  } else if (cmd == "b") {
    move_active = false;
    continuous_run = true;
    continuous_direction = Back;
    runMotor(Back, motor_state);
    motor_state = Back;
    sendMovement(MoveReport::BPLUS);
  } else if (cmd == "s") {
    move_active = false;
    is_timeout = true;
    continuous_run = false;
    sendMovement(MoveReport::T);
  } else if (cmd == "n") {
    move_active = false;
    continuous_run = false;
    is_timeout = false;
    if (digitalRead(ENDSTOP_3) == 0) {
      holdMotor();
      motor_state = Stop;
      sendMovement(MoveReport::S);
    } else {
      disableMotor();
      motor_state = Stop;
      sendMovement(MoveReport::O);
    }
  } else if (cmd == "o") {
    move_active = false;
    continuous_run = false;
    is_timeout = false;
    disableMotor();
    motor_state = Stop;
    sendMovement(MoveReport::O);
  } else if (cmd == "q") {
    reportAllStatus();
  } else if (cmd.startsWith("set_speed")) {
    String v = cmd.substring(String("set_speed").length());
    v.trim();
    const float num = v.toFloat();
    if (num > 0.0f) {
      speed_mm_s = num;
      current_vactual = mmpsToVactual(speed_mm_s);
    }
    sendMessage(String("speed ") + String(speed_mm_s, 2));
  } else if (cmd.startsWith("move")) {
    String v = cmd.substring(String("move").length());
    v.trim();
    const float dist = v.toFloat();
    move_active = true;
    continuous_run = false;
    is_timeout = false;
    move_direction = dist >= 0 ? Forward : Back;
    const float duration = absFloat(dist) / (speed_mm_s > 0.0F ? speed_mm_s : 1.0F) * 1000.0F;
    move_end_time = millis() + static_cast<uint32_t>(duration);
    runMotor(move_direction, motor_state);
    motor_state = move_direction;
    sendMovement(move_direction == Forward ? MoveReport::F : MoveReport::B);
  } else if (cmd.startsWith("set_timeout")) {
    String v = cmd.substring(String("set_timeout").length());
    v.trim();
    const int64_t num = v.toInt();
    if (num >= 0 && static_cast<uint64_t>(num) <= MAX_TIMEOUT) {
      timeout = static_cast<uint32_t>(num);
    }
    sendMessage(String("timeout ") + timeout);
  } else if (cmd.startsWith("set_multi_press_count")) {
    String v = cmd.substring(String("set_multi_press_count").length());
    v.trim();
    const int num = v.toInt();
    if (num > 0 && num < 10) {
      multi_press_count = static_cast<uint8_t>(num);
    }
    sendMessage(String("multi_press_count ") + multi_press_count);
  }
}

void process_serial_char(char c) {
  if (c == '\r') {
    return;
  }
  last_char_time = millis();
  if (c == '\n' || command_buffer.length() >= CMD_BUF_MAX_LEN) {
    String cmd = command_buffer;
    cmd.trim();
    command_buffer = "";
    if (cmd.length() > 0) {
      handleCommand(cmd);
    }
    return;
  }
  command_buffer += c;
}

void processSerialInput() {
  while (SerialUSB.available() > 0) {
    process_serial_char(static_cast<char>(SerialUSB.read()));
  }
  while (Serial2.available() > 0) {
    process_serial_char(static_cast<char>(Serial2.read()));
  }
  if (command_buffer.length() > 0 && millis() - last_char_time > CMD_TIMEOUT_MS) {
    command_buffer = "";
  }
}

void updateStatus() {
  filament_present = digitalRead(ENDSTOP_3) == 0;
  if (filament_present != last_filament_present) {
    sendMessage(filament_present ? "p1" : "p0");
    last_filament_present = filament_present;
    if (!filament_present) {
      disableMotor();
      continuous_run = false;
      move_active = false;
      sendMovement(MoveReport::O);
    }
  }
  sendMovement(currentMoveReport());
}

void buffer_debug() {
  // SerialUSB.print("buffer1_pos1_sensor_state:");
  // SerialUSB.println(buffer.buffer1_pos1_sensor_state);
  // SerialUSB.print("buffer1_pos2_sensor_state:");
  // SerialUSB.println(buffer.buffer1_pos2_sensor_state);
  // SerialUSB.print("buffer1_pos3_sensor_state:");
  // SerialUSB.println(buffer.buffer1_pos3_sensor_state);
  // SerialUSB.print("buffer1_material_swtich_state:");
  // SerialUSB.println(buffer.buffer1_material_swtich_state);
  // SerialUSB.print("key1:");
  // SerialUSB.println(buffer.key1);
  // SerialUSB.print("key2:");
  // SerialUSB.println(buffer.key2);
  static int idx = 0;
  if (idx < MAX_INDEX) {
    SerialUSB.print("i:");
    SerialUSB.println(idx);
    driver.GCONF(idx);
    driver.PWMCONF(idx);
    idx++;
  }
  const uint32_t gconf = driver.GCONF();
  const uint32_t chopconf = driver.CHOPCONF();
  const uint32_t pwmconf = driver.PWMCONF();
  if (driver.CRCerror) {
    SerialUSB.println("CRCerror");
  } else {
    SerialUSB.print("GCONF():0x");
    SerialUSB.println(gconf, HEX);
    SerialUSB.print("CHOPCONF():0x");
    String buf = String(chopconf, HEX);
    buf.toUpperCase();
    SerialUSB.println(buf);
    SerialUSB.print("PWMCONF():0x");
    buf = String(pwmconf, HEX);
    buf.toUpperCase();
    SerialUSB.println(buf);
    SerialUSB.println("");
  }
  delay(ONE_SECOND_MS);
}
