// Status Board for XIAO-ESP32-S3
// Ian Spurlock, Michigan Robotic Submarine - Fall 2026

#include <Wire.h>
#include "MS5837.h"
#include "TM1637Display.h"

constexpr int USER_LED = 21; // For testing

/* Time Constants */
// Since reading from the sensor takes up to 40ms,
// make sure this is at least 40.
constexpr unsigned long CYCLE_DELAY_MS = 50;

/* Serial Constants */
constexpr int BAUD_RATE = 9600;
constexpr uint8_t MESSAGE_HEADER[4] = { 0xFF, 0xFF, 0xFF, 0xFF };

/* Hall Effect Digital Pins */
// Note: These sensors are interchangeable:
constexpr int HALL_EFFECT_CHARM   = 2;
constexpr int HALL_EFFECT_STRANGE = 3;

/* Depth Sensor */
// Documentation: https://github.com/bluerobotics/BlueRobotics_MS5837_Library
// SDA: GPIO 5, SCL: GPIO 6
constexpr int DENSITY_FRESHWATER_KG_M3 = 997;
MS5837 depth_sensor;
float depth = 0;

/* Status LEDs */
// IDLE->[][]:'i', ESTOP->[][R]:'e', RUNNING->[G][]:'r', SENSOR_RESET->[G][R]:'s'
constexpr int STOPPED_LED = 7;
constexpr int ENABLED_LED = 44;

/* Voltage Readout */
// Documentation: https://github.com/avishorp/TM1637/tree/master 
// CLK: GPIO 8, DIO: GPIO 9
constexpr int VOLTAGE_READOUT_CLK = 8;
constexpr int VOLTAGE_READOUT_DIO = 9;
constexpr int VOLTAGE_READOUT_SAMPLE_SIZE = 20;
TM1637Display voltageReadout(VOLTAGE_READOUT_CLK, VOLTAGE_READOUT_DIO);

/* Voltage Divider */
constexpr int VOLTAGE_DIVIDER_INPUT = 4;
constexpr double VOLTAGE_READING_CONVERSION_MULTIPLIER = 5.63; // Might want to do more calibration

/* Voltage Variables (mV) */
int averageVoltage = 0.0;
double voltageSum = 0.0;
int voltageSampleCount = 0;

//////////////////////// CORE FUNCTIONALITY ////////////////////////

void setup() {
  Serial.begin(BAUD_RATE);

  //depthSensorInit(); // Commented for testing

  pinMode(HALL_EFFECT_CHARM,   INPUT_PULLUP);
  pinMode(HALL_EFFECT_STRANGE, INPUT_PULLUP);

  voltageReadout.setBrightness(7);
  voltageReadout.clear();

  pinMode(STOPPED_LED, OUTPUT);
  pinMode(ENABLED_LED, OUTPUT);

  pinMode(USER_LED, OUTPUT);
  digitalWrite(USER_LED, LOW); // Active-low (enabled for testing)
}

void loop() {
  updateVoltage();

  //updateDepth(); // Commented for testing
  if (serialInputAvailable()) updateStatusLEDs(); // Commented for testing

  digitalWrite(USER_LED, digitalRead(HALL_EFFECT_CHARM)); // For testing

  //outputToSerial(); // Commented for testing

  delay(CYCLE_DELAY_MS);
}

//////////////////////// DEPTH SENSOR ////////////////////////

/* Resets the depth sensor until it successfully initializes */
void depthSensorInit() {
  Wire.begin();
  while (!depth_sensor.init()) delay(3000);

  /* Automatic model detection in case the built in detection doesn't work:
  // Automatically set the model:
  depth_sensor.setModel(MS5837::MS5837_02BA);
  updateDepth();
  if (depth >= 2.4) depth_sensor.setModel(MS5837::MS5837_30BA); // The sensor must be placed 0.15m < depth < 2.40m for proper calibration
  */
  
  depth_sensor.setFluidDensity(DENSITY_FRESHWATER_KG_M3);
}

/* Tells the depth sensor to read new value and returns said value */
void updateDepth() {
  depth_sensor.read();
  depth = depth_sensor.depth();
}

//////////////////////// VOLTAGE READOUT ////////////////////////

/* Updates and displays average voltage after enough samples have been collected */
void updateVoltage() {
  voltageSum += analogReadMilliVolts(VOLTAGE_DIVIDER_INPUT) * VOLTAGE_READING_CONVERSION_MULTIPLIER;
  voltageSampleCount++;
  if (voltageSampleCount != VOLTAGE_READOUT_SAMPLE_SIZE) return;

  averageVoltage = voltageSum / VOLTAGE_READOUT_SAMPLE_SIZE;
  voltageSum = 0;
  voltageSampleCount = 0;
  voltageReadout.showNumberDecEx(averageVoltage / 10, 0b01000000, true);
}

//////////////////////// STATUS LEDS ////////////////////////

/* Reads Serial input and updates LEDs accordingly */
void updateStatusLEDs() {
  char status = inputFromSerial();
  digitalWrite(STOPPED_LED, status == 's' || status == 'e'); // Stopped if sensor-resetting or e-stopped.
  digitalWrite(ENABLED_LED, status == 's' || status == 'r'); // Enabled if sensor-resetting or running.
}

//////////////////////// SERIAL COMMUNICATION ////////////////////////

/* Check if data is available on Serial */
bool serialInputAvailable() {
  return Serial.available() != 0;
}

/* Receive data from Jetson over USB */
char inputFromSerial() {
  char lastReceived = '\0';
  while (serialInputAvailable()) lastReceived = Serial.read();
  return lastReceived; // TODO: Ensure software accounts for this
}

/* Send sensor data to Jetson over USB */
void outputToSerial() {
  Serial.write(MESSAGE_HEADER, 4);
  Serial.write((uint8_t)digitalRead(HALL_EFFECT_CHARM));
  Serial.write((uint8_t)digitalRead(HALL_EFFECT_STRANGE));
  Serial.write((uint8_t*)&depth, 4); // "(uint8_t*)&depth" treats the depth value as an array of bytes
  Serial.write((uint8_t*)&averageVoltage, 2); // TODO: Ensure software accounts for this
}
