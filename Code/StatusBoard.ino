// Status Board for XIAO-ESP32-S3
// Ian Spurlock, Michigan Robotic Submarine - Fall 2026

#include <Wire.h>
#include "MS5837.h"
#include "TM1637Display.h"

/* Time Constants */
// Since reading from the sensor takes up to 40ms,
// make sure this is at least 40.
constexpr unsigned long CYCLE_DELAY_MS = 50;

/* Serial Constants */
constexpr int BAUD_RATE = 9600;
constexpr u_int8_t MESSAGE_HEADER[4] = { 0xFF, 0xFF, 0xFF, 0xFF };

/* Hall Effect Digital Pins */
constexpr int HALL_EFFECT_CHARM   = 2;
constexpr int HALL_EFFECT_STRANGE = 3;

/* Depth Sensor */
// Documentation: https://github.com/bluerobotics/BlueRobotics_MS5837_Library
// SDA: GPIO 5, SCL: GPIO 6
constexpr int DENSITY_FRESHWATER_KG_M3 = 997;
const uint8_t DEPTH_SENSOR_MODEL = MS5837::MS5837_30BA; // TODO: Check if this should be _02BA
MS5837 depth_sensor;
float depth;

/* Status LEDs */
// IDLE->[][]:'i', ESTOP->[][R]:'e', RUNNING->[G][]:'r', SENSOR_RESET->[G][R]:'s'
constexpr int STOPPED_LED = 7;
constexpr int ENABLED_LED = 44;

/* Voltage Readout */
// Documentation: https://github.com/avishorp/TM1637/tree/master 
// DIO: GPIO 10, CLK: GPIO 11
constexpr int VOLTAGE_READOUT_DIO = 10;
constexpr int VOLTAGE_READOUT_CLK = 11;
constexpr int VOLTAGE_READOUT_SAMPLE_SIZE = 20;
TM1637Display voltageReadout(VOLTAGE_READOUT_CLK, VOLTAGE_READOUT_DIO);

/* Voltage Divider */
constexpr int VOLTAGE_DIVIDER_INPUT = 4;

/* Voltage Variables (mV) */
double averageVoltage = 0.0;
double voltageSum = 0.0;
int voltageSampleCount = 0;

//////////////////////// CORE FUNCTIONALITY ////////////////////////

void setup() {
  Serial.begin(BAUD_RATE);
  depthSensorInit();

  pinMode(HALL_EFFECT_CHARM,   INPUT_PULLUP);
  pinMode(HALL_EFFECT_STRANGE, INPUT_PULLUP);

  voltageReadout.setBrightness(7);
  voltageReadout.clear();

  pinMode(STOPPED_LED, OUTPUT);
  pinMode(ENABLED_LED, OUTPUT);
}

void loop() {
  updateVoltage();
  updateDepth();
  if (serialInputAvailable()) updateStatusLEDs();

  outputToSerial();
  delay(CYCLE_DELAY_MS);
}

//////////////////////// DEPTH SENSOR ////////////////////////

/* Resets the depth sensor until it successfully initializes */
void depthSensorInit() {
  Wire.begin();
  while (!depth_sensor.init()) delay(3000);
  depth_sensor.setModel(DEPTH_SENSOR_MODEL);
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
  voltageSum += analogReadMilliVolts(VOLTAGE_DIVIDER_INPUT) * 5.5814; // To be calibrated (theoretical: 5.5454545)
  voltageSampleCount++;
  if (voltageSampleCount != VOLTAGE_READOUT_SAMPLE_SIZE) return;

  averageVoltage = voltageSum / VOLTAGE_READOUT_SAMPLE_SIZE;
  voltageSum = 0;
  voltageSampleCount = 0;
  voltageReadout.showNumberDecEx(round(averageVoltage * 0.1), 0b01000000, true);
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
  while (Serial.available() > 1) Serial.read(); // Ensure serial input data doesn't build up
  return Serial.read(); // TODO: Ensure software accounts for this
}

/* Send sensor data to Jetson over USB */
void outputToSerial() {
  Serial.write(MESSAGE_HEADER, 4);
  Serial.write(digitalRead(HALL_EFFECT_CHARM));
  Serial.write(digitalRead(HALL_EFFECT_STRANGE));
  Serial.write((u_int8_t*)&depth, sizeof(depth)); // "(u_int8_t*)&depth" treats the depth value as an array of bytes
  Serial.write(static_cast<int>(averageVoltage)); // TODO: Ensure software accounts for this
}
