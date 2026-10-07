// Status Board for XIAO-ESP32-S3
// Michigan Robotic Submarine - Fall 2026

#include <Wire.h>
#include <cstddef>
#include "MS5837.h"
#include "TM1637Display.h"

/* Time Constants */
// Since reading from the sensor takes up to 40ms,
// make sure this is at least 40.
constexpr unsigned long MEASUREMENT_DELAY_MS = 50;
const int BAUD_RATE = 9600;

/* Fluid Density */
const int DENSITY_FRESHWATER_KG_M3  = 997;

/* Hall Effect Digital Pins */
const int HALL_EFFECT_CHARM   = 2;
const int HALL_EFFECT_STRANGE = 3;

/* Depth Sensor */
// Documentation: https://github.com/bluerobotics/BlueRobotics_MS5837_Library
// SDA: GPIO 5, SCL: GPIO 6
MS5837 depth_sensor;

/* Status LEDs */
// IDLE->[][]:'i', ESTOP->[][R]:'e', RUNNING->[G][]:'r', SENSOR_RESET->[G][R]:'s'
const int STOPPED_LED = 7;
const int ENABLED_LED = 44;

/* Voltage Readout */
// Documentation: https://github.com/avishorp/TM1637/tree/master 
// DIO: GPIO 10, CLK: GPIO 11
const int VOLTAGE_READOUT_DIO = 10;
const int VOLTAGE_READOUT_CLK = 11;
const int VOLTAGE_READOUT_SAMPLE_SIZE = 20;
TM1637Display voltageReadout(CLK, DIO);

/* Voltage Divider */
const int VOLTAGE_DIVIDER_INPUT = 4;

/* Voltage Variables (All in mV) */
double averageVoltage = 0.0;
double voltageSum = 0.0;
int voltageSampleCount = 0;

//////////////////////// DEPTH SENSOR ////////////////////////

void depthSensorInit() {
  Wire.begin();
  while (!depth_sensor.init()) {
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }
  depth_sensor.setModel(MS5837::MS5837_30BA);
  depth_sensor.setFluidDensity(DENSITY_FRESHWATER_KG_M3);
}

long getDepth() {
  depth_sensor.read();
  float rawDepth = depth_sensor.depth();

  //Recasting data to a long because floats can't be bit shifted
  float *f_ptr = &rawDepth;
  return *reinterpret_cast<long*>(f_ptr);
}

//////////////////////// VOLTAGE READOUT ////////////////////////

/* Returns the measured voltage value between the probes */
double getVoltageReading() {
  return analogReadMilliVolts(VOLTAGE_DIVIDER_INPUT) * 5.5814; // Will need to be calibrated (theoretical: 5.5454545)
}

/* Updates and displays average voltage after enough samples have been collected */
void updateVoltage() {
  voltageSum += getVoltageReading();
  voltageSampleCount++;
  if (voltageSampleCount != VOLTAGE_READOUT_SAMPLE_SIZE) return;

  averageVoltage = voltageSum / VOLTAGE_READOUT_SAMPLE_SIZE;
  voltageSum = 0;
  voltageSampleCount = 0;
  voltageReadout.showNumberDecEx(round(averageVoltage * 0.1), 0b01000000, true);
}

//////////////////////// STATUS LEDS ////////////////////////

void updateStatusLEDs() {
  char status = inputFromSerial();
  setStatus(
    status == 's' || status == 'e', // Stopped if sensor-resetting or e-stopped.
    status == 's' || status == 'r'  // Enabled if sensor-resetting or running.
  );
}

void setStatus(bool isStopped, bool isEnabled) {
  digitalWrite(STOPPED_LED, isStopped ? HIGH : LOW);
  digitalWrite(ENABLED_LED, isEnabled ? HIGH : LOW);
}

//////////////////////// SERIAL COMMUNICATION ////////////////////////

/* Check if data is available on Serial */
bool serialInputAvailable() {
  return Serial.available() != 0;
}

/* Receive data from Jetson over USB */
char inputFromSerial() {
  return Serial.read(); // TODO: Ensure software accounts for this
}

/* Send sensor data to Jetson over USB */
void outputToSerial(int hallEffectCharm, int hallEffectStrange, long depth, int voltageMillivolts) {
  //Write to serial a message header that the Jetson uses to confirm the start of a message
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);

  //Begin writing data to serial one byte at a time
  Serial.write(hallEffectStrange); // Charm and Strange are at beginning and end to prevent header check on Jetson from looking at depth
  Serial.write(depth & 0xFF);
  Serial.write((depth >>  8) & 0xFF);
  Serial.write((depth >>  16) & 0xFF);
  Serial.write((depth >>  24) & 0xFF);
  Serial.write(hallEffectCharm);
  Serial.write(voltageMillivolts); // TODO: Ensure software accounts for this
}

//////////////////////// CORE FUNCTIONALITY ////////////////////////

void setup() {
  Serial.begin(BAUD_RATE);

  pinMode(HALL_EFFECT_CHARM,   INPUT_PULLUP);
  pinMode(HALL_EFFECT_STRANGE, INPUT_PULLUP);

  voltageReadout.setBrightness(7);
  voltageReadout.clear();

  pinMode(STOPPED_LED, OUTPUT);
  pinMode(ENABLED_LED, OUTPUT);

  depthSensorInit();
}

void loop() {
  updateVoltage();

  if (serialInputAvailable()) updateStatusLEDs();

  outputToSerial(
    digitalRead(HALL_EFFECT_CHARM),
    digitalRead(HALL_EFFECT_STRANGE),
    getDepth(),
    averageVoltage
  );

  delay(MEASUREMENT_DELAY_MS);
}
