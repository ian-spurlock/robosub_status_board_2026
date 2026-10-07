// Status Board for XIAO-ESP32-S3
// Michigan Robotic Submarine - Fall 2026

#include <Wire.h>
#include "MS5837.h"
#include "TM1637Display.h"

/* Time Constants */
// Since reading from the sensor takes up to 40ms,
// make sure this is at least 40.
constexpr unsigned long MEASUREMENT_DELAY_MS = 50;

/* Fluid Density */
// Units are in kg/m^3
const int BAUD_RATE = 9600;
const int DENSITY_FRESHWATER  = 997;

/* Hall Effect Digital Pins */
const int HALL_EFFECT_CHARM   = 2;
const int HALL_EFFECT_STRANGE = 3;

/* Depth Sensor */
// Documentation: https://github.com/bluerobotics/BlueRobotics_MS5837_Library
// SDA: GPIO 5, SCL: GPIO 6
MS5837 depth_sensor;

/* Voltage Readout */
// Documentation: https://github.com/avishorp/TM1637/tree/master 
// DIO: GPIO 10, CLK: GPIO 11
const int VOLTAGE_READOUT_DIO = 10;
const int VOLTAGE_READOUT_CLK = 11;
const int VOLTAGE_READOUT_SAMPLE_SIZE = 20;
const int VOLTAGE_READOUT_SAMPLE_DELAY = 2; // As multiple of measurement delay
const int VOLTAGE_SAMPLE_COUNT_THRESHOLD = VOLTAGE_READOUT_SAMPLE_SIZE * VOLTAGE_READOUT_SAMPLE_DELAY;
TM1637Display voltageReadout(CLK, DIO);

/* Voltage Divider */
const int VOLTAGE_DIVIDER_OUTPUT = 4;

/* Voltage Variables */
double averageVoltage = 0.0;
double voltageSum = 0.0;
int voltageSampleCount = 0;

//////////////////////// DEPTH SENSOR ////////////////////////

void depthSensorInit() {
  //Wait for Depth Sensor to initialize
  Wire.begin();
  while (!depth_sensor.init()) {
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }
  depth_sensor.setModel(MS5837::MS5837_30BA);
  depth_sensor.setFluidDensity(DENSITY_FRESHWATER);
}

long getDepth() {
  depth_sensor.read();
  float rawDepth = depth_sensor.depth();

  //Recasting depth data to a long because floats can't be bit shifted
  float *f_ptr = &rawDepth;
  return *reinterpret_cast<long*>(f_ptr);
}

//////////////////////// VOLTAGE READOUT ////////////////////////

/* Returns the measured voltage value between the probes */
double getVoltageReading() {
  double voltageDividerOutput = analogReadMilliVolts(VOLTAGE_DIVIDER_OUTPUT) / 1000.0;
  return voltageDividerOutput * 5.545454; // Will need to be calibrated
}

/* Updates and displays average voltage after enough samples have been collected */
void updateVoltage() {
  voltageSampleCount++;
  // Sample in accordance with the voltage sample rate instead of the default sensor measurement rate
  if (voltageSampleCount % VOLTAGE_READOUT_SAMPLE_DELAY == 0) voltageSum += getVoltageReading();
  if (voltageSampleCount != VOLTAGE_SAMPLE_COUNT_THRESHOLD) return;
  // TODO: Ask if the exact sample rate matters this much

  // Reset sampling and display
  averageVoltage = voltageSum / VOLTAGE_READOUT_SAMPLE_SIZE;
  voltageSampleCount = 0;
  voltageSum = 0;
  voltageReadout.showNumberDecEx(round(averageVoltage * 100), 0b01000000, true);
}

//////////////////////// SERIAL COMMUNICATION ////////////////////////

/* Send sensor data to Jetson over USB */
void outputToSerial(int hallEffectCharm, int hallEffectStrange, long depth) {
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
}

//////////////////////// CORE FUNCTIONALITY ////////////////////////

void setup() {
  Serial.begin(BAUD_RATE);

  pinMode(HALL_EFFECT_CHARM,   INPUT_PULLUP);
  pinMode(HALL_EFFECT_STRANGE, INPUT_PULLUP);

  voltageReadout.setBrightness(7);
  voltageReadout.clear();

  depthSensorInit();
}

void loop() {
  //TODO: Get status from Serial and set LEDs accordingly

  updateVoltage();

  outputToSerial(
    digitalRead(HALL_EFFECT_CHARM),
    digitalRead(HALL_EFFECT_STRANGE),
    getDepth()
  );

  delay(MEASUREMENT_DELAY_MS);
}
