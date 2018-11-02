/**
 * Battery Monitor (Firmware-Side)
 * This uses the microcontroller for its ADC capabilities
 * The input is read and voltage is estimated with calibration
 * This value is published to "/voltage_monitor/voltage" using rosserial
 *
 * @author Jeremy Mallette
 */

/**
 * Dependencies ---------------------------------------------------------------
 */
#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Float64.h>

#include "calib.h"

/**
 * Definitions ----------------------------------------------------------------
 */
#define V_METER A1          // The pin connected to the volt meter
#define MEAN_RANGE 500      // The size of the analog reading dataset
#define RES_RATIO 0.2       // The voltage divider resistor ratio

/**
 * Global Variables
 */
double ref = 0;             // Vcc calc << back-eval from 1.1V internal ref
std_msgs::Float64 voltage;  // Voltage to be published on the ros network

/**
 * Ros Setup ------------------------------------------------------------------
 */
ros::NodeHandle nh;
ros::Publisher feed("/voltage_monitor/voltage", &voltage);

/**
 * Applies the calibration ----------------------------------------------------
 */
float apply_calib(float input)
{
  return ((input - BIAS) / SCALE);
}

/**
 * Compare Vcc against Internal 1.1 volts to get accurate Vcc Reference -------
 */
long read_vcc()
{
  long result;

  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);

  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA,ADSC));

  result = ADCL;
  result |= ADCH<<8;

  result = 1125300L / result;
  //result = 1125200L / result;

  return result;
}

/**
 * Setup ----------------------------------------------------------------------
 */
void setup()
{
  nh.initNode();
  nh.advertise(feed);
}

/**
 * Loop -----------------------------------------------------------------------
 */
void loop()
{
  float reading = -1.0;
  float theoretical = -1.0;

  // Back-evaluate the reference voltage
  ref = read_vcc() / 1000.0;

  // Sum a dataset of analog readings
  double sample_sum = 0;
  for (int i = 0; i < MEAN_RANGE; i++)
  {
    sample_sum += analogRead(V_METER);
    delay(1);
  }

  // Reading is the mean of the collected data set and estimating
  reading = sample_sum / MEAN_RANGE;
  theoretical = ((reading / 1024.0) * ref) / RES_RATIO;

  // Apply the calibration linear transform
  voltage.data = apply_calib(theoretical);

  feed.publish(&voltage);
  nh.spinOnce();

  delay(500);
}
