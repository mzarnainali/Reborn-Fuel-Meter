#ifndef ESP32_TRIP_CONFIG_H_
#define ESP32_TRIP_CONFIG_H_

/*
 * Trip parameters
 */

// GPIO Definitions
#define RELAY_PIN 4
#define INJ_PIN GPIO_NUM_18
#define VSS1_PIN GPIO_NUM_27
#define CRANK_PIN GPIO_NUM_16
#define A_PIN 23
#define B_PIN 13
#define C_PIN 32
#define D_PIN 33
#define DAC_CH1 25
#define VSS_OUTPUT_PIN GPIO_NUM_5
#define VSS_RELAY_PIN 17
#define TEMP_SENSOR_PIN 14 // INPUT ONLY PIN 34
// 21 -  22 I2C NOT USED

// The maximum allowed time between opening and closing the injector
#define INJ_DELTA_MAX 200000 // 200ms
#define INJ_DELTA_MIN 8695   // 300ms
#define INJ_TIME_MAX 50000   // 50ms
// Amount of time for an injector to have injected 1 liter of fuel
#define INJ_USEC_LITER_D 72966250 ///   (81,075,000 => for 185cc/min * 4 injector)
// InstantLph = 3600000000.0f / INJ_USEC_LITER) * getDuty()
// INJ_USEC_LITER = 3600000000.0f  / InstantLph * getDuty()

// Amount of time to consider the engine to be stopped
#define INJ_TICKS_TIMEOUT 2000000 // 200ms   org=>1000000
// Amount of time to consider the vehicle to be stopped
#define VSS_TICKS_TIMEOUT 1000000 // 100ms
#define DIVIDER 8                 // Divider to reduce load

// The maximum allowed time between VSS pulses to calculate velocity
#define VSS_DELTA_MAX 100000 // 100ms

#define CRANK_DELTA_MIN 9000      // 9ms mean 6700 RPM
#define CRANK_DELTA_TEETH_MIN 750 // mean 6700 RPM

#define CRANK_DELTA_MAX 120000      // mean 500RPM
#define CRANK_TEETH_DELTA_MAX 10000 // mean 500RPM

// Amount of VSS pulses to have travelled 1 Km
#define VSS_PULSE_KM_D 115500.0f // (105000.0f => sync with meter) | (120000 => sync with GPS) | (115,500=> maybe sync with OBD2)
// Speed= 3600/VSS_PULSE_KM  * VSS_PLUS
// VSS_PULSE_KM = 3600 / Speed * VSS_PLUS

/*
120000 plus required for 1KM
7333 plus / s at speed of 220km/h
for 1km @ 220km/h speed it take 16.4seconds
232.3 plus per rotation of type with 615mm diameter
136 μs period between 2 pluses @ speed of 220km/h
30 ms period between 2 pluses @ speed of 1km/h
*/

#endif
