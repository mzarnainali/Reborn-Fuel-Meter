#ifndef TRIP_H_
#define TRIP_H_

#include <Arduino.h>
#include "driver/mcpwm.h"
#include "driver/timer.h"

/**
 * @brief Class for handling fuel and velocity calculations
 */
class Trip
{
public:
  void begin(void);

  uint_fast16_t getRpm(void);
  float getLiters(void);
  uint64_t getTotalInjectionMs(void);
  void resetLiters(void);
  float getKm(void);
  void resetKm(void);
  float getKmh(void);
  float getEfficiency();
  float getLph(void);
  float getInstantLkm();
  uint32_t getInjectionTime();
  float getVel();
  float getDuty(void);
  float getInjFreq();

  float INJ_USEC_LITER = 0.0f;
  float VSS_PULSE_KM = 0.0f;

private:
  static Trip *sTrip;

  volatile uint32_t lastInjStage = 0;
  volatile uint32_t latestInjectionTime = 0;
  volatile uint64_t injOpenTimestamp = 0;
  volatile uint32_t latestInjectionPeriod = 0;
  volatile uint32_t totalInjectionPulses = 0;
  volatile uint64_t totalInjectionTime = 0;

  volatile uint64_t latestVssTimestamp = 0;
  volatile uint32_t latestVssPeriod = 0;
  volatile uint32_t totalVssPulses = 0;

  volatile uint64_t lastCrankTimestamp = 0;
  volatile uint32_t lastCrankPeriod = 0;
  volatile uint32_t prevCrankPeriod = 0;
  volatile uint32_t currentCrankPulse = 0;
  volatile uint64_t lastCrankFinalTimestamp = 0;
  volatile uint32_t lastCrankFinalPeriod = 0;
  volatile bool isStopped = true;

  float previousSpeed = 0.0f;

  // float getVel();
  // float getVel2();

  // static IRAM_ATTR void updateTripInjISR(void *);
  // static IRAM_ATTR void updateTripVssISR(void *);
  // static IRAM_ATTR void updateTripCrankISR(void *);
  // void injChange();
  // void vssPulse();
  // void crankPulse();

  static IRAM_ATTR void tripInjISRHandler(void *);
  static IRAM_ATTR void tripVssISRHandler(void *);

  static IRAM_ATTR bool timeoutInjISR(void *);
  static IRAM_ATTR bool timeoutVssISR(void *);

  void injPulse(uint32_t, uint32_t);
  void vssPulse(uint32_t, uint32_t);
  void timeoutInj();
  void timeoutVss();

  const timer_config_t timeout_timer_conf = {
      .alarm_en = TIMER_ALARM_EN,
      .counter_en = TIMER_PAUSE,
      .intr_type = TIMER_INTR_LEVEL,
      .counter_dir = TIMER_COUNT_DOWN,
      .auto_reload = TIMER_AUTORELOAD_DIS,
      .divider = 8};
};

#endif // TRIP_H_
