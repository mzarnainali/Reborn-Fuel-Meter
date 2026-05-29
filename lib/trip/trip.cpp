// https://github.com/GAsplund/esp32-trip-meter/tree/main/src/fuel_economy_meter migrate 2 this

#include "trip.h"
#include <arduino.h>
#include "esp_timer.h"
#include "config.h"

#define FILTER_VSS 0

Trip *Trip::sTrip = 0;

/**
 * Starts the VSS interrupts and adds interrupts for the injector and VSS sensor
 */
void Trip::begin(void)
{
  sTrip = this;

  Serial.println("Setting up injector ISR");
  gpio_pad_select_gpio(INJ_PIN);
  gpio_set_direction(INJ_PIN, GPIO_MODE_INPUT);
  gpio_set_pull_mode(INJ_PIN, GPIO_PULLUP_ONLY);

  Serial.println("Setting up VSS ISR");
  gpio_pad_select_gpio(VSS1_PIN);
  gpio_set_direction(VSS1_PIN, GPIO_MODE_INPUT);
  gpio_set_pull_mode(VSS1_PIN, GPIO_PULLUP_ONLY);

  // Serial.println("Setting up CRANK ISR");
  // gpio_pad_select_gpio(CRANK_PIN);
  // gpio_set_direction(CRANK_PIN, GPIO_MODE_INPUT);
  // gpio_set_pull_mode(CRANK_PIN, GPIO_PULLUP_ONLY);

  Serial.println("Enabling ISR");
  gpio_install_isr_service(0);
  gpio_set_intr_type(INJ_PIN, GPIO_INTR_ANYEDGE);
  gpio_set_intr_type(VSS1_PIN, GPIO_INTR_POSEDGE);
  // gpio_set_intr_type(CRANK_PIN, GPIO_INTR_POSEDGE);

  gpio_isr_handler_add(INJ_PIN, Trip::updateTripInjISR, (void *)INJ_PIN);
  gpio_isr_handler_add(VSS1_PIN, Trip::updateTripVssISR, (void *)VSS1_PIN);
  // gpio_isr_handler_add(CRANK_PIN, Trip::updateTripCrankISR, (void *)CRANK_PIN);
}

/**
 * @brief Handles a pulse change in the injector signal
 *
 * Used to calculate total and momentary injection time.
 * Injector is considered open when the signal is low due to MOSFET pulling it down.
 *
 *     ╭―(+ Inj -)―╮
 *     +           o - pull-down signal
 *     -           / - MOSFET
 *     ╰―――――――――――╯
 */
void Trip::injChange()
{
  uint64_t changeTimestamp = esp_timer_get_time();
  if (gpio_get_level(INJ_PIN) == 1)
  // if ((REG_READ(GPIO_IN_REG) & INJ_PIN) == 1)
  {
    if (this->lastInjStage == 1)
      return;

    this->lastInjStage = 1;
    if (this->injOpenTimestamp > 0)
    {
      uint32_t delta = changeTimestamp - this->injOpenTimestamp;
      if (delta < INJ_DELTA_MAX && delta > INJ_DELTA_MIN)
      {
        this->latestInjectionPeriod = delta;
      }
    }
    this->injOpenTimestamp = changeTimestamp;
    this->totalInjectionPulses += 1;
  }
  else
  {
    if (this->lastInjStage == 0)
      return;
    this->lastInjStage = 0;
    uint32_t delta = changeTimestamp - this->injOpenTimestamp;
    // this->latestInjectionTimeDebug = delta;
    if (delta < INJ_TIME_MAX && (this->latestInjectionPeriod == 0 || delta <= this->latestInjectionPeriod))
    {
      this->latestInjectionTime = delta;
      this->totalInjectionTime += this->latestInjectionTime;
    }
  }
}

void IRAM_ATTR Trip::updateTripInjISR(void *)
{
  // if (sTrip != 0)
  sTrip->injChange();
}

/**
 * @brief Handles a pulse in the VSS signal
 *
 * Gets the timestamp of the pulse and calculates the period between the current and the last pulse.
 * If the period is not too long, it is stored as the latest period.
 *
 * Since the period is only used to calculate velocity, the delta is not needed when the period is
 * too long. (i.e. when the vehicle is stopped)
 */
void Trip::vssPulse()
{
  uint64_t pulseTimestamp = esp_timer_get_time();
  ++totalVssPulses;
  uint32_t delta = pulseTimestamp - latestVssTimestamp;
  latestVssTimestamp = pulseTimestamp;
  if (delta < VSS_DELTA_MAX && delta > VSS_DELTA_MIN)
  {
#if not FILTER_VSS
    latestVssPeriod = delta;
#endif

#if FILTER_VSS
    if (prevVssPeriod == 0 || vssRejectCount >= 5)
    {
      latestVssPeriod = prevVssPeriod = delta;
      vssRejectCount = 0;
    }
    else
    {
      float change = ((float)delta - prevVssPeriod) / prevVssPeriod;

      // Accept if change is within tolerance depending on direction
      bool accept = (change >= 0) ? (change <= 0.10f)   // +10% tolerance on increase
                                  : (change >= -0.25f); // -25% tolerance on decrease

      if (accept)
      {
        latestVssPeriod = prevVssPeriod = delta;
        vssRejectCount = 0;
      }
      else
      {
        ++vssRejectCount;
      }
    }
#endif
  }
}

void IRAM_ATTR Trip::updateTripVssISR(void *)
{
  // if (sTrip != 0)
  sTrip->vssPulse();
}

void Trip::crankPulse()
{
  uint64_t pulseTimestamp = esp_timer_get_time();
  this->currentCrankPulse += 1;
  this->prevCrankPeriod = this->lastCrankPeriod;
  uint32_t delta = pulseTimestamp - this->lastCrankTimestamp;
  if (delta < CRANK_TEETH_DELTA_MAX && delta > CRANK_DELTA_TEETH_MIN)
  {
    this->lastCrankPeriod = delta;
  }

  this->lastCrankTimestamp = pulseTimestamp;
  if (this->lastCrankPeriod < (this->prevCrankPeriod / 2) && this->currentCrankPulse > 12) // 3/10
  {
    this->currentCrankPulse = 0;
    uint32_t deltaFinal = pulseTimestamp - this->lastCrankFinalTimestamp;
    if (deltaFinal < CRANK_DELTA_MAX && deltaFinal > CRANK_DELTA_MIN)
    {
      this->lastCrankFinalPeriod = deltaFinal;
    }
    this->lastCrankFinalTimestamp = pulseTimestamp;
  }
}

void IRAM_ATTR Trip::updateTripCrankISR(void *)
{
  // if (sTrip != 0)
  sTrip->crankPulse();
}

/**
 * @brief Calculates current engine RPM
 *
 * Uses injector pulse period to calculate the rotation speed of the engine in RPM
 *
 * Given that one single injector fires per engine rotation, the RPM is calculated
 * by dividing 60 seconds by the injector pulse period.
 *
 * Since f = 1/T, the formula is 60/T
 * This implicitly converts from rotations per second (Hz) to rotations per minute.
 * Since the injector pulse period is in microseconds, we multiply by 1e6 to get seconds.
 *
 * @return the current engine RPM
 */
uint_fast16_t Trip::getRpm(void) { return (this->latestInjectionPeriod > 0 && esp_timer_get_time() - this->injOpenTimestamp < INJ_DELTA_MAX) ? 60000000 / this->latestInjectionPeriod * 2 : 0; }

uint_fast16_t Trip::getRpmCrank(void) { return (this->lastCrankFinalPeriod > 0 && esp_timer_get_time() - this->lastCrankFinalTimestamp < CRANK_DELTA_MAX) ? 60000000 / this->lastCrankFinalPeriod : 0; }
/**
 * Calculates the total fuel spent
 */
// float Trip::getLiters(void) { return this->totalInjectionTime * (1 / INJ_USEC_LITER); }
float Trip::getLiters(void) { return this->totalInjectionTime / INJ_USEC_LITER; }

uint64_t Trip::getTotalInjectionMs(void) { return this->totalInjectionTime; }

void Trip::resetLiters(void)
{
  this->totalInjectionTime = 0;
}

/**
 * @brief Calculates the duty cycle on the injector
 */
float Trip::getDuty(void) { return (this->latestInjectionPeriod > 0 && esp_timer_get_time() - this->injOpenTimestamp < INJ_DELTA_MAX) ? (float)this->latestInjectionTime / (float)this->latestInjectionPeriod : 0; }

/**
 * @brief Calculates momentary fuel consumption
 */
float Trip::getLph(void) { return (3600000000.0f / INJ_USEC_LITER) * getDuty(); }

/**
 * Calculates the total distance traveled in km
 */
float Trip::getKm(void)
{
  return this->totalVssPulses * (1 / VSS_PULSE_KM);
  // 120000 pluses for 1KM
  // return this->totalVssPulses * 0.00000833333;
}

void Trip::resetKm(void) { this->totalVssPulses = 0; }

/**
 * Calculates the velocity in km/h
 */
float Trip::getKmh(void)
{
  return (3600.0f / VSS_PULSE_KM) * this->getVel();
  // return 0.03 * this->getVel();
  //  if (esp_timer_get_time() - this->latestVssTimestamp > VSS_DELTA_MAX)
  //{
  //    return 0;
  // }
  //  else
  //  {
  //  float speed = /*(3600 / VSS_PULSE_KM)*/ 0.03 * this->getVel();
  //  return speed;
  //  if (abs(speed - previousSpeed) < 0.5)
  //  {
  //   previousSpeed = speed;
  // }
  //  return previousSpeed;
  //  }
}

/**
 * @brief Calculates the momentary fuel efficiency
 *
 * Calculates the fuel efficiency based on the velocity and fuel consumption rate.
 * Efficiency is calculated per km, which is multiplied by 100 to get a proper efficiency value.
 *
 * @return the momentary efficiency in L/100km
 */
float Trip::getEfficiency()
{
  float kmh = this->getKmh();
  return (kmh > 0) ? (this->getLph() * 100) / kmh : 0;
}

float Trip::getInstantLkm()
{
  float kmh = this->getKmh();
  float lph = this->getLph();
  return (kmh > 0 && lph > 0) ? (kmh / lph) : 0;
}

/**
 * @brief Gets the amount of VSS pulses per second
 *
 * Calculated using f = 1/T, where T is the latest VSS pulse period.
 * We also multiply by 1e6 to convert from seconds to microseconds.
 *
 * @return the amount of VSS pulses per second based on the latest period
 */
float Trip::getVel() { return (this->latestVssPeriod > 0 && esp_timer_get_time() - this->latestVssTimestamp < VSS_DELTA_MAX) ? 1000000.0f / this->latestVssPeriod : 0; }

uint32_t Trip::getInjectionTime()
{
  return (this->latestInjectionTime > 0 && esp_timer_get_time() - this->injOpenTimestamp < INJ_DELTA_MAX) ? this->latestInjectionTime : 0;
}

float Trip::getInjFreq() { return (this->latestInjectionPeriod > 0 && esp_timer_get_time() - this->injOpenTimestamp < INJ_DELTA_MAX) ? 1000000.0f / this->latestInjectionPeriod : 0; }
