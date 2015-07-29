#include "ch.h"
#include "hal.h"
#include "ext.h"
#include "radio.h"
#include "orchard-events.h"

event_source_t rf_pkt_rdy;
event_source_t gpiox_rdy;

event_source_t celcius_rdy;
event_source_t mic_rdy;
event_source_t usbdet_rdy;

event_source_t radio_page;
event_source_t radio_sex;
event_source_t radio_app;

#if ORCHARD_BOARD_REV != ORCHARD_REV_EVT1
static void gpiox_rdy_cb(EXTDriver *extp, expchannel_t channel) {

  (void)extp;
  (void)channel;

  chSysLockFromISR();
  chEvtBroadcastI(&gpiox_rdy);
  chSysUnlockFromISR();
}
#endif

static const EXTConfig ext_config = {
  {
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART, radioInterrupt, PORTC, 4},
#if ORCHARD_BOARD_REV != ORCHARD_REV_EVT1
    {EXT_CH_MODE_FALLING_EDGE | EXT_CH_MODE_AUTOSTART, gpiox_rdy_cb, PORTD, 4},
#endif
  }
};

void orchardEventsStart(void) {

  chEvtObjectInit(&rf_pkt_rdy);
  chEvtObjectInit(&gpiox_rdy);

  // ADC-related events
  chEvtObjectInit(&celcius_rdy);
  chEvtObjectInit(&mic_rdy);
  chEvtObjectInit(&usbdet_rdy);

  // radio protocol events
  chEvtObjectInit(&radio_page);
  chEvtObjectInit(&radio_sex);
  chEvtObjectInit(&radio_app);

  extStart(&EXTD1, &ext_config);
}
