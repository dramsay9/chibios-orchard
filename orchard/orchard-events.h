#ifndef __ORCHARD_EVENTS__
#define __ORCHARD_EVENTS__

/* Orchard event wrappers.
   These simplify the ChibiOS eventing system.  To use, initialize the event
   table, hook an event, and then dispatch events.

  static void shell_termination_handler(eventid_t id) {
    chprintf(stream, "Shell exited.  Received id %d\r\n", id);
  }

  void main(int argc, char **argv) {
    struct evt_table events;

    // Support up to 8 events
    evtTableInit(events, 8);

    // Call shell_termination_handler() when 'shell_terminated' is emitted
    evtTableHook(events, shell_terminated, shell_termination_handler);

    // Dispatch all events
    while (TRUE)
      chEvtDispatch(evtHandlers(events), chEvtWaitOne(ALL_EVENTS));
   }
 */

struct evt_table {
  int size;
  int next;
  evhandler_t *handlers;
  event_listener_t *listeners;
};

#define evtTableInit(table, capacity)                                       \
  do {                                                                      \
    static evhandler_t handlers[capacity];                                  \
    static event_listener_t listeners[capacity];                            \
    table.size = capacity;                                                  \
    table.next = 0;                                                         \
    table.handlers = handlers;                                              \
    table.listeners = listeners;                                            \
  } while(0)

#define evtTableHook(table, event, callback)                                \
  do {                                                                      \
    if (CH_DBG_ENABLE_ASSERTS != FALSE)                                     \
      if (table.next >= table.size)                                         \
        chSysHalt("event table overflow");                                  \
    chEvtRegister(&event, &table.listeners[table.next], table.next);        \
    table.handlers[table.next] = callback;                                  \
    table.next++;                                                           \
  } while(0)

#define evtHandlers(table)                                                  \
    table.handlers

#define evtListeners(table)                                                 \
    table.listeners

extern event_source_t ble_rdy;
extern event_source_t rf_pkt_rdy;

void orchardEventsStart(void);

#endif /* __ORCHARD_EVENTS__ */