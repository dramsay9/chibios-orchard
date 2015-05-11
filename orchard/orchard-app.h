#ifndef __ORCHARD_APP_H__
#define __ORCHARD_APP_H__

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "shell.h"
#include "orchard.h"
#include "gfx.h"

struct _OrchardApp;
typedef struct _OrchardApp OrchardApp;

/* Emitted to an app when it's about to be terminated */
extern event_source_t orchard_app_terminate;

/* Emitted to the system after the app has terminated */
extern event_source_t orchard_app_terminated;

void orchardAppInit(void);
void orchardAppRestart(void);
void orchardAppWatchdog(void);
void orchardAppRun(const OrchardApp *app);

typedef struct _OrchardAppContext {
  uint32_t keymask;
  bool should_exit;
  uint32_t priv_size;
  void *priv;
} OrchardAppContext;

typedef enum _OrchardAppEventType {
  keyEvent,
  appEvent,
} OrchardAppEventType;

/* ------- */

typedef enum _OrchardAppEventKeyFlag {
  keyUp = 0,
  keyDown = 1,
} OrchardAppEventKeyFlag;

typedef struct _OrchardAppKeyEvent {
  uint8_t   code;
  uint8_t   flags;
} OrchardAppKeyEvent;

/* ------- */

typedef enum _OrchardAppLifeEventFlag {
  appStart,
  appTerminate,
} OrchardAppLifeEventFlag;

typedef struct _OrchardAppLifeEvent {
  uint8_t   event;
} OrchardAppLifeEvent;

/* ------- */

typedef struct _OrchardAppEvent {
  OrchardAppEventType   type;
  union {
    OrchardAppKeyEvent  key;
    OrchardAppLifeEvent app;
  };
} OrchardAppEvent;

typedef struct _OrchardApp {
  char *name;
  uint32_t (*init)(OrchardAppContext *context);
  void (*start)(OrchardAppContext *context);
  void (*event)(OrchardAppContext *context, OrchardAppEvent *event);
  void (*exit)(OrchardAppContext *context);
} OrchardApp;

#define orchard_app_start()                                                   \
({                                                                            \
  static char start[0] __attribute__((unused,                                 \
    aligned(4), section(".chibi_list_app_1")));                               \
  (const OrchardApp *)&start;                                                 \
})

#define orchard_app(_name, _init, _start, _event, _exit)                        \
  const OrchardApp _orchard_app_list_##_init##_start##_event##_exit           \
  __attribute__((unused, aligned(4), section(".chibi_list_app_2_" # _init # _start # _event # _exit))) =  \
     { _name, _init, _start, _event, _exit }

#define orchard_app_end()                                                     \
  const OrchardApp _orchard_app_list_final                                    \
  __attribute__((unused, aligned(4), section(".chibi_list_app_3_end"))) =     \
     { NULL, NULL, NULL, NULL, NULL }

#endif /* __ORCHARD_APP_H__ */
