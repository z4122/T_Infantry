#ifndef DRIVERS_LED_USER_H
#define DRIVERS_LED_USER_H

typedef enum{off, on, blink} LedStatus_t;
extern LedStatus_t ledGreenStatus, ledRedStatus;

#endif
