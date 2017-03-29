#ifndef FRAMEWORK_LED_H
#define FRAMEWORK_LED_H

typedef enum{off, on, blink} LedStatus_t;
extern LedStatus_t ledGStatus, ledRStatus;

void ledGTask(void const * argument);
void ledRTask(void const * argument);

#endif
