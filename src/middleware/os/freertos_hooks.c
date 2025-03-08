#include "FreeRTOS.h"
#include "task.h"

void vApplicationMallocFailedHook(void) {
    /* Called if a call to pvPortMalloc() fails because there is insufficient
    free memory available in the FreeRTOS heap. */
    for(;;);
}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) {
    /* Called when a stack overflow is detected. */
    (void) pxTask;
    (void) pcTaskName;
    for(;;);
}

void vApplicationIdleHook(void) {
    /* Called when the idle task is running */
}

void vApplicationTickHook(void) {
    /* Called during each tick interrupt */
}