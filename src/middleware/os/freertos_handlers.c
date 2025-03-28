/* Implementation of SVC and PendSV handlers that call FreeRTOS functions */
#include "FreeRTOS.h"

/* These are the actual FreeRTOS handler implementations */
extern void
vPortSVCHandler(void);
extern void
xPortPendSVHandler(void);

/* These are the names your processor expects */
void
SVC_Handler(void)
{
  vPortSVCHandler();
}

void
PendSV_Handler(void)
{
  xPortPendSVHandler();
}