#include <math.h>

#include "mode_switch.h"

#include "FreeRTOS.h"
#include "task.h"
#include "config.h"
#include "motors.h"
#include "system.h"
#include "stabilizer.h"

bool isAgressive = false;
static bool isInit = false;

static void modeSwitchTask(void* param)
{
  uint32_t lastWakeTime;

  // sets the id number in TASK_MODE_SWITCH_ID_NBR inside FreeRTOSConfig.h
  //(included with FreeRTOS.h) to this task (0 denotes this)
  vTaskSetApplicationTaskTag(0, (void*)TASK_MODE_SWITCH_ID_NBR);

  lastWakeTime = xTaskGetTickCount();

  while (1) {
    // some kind of event listening
    // this makes it run with a frequency
    // F2T comes from FreeRTOSConfig.h
    // TODO: find out how it should be triggered
    vTaskDelayUntil(&lastWakeTime, F2T(1)); // 1Hz
    // other possibility
    // vTaskSuspend() will be invoked again by vTaskResume()

    // actual code for task
    while (!xSemaphoreTake(canUseStateGainMutex, portMAX_DELAY));
      // TODO switch isAggressive on and off dependent on user inputs
    xSemaphoreGive(canUseStateGainMutex);
  }
}

void modeSwitchInit(void)
{
  if(isInit)
    return;

    // finals are decleared in config.h
    xTaskCreate(modeSwitchTask, MODE_SWITCH_TASK_NAME,
                MODE_SWITCH_TASK_STACKSIZE, NULL, MODE_SWITCH_TASK_PRI, NULL);

    isInit = true;
    isAgressive = false;
}

bool modeSwitchTest(void)
{
  bool pass = true;
  // do tests

  return pass;
}
