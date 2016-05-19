#include <math.h>

#include "mode_switch.h"

#include "FreeRTOS.h"
#include "task.h"
#include "config.h"
#include "motors.h"
#include "system.h"
#include "stabilizer.h"
#include "log.h"
#include "param.h"

bool isAgressive = false;
static bool isInit = false;
int32_t changeMode = 0;


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

    if(changeMode){
      while (!xSemaphoreTake(canUseStateGainMutex, portMAX_DELAY));
      isEco = !isEco;
      changeMode=0;
      xSemaphoreGive(canUseStateGainMutex);
    }


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

PARAM_GROUP_START(setMode)
PARAM_ADD(PARAM_INT32, changeMode, &changeMode)
PARAM_GROUP_STOP(setMode)
