#include <math.h>

#include "mode_switch.h"

#include "FreeRTOS.h"
#include "task.h"
#include "config.h"
#include "motors.h"

uint32_t motorPowerM1;  // Motor 1 power output (16bit value used: 0 - 65535)
uint32_t motorPowerM2;  // Motor 2 power output (16bit value used: 0 - 65535)
uint32_t motorPowerM3;  // Motor 3 power output (16bit value used: 0 - 65535)
uint32_t motorPowerM4;  // Motor 4 power output (16bit value used: 0 - 65535)

bool isOn;

static bool isInit;

static uint16_t limitThrust(int32_t value);

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
    vTaskDelayUntil(&lastWakeTime, F2T(1)); // 1Hz
    // other possibility
    // vTaskSuspend() will be invoked again by vTaskResume()

    // actual code for task
    isOn = !isOn; // flip the boolean
    motorPowerM2 = limitThrust(fabs(100*isOn));
    motorsSetRatio(MOTOR_M2, motorPowerM2);
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
    isOn = false;
}

bool modeSwitchTest(void)
{
  bool pass = true;
  // do tests

  return pass;
}

static uint16_t limitThrust(int32_t value)
{
  return limitUint16(value);
}
