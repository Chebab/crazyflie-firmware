#include <math.h>

#include "reference_generator.h"

#include "FreeRTOS.h"
#include "task.h"
#include "config.h"
#include "motors.h"
#include "system.h"

#include "stabilizer.h"

// TODO find and solve dependencies for commander

float zAccDesired;
float eulerRollDesired;   // Measured roll angle in deg
float eulerPitchDesired;  // Measured pitch angle in deg
float eulerYawDesired;    // Measured yaw angle in deg

static bool isInit;
static uint16_t limitThrust(int32_t value);

static void referenceGeneratorTask(void* param)
{
  uint32_t lastWakeTime;

  // sets the id number in TASK_MODE_SWITCH_ID_NBR inside FreeRTOSConfig.h
  //(included with FreeRTOS.h) to this task (0 denotes this)
  vTaskSetApplicationTaskTag(0, (void*)TASK_REFERENCE_GENERATOR_ID_NBR);

  lastWakeTime = xTaskGetTickCount();

  while (1) { // TODO fix the entire function
    // some kind of event listening
    // this makes it run with a frequency
    // F2T comes from FreeRTOSConfig.h
    vTaskDelayUntil(&lastWakeTime, F2T(1)); // 1Hz
    // other possibility
    // vTaskSuspend() will be invoked again by vTaskResume()

    // actual code for task
    if (imu6IsCalibrated())
    {
      while (!xSemaphoreTake(canUseReferenceMutex, portMAX_DELAY));

      // read references from controller
      commanderGetRPY(&eulerRollDesired, &eulerPitchDesired, &eulerYawDesired);
      // used if you want controller to control rates of angles instead
      //commanderGetRPYType(&rollType, &pitchType, &yawType);

      // TODO read acceleration reference
      xSemaphoreGive(canUseReferenceMutex);
    }
  }
}

void referenceGeneratorInit(void)
{
  if(isInit)
    return;

    // finals are decleared in config.h
    xTaskCreate(referenceGeneratorTask, REFERENCE_GENERATOR_TASK_NAME,
                REFERENCE_GENERATOR_TASK_STACKSIZE, NULL,
                REFERENCE_GENERATOR_TASK_PRI, NULL);

    isInit = true;
}

bool referenceGeneratorTest(void)
{
  bool pass = true;
  // do tests

  return pass;
}

static uint16_t limitThrust(int32_t value)
{
  return limitUint16(value);
}
