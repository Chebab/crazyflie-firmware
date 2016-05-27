#include <math.h>

#include "reference_generator.h"

#include "FreeRTOS.h"
#include "task.h"
#include "config.h"
#include "motors.h"
#include "system.h"
#include "log.h"
#include "commander.h"
#include "imu.h"
//#include "position_estimator_altitude.h"

#include "stabilizer.h"

// TODO find and solve dependencies for commander

static float zVelDesired;
static float eulerRollDesired;   // Measured roll angle in deg
static float eulerPitchDesired;  // Measured pitch angle in deg
static float eulerYawDesired;    // Measured yaw angle in deg
float reference[STATE_SIZE];


static bool isInit;

static void referenceGeneratorTask(void* param)
{
  uint32_t lastWakeTime;

  // sets the id number in TASK_MODE_SWITCH_ID_NBR inside FreeRTOSConfig.h
  //(included with FreeRTOS.h) to this task (0 denotes this)
  vTaskSetApplicationTaskTag(0, (void*)TASK_REFERENCE_GENERATOR_ID_NBR);

  lastWakeTime = xTaskGetTickCount();
  unsigned int update = 0;
  while (1) {

    vTaskDelayUntil(&lastWakeTime, F2T(250)); // 250Hz


    update++;
    // actual code for task
    if (imu6IsCalibrated())
    {


      // read references from controller
      commanderGetRPY(&eulerRollDesired, &eulerPitchDesired, &eulerYawDesired);

      // Convet to radians
      const float degToRad = 3.14f/180.0f;
      // Get the filtered velocity
      commanderGetZVelocity(&zVelDesired);

      while (!xSemaphoreTake(canUseReferenceMutex, portMAX_DELAY));
      // update the reference values

      // change the direction to compensate for up-side-down model
      reference[0] = 0;//-zVelDesired;

      reference[1] = eulerRollDesired*degToRad;
      reference[2] = eulerPitchDesired*degToRad;
      reference[3] = eulerYawDesired*degToRad;

      //release reference mutex
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
    reference[0]=-MIN_VELZ;
    int i = 1;
    for(;i<STATE_SIZE;i++){
      reference[i] = 0;
    }

}

bool referenceGeneratorTest(void)
{
  bool pass = true;
  // do tests

  return pass;
}
LOG_GROUP_START(velz)
LOG_ADD(LOG_FLOAT, velz, &zVelDesired)
LOG_GROUP_STOP(velz)
