/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "system.h"
#include "pm.h"
#include "stabilizer.h"
#include "reference_generator.h"
#include "mode_switch.h"
#include "commander.h"
#include "attitude_controller.h"
#include "sensfusion6.h"
#include "imu.h"
#include "motors.h"
#include "log.h"
#include "pid.h"
#include "param.h"
#include "sitaw.h"
#ifdef PLATFORM_CF1
  #include "ms5611.h"
#else
  #include "lps25h.h"
#endif
#include "num.h"
#include "position_estimator.h"
#include "position_controller.h"
#include "altitude_hold.h"


/**
 * Defines in what divided update rate should the attitude
 * control loop run relative the rate control loop.
 */
#define ATTITUDE_UPDATE_RATE_DIVIDER  2
#define ATTITUDE_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER)) // 250hz

#define ALTHOLD_UPDATE_RATE_DIVIDER  5
#define ALTHOLD_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ALTHOLD_UPDATE_RATE_DIVIDER))   // 100hz

#define STATE_SIZE 7
#define INPUT_SIZE 4

static Axis3f gyro; // Gyro axis data in deg/s
static Axis3f acc;  // Accelerometer axis data in mG
static Axis3f mag;  // Magnetometer axis data in testla

static float eulerRollActual;   // Measured roll angle in deg
static float eulerPitchActual;  // Measured pitch angle in deg
static float eulerYawActual;    // Measured yaw angle in deg
static float rollRate;
static float pitchRate;
static float yawRate;

// Controller matrices
const float K_agg[INPUT_SIZE][STATE_SIZE] = {{-0.481824, -0.000000, 0.436087, 0.000472, -0.000000, 0.138356, 0.000150 },
{-0.481824, -0.424516, 0.000000, -0.000472, -0.134684, 0.000000, -0.000150 },
{-0.481824, -0.000000, -0.436087, 0.000472, -0.000000, -0.138356, 0.000150 },
{-0.481824, 0.424516, 0.000000, -0.000472, 0.134684, 0.000000, -0.000150}
};
const float Kr_agg[INPUT_SIZE][STATE_SIZE] = {{-0.481824, -0.000000, 0.436087, 0.000472, -0.000000, 0.138356, 0.000150 },
{-0.481824, -0.424516, 0.000000, -0.000472, -0.134684, 0.000000, -0.000150 },
{-0.481824, -0.000000, -0.436087, 0.000472, -0.000000, -0.138356, 0.000150 },
{-0.481824, 0.424516, 0.000000, -0.000472, 0.134684, 0.000000, -0.000150}
};

uint16_t actuatorThrust;  // Actuator output for thrust base

uint32_t motorPowerM1;  // Motor 1 power output (16bit value used: 0 - 65535)
uint32_t motorPowerM2;  // Motor 2 power output (16bit value used: 0 - 65535)
uint32_t motorPowerM3;  // Motor 3 power output (16bit value used: 0 - 65535)
uint32_t motorPowerM4;  // Motor 4 power output (16bit value used: 0 - 65535)

static float states[STATE_SIZE]={0,0,0,0,0,0,0};
static float reference[STATE_SIZE]={0,0,0,0,0,0,0};
static float thrusts[INPUT_SIZE]={0,0,0,0};

static bool isInit = false;

static uint16_t limitThrust(float value);
static void convertAngles(float rollCrazyframe, float pitchCrazyFrame, float yawCrazyFrame,
float rollRateCrazyFrame, float pitchRateCrazyFrame, float yawRateCrazyFrame);

static void LQR(float currentStates[7]);
static int32_t thrust2PWM(float thrust);

static void stabilizerTask(void* param)
{
  uint32_t attitudeCounter = 0;
  uint32_t lastWakeTime;

  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  lastWakeTime = xTaskGetTickCount ();

  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(IMU_UPDATE_FREQ)); //1Hz before: IMU_UPDATE_FREQ=500Hz

    // Magnetometer not yet used more then for logging.
    imu9Read(&gyro, &acc, &mag);

    if (imu6IsCalibrated())
    {
      // 250HZ
      if (++attitudeCounter >= ATTITUDE_UPDATE_RATE_DIVIDER)
      {
        sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, ATTITUDE_UPDATE_DT);
        sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);

        // convert from the crazyflie frame to ours
        // TODO find out if gyro.x are actual values in degrees or radians
        convertAngles(eulerRollActual, eulerPitchActual, eulerYawActual,
          gyro.x, gyro.y, gyro.z);

        // try to take the semaphore until it is possible
        // TODO maybe move semaphores to LQR()
        //while (!xSemaphoreTake(canUseReferenceMutex, portMAX_DELAY));
        //while (!xSemaphoreTake(canUseStateGain, portMAX_DELAY));
        states[0]=0;
        states[1]=eulerRollActual;
        states[2]=eulerPitchActual;
        states[3]=eulerYawActual;
        states[4]=rollRate;
        states[5]=pitchRate;
        states[6]=yawRate;

        //states = {acc.z,eulerRollActual, eulerPitchActual, eulerYawActual,
          //rollRate, pitchRate, yawRate}; // TODO make shure this are all correct types and units
        // TODO: remember that angles are in degrees?
        LQR(states); // uses the reference and is therefore inside semaphore protection

        //xSemaphoreGive(canUseStateGainMutex);
        //xSemaphoreGive(canUseReferenceMutex);

        // Set motors depending on the euler angles
        // TODO: set values based on thrusts from LQR
        // TODO: find how to transform them into pwm

        motorPowerM1 = limitThrust(fabs(thrusts[0]));
        motorPowerM2 = limitThrust(fabs(thrusts[1]));
        motorPowerM3 = limitThrust(fabs(thrusts[2]));
        motorPowerM4 = limitThrust(fabs(thrusts[3]));

/*
        motorsSetRatio(MOTOR_M1, motorPowerM1);
        motorsSetRatio(MOTOR_M2, motorPowerM2);
        motorsSetRatio(MOTOR_M3, motorPowerM3);
        motorsSetRatio(MOTOR_M4, motorPowerM4);
*/

        attitudeCounter = 0;
      }
    }

  }
}

void stabilizerInit(void)
{
  if(isInit)
    return;

  motorsInit(motorMapDefaultBrushed);
  imu6Init();
  sensfusion6Init();
  attitudeControllerInit();

  xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME,
              STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);

  isInit = true;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= motorsTest();
  pass &= imu6Test();
  pass &= sensfusion6Test();
  pass &= attitudeControllerTest();

  return pass;
}

static void LQR_new(float *currentStates,float* reference,float *output,float **K, float **Kr){
  int i = 0;
  int j = 0;
  for(;i<INPUT_SIZE;i++){
    for(;j<STATE_SIZE;j++){
      output[i]+=Kr[i][j]*reference[i]-K[i][j]*currentStates[j];
    }
  }
}

static void LQR(float currentStates[7])
{
  ;

  //if (isAgressive) {
    float K[4][7] = {{-0.481824, -0.000000, 0.436087, 0.000472, -0.000000, 0.138356, 0.000150 },
{-0.481824, -0.424516, 0.000000, -0.000472, -0.134684, 0.000000, -0.000150 },
{-0.481824, -0.000000, -0.436087, 0.000472, -0.000000, -0.138356, 0.000150 },
{-0.481824, 0.424516, 0.000000, -0.000472, 0.134684, 0.000000, -0.000150}
};
    float Kr[4][7] = {{-0.481824, -0.000000, 0.436087, 0.000472, -0.000000, 0.138356, 0.000150 },
{-0.481824, -0.424516, 0.000000, -0.000472, -0.134684, 0.000000, -0.000150 },
{-0.481824, -0.000000, -0.436087, 0.000472, -0.000000, -0.138356, 0.000150 },
{-0.481824, 0.424516, 0.000000, -0.000472, 0.134684, 0.000000, -0.000150}
};
  //}
  /*
  else // TODO change the matrixes for the diferent modes
  {
    K = {{-0.481824, -0.000000, 0.436087, 0.000472, -0.000000, 0.138356, 0.000150 },
{-0.481824, -0.424516, 0.000000, -0.000472, -0.134684, 0.000000, -0.000150 },
{-0.481824, -0.000000, -0.436087, 0.000472, -0.000000, -0.138356, 0.000150 },
{-0.481824, 0.424516, 0.000000, -0.000472, 0.134684, 0.000000, -0.000150}
};
    Kr = {{-0.481824, -0.000000, 0.436087, 0.000472, -0.000000, 0.138356, 0.000150 },
{-0.481824, -0.424516, 0.000000, -0.000472, -0.134684, 0.000000, -0.000150 },
{-0.481824, -0.000000, -0.436087, 0.000472, -0.000000, -0.138356, 0.000150 },
{-0.481824, 0.424516, 0.000000, -0.000472, 0.134684, 0.000000, -0.000150}
};
  }*/

  int out;
  int state;
  for (out=0; out<4; out++) {
    for (state=0; state<7; state++) {
      thrusts[out] += Kr[out][state]*reference[state]
                      -K[out][state]*currentStates[state];
    }
  }

}

static void convertAngles(
  float rollCrazyFrame, float pitchCrazyFrame, float yawCrazyFrame,
  float rollRateCrazyFrame, float pitchRateCrazyFrame, float yawRateCrazyFrame)
{
  const float degToRad = 3.14/180;
  const float k =1/sqrt(2.0)*degToRad;
  eulerRollActual =
      (float) k*(rollCrazyFrame - pitchCrazyFrame);
  rollRate =
      (float) k*(rollRateCrazyFrame - pitchRateCrazyFrame);
  eulerPitchActual =
      (float) -k*(rollCrazyFrame + pitchCrazyFrame);
  pitchRate =
      (float) -k*(rollRateCrazyFrame + pitchRateCrazyFrame);
  // yaw are the same in crazyframe and ours
  eulerYawActual = yawCrazyFrame*degToRad;
  yawRate = yawRateCrazyFrame*degToRad;
}

// convert thrust to pwm according to article on Bitcraze
// TODO check if type conversion really is correct
// TODO round to nearest integer before type casting
static int32_t thrust2PWM(float thrust)
{
  static float a = 0.000409;
  static float b = 0.1405;
  static float c = -0.000099;

  thrust = thrust/9.81; // make thrust to g

  return (int32_t) -b/(2*a)+sqrt((thrust-c)/a + b*b/(4*a*a));
}


static uint16_t limitThrust(float value)
{
  uint16_t intValue;
  value = (float) value/9.81; // convert to g
  if (value <= 0.0f) {
    intValue = 0;
  }
  else if (value >= 60.0f) {
    intValue = 65526;
  }
  else
  {
    float scale = (value/60.0);
    value = 65526.0*scale;
    intValue = (uint16_t) floor(value + 0.5); // round to nearest integer
  }


  return intValue;
}

LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &eulerRollActual)
LOG_ADD(LOG_FLOAT, pitch, &eulerPitchActual)
LOG_ADD(LOG_FLOAT, yaw, &eulerYawActual)
LOG_ADD(LOG_UINT16, thrust, &actuatorThrust)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &acc.x)
LOG_ADD(LOG_FLOAT, y, &acc.y)
LOG_ADD(LOG_FLOAT, z, &acc.z)
LOG_GROUP_STOP(acc)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &gyro.x)
LOG_ADD(LOG_FLOAT, y, &gyro.y)
LOG_ADD(LOG_FLOAT, z, &gyro.z)
LOG_GROUP_STOP(gyro)

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &mag.x)
LOG_ADD(LOG_FLOAT, y, &mag.y)
LOG_ADD(LOG_FLOAT, z, &mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m4, &motorPowerM4)
LOG_ADD(LOG_INT32, m1, &motorPowerM1)
LOG_ADD(LOG_INT32, m2, &motorPowerM2)
LOG_ADD(LOG_INT32, m3, &motorPowerM3)
LOG_GROUP_STOP(motor)
