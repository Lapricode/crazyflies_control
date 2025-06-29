/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
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
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "controller.h"
#include "controller_pid.h"

#include "log.h"
#include "param.h"
#include "math.h"
#include "math3d.h"

#include "debug.h"

#define DEBUG_MODULE "MY_CONTROLLER"
#define ATTITUDE_UPDATE_DT (float)(1.0f / ATTITUDE_RATE)

typedef struct mat_4_12
{
  float m[4][12];
};

static struct mat_4_12 Kinf =
    {{{-7.23607257e-01f, 7.23607257e-01f, 1.55913072e+00f, 5.89179228e+00f, 7.83138464e-15f, 6.03580426e-02f, 1.30523077e-15f, -1.51390818e+00f, 2.22715559e+00f, 1.12469478e+00f, 1.63153847e-15f, 6.12650533e-02f},
      {7.23607257e-01f, 7.23607257e-01f, 1.55913072e+00f, -3.45722499e-14f, -5.89179228e+00f, -6.03580426e-02f, -1.51390818e+00f, 9.10608368e-15f, 2.22715559e+00f, -6.94531806e-15f, -1.12469478e+00f, -6.12650533e-02f},
      {7.23607257e-01f, -7.23607257e-01f, 1.55913072e+00f, -5.89179228e+00f, 3.81482621e-15f, 6.03580426e-02f, 3.17902184e-16f, 1.51390818e+00f, 2.22715559e+00f, -1.12469478e+00f, 7.94755460e-16f, 6.12650533e-02f},
      {-7.23607257e-01f, -7.23607257e-01f, 1.55913072e+00f, -4.45307363e-14f, 5.89179228e+00f, -6.03580426e-02f, 1.51390818e+00f, 1.14419253e-14f, 2.22715559e+00f, -8.96799550e-15f, 1.12469478e+00f, -6.12650533e-02f}}};
static struct mat33 CRAZYFLIE_INERTIA =
    {{{16.6e-6f, 0.83e-6f, 0.72e-6f},
      {0.83e-6f, 16.6e-6f, 1.8e-6f},
      {0.72e-6f, 1.8e-6f, 29.3e-6f}}};
static bool isInit = false;

void appMain()
{
  DEBUG_PRINT("Waiting for activation ...\n");

  while (1)
  {
    vTaskDelay(M2T(2000));

    DEBUG_PRINT("My Controller!\n");
  }
}

// static float capAngle(float angle)
// {
//   float result = angle;
//   while (result > 180.0f)
//   {
//     result -= 360.0f;
//   }
//   while (result < -180.0f)
//   {
//     result += 360.0f;
//   }
//   return result;
// }

void attitudeControllerInit(const float updateDt)
{
  if (isInit)
    return;

  // add stuff here

  isInit = true;
}

void positionControllerInit(void)
{
  return;
}

bool attitudeControllerTest()
{
  return isInit;
}

void controllerOutOfTreeInit(void)
{
  attitudeControllerInit(ATTITUDE_UPDATE_DT);
  positionControllerInit();
}

bool controllerOutOfTreeTest(void)
{
  bool pass = true;
  pass &= attitudeControllerTest();
  return pass;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint,
                         const sensorData_t *sensors,
                         const state_t *state,
                         const stabilizerStep_t tick)
{
  controllerPid(control, setpoint, sensors, state, tick);
}

LOG_GROUP_START(LQRcontroller)
// /**
//  *@brief Thrust command
//  */
// LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_GROUP_STOP(LQRcontroller)
