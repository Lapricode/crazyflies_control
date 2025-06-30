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
#define TOL 1e-6f

typedef struct mat_4_12
{
  float m[4][12];
};

typedef struct vec_12
{
  float m[12];
};
// typedef float vec_12[12];

typedef struct my_state_s
{
  float rw[3];
  float qwb[4];
  float vb[3];
  float omegab[3];
} my_state_t;

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

    DEBUG_PRINT("My LQR Controller!\n");
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

struct mat33 Rq_mat(const float q[4])
{
  float w = q[0], x = q[1], y = q[2], z = q[3];
  struct mat33 R;

  R.m[0][0] = w * w + x * x - y * y - z * z;
  R.m[0][1] = 2.0f * (x * y - w * z);
  R.m[0][2] = 2.0f * (x * z + w * y);

  R.m[1][0] = 2.0f * (x * y + w * z);
  R.m[1][1] = w * w - x * x + y * y - z * z;
  R.m[1][2] = 2.0f * (y * z - w * x);

  R.m[2][0] = 2.0f * (x * z - w * y);
  R.m[2][1] = 2.0f * (y * z + w * x);
  R.m[2][2] = w * w - x * x - y * y + z * z;

  return R;
}

float trace_mat33(struct mat33 R)
{
  return R.m[0][0] + R.m[1][1] + R.m[2][2];
}

void mat33_transpose_mul(struct mat33 A, struct mat33 B, struct mat33 *result)
{
  // result = Bᵀ * A
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
    {
      result->m[i][j] = 0.0f;
      for (int k = 0; k < 3; ++k)
        result->m[i][j] += B.m[k][i] * A.m[k][j];
    }
}

void SO3_minus_right(struct mat33 R1, struct mat33 R2, float out[3])
{
  struct mat33 R_rel;
  mat33_transpose_mul(R1, R2, &R_rel); // R_rel = R2ᵀ * R1

  float tr = trace_mat33(R_rel);
  float cos_theta = (tr - 1.0f) / 2.0f;
  if (cos_theta > 1.0f)
    cos_theta = 1.0f;
  if (cos_theta < -1.0f)
    cos_theta = -1.0f;
  float theta = acosf(cos_theta);

  float tau_hat[3][3];
  if (fabsf(theta) >= TOL)
  {
    float scale = theta / (2.0f * sinf(theta));
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        tau_hat[i][j] = scale * (R_rel.m[i][j] - R_rel.m[j][i]);
  }
  else
  {
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        tau_hat[i][j] = 0.5f * (R_rel.m[i][j] - R_rel.m[j][i]);
    theta = 0.0f;
  }

  float tau[3] = {
      tau_hat[2][1],
      tau_hat[0][2],
      tau_hat[1][0]};

  if (fabsf(theta) >= TOL)
  {
    for (int i = 0; i < 3; ++i)
      out[i] = tau[i]; // tau = theta * u
  }
  else
  {
    out[0] = 0.0f;
    out[1] = 0.0f;
    out[2] = 0.0f; // default direction was [0, 0, 1], but theta=0
  }
}

struct vec_12 compute_state_error(const my_state_t *state_cur, const my_state_t *state_ref)
{
  struct vec_12 error;

  // Extract state components
  const float *rw_1 = state_cur->rw;
  const float *qwb_1 = state_cur->qwb;
  const float *vb_1 = state_cur->vb;
  const float *omegab_1 = state_cur->omegab;

  const float *rw_2 = state_ref->rw;
  const float *qwb_2 = state_ref->qwb;
  const float *vb_2 = state_ref->vb;
  const float *omegab_2 = state_ref->omegab;

  // Rotation matrices
  struct mat33 Rwb_1 = Rq_mat(qwb_1);
  struct mat33 Rwb_2 = Rq_mat(qwb_2);

  // Compute errors
  float rw_error[3];
  for (int i = 0; i < 3; i++)
  {
    rw_error[i] = rw_1[i] - rw_2[i];
  }

  float Rwb_error[3];                       // placeholder result of SO3.minus_right
  SO3_minus_right(Rwb_1, Rwb_2, Rwb_error); // Assume fills Rwb_error[3]

  float vb_error[3];
  for (int i = 0; i < 3; i++)
  {
    vb_error[i] = vb_1[i] - vb_2[i];
  }

  float omegab_error[3];
  for (int i = 0; i < 3; i++)
  {
    omegab_error[i] = omegab_1[i] - omegab_2[i];
  }

  // Concatenate into error vector
  for (int i = 0; i < 3; i++)
  {
    error.m[i] = rw_error[i];
    error.m[i + 3] = Rwb_error[i];
    error.m[i + 6] = vb_error[i];
    error.m[i + 9] = omegab_error[i];
  }

  return error;
}

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

