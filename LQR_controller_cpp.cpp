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

// #include "controller.h"
// #include "controller_pid.h"

#include "log.h"
#include "param.h"
#include "math.h"
#include "math3d.h"
#include "debug.h"

#include "physicalConstants.h"
#include "power_distribution.h"
#include "platform_defaults.h"

#define ATTITUDE_UPDATE_DT (float)(1.0f / ATTITUDE_RATE)
#define TOL 1e-6f

static bool isInit = false;

typedef struct mat_4_12_s
{
  float m[4][12];
} mat_4_12_t;

typedef struct vec_12_s
{
  float m[12];
} vec_12_t;
// typedef float vec_12_t[12];

typedef struct my_state_s
{
  float rw[3];     // crazyflie's position w.r.t. world frame
  float qwb[4];    // crazyflie's body orientation in quaternion form w.r.t. world frame
  float vb[3];     // crazyflie's linear velocity w.r.t body frame
  float omegab[3]; // crazyflie's angular velocity w.r.t body frame
} my_state_t;

// static mat_4_12_t Kinf =
//     {{{-4.05881016e+02f, 4.18693028e+02f, 4.98479582e+02f, -2.29752767e+03f, -2.22107965e+03f, 5.11762255e+02f, -5.90581176e+02f, 6.09658672e+02f, 5.87491481e+02f, -4.26743112e+02f, -4.11200457e+02f, 5.25880761e+02f},
//       {3.99678379e+02f, 4.28156029e+02f, 4.98479582e+02f, -2.35582170e+03f, 2.18513105e+03f, -4.99573788e+02f, 5.81415635e+02f, 6.23872664e+02f, 5.87491481e+02f, -4.38998782e+02f, 4.04145594e+02f, -5.13823162e+02f},
//       {4.45756152e+02f, -4.34418218e+02f, 4.98479582e+02f, 2.39209435e+03f, 2.46056279e+03f, 4.69481138e+02f, 6.50063668e+02f, -6.33124728e+02f, 5.87491481e+02f, 4.46113300e+02f, 4.60190785e+02f, 4.84057745e+02f},
//       {-4.39553515e+02f, -4.12430839e+02f, 4.98479582e+02f, 2.26125502e+03f, -2.42461419e+03f, -4.81669605e+02f, -6.40898127e+02f, -6.00406609e+02f, 5.87491481e+02f, 4.19628594e+02f, -4.53135923e+02f, -4.96115344e+02f}}};

// static struct mat33 CRAZYFLIE_INERTIA =
//     {{{16.6e-6f, 0.83e-6f, 0.72e-6f},
//       {0.83e-6f, 16.6e-6f, 1.8e-6f},
//       {0.72e-6f, 1.8e-6f, 29.3e-6f}}};

// define some log variables
static float posw;

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

vec_12_t compute_state_error(const my_state_t *state_cur, const my_state_t *state_ref)
{
  vec_12_t error;

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

void attitudeControlInit(const float updateDt)
{
  if (isInit)
    return;

  // add stuff here

  isInit = true;
}

void positionControlInit(void)
{
  return;
}

bool attitudeControlTest()
{
  return isInit;
}

void controllerOutOfTreeInit(void)
{
  attitudeControlInit(ATTITUDE_UPDATE_DT);
  positionControlInit();
  return;
}

bool controllerOutOfTreeTest(void)
{
  bool pass = true;
  pass &= attitudeControlTest();
  return pass;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint,
                         const sensorData_t *sensors,
                         const state_t *state,
                         const stabilizerStep_t tick)
{
  // rotor speed (rad/sec) w.r.t. PWM: sqrt(8e-4f * x^2 + 53.33 * x)
  // PWM w.r.t. rotor speed (rad/sec): -33333 + sqrt(1250 * x^2 + 11111*10^5)

  // controlModeForce for the control
  control->controlMode = controlModeForce;

  struct vec rw = mkvec(state->position.x,
                        state->position.y,
                        state->position.z);
  struct quat qwb = mkquat(state->attitudeQuaternion.w,
                           state->attitudeQuaternion.x,
                           state->attitudeQuaternion.y,
                           state->attitudeQuaternion.z);
  struct vec vb = mkvec(state->velocity.x,
                        state->velocity.y,
                        state->velocity.z);
  struct vec omegab = mkvec(radians(sensors->gyro.x),
                            radians(sensors->gyro.y),
                            radians(sensors->gyro.z));
  my_state_t state_cur = {
      .rw = {rw.x, rw.y, rw.z},
      .qwb = {qwb.w, qwb.x, qwb.y, qwb.z},
      .vb = {vb.x, vb.y, vb.z},
      .omegab = {omegab.x, omegab.y, omegab.z},
  };

  DEBUG_PRINT("Current state: rw=[%.3f, %.3f, %.3f], qwb=[%.3f, %.3f, %.3f, %.3f], vb=[%.3f, %.3f, %.3f], omegab=[%.3f, %.3f, %.3f]\n",
              (double)state_cur.rw[0], (double)state_cur.rw[1], (double)state_cur.rw[2],
              (double)state_cur.qwb[0], (double)state_cur.qwb[1], (double)state_cur.qwb[2], (double)state_cur.qwb[3],
              (double)state_cur.vb[0], (double)state_cur.vb[1], (double)state_cur.vb[2],
              (double)state_cur.omegab[0], (double)state_cur.omegab[1], (double)state_cur.omegab[2]);
  // controllerPid(control, setpoint, sensors, state, tick);
}

LOG_GROUP_START(LQRcontroller)
/**
 * @brief Position w.r.t world frame
 */
LOG_ADD(LOG_FLOAT, posw, &posw)
/**
 * @brief Linear velocity w.r.t world frame
 */
// LOG_ADD(LOG)
LOG_GROUP_STOP(LQRcontroller)

