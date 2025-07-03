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

#include "physicalConstants.h"
#include "power_distribution.h"
#include "platform_defaults.h"

#define ATTITUDE_UPDATE_DT (float)(1.0f / ATTITUDE_RATE)
#define TOL 1e-6f

static bool isInit = false;

typedef struct mat_4_12_s
{
  float m[4][12];
} mat_4_12_t; // 4x12 matrix

typedef struct mat_3_3_s
{
  float m[3][3];
} mat_3_3_t; // 3x3 matrix

typedef struct vec_3_s
{
  float v[3];
} vec_3_t; // 3x1 column vector

typedef struct vec_12_s
{
  float v[12];
} vec_12_t; // 12x1 column vector

typedef struct quat_s
{
  float q[4];
} quat_t; // quaternion

typedef struct cf_state_s
{
  vec_3_t rw; // position w.r.t. world frame
  quat_t qwb; // body orientation in quaternion form w.r.t. world frame
  vec_3_t vb; // linear velocity w.r.t body frame
  vec_3_t ob; // angular velocity w.r.t body frame
} cf_state_t; // crazyflie's state structure

// static mat_3_3_t CRAZYFLIE_INERTIA =
//     {{{16.6e-6f, 0.83e-6f, 0.72e-6f},
//       {0.83e-6f, 16.6e-6f, 1.8e-6f},
//       {0.72e-6f, 1.8e-6f, 29.3e-6f}}};

// define some logging variables
static float posw_x;
static float posw_y;
static float posw_z;
static float qwb_w;
static float qwb_x;
static float qwb_y;
static float qwb_z;
static float roll;
static float pitch;
static float yaw;
static float velb_x;
static float velb_y;
static float velb_z;
static float omegab_x;
static float omegab_y;
static float omegab_z;

// // define LQR controller variables
// static mat_4_12_t Kinf =
//     {{{-4.05881016e+02f, 4.18693028e+02f, 4.98479582e+02f, -2.29752767e+03f, -2.22107965e+03f, 5.11762255e+02f, -5.90581176e+02f, 6.09658672e+02f, 5.87491481e+02f, -4.26743112e+02f, -4.11200457e+02f, 5.25880761e+02f},
//       {3.99678379e+02f, 4.28156029e+02f, 4.98479582e+02f, -2.35582170e+03f, 2.18513105e+03f, -4.99573788e+02f, 5.81415635e+02f, 6.23872664e+02f, 5.87491481e+02f, -4.38998782e+02f, 4.04145594e+02f, -5.13823162e+02f},
//       {4.45756152e+02f, -4.34418218e+02f, 4.98479582e+02f, 2.39209435e+03f, 2.46056279e+03f, 4.69481138e+02f, 6.50063668e+02f, -6.33124728e+02f, 5.87491481e+02f, 4.46113300e+02f, 4.60190785e+02f, 4.84057745e+02f},
//       {-4.39553515e+02f, -4.12430839e+02f, 4.98479582e+02f, 2.26125502e+03f, -2.42461419e+03f, -4.81669605e+02f, -6.40898127e+02f, -6.00406609e+02f, 5.87491481e+02f, 4.19628594e+02f, -4.53135923e+02f, -4.96115344e+02f}}};
// static float control_speed[4] = {0};
// static float max_thrust = 0.156;    // the maximum thrust (in N) provided by one motor
static int debug_print_counter = 0; // a counter for debug printings rate

void appMain()
{
  DEBUG_PRINT("Waiting for activation ...\n");

  while (1)
  {
    vTaskDelay(M2T(2000));

    // DEBUG_PRINT("My LQR Controller!\n");
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

mat_3_3_t Rq_mat(quat_t q)
{
  mat_3_3_t R;
  float w = q.q[0], x = q.q[1], y = q.q[2], z = q.q[3];

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

// Function to compute the transpose of a 3x3 matrix
static mat_3_3_t mat33_transpose(mat_3_3_t A)
{
  mat_3_3_t At;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      At.m[i][j] = A.m[j][i];
  return At;
}

// function to compute the product A * b, where A is a 3x3 matrix and b is a 3x1 column vector
static vec_3_t mat33_vec3_multiply(mat_3_3_t A, vec_3_t b)
{
  vec_3_t result;
  for (int i = 0; i < 3; ++i)
  {
    result.v[i] = A.m[i][0] * b.v[0] + A.m[i][1] * b.v[1] + A.m[i][2] * b.v[2];
  }
  return result;
}

// function to compute the product A * B, where A, B are 3x3 matrices
static mat_3_3_t mat33_mat33_multiply(mat_3_3_t A, mat_3_3_t B)
{
  mat_3_3_t result;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
    {
      result.m[i][j] = 0.0f;
      for (int k = 0; k < 3; ++k)
        result.m[i][j] += A.m[i][k] * B.m[k][j];
    }
  return result;
}

// function to compute the right minus operator of the SO3 group
static vec_3_t SO3_minus_right(mat_3_3_t R1, mat_3_3_t R2)
{
  mat_3_3_t R_rel;
  vec_3_t result;
  R_rel = mat33_mat33_multiply(mat33_transpose(R2), R1); // R_rel = R2ᵀ * R1

  float tr = R_rel.m[0][0] + R_rel.m[1][1] + R_rel.m[2][2];
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
      result.v[i] = tau[i]; // tau = theta * u
  }
  else
  {
    result.v[0] = 0.0f;
    result.v[1] = 0.0f;
    result.v[2] = 0.0f; // default direction was [0, 0, 1], but theta=0
  }
  return result;
}

// function to compute the state error (state_current - state_reference)
static vec_12_t compute_state_error(const cf_state_t state_cur, const cf_state_t state_ref)
{
  vec_12_t error;

  // Extract state components
  vec_3_t rw_1 = state_cur.rw;
  quat_t qwb_1 = state_cur.qwb;
  vec_3_t vb_1 = state_cur.vb;
  vec_3_t ob_1 = state_cur.ob;

  vec_3_t rw_2 = state_ref.rw;
  quat_t qwb_2 = state_ref.qwb;
  vec_3_t vb_2 = state_ref.vb;
  vec_3_t ob_2 = state_ref.ob;

  // Rotation matrices
  mat_3_3_t Rwb_1 = Rq_mat(qwb_1);
  mat_3_3_t Rwb_2 = Rq_mat(qwb_2);

  // Compute errors
  float rw_error[3];
  for (int i = 0; i < 3; i++)
  {
    rw_error[i] = rw_1.v[i] - rw_2.v[i];
  }

  vec_3_t Rwb_error;                         // placeholder result of SO3.minus_right
  Rwb_error = SO3_minus_right(Rwb_1, Rwb_2); // Assume fills Rwb_error[3]

  float vb_error[3];
  for (int i = 0; i < 3; i++)
  {
    vb_error[i] = vb_1.v[i] - vb_2.v[i];
  }

  float ob_error[3];
  for (int i = 0; i < 3; i++)
  {
    ob_error[i] = ob_1.v[i] - ob_2.v[i];
  }

  // Concatenate into error vector
  for (int i = 0; i < 3; i++)
  {
    error.v[i] = rw_error[i];
    error.v[i + 3] = Rwb_error.v[i];
    error.v[i + 6] = vb_error[i];
    error.v[i + 9] = ob_error[i];
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
  // This loop runs at approximately 1000 Hz rate.
  // rotor speed (rad/sec) w.r.t. PWM:        omegar = sqrt(8e-4f * PWM^2 + 53.33 * PWM)
  // PWM w.r.t. rotor speed (rad/sec):        PWM = -33333 + sqrt(1250 * omegar^2 + 11111*10^5)
  // thrust (N) w.r.t rotor_speed (rad/sec):  Fi = (9/4)*10^(-8) * ui^2

  // controlModeForce for the control
  control->controlMode = controlModeForce;

  // get the current state of the crazyflie
  vec_3_t vw_cur = {{state->velocity.x, state->velocity.y, state->velocity.z}};
  quat_t qwb_cur = {{state->attitudeQuaternion.w, state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z}};
  mat_3_3_t Rwb_cur = Rq_mat(qwb_cur);
  vec_3_t vb_cur = mat33_vec3_multiply(mat33_transpose(Rwb_cur), vw_cur);
  cf_state_t state_cur = {
      .rw = {{state->position.x,
              state->position.y,
              state->position.z}},
      .qwb = {{state->attitudeQuaternion.w,
               state->attitudeQuaternion.x,
               state->attitudeQuaternion.y,
               state->attitudeQuaternion.z}},
      .vb = {{vb_cur.v[0],
              vb_cur.v[1],
              vb_cur.v[2]}},
      .ob = {{radians(sensors->gyro.x),
              radians(sensors->gyro.y),
              radians(sensors->gyro.z)}},
  };
  vec_3_t attb_cur = {{radians(state->attitude.roll), -radians(state->attitude.pitch), radians(state->attitude.yaw)}};

  // get the reference state of the crazyflie (in which state it should end up)
  cf_state_t state_ref = {
      .rw = {{setpoint->position.x,
              setpoint->position.y,
              setpoint->position.z}},
      .qwb = {{setpoint->attitudeQuaternion.w,
               setpoint->attitudeQuaternion.x,
               setpoint->attitudeQuaternion.y,
               setpoint->attitudeQuaternion.z}},
      .vb = {{0.0, 0.0, 0.0}},
      .ob = {{0.0, 0.0, 0.0}},
  };
  vec_3_t attb_ref = {{radians(setpoint->attitude.roll), -radians(setpoint->attitude.pitch), radians(setpoint->attitude.yaw)}};

  // create the control signal
  vec_12_t state_error = compute_state_error(state_cur, state_ref);
  // control->normalizedForces = ;

  // print some data for debugging
  if (debug_print_counter % 500 == 0)
  {
    debug_print_counter = 0;
    DEBUG_PRINT("Current state [%lu]: rw(m) = [%.3f, %.3f, %.3f],\t\t qwb = [%.3f, %.3f, %.3f, %.3f],\t\t attb(deg) = [%.3f, %.3f, %.3f],\n\t\t vb(m/s) = [%.3f, %.3f, %.3f],\t\t ob(deg/s) = [%.3f, %.3f, %.3f]\n",
                tick,
                (double)state_cur.rw.v[0], (double)state_cur.rw.v[1], (double)state_cur.rw.v[2],
                (double)state_cur.qwb.q[0], (double)state_cur.qwb.q[1], (double)state_cur.qwb.q[2], (double)state_cur.qwb.q[3],
                (double)degrees(attb_cur.v[0]), (double)degrees(attb_cur.v[1]), (double)degrees(attb_cur.v[2]),
                (double)state_cur.vb.v[0], (double)state_cur.vb.v[1], (double)state_cur.vb.v[2],
                (double)degrees(state_cur.ob.v[0]), (double)degrees(state_cur.ob.v[1]), (double)degrees(state_cur.ob.v[2]));
    DEBUG_PRINT("Reference state [%lu]: rw(m) = [%.3f, %.3f, %.3f],\t\t qwb = [%.3f, %.3f, %.3f, %.3f],\t\t attb(deg) = [%.3f, %.3f, %.3f]\n",
                tick,
                (double)state_cur.rw.v[0], (double)state_cur.rw.v[1], (double)state_cur.rw.v[2],
                (double)state_ref.qwb.q[0], (double)state_ref.qwb.q[1], (double)state_ref.qwb.q[1], (double)state_ref.qwb.q[2],
                (double)degrees(attb_ref.v[0]), (double)degrees(attb_ref.v[1]), (double)degrees(attb_ref.v[2]));
    DEBUG_PRINT("State error [%lu]: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                tick,
                (double)state_error.v[0], (double)state_error.v[1], (double)state_error.v[2],
                (double)state_error.v[3], (double)state_error.v[4], (double)state_error.v[5],
                (double)state_error.v[6], (double)state_error.v[7], (double)state_error.v[8],
                (double)state_error.v[9], (double)state_error.v[10], (double)state_error.v[11]);
  }
  debug_print_counter += 1;

  // store data to the logging variables
  posw_x = state_cur.rw.v[0];
  posw_y = state_cur.rw.v[1];
  posw_z = state_cur.rw.v[2];
  qwb_w = state_cur.qwb.q[0];
  qwb_x = state_cur.qwb.q[1];
  qwb_y = state_cur.qwb.q[2];
  qwb_z = state_cur.qwb.q[3];
  roll = attb_cur.v[0];
  pitch = attb_cur.v[1];
  yaw = attb_cur.v[2];
  velb_x = state_cur.vb.v[0];
  velb_y = state_cur.vb.v[1];
  velb_z = state_cur.vb.v[2];
  omegab_x = state_cur.ob.v[0];
  omegab_y = state_cur.ob.v[1];
  omegab_z = state_cur.ob.v[2];

  controllerPid(control, setpoint, sensors, state, tick);
}

LOG_GROUP_START(LQRcontroller)
/**
 * @brief Position x (m) - world frame
 */
LOG_ADD(LOG_FLOAT, posw_x, &posw_x)
/**
 * @brief Position y (m) - world frame
 */
LOG_ADD(LOG_FLOAT, posw_y, &posw_y)
/**
 * @brief Position z (m) - world frame
 */
LOG_ADD(LOG_FLOAT, posw_z, &posw_z)
/**
 * @brief Body Quaternion w - world frame
 */
LOG_ADD(LOG_FLOAT, qwb_w, &qwb_w)
/**
 * @brief Body Quaternion x - world frame
 */
LOG_ADD(LOG_FLOAT, qwb_x, &qwb_x)
/**
 * @brief Body Quaternion y - world frame
 */
LOG_ADD(LOG_FLOAT, qwb_y, &qwb_y)
/**
 * @brief Body Quaternion z - world frame
 */
LOG_ADD(LOG_FLOAT, qwb_z, &qwb_z)
/**
 * @brief Roll angle (deg)
 */
LOG_ADD(LOG_FLOAT, roll, &roll)
/**
 * @brief Pitch angle (deg)
 */
LOG_ADD(LOG_FLOAT, pitch, &pitch)
/**
 * @brief Yaw angle (deg)
 */
LOG_ADD(LOG_FLOAT, yaw, &yaw)
/**
 * @brief Linear velocity x (m/s) - body frame
 */
LOG_ADD(LOG_FLOAT, velb_x, &velb_x)
/**
 * @brief Linear velocity y (m/s) - body frame
 */
LOG_ADD(LOG_FLOAT, velb_y, &velb_y)
/**
 * @brief Linear velocity z (m/s) - body frame
 */
LOG_ADD(LOG_FLOAT, velb_z, &velb_z)
/**
 * @brief Angular velocity x (deg/s) - body frame
 */
LOG_ADD(LOG_FLOAT, omegab_x, &omegab_x)
/**
 * @brief Angular velocity y (deg/s) - body frame
 */
LOG_ADD(LOG_FLOAT, omegab_y, &omegab_y)
/**
 * @brief Angular velocity z (deg/s) - body frame
 */
LOG_ADD(LOG_FLOAT, omegab_z, &omegab_z)
LOG_GROUP_STOP(LQRcontroller)
