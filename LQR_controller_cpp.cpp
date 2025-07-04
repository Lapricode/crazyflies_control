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

typedef struct mat_3_3_s
{
  float m[3][3];
} mat_3_3_t; // 3x3 matrix

typedef union vec_3_u
{
  float v[3];
  struct
  {
    float x, y, z;
  };
} vec_3_t; // 3x1 column vector

typedef struct vec_4_u
{
  float v[4];
  struct
  {
    float v1, v2, v3, v4;
  };
} vec_4_t; // 4x1 column vector

typedef union quat_u
{
  float q[4];
  struct
  {
    float w, x, y, z;
  };
} quat_t; // quaternion

typedef struct cf_state_s
{
  vec_3_t rw; // position w.r.t. world frame
  quat_t qwb; // body orientation in quaternion form w.r.t. world frame
  vec_3_t vb; // linear velocity w.r.t body frame
  vec_3_t ob; // angular velocity w.r.t body frame
} cf_state_t; // crazyflie's state structure

// // define some logging variables
// static float posw_x;
// static float posw_y;
// static float posw_z;
// static float qwb_w;
// static float qwb_x;
// static float qwb_y;
// static float qwb_z;
// static float roll;
// static float pitch;
// static float yaw;
// static float velb_x;
// static float velb_y;
// static float velb_z;
// static float omegab_x;
// static float omegab_y;
// static float omegab_z;
// static float posw_x_ref;
// static float posw_y_ref;
// static float posw_z_ref;
// static float yaw_ref;
// static float ctrl_speed_m1;
// static float ctrl_speed_m2;
// static float ctrl_speed_m3;
// static float ctrl_speed_m4;
// static float ctrl_thrust_m1;
// static float ctrl_thrust_m2;
// static float ctrl_thrust_m3;
// static float ctrl_thrust_m4;
// static float ctrl_norm_thrust_m1;
// static float ctrl_norm_thrust_m2;
// static float ctrl_norm_thrust_m3;
// static float ctrl_norm_thrust_m4;
static int debug_print_counter = 0; // a counter for debug printing rate

// define some crazyflie model parameters
static const float g = 9.81f;  // gravity's acceleration (in m/sec^2)
static const float M = 0.033f; // mass (in kg)
static const float L = 0.046f; // arm length (in m)
// static const mat_3_3_t CRAZYFLIE_INERTIA =
//     {{{16.6e-6f, 0.83e-6f, 0.72e-6f},
//       {0.83e-6f, 16.6e-6f, 1.8e-6f},
//       {0.72e-6f, 1.8e-6f, 29.3e-6f}}};
static const float kf = 2.25e-08f; // the coefficient parameter of the square model: thrust (N) vs. rotor_speed (rad/sec), for a single motor
static const float kt = 1.34e-10f; // the coefficient parameter of the square model: torque (N*m) vs. rotor_speed (rad/sec), for a single motor, kt = 0.00596 * kf

// define the LQR controller variables
// general control variables
// the Kinf constant matrix of the LQR controller
static const float Kinf[4][12] = {
    {-4.44452152e+02f, 4.33024173e+02f, 4.98479582e+02f, -2.38404006e+03f, -2.45304027e+03f, -4.69379790e+02f, -6.48139092e+02f, 6.31066494e+02f, 5.87491481e+02f, -4.44537802e+02f, -4.58722942e+02f, -4.84044820e+02f},
    {4.38138384e+02f, 4.10781651e+02f, 4.98479582e+02f, -2.25180314e+03f, 2.41653293e+03f, 4.81660322e+02f, 6.38815301e+02f, 5.97977153e+02f, 5.87491481e+02f, -4.17792135e+02f, 4.51574532e+02f, 4.96197310e+02f},
    {4.04226800e+02f, -4.17155328e+02f, 4.98479582e+02f, 2.28863647e+03f, 2.21151366e+03f, -5.11962778e+02f, 5.88138453e+02f, -6.07387985e+02f, 5.87491481e+02f, 4.25000554e+02f, 4.09323963e+02f, -5.26179601e+02f},
    {-3.97913033e+02f, -4.26650497e+02f, 4.98479582e+02f, 2.34720674e+03f, -2.17500632e+03f, 4.99682247e+02f, -5.78814662e+02f, -6.21655663e+02f, 5.87491481e+02f, 4.37329383e+02f, -4.02175552e+02f, 5.14027111e+02f}};
static float hover_speeds[4]; // the angular speeds (in rad/sec) of the 4 rotors, for the crazyflie to hover
void init_hover()
{
  for (int i = 0; i < 4; ++i)
    hover_speeds[i] = sqrtf(M * g / 4.0f / kf); //  almost 1900.f rad/sec
}
static float control_speeds[4] = {0.0f, 0.0f, 0.0f, 0.0f};  // the controlled angular speeds (in rad/sec) of the 4 rotors
static float control_thrusts[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // the controlled thrusts (in N) of the 4 rotors

// for the forces-torques control mode (controlModeForceTorque)
static float control_thrust_total = 0.0f;                   // the total thrust (in N) generated by the 4 rotors
static vec_3_t control_body_torques = {{0.0f, 0.0f, 0.0f}}; // the body torques for each axis (x, y, z)

// for the normalized forces control mode (controlModeForce)
static const float max_thrust = 0.156f;                      // the maximum thrust (in N) generated by only 1 motor
static float control_norm_thrusts[4] = {0.f, 0.f, 0.f, 0.f}; // the controlled normalized thrusts, in [0, 1], of the 4 rotors

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

// clamp a float value betweeen 0 and 1
float clamp_to_unit_interval(float value)
{
  if (value < 0.0f)
    return 0.0f;
  if (value > 1.0f)
    return 1.0f;
  return value;
}

// convert roll, pitch, yaw angles to the corresponding quaternion
quat_t qrpy_quat(vec_3_t rpy) // rpy carries the roll, pitch, and yaw angles in radians
{
  quat_t q;
  float cr = cosf(0.5f * rpy.x);
  float sr = sinf(0.5f * rpy.x);
  float cp = cosf(0.5f * rpy.y);
  float sp = sinf(0.5f * rpy.y);
  float cy = cosf(0.5f * rpy.z);
  float sy = sinf(0.5f * rpy.z);

  q.q[0] = cr * cp * cy + sr * sp * sy;
  q.q[1] = sr * cp * cy - cr * sp * sy;
  q.q[2] = cr * sp * cy + sr * cp * sy;
  q.q[3] = cr * cp * sy - sr * sp * cy;

  return q;
}

// convert a quaternion to the corresponding rotation matrix
mat_3_3_t Rq_mat(quat_t q) // q is the quaternion
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

// compute the transpose of a 3x3 matrix
static mat_3_3_t mat33_transpose(mat_3_3_t A)
{
  mat_3_3_t At;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      At.m[i][j] = A.m[j][i];
  return At;
}

// compute the product A * b, where A is a 3x3 matrix and b is a 3x1 column vector
static vec_3_t mat33_vec3_multiply(mat_3_3_t A, vec_3_t b)
{
  vec_3_t result;
  for (int i = 0; i < 3; ++i)
  {
    result.v[i] = A.m[i][0] * b.v[0] + A.m[i][1] * b.v[1] + A.m[i][2] * b.v[2];
  }
  return result;
}

// compute the product A * b, where A is a 4x12 matrix and b is a 12x1 column vector
static void mat412_vec12_multiply(const float A[4][12], const float b[12], float *result)
{
  for (int i = 0; i < 3; ++i)
  {
    float sum = 0.0f;
    for (int j = 0; j < 12; ++j)
    {
      sum += A[i][j] * b[j];
    }
    result[i] = sum;
  }
}

// // compute the product A * b, where A is a MxN matrix and b is a Nx1 column vector
// static void mat_vec_multiply(const float **A, int M, int N, const float *b, float *result)
// {
//   for (int i = 0; i < M; ++i)
//   {
//     float sum = 0.0f;
//     for (int j = 0; j < N; ++j)
//     {
//       sum += A[i][j] * b[j];
//     }
//     result[i] = sum;
//   }
// }

// compute the product A * B, where A, B are 3x3 matrices
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

// compute the right minus operator of the SO3 group
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

// compute the state error (state_current - state_reference)
static void compute_state_error(const cf_state_t state_cur, const cf_state_t state_ref, float *error)
{
  // extract state components
  vec_3_t rw_1 = state_cur.rw;
  quat_t qwb_1 = state_cur.qwb;
  vec_3_t vb_1 = state_cur.vb;
  vec_3_t ob_1 = state_cur.ob;

  vec_3_t rw_2 = state_ref.rw;
  quat_t qwb_2 = state_ref.qwb;
  vec_3_t vb_2 = state_ref.vb;
  vec_3_t ob_2 = state_ref.ob;

  // rotation matrices
  mat_3_3_t Rwb_1 = Rq_mat(qwb_1);
  mat_3_3_t Rwb_2 = Rq_mat(qwb_2);

  // compute errors
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

  // concatenate into error vector
  for (int i = 0; i < 3; i++)
  {
    error[i] = rw_error[i];
    error[i + 3] = Rwb_error.v[i];
    error[i + 6] = vb_error[i];
    error[i + 9] = ob_error[i];
  }
}

static bool isInit = false;

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
  // controllerPidInit();
  attitudeControlInit(ATTITUDE_UPDATE_DT);
  positionControlInit();
  return;
}

bool controllerOutOfTreeTest(void)
{
  // return controllerPidTest();
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

  // rotor_speed (rad/sec) vs. PWM:         omegar = sqrt(8e-4f * PWM^2 + 53.33 * PWM)
  // PWM vs. rotor_speed (rad/sec):         PWM = -33333 + sqrt(1250 * omegar^2 + 11111*10^5)
  // thrust (N) vs. rotor_speed (rad/sec):  Fi = kf * ui^2 = (9/4)*10^(-8) * ui^2

  // get the current state of the crazyflie
  vec_3_t vw_cur = {{state->velocity.x, state->velocity.y, state->velocity.z}};
  quat_t qwb_cur = {{state->attitudeQuaternion.w, state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z}};
  vec_3_t rpyb_cur = {{radians(state->attitude.roll), -radians(state->attitude.pitch), radians(state->attitude.yaw)}};
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
      .vb = {{vb_cur.x,
              vb_cur.y,
              vb_cur.z}},
      .ob = {{radians(sensors->gyro.x),
              radians(sensors->gyro.y),
              radians(sensors->gyro.z)}},
  };

  // get the reference state of the crazyflie (in which state it should end up)
  vec_3_t rpyb_ref = {{0., 0., radians(setpoint->attitude.yaw)}};
  // vec_3_t rpyb_ref = {{radians(setpoint->attitude.roll), -radians(setpoint->attitude.pitch), radians(setpoint->attitude.yaw)}};
  quat_t qwb_ref = qrpy_quat(rpyb_ref);
  cf_state_t state_ref = {
      .rw = {{setpoint->position.x,
              setpoint->position.y,
              setpoint->position.z}},
      .qwb = {{qwb_ref.w,
               qwb_ref.x,
               qwb_ref.y,
               qwb_ref.z}},
      .vb = {{0.0, 0.0, 0.0}},
      .ob = {{0.0, 0.0, 0.0}},
  };

  // compute the control vector signal
  float state_error[3];
  compute_state_error(state_cur, state_ref, state_error);
  float control_feedback[4];
  mat412_vec12_multiply(Kinf, state_error, control_feedback);
  for (int i = 0; i < 4; ++i)
  {
    control_speeds[i] = hover_speeds[i] - control_feedback[i];
    control_thrusts[i] = kf * powf(control_speeds[i], 2.0f);
  }
  // using the forces-torques control mode
  control->controlMode = controlModeForceTorque;
  control_thrust_total = 0.0;
  for (int i = 0; i < 4; ++i)
  {
    control_thrust_total += control_thrusts[i];
  }
  control_body_torques.x = L * kf * (powf(control_speeds[0], 2.0f) - powf(control_speeds[2], 2.0f));
  control_body_torques.y = L * kf * (powf(control_speeds[3], 2.0f) - powf(control_speeds[1], 2.0f));
  control_body_torques.z = kt * (powf(control_speeds[1], 2.0f) + powf(control_speeds[3], 2.0f) - powf(control_speeds[0], 2.0f) - powf(control_speeds[2], 2.0f));
  control->thrustSi = control_thrust_total;
  control->torqueX = control_body_torques.x;
  control->torqueY = control_body_torques.y;
  control->torqueZ = control_body_torques.z;
  // // using the normalized forces control mode
  // control->controlMode = controlModeForce;
  // for (int i = 0; i < 4; ++i)
  // {
  //   control_norm_thrusts[i] = clamp_to_unit_interval(control_thrusts[i] / max_thrust);
  //   control->normalizedForces[i] = control_norm_thrusts[i];
  // }

  // print some data for debugging
  if (debug_print_counter % 1000 == 0)
  {
    debug_print_counter = 0;
    DEBUG_PRINT("Current state [%lu]: rw(m) = [%.3f, %.3f, %.3f],\t\t qwb = [%.3f, %.3f, %.3f, %.3f],\t\t rpyb(deg) = [%.3f, %.3f, %.3f],\n\t\t vb(m/s) = [%.3f, %.3f, %.3f],\t\t ob(deg/s) = [%.3f, %.3f, %.3f]\n",
                tick,
                (double)state_cur.rw.x, (double)state_cur.rw.y, (double)state_cur.rw.z,
                (double)state_cur.qwb.w, (double)state_cur.qwb.x, (double)state_cur.qwb.y, (double)state_cur.qwb.z,
                (double)degrees(rpyb_cur.x), (double)degrees(rpyb_cur.y), (double)degrees(rpyb_cur.z),
                (double)state_cur.vb.x, (double)state_cur.vb.y, (double)state_cur.vb.z,
                (double)degrees(state_cur.ob.x), (double)degrees(state_cur.ob.y), (double)degrees(state_cur.ob.z));
    DEBUG_PRINT("Reference state [%lu]: rw(m) = [%.3f, %.3f, %.3f],\t\t qwb = [%.3f, %.3f, %.3f, %.3f],\t\t rpyb(deg) = [%.3f, %.3f, %.3f]\n",
                tick,
                (double)state_ref.rw.x, (double)state_ref.rw.y, (double)state_ref.rw.z,
                (double)state_ref.qwb.w, (double)state_ref.qwb.x, (double)state_ref.qwb.y, (double)state_ref.qwb.z,
                (double)degrees(rpyb_ref.x), (double)degrees(rpyb_ref.y), (double)degrees(rpyb_ref.z));
    DEBUG_PRINT("State error [%lu]: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n",
                tick,
                (double)state_error[0], (double)state_error[1], (double)state_error[2],
                (double)state_error[3], (double)state_error[4], (double)state_error[5],
                (double)state_error[6], (double)state_error[7], (double)state_error[8],
                (double)state_error[9], (double)state_error[10], (double)state_error[11]);
    DEBUG_PRINT("Motors speeds (rad/sec) [%lu]: m1 = %.0f,\t\t m2 = %.0f,\t\t m3 = %.0f,\t\t m4 = %.0f\n",
                tick,
                (double)control_speeds[0], (double)control_speeds[1], (double)control_speeds[2], (double)control_speeds[3]);
    DEBUG_PRINT("Motors thrusts (N) [%lu]: m1 = %.3f (%.3f),\t\t m2 = %.3f (%.3f),\t\t m3 = %.3f (%.3f),\t\t m4 = %.3f (%.3f)\n",
                tick,
                (double)control_thrusts[0], (double)control_norm_thrusts[0], (double)control_thrusts[1], (double)control_norm_thrusts[1],
                (double)control_thrusts[2], (double)control_norm_thrusts[2], (double)control_thrusts[3], (double)control_norm_thrusts[3]);
    DEBUG_PRINT("\n");
  }
  debug_print_counter += 1;

  // // store data to the logging variables
  // posw_x = state_cur.rw.x;
  // posw_y = state_cur.rw.y;
  // posw_z = state_cur.rw.z;
  // qwb_w = state_cur.qwb.w;
  // qwb_x = state_cur.qwb.x;
  // qwb_y = state_cur.qwb.y;
  // qwb_z = state_cur.qwb.z;
  // roll = rpyb_cur.x;
  // pitch = rpyb_cur.y;
  // yaw = rpyb_cur.z;
  // velb_x = state_cur.vb.x;
  // velb_y = state_cur.vb.y;
  // velb_z = state_cur.vb.z;
  // omegab_x = state_cur.ob.x;
  // omegab_y = state_cur.ob.y;
  // omegab_z = state_cur.ob.z;
  // posw_x_ref = state_ref.rw.x;
  // posw_y_ref = state_ref.rw.y;
  // posw_z_ref = state_ref.rw.z;
  // yaw_ref = rpyb_ref.z;
  // ctrl_speed_m1 = control_speeds[0];
  // ctrl_speed_m2 = control_speeds[1];
  // ctrl_speed_m3 = control_speeds[2];
  // ctrl_speed_m4 = control_speeds[3];
  // ctrl_thrust_m1 = control_thrusts[0];
  // ctrl_thrust_m2 = control_thrusts[1];
  // ctrl_thrust_m3 = control_thrusts[2];
  // ctrl_thrust_m4 = control_thrusts[3];
  // ctrl_norm_thrust_m1 = control_norm_thrusts[0];
  // ctrl_norm_thrust_m2 = control_norm_thrusts[1];
  // ctrl_norm_thrust_m3 = control_norm_thrusts[2];
  // ctrl_norm_thrust_m4 = control_norm_thrusts[3];

  // controllerPid(control, setpoint, sensors, state, tick);
}

LOG_GROUP_START(LQRcontroller)
// /**
//  * @brief Position x (m) - world frame
//  */
// LOG_ADD(LOG_FLOAT, posw_x, &posw_x)
// /**
//  * @brief Position y (m) - world frame
//  */
// LOG_ADD(LOG_FLOAT, posw_y, &posw_y)
// /**
//  * @brief Position z (m) - world frame
//  */
// LOG_ADD(LOG_FLOAT, posw_z, &posw_z)
// /**
//  * @brief Body Quaternion w - world frame
//  */
// LOG_ADD(LOG_FLOAT, qwb_w, &qwb_w)
// /**
//  * @brief Body Quaternion x - world frame
//  */
// LOG_ADD(LOG_FLOAT, qwb_x, &qwb_x)
// /**
//  * @brief Body Quaternion y - world frame
//  */
// LOG_ADD(LOG_FLOAT, qwb_y, &qwb_y)
// /**
//  * @brief Body Quaternion z - world frame
//  */
// LOG_ADD(LOG_FLOAT, qwb_z, &qwb_z)
// /**
//  * @brief Roll angle (deg)
//  */
// LOG_ADD(LOG_FLOAT, roll, &roll)
// /**
//  * @brief Pitch angle (deg)
//  */
// LOG_ADD(LOG_FLOAT, pitch, &pitch)
// /**
//  * @brief Yaw angle (deg)
//  */
// LOG_ADD(LOG_FLOAT, yaw, &yaw)
// /**
//  * @brief Linear velocity x (m/s) - body frame
//  */
// LOG_ADD(LOG_FLOAT, velb_x, &velb_x)
// /**
//  * @brief Linear velocity y (m/s) - body frame
//  */
// LOG_ADD(LOG_FLOAT, velb_y, &velb_y)
// /**
//  * @brief Linear velocity z (m/s) - body frame
//  */
// LOG_ADD(LOG_FLOAT, velb_z, &velb_z)
// /**
//  * @brief Angular velocity x (deg/s) - body frame
//  */
// LOG_ADD(LOG_FLOAT, omegab_x, &omegab_x)
// /**
//  * @brief Angular velocity y (deg/s) - body frame
//  */
// LOG_ADD(LOG_FLOAT, omegab_y, &omegab_y)
// /**
//  * @brief Angular velocity z (deg/s) - body frame
//  */
// LOG_ADD(LOG_FLOAT, omegab_z, &omegab_z)
// /**
//  * @brief Reference position x (m) - world frame
//  */
// LOG_ADD(LOG_FLOAT, posw_x_ref, &posw_x_ref)
// /**
//  * @brief Reference position y (m) - world frame
//  */
// LOG_ADD(LOG_FLOAT, posw_y_ref, &posw_y_ref)
// /**
//  * @brief Reference position z (m) - world frame
//  */
// LOG_ADD(LOG_FLOAT, posw_z_ref, &posw_z_ref)
// /**
//  * @brief Control speed (rad/sec) for motor 1
//  */
// LOG_ADD(LOG_FLOAT, ctrl_speed_m1, &ctrl_speed_m1)
// /**
//  * @brief Control speed (rad/sec) for motor 2
//  */
// LOG_ADD(LOG_FLOAT, ctrl_speed_m2, &ctrl_speed_m2)
// /**
//  * @brief Control speed (rad/sec) for motor 3
//  */
// LOG_ADD(LOG_FLOAT, ctrl_speed_m3, &ctrl_speed_m3)
// /**
//  * @brief Control speed (rad/sec) for motor 4
//  */
// LOG_ADD(LOG_FLOAT, ctrl_speed_m4, &ctrl_speed_m4)
// /**
//  * @brief Control thrust (N) for motor 1
//  */
// LOG_ADD(LOG_FLOAT, ctrl_thrust_m1, &ctrl_thrust_m1)
// /**
//  * @brief Control thrust (N) for motor 2
//  */
// LOG_ADD(LOG_FLOAT, ctrl_thrust_m2, &ctrl_thrust_m2)
// /**
//  * @brief Control thrust (N) for motor 3
//  */
// LOG_ADD(LOG_FLOAT, ctrl_thrust_m3, &ctrl_thrust_m3)
// /**
//  * @brief Control thrust (N) for motor 4
//  */
// LOG_ADD(LOG_FLOAT, ctrl_thrust_m4, &ctrl_thrust_m4)
// /**
//  * @brief Control normalized thrust for motor 1
//  */
// LOG_ADD(LOG_FLOAT, ctrl_norm_thrust_m1, &ctrl_norm_thrust_m1)
// /**
//  * @brief Control normalized thrust for motor 2
//  */
// LOG_ADD(LOG_FLOAT, ctrl_norm_thrust_m2, &ctrl_norm_thrust_m2)
// /**
//  * @brief Control normalized thrust for motor 3
//  */
// LOG_ADD(LOG_FLOAT, ctrl_norm_thrust_m3, &ctrl_norm_thrust_m3)
// /**
//  * @brief Control normalized thrust for motor 4
//  */
// LOG_ADD(LOG_FLOAT, ctrl_norm_thrust_m4, &ctrl_norm_thrust_m4)
LOG_GROUP_STOP(LQRcontroller)
