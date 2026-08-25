/**
  * @file       chassis_power_control.c/h
  * @brief      Wheeled-leg chassis power control
  *             Based on HKUST ENTERPRIZE 2024 PowerModule approach.
  *
  * @note  CRITICAL FIX vs original DJI template:
  *        This version constrains wheel_T (torque, Nm) instead of
  *        given_current.  The original wrote to given_current which
  *        was immediately overwritten by the downstream conversion
  *            given_current = wheel_T / 0.000396211f
  *        making the original power control completely ineffective.
  */
#ifndef CHASSIS_POWER_CONTROL_H
#define CHASSIS_POWER_CONTROL_H

#include "chassisR_task.h"
#include "main.h"

/* ================================================================
 * Tunable model parameters
 * After flashing, monitor power_dbg.cmd_power vs real chassis power
 * (from referee/capacitor) and adjust k1, k2, k3 until they match.
 * ================================================================ */
#define POWER_K1            0.5f    /* |ω| loss coefficient   (friction+iron, W·s/rad)  */
#define POWER_K2            0.78f   /* τ² loss coefficient    (copper loss,   W/Nm²)     */
#define POWER_K3            4.0f    /* board static loss      (W) — measure at motor idle */

/* ================================================================
 * Energy-loop parameters
 * ================================================================ */
#define POWER_HALF_BUDGET   0.5f    /* this board's share of total referee power limit   */
#define POWER_BUFF_TARGET   30.0f   /* target buffer energy (J); tune to ~half of 60J    */
#define POWER_KP_ENERGY     2.0f    /* energy-loop Kp on sqrt error  (W / sqrt(J))       */
#define POWER_KD_ENERGY     0.05f   /* energy-loop Kd on sqrt error rate                 */
#define POWER_MIN_PMAX      12.0f   /* hard floor on P_max (W)                           */
#define POWER_NO_REF_LIMIT  22.0f   /* fallback P_max when referee is offline (W)        */

/* ================================================================
 * Physical hard limit
 * ================================================================ */
#define WHEEL_T_LIMIT       4.2f    /* max wheel torque (Nm); matches mySaturate in task  */

/* ================================================================
 * Debug structure — add all fields to Vofa+ or J-Scope
 * ================================================================ */
typedef struct
{
    float cmd_power;    /* model-predicted power before constraint (W)   */
    float P_max;        /* current power budget this cycle (W)           */
    float tau_in;       /* wheel_T from LQR before limiting (Nm)         */
    float tau_out;      /* wheel_T after limiting (Nm)                   */
    float e_sqrt;       /* sqrt energy error: sqrt(target)-sqrt(feedback) */
    float buf_energy;   /* raw chassis buffer energy feedback (J)        */
    uint8_t active;     /* 1 = limiting is currently active              */
} power_debug_t;

extern power_debug_t power_dbg;
extern void chassis_power_control(chassis_t *chassis_power_control);

#endif
