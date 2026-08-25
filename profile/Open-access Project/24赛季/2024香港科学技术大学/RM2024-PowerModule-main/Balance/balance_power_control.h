/**
 * @file    balance_power_control.h
 * @brief   Wheeled-leg (balance) chassis power control — C implementation
 *          Reference: HKUST ENTERPRIZE 2024 PowerModule (Balance folder)
 *
 * ── What this module does ────────────────────────────────────────────────
 *  1. Motor power model   P = τ·ω + k1·|ω| + k2·τ² + k3
 *     Predicts power consumption from motor torque command and speed
 *     without needing current-sensor hardware.
 *
 *  2. Energy loop (sqrt-based, HKUST approach)
 *     e     = √E_target − √E_feedback
 *     P_max = P_ref − Kp·e − Kd·ė
 *     Non-linear: cuts power sharply when buffer is nearly empty;
 *     allows slight boost when buffer is full.
 *
 *  3. Quadratic solver → max wheel torque
 *     Solves k2·τ²+ ω·τ + (k1|ω|+k3−P_max) = 0 for the largest τ
 *     that keeps power within budget. Imaginary-root fallback (−B/2A)
 *     ensures the motor always receives a valid command.
 *
 * ── Integration checklist ────────────────────────────────────────────────
 *  □ Provide the four accessor macros at the bottom of this header
 *    (or replace function-body calls if your project uses a different API).
 *  □ Call balance_power_control_init() once at startup.
 *  □ Call balance_power_control_update() every 1 ms inside your
 *    chassis control loop, BEFORE converting wheel_T → CAN current.
 *  □ After the call, read back wheel_T — it will be ≤ the power limit.
 *  □ Apply the same file to the OTHER board (left/right) independently.
 *
 * ── Critical bug this module fixes (vs. DJI template) ────────────────────
 *  The original DJI template wrote the limited value to given_current,
 *  which was immediately overwritten by  given_current = wheel_T / K
 *  in the task file, making power control completely ineffective.
 *  This version writes back to wheel_T, which is the correct target.
 */

#ifndef BALANCE_POWER_CONTROL_H
#define BALANCE_POWER_CONTROL_H

#include <stdint.h>

/* ══════════════════════════════════════════════════════════════════════════
 * Section 1 — Motor power model parameters
 *
 * Calibration procedure:
 *   k3: disable all motors, read chassis power from referee → that value is k3.
 *   k1, k2: run chassis normally, compare power_dbg.cmd_power with the
 *           real chassis power from referee until curves match.
 * ══════════════════════════════════════════════════════════════════════════ */
#define BPC_K1          0.5f    /* |ω| loss coeff  — friction + iron loss  (W·s/rad)  */
#define BPC_K2          0.78f   /* τ² loss coeff   — copper / resistive loss (W/Nm²)  */
#define BPC_K3          4.0f    /* board static loss (W) — calibrate at motor idle     */

/* ══════════════════════════════════════════════════════════════════════════
 * Section 2 — Energy loop parameters
 * ══════════════════════════════════════════════════════════════════════════ */
#define BPC_HALF_BUDGET     0.5f    /* this board's share of total referee power limit  */
#define BPC_BUFF_TARGET     30.0f   /* target buffer energy (J); ≈ half of 60 J total  */
#define BPC_KP_ENERGY       2.0f    /* energy-loop Kp on sqrt error  (W / √J)          */
#define BPC_KD_ENERGY       0.05f   /* energy-loop Kd on sqrt error rate               */
#define BPC_MIN_PMAX        12.0f   /* hard floor for P_max (W)                        */
#define BPC_NO_REF_LIMIT    22.0f   /* conservative fallback when referee is offline(W) */

/* ══════════════════════════════════════════════════════════════════════════
 * Section 3 — Physical hard limit
 * ══════════════════════════════════════════════════════════════════════════ */
#define BPC_WHEEL_T_LIMIT   4.2f    /* maximum wheel torque (Nm); match mySaturate()   */

/* ══════════════════════════════════════════════════════════════════════════
 * Section 4 — Real-time debug structure
 * Monitor all fields via Vofa+, J-Scope, or any serial plotter.
 * ══════════════════════════════════════════════════════════════════════════ */
typedef struct
{
    float   cmd_power;  /* model-predicted power before constraint  (W)           */
    float   P_max;      /* power budget computed this cycle         (W)           */
    float   tau_in;     /* wheel_T from LQR / controller, pre-limit (Nm)          */
    float   tau_out;    /* wheel_T after power limiting             (Nm)          */
    float   e_sqrt;     /* sqrt energy error: √target − √feedback                 */
    float   buf_energy; /* raw buffer-energy feedback               (J)           */
    uint8_t active;     /* 1 = limiting is currently active                       */
} bpc_debug_t;

extern bpc_debug_t bpc_dbg;

/* ══════════════════════════════════════════════════════════════════════════
 * Section 5 — Project-specific accessor macros
 *
 * Replace these four macros to match your project's API.
 * The defaults match the 1.12 project structure.
 * ══════════════════════════════════════════════════════════════════════════ */

/*  float chassis_power, float chassis_power_buffer  ← outputs */
#define BPC_GET_POWER_AND_BUFFER(pwr, buf) \
    get_chassis_power_and_buffer((pwr), (buf))

/*  uint16_t* max_power_limit  ← output */
#define BPC_GET_MAX_POWER(p_limit) \
    get_chassis_max_power((p_limit))

/*  returns non-zero when referee is offline */
#define BPC_REFEREE_IS_OFFLINE() \
    toe_is_error(REFEREE_TOE)

/*  wheel angular velocity field (rad/s) in your motor struct  */
#define BPC_WHEEL_OMEGA(chassis)  ((chassis)->wheel_motor[0].vel)

/*  wheel torque command field (Nm) in your motor struct        */
#define BPC_WHEEL_TAU(chassis)    ((chassis)->wheel_motor[0].wheel_T)

/* ══════════════════════════════════════════════════════════════════════════
 * Section 6 — Public API
 * ══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  One-time initialisation (resets energy-loop state).
 *         Call once before the control loop starts.
 */
void balance_power_control_init(void);

/**
 * @brief  Per-cycle power control update — call every 1 ms.
 *
 * @param  chassis  Pointer to your chassis struct.
 *                  Must expose the fields referenced by the macros above.
 *
 * The function:
 *   1. Reads buffer energy and referee power limit.
 *   2. Computes P_max via the sqrt energy loop.
 *   3. Predicts motor power consumption using the model.
 *   4. If over budget, solves the quadratic and writes a limited
 *      wheel_T back via BPC_WHEEL_TAU(chassis).
 *   5. Updates bpc_dbg for monitoring.
 *
 * Call this BEFORE the conversion  given_current = wheel_T / K
 * so the limited torque is the one sent over CAN.
 */
void balance_power_control_update(void *chassis);

#endif /* BALANCE_POWER_CONTROL_H */
