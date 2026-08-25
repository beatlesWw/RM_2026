/**
 * @file    balance_power_control.c
 * @brief   Wheeled-leg (balance) chassis power control — C implementation
 *          Reference: HKUST ENTERPRIZE 2024 PowerModule (Balance folder)
 *
 * ── Algorithm overview ───────────────────────────────────────────────────
 *
 *  Power model (single wheel, HKUST formulation):
 *
 *      P_i = τ·ω + k1·|ω| + k2·τ² + k3
 *
 *  where:
 *      τ     wheel output-shaft torque command (Nm)
 *      ω     wheel output-shaft angular velocity (rad/s)
 *      k1    speed-proportional loss (mechanical friction, iron loss)
 *      k2    torque-squared loss (copper / I²R loss)
 *      k3    board static loss (measured with motors disabled)
 *
 *  Energy loop (non-linear, HKUST approach):
 *
 *      e     = √E_target − √E_feedback
 *      P_max = P_ref − Kp·e − Kd·ė
 *
 *  Using √E instead of E makes the response non-linear:
 *  the controller cuts power sharply when energy is nearly gone,
 *  while allowing a slight boost when the buffer is full.
 *
 *  Quadratic constraint (if P_cmd > P_max):
 *
 *      k2·τ² + ω·τ + (k1·|ω| + k3 − P_max) = 0
 *      A = k2,  B = ω,  C = k1·|ω| + k3 − P_max
 *
 *  Discriminant cases:
 *      Δ < 0 : no real root → use vertex −B/(2A) as safest approximation
 *      Δ ≥ 0 : two roots → pick the root whose sign matches the original
 *               torque command (preserves motion direction)
 *
 * ── Calibration guide ────────────────────────────────────────────────────
 *
 *  Step 1 — Measure k3 (static board loss)
 *    Disable all motors (start_flag = 0).
 *    Read chassis power from referee system.
 *    Set BPC_K3 = that measured value (typically 3–6 W).
 *
 *  Step 2 — Verify model accuracy
 *    Run the robot normally. In Vofa+/J-Scope plot:
 *      bpc_dbg.cmd_power   (model prediction)
 *      chassis_power       (referee real-time measurement)
 *    Adjust BPC_K1 and BPC_K2 until the two curves overlap.
 *    Typical converged values: K1 ≈ 0.3–0.8,  K2 ≈ 0.5–1.2
 *
 *  Step 3 — Tune the energy loop
 *    Watch bpc_dbg.buf_energy.  Target: stays near BPC_BUFF_TARGET.
 *    - Keeps draining to 0  → increase BPC_KP_ENERGY
 *    - Oscillates around target → decrease BPC_KP_ENERGY,
 *                                  increase BPC_KD_ENERGY
 *    - Robot feels sluggish     → decrease BPC_KP_ENERGY or
 *                                  raise BPC_BUFF_TARGET
 */

#include "balance_power_control.h"
#include <math.h>   /* sqrtf, fabsf */

/* ── Global debug struct ─────────────────────────────────────────────── */
bpc_debug_t bpc_dbg;

/* ── Module-private state ────────────────────────────────────────────── */
static float s_last_e_sqrt = 0.0f;

/* ═══════════════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════════════ */

void balance_power_control_init(void)
{
    s_last_e_sqrt = 0.0f;

    bpc_dbg.cmd_power  = 0.0f;
    bpc_dbg.P_max      = 0.0f;
    bpc_dbg.tau_in     = 0.0f;
    bpc_dbg.tau_out    = 0.0f;
    bpc_dbg.e_sqrt     = 0.0f;
    bpc_dbg.buf_energy = 0.0f;
    bpc_dbg.active     = 0U;
}

void balance_power_control_update(void *chassis)
{
    float chassis_power        = 0.0f;
    float chassis_power_buffer = 0.0f;
    unsigned short ref_max_power = 45U;   /* default referee power limit (W) */

    /* ------------------------------------------------------------------
     * Step 1: Read feedback from referee system
     * ------------------------------------------------------------------ */
    BPC_GET_POWER_AND_BUFFER(&chassis_power, &chassis_power_buffer);

    /* ------------------------------------------------------------------
     * Step 2: Compute this board's power budget via the energy loop
     *
     *   P_ref = total_referee_limit × HALF_BUDGET   (split across 2 boards)
     *   e     = √E_target − √E_feedback
     *   P_max = P_ref − Kp·e − Kd·ė
     * ------------------------------------------------------------------ */
    float P_ref;
    if (BPC_REFEREE_IS_OFFLINE())
    {
        /* Referee offline: use safe conservative fallback */
        P_ref = BPC_NO_REF_LIMIT;
    }
    else
    {
        BPC_GET_MAX_POWER(&ref_max_power);
        P_ref = (float)ref_max_power * BPC_HALF_BUDGET;
    }

    float buf_safe    = (chassis_power_buffer > 0.0f) ? chassis_power_buffer : 0.0f;
    float sqrt_target = sqrtf(BPC_BUFF_TARGET);
    float sqrt_fb     = sqrtf(buf_safe);
    float e_sqrt      = sqrt_target - sqrt_fb;

    /* Derivative term — multiply by 1000 to convert per-tick to per-second
     * (assumes this function is called at 1 kHz)                          */
    float de_sqrt  = (e_sqrt - s_last_e_sqrt) * 1000.0f;
    s_last_e_sqrt  = e_sqrt;

    float P_max = P_ref - BPC_KP_ENERGY * e_sqrt - BPC_KD_ENERGY * de_sqrt;
    if (P_max < BPC_MIN_PMAX)
        P_max = BPC_MIN_PMAX;

    /* ------------------------------------------------------------------
     * Step 3: Read wheel state
     *   omega : output-shaft angular velocity  (rad/s)
     *   tau   : torque command from LQR/PID    (Nm)
     * ------------------------------------------------------------------ */
    float omega = BPC_WHEEL_OMEGA(chassis);
    float tau   = BPC_WHEEL_TAU(chassis);

    /* ------------------------------------------------------------------
     * Step 4: Predict power
     *   P = τ·ω + k1·|ω| + k2·τ² + k3
     * ------------------------------------------------------------------ */
    float cmd_power = tau * omega
                    + BPC_K1 * fabsf(omega)
                    + BPC_K2 * tau * tau
                    + BPC_K3;

    /* Update debug (always, even on early return) */
    bpc_dbg.cmd_power  = cmd_power;
    bpc_dbg.P_max      = P_max;
    bpc_dbg.tau_in     = tau;
    bpc_dbg.e_sqrt     = e_sqrt;
    bpc_dbg.buf_energy = chassis_power_buffer;

    /* ------------------------------------------------------------------
     * Step 5: Within budget — pass through unchanged
     * ------------------------------------------------------------------ */
    if (cmd_power <= P_max)
    {
        bpc_dbg.tau_out = tau;
        bpc_dbg.active  = 0U;
        return;
    }

    /* ------------------------------------------------------------------
     * Step 6: Over budget — solve quadratic for max feasible τ
     *
     *   k2·τ² + ω·τ + (k1·|ω| + k3 − P_max) = 0
     *
     *   A = k2
     *   B = ω
     *   C = k1·|ω| + k3 − P_max
     *   Δ = B² − 4AC
     *
     * Δ < 0  (imaginary roots):
     *   No torque satisfies the power budget at this speed.
     *   Use the vertex  τ = −B/(2A)  as the closest approximation.
     *   This guarantees the motor still receives a bounded command.
     *
     * Δ ≥ 0  (real roots):
     *   Two solutions exist. Both satisfy P = P_max.
     *   Pick the root whose sign matches the original command to
     *   preserve the direction of motion.
     * ------------------------------------------------------------------ */
    float A     = BPC_K2;
    float B     = omega;
    float C     = BPC_K1 * fabsf(omega) + BPC_K3 - P_max;
    float Delta = B * B - 4.0f * A * C;

    float tau_limited;
    if (Delta <= 0.0f)
    {
        /* Vertex approximation — no real root */
        tau_limited = -B / (2.0f * A);
    }
    else
    {
        float sqrt_d = sqrtf(Delta);
        float tau1   = (-B + sqrt_d) / (2.0f * A);
        float tau2   = (-B - sqrt_d) / (2.0f * A);

        /* Choose the root that matches the original torque direction */
        tau_limited  = (tau >= 0.0f) ? tau1 : tau2;
    }

    /* ------------------------------------------------------------------
     * Step 7: Clamp to physical limit and write back
     *
     * IMPORTANT — call this function BEFORE the conversion
     *     given_current = wheel_T / torque_constant
     * so the CAN command uses the power-limited torque value.
     * ------------------------------------------------------------------ */
    if (tau_limited >  BPC_WHEEL_T_LIMIT) tau_limited =  BPC_WHEEL_T_LIMIT;
    if (tau_limited < -BPC_WHEEL_T_LIMIT) tau_limited = -BPC_WHEEL_T_LIMIT;

    BPC_WHEEL_TAU(chassis) = tau_limited;

    bpc_dbg.tau_out = tau_limited;
    bpc_dbg.active  = 1U;
}
