/**
  * @file       chassis_power_control.c
  * @brief      Wheeled-leg chassis power control
  *             Based on HKUST ENTERPRIZE 2024 PowerModule.
  *
  * Improvements over original DJI template
  * -----------------------------------------
  * 1. CRITICAL BUG FIX: constrains wheel_T (Nm) instead of given_current.
  *    Original wrote to given_current which was IMMEDIATELY overwritten by
  *        given_current = wheel_T / 0.000396211f
  *    in chassisR_task.c, making the original power control do nothing.
  *
  * 2. sqrt-based non-linear energy loop (HKUST approach):
  *        e = sqrt(E_target) - sqrt(E_feedback)
  *        P_max = P_ref - Kp*e - Kd*e_dot
  *    Aggressively cuts power when energy is low; allows boost when full.
  *
  * 3. Imaginary-root fallback: uses vertex approximation (-B/2A) instead
  *    of silently skipping (which leaves the motor uncontrolled).
  *
  * 4. Referee offline fallback via toe_is_error(REFEREE_TOE).
  *
  * 5. power_debug_t struct for real-time monitoring via Vofa+/J-Scope.
  */
#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"
#include "detect_task.h"

/* Global debug struct — monitor via Vofa+ or J-Scope */
power_debug_t power_dbg;

void chassis_power_control(chassis_t *chassis_power_control)
{
    fp32 chassis_power        = 0.0f;
    fp32 chassis_power_buffer = 0.0f;
    uint16_t ref_max_power    = 45;

    /* ----------------------------------------------------------
     * Step 1: Get referee feedback
     * ---------------------------------------------------------- */
    get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer);

    /* ----------------------------------------------------------
     * Step 2: Compute this board's power budget (energy loop)
     *
     * Using HKUST sqrt-based error for non-linear response:
     *   e     = sqrt(E_target) - sqrt(E_feedback)
     *   P_max = P_ref * HALF_BUDGET - Kp*e - Kd*e_dot
     *
     * Effect: when buffer is nearly empty, error is large and
     * P_max drops sharply; when buffer is full, P_max can rise
     * slightly above the referee limit.
     * ---------------------------------------------------------- */
    fp32 P_ref;
    if (toe_is_error(REFEREE_TOE))
    {
        P_ref = POWER_NO_REF_LIMIT;
    }
    else
    {
        get_chassis_max_power(&ref_max_power);
        P_ref = (fp32)ref_max_power * POWER_HALF_BUDGET;
    }

    static fp32 last_e_sqrt = 0.0f;
    fp32 buf_safe    = (chassis_power_buffer > 0.0f) ? chassis_power_buffer : 0.0f;
    fp32 sqrt_target = sqrtf(POWER_BUFF_TARGET);
    fp32 sqrt_fb     = sqrtf(buf_safe);
    fp32 e_sqrt      = sqrt_target - sqrt_fb;
    fp32 de_sqrt     = (e_sqrt - last_e_sqrt) * 1000.0f; /* scale to per-second */
    last_e_sqrt      = e_sqrt;

    fp32 P_max = P_ref - POWER_KP_ENERGY * e_sqrt - POWER_KD_ENERGY * de_sqrt;
    if (P_max < POWER_MIN_PMAX)
        P_max = POWER_MIN_PMAX;

    /* ----------------------------------------------------------
     * Step 3: Read wheel state from motor feedback
     *   omega : output-shaft angular velocity (rad/s)
     *   tau   : LQR-commanded wheel torque (Nm)
     * ---------------------------------------------------------- */
    fp32 omega = chassis_power_control->wheel_motor[0].vel;    /* rad/s */
    fp32 tau   = chassis_power_control->wheel_motor[0].wheel_T; /* Nm   */

    /* ----------------------------------------------------------
     * Step 4: Predict power with HKUST-style model
     *   P = tau*omega + k1*|omega| + k2*tau^2 + k3
     *
     * Physical interpretation:
     *   tau*omega  : effective mechanical output power
     *   k1*|omega| : speed-proportional loss (friction, iron loss)
     *   k2*tau^2   : torque-squared loss (copper / resistive loss)
     *   k3         : board static loss (measured at motor idle)
     * ---------------------------------------------------------- */
    fp32 cmd_power = tau * omega
                   + POWER_K1 * fabsf(omega)
                   + POWER_K2 * tau * tau
                   + POWER_K3;

    /* Fill debug before potential early return */
    power_dbg.cmd_power  = cmd_power;
    power_dbg.P_max      = P_max;
    power_dbg.tau_in     = tau;
    power_dbg.e_sqrt     = e_sqrt;
    power_dbg.buf_energy = chassis_power_buffer;

    /* ----------------------------------------------------------
     * Step 5: No constraint needed
     * ---------------------------------------------------------- */
    if (cmd_power <= P_max)
    {
        power_dbg.tau_out = tau;
        power_dbg.active  = 0;
        return;
    }

    /* ----------------------------------------------------------
     * Step 6: Solve for max tau given P_max
     *
     * Rearrange model as quadratic in tau:
     *   k2*tau^2 + omega*tau + (k1*|omega| + k3 - P_max) = 0
     *   A = k2,  B = omega,  C = k1*|omega| + k3 - P_max
     *
     * Delta < 0: no real solution → use vertex -B/(2A) as best
     *            approximation (motor still gets a command).
     * Delta >= 0: two roots → pick the one matching the sign of
     *             the original LQR output to preserve direction.
     * ---------------------------------------------------------- */
    fp32 A     = POWER_K2;
    fp32 B     = omega;
    fp32 C     = POWER_K1 * fabsf(omega) + POWER_K3 - P_max;
    fp32 Delta = B * B - 4.0f * A * C;

    fp32 tau_limited;
    if (Delta <= 0.0f)
    {
        tau_limited = -B / (2.0f * A);
    }
    else
    {
        fp32 sqrt_d = sqrtf(Delta);
        fp32 tau1   = (-B + sqrt_d) / (2.0f * A);
        fp32 tau2   = (-B - sqrt_d) / (2.0f * A);
        tau_limited = (tau >= 0.0f) ? tau1 : tau2;
    }

    /* ----------------------------------------------------------
     * Step 7: Clamp and write back to wheel_T
     *
     * NOTE: This function is called INSIDE chassisR_control_loop.
     * After the loop returns, chassisR_task.c converts wheel_T to
     * given_current and sends it via CAN.  Writing wheel_T here
     * therefore correctly limits the final motor command.
     * ---------------------------------------------------------- */
    if (tau_limited >  WHEEL_T_LIMIT) tau_limited =  WHEEL_T_LIMIT;
    if (tau_limited < -WHEEL_T_LIMIT) tau_limited = -WHEEL_T_LIMIT;

    chassis_power_control->wheel_motor[0].wheel_T = tau_limited;

    power_dbg.tau_out = tau_limited;
    power_dbg.active  = 1;
}
