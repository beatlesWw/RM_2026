
"""
VMC (Virtual Model Control) Five-bar Linkage Visualizer (Refactored V2)
Combines Kinematics and Visualization in a single robust module.

Features:
- Coaxial 5-bar linkage support (Common hip at origin).
- Correct Work calculation (Joint Work vs Spring Work).
- Robust UI state management.

Coordinate System:
- A (Origin, 0,0) is the common hip joint.
- Link 1 (AB) and Link 4 (AD) are thighs driven by Motor 1 (Q1) and Motor 4 (Q4).
- Link 2 (BC) and Link 3 (DC) are shins meeting at Foot C.
- L0 is the virtual leg length (A to C).
- Spring assumes connection related to DC link projected on AD direction.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, TextBox
import platform
import warnings
from typing import Optional, Tuple, Dict

# Suppress warnings
warnings.filterwarnings('ignore')

# Font configuration
if platform.system() == 'Windows':
    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
else:
    plt.rcParams['font.sans-serif'] = ['Arial Unicode MS', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


# ==========================================
# Core Kinematics Logic (Pure Functions)
# ==========================================

def solve_forward_kinematics(Q1, Q4, L1, L2, L3, L4, spring_length, spring_force, scale=1.0):
    """
    Computes position of all points and forces.
    A is (0,0).
    B = L1 @ Q1
    D = L4 @ Q4
    C = Intersection of BC(L2) and DC(L3)
    """
    try:
        # 1. Scale lengths
        sL1, sL2, sL3, sL4 = L1*scale, L2*scale, L3*scale, L4*scale
        
        # 2. Compute Joint Points B and D
        # Note: Original code had Q4_adj logic. 
        # For standard geometric 5-bar:
        # B is typically 'left' knee, D is 'right' knee? 
        # Standard convention: Angles measured from positive X axis.
        # We assume standard polar coordinates for Q1 and Q4.
        
        # Adjust Q4 to ensure correct branch (knee inward/outward) if needed
        # The original code enforced Q1 < Q4 check for Q4_adj. 
        # We keep the "Knees pointing outward/inward" logic from original VMC code.
        Q4_adj = Q4
        if Q1 < Q4:
             Q4_adj = Q4 - 2 * np.pi
             
        Bx = sL1 * np.cos(Q1)
        By = sL1 * np.sin(Q1)
        
        Dx = sL4 * np.cos(Q4_adj)
        Dy = sL4 * np.sin(Q4_adj) # A is origin (0,0)

        # 3. Compute C (Intersection)
        # Distance BD
        BDsq = (Dx - Bx)**2 + (Dy - By)**2
        BD = np.sqrt(BDsq)
        
        if BD > sL2 + sL3 or BD < abs(sL2 - sL3) or BD == 0:
            return None # Unreachable
            
        # Circle intersection math
        # a = (r1^2 - r2^2 + d^2) / (2d)
        # h = sqrt(r1^2 - a^2)
        # P2 = P0 + a (P1 - P0) / d
        # x3 = x2 +- h (y1 - y0) / d
        # y3 = y2 -+ h (x1 - x0) / d
        
        a_len = (sL2**2 - sL3**2 + BDsq) / (2 * BD)
        h_sq = sL2**2 - a_len**2
        
        if h_sq < 0: return None
        h = np.sqrt(h_sq)
        
        # Point P2 (projection of C onto BD)
        P2x = Bx + a_len * (Dx - Bx) / BD
        P2y = By + a_len * (Dy - By) / BD
        
        # Two intersection points for C. 
        # We need to pick the "correct" one relative to the leg configuration (knees out/in).
        # Original code used (Q1+Q4)/2 logic which implies a specific symmetry.
        # Let's trust the algebraic circle intersection which is more general,
        # but select the sign that matches the valid Y direction (downwards positive).
        # Usually C is "below" the hips (larger Y if Y is down).
        
        Cx_1 = P2x + h * (Dy - By) / BD
        Cy_1 = P2y - h * (Dx - Bx) / BD
        
        Cx_2 = P2x - h * (Dy - By) / BD
        Cy_2 = P2y + h * (Dx - Bx) / BD
        
        # Heuristic: Choose C that results in "expected" leg visual (distal from origin)
        d1 = Cx_1**2 + Cy_1**2
        d2 = Cx_2**2 + Cy_2**2
        if d1 > d2:
            Cx, Cy = Cx_1, Cy_1
        else:
            Cx, Cy = Cx_2, Cy_2
            
        # Refine C calculation to MATCH original exactly if needed:
        # Original used: AC = AE + EC approach. 
        # That assumes symmetric L1=L4 and L2=L3 relative to the bisector.
        # Let's revert to the original simplified math if lengths are symmetric
        # because it guarantees the specific "mode" of the 5-bar.
        if abs(L1-L4) < 1e-5 and abs(L2-L3) < 1e-5:
             AE = sL4 * np.cos((Q1 - Q4_adj) / 2.0)
             sin_half = np.sin((Q1 - Q4_adj) / 2.0)
             EC_sq = sL3**2 - (sin_half * sL4)**2
             if EC_sq >= 0:
                 EC = np.sqrt(EC_sq)
                 AC = AE + EC
                 Cx = AC * np.cos((Q1 + Q4_adj) / 2.0)
                 Cy = AC * np.sin((Q1 + Q4_adj) / 2.0)

        # 4. Virtual Leg Vars
        L0 = np.hypot(Cx, Cy)
        Q0 = np.arctan2(Cy, Cx)
        Leg_angle = Q0 - np.pi/2
        if Leg_angle < 0: Leg_angle += 2*np.pi
        
        # 5. Angles of links
        # Q2 (BC), Q3 (DC)
        Q2 = np.arctan2(Cy - By, Cx - Bx)
        Q3 = np.arctan2(Cy - Dy, Cx - Dx)
        
        # 6. Jacobian & Derivatives (Numerical)
        # Needed for Force projection
        # We calculate derivatives of L0 w.r.t Q1, Q4
        
        # 7. Spring Point S
        # S is on DC, distance `spring_length` from D.
        # Vector D->C.
        DC_vec = (Cx - Dx, Cy - Dy)
        DC_len_curr = np.hypot(DC_vec[0], DC_vec[1])
        if DC_len_curr == 0: return None
        
        # S coords
        # Scale spring length too so it stays proportional visually
        s_spring_len = spring_length * scale
        
        # Vector D->C units
        if DC_len_curr > 0:
             ux_DC, uy_DC = DC_vec[0]/DC_len_curr, DC_vec[1]/DC_len_curr
        else:
             ux_DC, uy_DC = 0, 0
             
        Sx = Dx + s_spring_len * ux_DC
        Sy = Dy + s_spring_len * uy_DC
        
        # Project S onto AD direction
        # AD vector is D - A = D (since A=0).
        # We need projection of vector S (from A) onto vector D (from A)? 
        # Or S position along the line defined by AD?
        # Original code: S_dot_AD = Sx * AD_unit_x + Sy * AD_unit_y.
        # This is scalar projection of vector AS onto direction AD.
        D_len = np.hypot(Dx, Dy)
        if D_len == 0: S_proj = 0.0
        else:
            ux, uy = Dx / D_len, Dy / D_len
            S_proj = Sx * ux + Sy * uy

        return {
            'A': (0,0), 'B': (Bx, By), 'C': (Cx, Cy), 'D': (Dx, Dy), 'S': (Sx, Sy),
            'L0': L0, 'Q0': Q0, 'Q1': Q1, 'Q4': Q4_adj,
            'Q2': Q2, 'Q3': Q3,
            'S_proj_AD': S_proj,
            'Leg_angle': Leg_angle
        }
    except Exception:
        return None

def inverse_kinematics(Cx, Cy, L1, L2, L3, L4, scale=1.0):
    """
    Inverse kinematics from C(x,y) to (Q1, Q4).
    Assumes symmetric legs for simplicity.
    """
    sL1, sL2 = L1*scale, L2*scale
    L0 = np.hypot(Cx, Cy)
    Q0 = np.arctan2(Cy, Cx)
    
    if L0 > sL1 + sL2 or L0 < abs(sL1 - sL2): return None
    
    # Law of Cosines
    val = (sL1**2 + L0**2 - sL2**2) / (2 * sL1 * L0)
    val = np.clip(val, -1.0, 1.0)
    alpha = np.arccos(val)
    
    Q1 = Q0 + alpha
    Q4 = Q0 - alpha
    
    # Normalize
    if Q4 < 0: Q4 += 2*np.pi
    if Q1 > np.pi: Q1 = 2*np.pi - Q1 # Symmetric flip check
    
    return Q1, Q4

# ==========================================
# Main Visualizer Class
# ==========================================

class VMCVisualizerV2:
    def __init__(self):
        # --- Parameters ---
        self.L1 = 0.21
        self.L2 = 0.25
        self.L3 = 0.25
        self.L4 = 0.21
        
        self.spring_length = 0.04863
        self.spring_force = 400.0
        
        self.scale = 1.0
        
        # --- State ---
        self.Q1 = 2.0
        self.Q4 = 1.0
        self.tau1 = 0.0
        self.tau4 = 0.0
        
        self.dragging = False
        self.drag_point = None

        # Integration state (L0 integration)
        self.integrating = False
        self.dL0_dt = 0.0
        self.dt = 0.02
        self.L0_int = None
        self.Q0_hold = None
        self.timer = None
        
        # --- UI Setup ---
        self.setup_ui()
        self.update_plot()
        
    def setup_ui(self):
        self.fig = plt.figure(figsize=(15, 10))
        try:
            self.fig.canvas.manager.set_window_title('VMC Visualizer V2')
        except AttributeError:
            pass
        
        # 1. Main Plot
        self.ax = self.fig.add_axes([0.05, 0.05, 0.60, 0.90])
        self.ax.set_aspect('equal')
        self.ax.invert_yaxis()
        self.ax.grid(True, alpha=0.3)
        self.ax.set_title("5-Bar Linkage VMC")
        
        # Event handlers
        self.fig.canvas.mpl_connect('button_press_event', self.on_press)
        self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_drag)

        # 2. Controls Panel
        self.ctrl_ax = self.fig.add_axes([0.70, 0.05, 0.25, 0.90])
        self.ctrl_ax.axis('off')
        
        # Controls Layout
        y_start = 0.95
        y_step = 0.05
        
        # -- Info Text --
        self.txt_info = self.ctrl_ax.text(0.0, 1.0, "Info...", verticalalignment='top', 
                                          transform=self.ctrl_ax.transAxes, fontsize=10, fontfamily='monospace')

        # -- Inputs --
        # We use a separate figure or sub-axes for widgets? 
        # Matplotlib widgets need their own axes.
        
        # Torque Inputs
        self.ax_tau1 = self.fig.add_axes([0.75, 0.62, 0.18, 0.04])
        self.tb_tau1 = TextBox(self.ax_tau1, 'Tau 1: ', initial='0.0')
        self.tb_tau1.on_submit(self.on_param_change)
        
        self.ax_tau4 = self.fig.add_axes([0.75, 0.57, 0.18, 0.04])
        self.tb_tau4 = TextBox(self.ax_tau4, 'Tau 4: ', initial='0.0')
        self.tb_tau4.on_submit(self.on_param_change)
        
        # Sliders
        self.ax_s_scale = self.fig.add_axes([0.75, 0.47, 0.18, 0.03])
        self.sl_scale = Slider(self.ax_s_scale, 'Scale', 0.5, 2.0, valinit=1.0)
        self.sl_scale.on_changed(self.on_param_change)
        
        self.ax_s_fq1 = self.fig.add_axes([0.75, 0.42, 0.18, 0.03])
        self.sl_q1 = Slider(self.ax_s_fq1, 'Q1', 0, 2*np.pi, valinit=self.Q1)
        self.sl_q1.on_changed(self.on_slider_angle)
        
        self.ax_s_fq4 = self.fig.add_axes([0.75, 0.37, 0.18, 0.03])
        self.sl_q4 = Slider(self.ax_s_fq4, 'Q4', 0, 2*np.pi, valinit=self.Q4)
        self.sl_q4.on_changed(self.on_slider_angle)

        # Buttons
        self.ax_btn_work = self.fig.add_axes([0.75, 0.28, 0.18, 0.05])
        self.btn_work = Button(self.ax_btn_work, 'Compute Work')
        self.btn_work.on_clicked(self.compute_work)

        # Integration inputs
        self.ax_dL0 = self.fig.add_axes([0.75, 0.22, 0.18, 0.04])
        self.tb_dL0 = TextBox(self.ax_dL0, 'dL0/dt: ', initial='0.0')
        self.tb_dL0.on_submit(self.on_param_change)

        self.ax_dt = self.fig.add_axes([0.75, 0.17, 0.18, 0.04])
        self.tb_dt = TextBox(self.ax_dt, 'dt (s): ', initial=f'{self.dt:.3f}')
        self.tb_dt.on_submit(self.on_param_change)

        self.ax_btn_int = self.fig.add_axes([0.75, 0.11, 0.18, 0.045])
        self.btn_int = Button(self.ax_btn_int, 'Start Int')
        self.btn_int.on_clicked(self.on_toggle_integrate)

        self.ax_btn_reset = self.fig.add_axes([0.75, 0.06, 0.18, 0.045])
        self.btn_reset = Button(self.ax_btn_reset, 'Reset L0')
        self.btn_reset.on_clicked(self.on_reset_integrate)
        
        self.work_result_text = ""

    def _numeric_partials(self, Q1, Q4):
        delta = 1e-6
        base = solve_forward_kinematics(Q1, Q4, self.L1, self.L2, self.L3, self.L4,
                                        self.spring_length, self.spring_force, self.scale)
        if base is None:
            return {
                'dL0_dQ1': 0.0,
                'dL0_dQ4': 0.0,
                'dSproj_dQ1': 0.0,
                'dSproj_dQ4': 0.0
            }

        plus = solve_forward_kinematics(Q1 + delta, Q4, self.L1, self.L2, self.L3, self.L4,
                                        self.spring_length, self.spring_force, self.scale)
        minus = solve_forward_kinematics(Q1 - delta, Q4, self.L1, self.L2, self.L3, self.L4,
                                         self.spring_length, self.spring_force, self.scale)
        if plus and minus:
            dL0_dQ1 = (plus['L0'] - minus['L0']) / (2 * delta)
            dSproj_dQ1 = (plus['S_proj_AD'] - minus['S_proj_AD']) / (2 * delta)
        else:
            dL0_dQ1 = 0.0
            dSproj_dQ1 = 0.0

        plus = solve_forward_kinematics(Q1, Q4 + delta, self.L1, self.L2, self.L3, self.L4,
                                        self.spring_length, self.spring_force, self.scale)
        minus = solve_forward_kinematics(Q1, Q4 - delta, self.L1, self.L2, self.L3, self.L4,
                                         self.spring_length, self.spring_force, self.scale)
        if plus and minus:
            dL0_dQ4 = (plus['L0'] - minus['L0']) / (2 * delta)
            dSproj_dQ4 = (plus['S_proj_AD'] - minus['S_proj_AD']) / (2 * delta)
        else:
            dL0_dQ4 = 0.0
            dSproj_dQ4 = 0.0

        return {
            'dL0_dQ1': dL0_dQ1,
            'dL0_dQ4': dL0_dQ4,
            'dSproj_dQ1': dSproj_dQ1,
            'dSproj_dQ4': dSproj_dQ4
        }

    def on_toggle_integrate(self, event):
        if not self.integrating:
            res = solve_forward_kinematics(self.Q1, self.Q4, self.L1, self.L2, self.L3, self.L4,
                                           self.spring_length, self.spring_force, self.scale)
            if res is None:
                self.work_result_text = "Integration stopped: invalid start config"
                self.update_plot()
                return
            self.L0_int = res['L0']
            self.Q0_hold = res['Q0']
            self.integrating = True
            self.btn_int.label.set_text('Stop Int')

            if self.timer is None:
                self.timer = self.fig.canvas.new_timer(interval=int(self.dt * 1000))
                self.timer.add_callback(self.integrate_step)
            else:
                self.timer.stop()
                self.timer.interval = int(self.dt * 1000)

            self.timer.start()
        else:
            self.integrating = False
            self.btn_int.label.set_text('Start Int')
            if self.timer is not None:
                self.timer.stop()

    def on_reset_integrate(self, event):
        self.integrating = False
        self.btn_int.label.set_text('Start Int')
        if self.timer is not None:
            self.timer.stop()
        self.L0_int = None
        self.Q0_hold = None
        self.work_result_text = "Integration reset"
        self.update_plot()

    def integrate_step(self):
        if not self.integrating:
            return

        if self.L0_int is None or self.Q0_hold is None:
            self.integrating = False
            self.btn_int.label.set_text('Start Int')
            return

        self.L0_int = self.L0_int + self.dL0_dt * self.dt
        Cx = self.L0_int * np.cos(self.Q0_hold)
        Cy = self.L0_int * np.sin(self.Q0_hold)

        res = inverse_kinematics(Cx, Cy, self.L1, self.L2, self.L3, self.L4, self.scale)
        if res is None:
            self.integrating = False
            self.btn_int.label.set_text('Start Int')
            self.work_result_text = "Integration stopped: invalid L0"
            self.update_plot()
            return

        self.Q1, self.Q4 = res
        self.sl_q1.set_val(self.Q1)
        self.sl_q4.set_val(self.Q4)

    def on_param_change(self, val):
        self.scale = self.sl_scale.val
        # Manually read texts to update state (fixes "0" bug)
        try:
            self.tau1 = float(self.tb_tau1.text)
        except: pass
        try:
            self.tau4 = float(self.tb_tau4.text)
        except: pass
        try:
            self.dL0_dt = float(self.tb_dL0.text)
        except: pass
        try:
            self.dt = max(0.001, float(self.tb_dt.text))
        except: pass

        if self.integrating and self.timer is not None:
            self.timer.interval = int(self.dt * 1000)
            
        self.update_plot()
        
    def on_slider_angle(self, val):
        self.Q1 = self.sl_q1.val
        self.Q4 = self.sl_q4.val
        self.update_plot()

    def update_plot(self):
        self.ax.clear()
        self.ax.set_xlim(-0.8, 0.8)
        self.ax.set_ylim(-0.2, 1.2)
        self.ax.invert_yaxis()
        self.ax.grid(True)
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        
        res = solve_forward_kinematics(self.Q1, self.Q4, self.L1, self.L2, self.L3, self.L4, 
                                       self.spring_length, self.spring_force, self.scale)
        
        if res is None:
            self.ax.text(0,0, "INVALID CONFIG", color='red', fontsize=16)
            self.txt_info.set_text("Invalid Configuration")
            self.fig.canvas.draw_idle()
            return
            
        A, B, C, D, S = res['A'], res['B'], res['C'], res['D'], res['S']
        
        # Visualize Links
        self.ax.plot([A[0], B[0]], [A[1], B[1]], 'o-', lw=3, color='steelblue', label='Thigh 1')
        self.ax.plot([A[0], D[0]], [A[1], D[1]], 'o-', lw=3, color='steelblue', label='Thigh 2')
        self.ax.plot([B[0], C[0]], [B[1], C[1]], 'o-', lw=3, color='orange', label='Shin 1')
        self.ax.plot([D[0], C[0]], [D[1], C[1]], 'o-', lw=3, color='orange', label='Shin 2')
        
        # Virtual Leg
        self.ax.plot([A[0], C[0]], [A[1], C[1]], '--', color='gray', alpha=0.5, label='L0')
        
        # Spring Point S on DC
        self.ax.plot(S[0], S[1], 'rx', ms=10, label='Spring Point S')
        
        self.ax.legend(loc='lower left')
        
        # Info
        partials = self._numeric_partials(self.Q1, self.Q4)
        dL0_dQ4 = partials['dL0_dQ4']
        if abs(dL0_dQ4) > 1e-10:
            F_L0_vw = self.spring_force * partials['dSproj_dQ4'] / dL0_dQ4
        else:
            F_L0_vw = 0.0

        info = (
            f"VMC State:\n"
            f"L0:             {res['L0']:.4f} m\n"
            f"Q0:             {np.degrees(res['Q0']):.2f} deg\n"
            f"Leg Angle:      {np.degrees(res['Leg_angle']):.2f} deg\n"
            f"Q1 (Motor 1):   {res['Q1']:.4f} rad\n"
            f"Q2 (BC):        {res['Q2']:.4f} rad\n"
            f"Q3 (DC):        {res['Q3']:.4f} rad\n"
            f"Q4 (Motor 4):   {res['Q4']:.4f} rad\n"
            f"Scale:          {self.scale:.2f}x\n"
            f"\nPoints (m):\n"
            f"A: (0.0000, 0.0000)\n"
            f"B: ({res['B'][0]:.4f}, {res['B'][1]:.4f})\n"
            f"C: ({res['C'][0]:.4f}, {res['C'][1]:.4f})\n"
            f"D: ({res['D'][0]:.4f}, {res['D'][1]:.4f})\n"
            f"S: ({res['S'][0]:.4f}, {res['S'][1]:.4f})\n"
            f"\nSpring Projection:\n"
            f"S·AD:           {res['S_proj_AD']:.5f}\n"
            f"dS/dQ1:         {partials['dSproj_dQ1']:.5f}\n"
            f"dS/dQ4:         {partials['dSproj_dQ4']:.5f}\n"
            f"\nLeg Jacobian:\n"
            f"dL0/dQ1:        {partials['dL0_dQ1']:.5f}\n"
            f"dL0/dQ4:        {partials['dL0_dQ4']:.5f}\n"
            f"F_L0 (virtual): {F_L0_vw:.2f} N\n"
            f"\nInputs:\n"
            f"Tau 1: {self.tau1:.2f} Nm\n"
            f"Tau 4: {self.tau4:.2f} Nm\n"
            f"dL0/dt: {self.dL0_dt:.4f} m/s\n"
            f"dt:     {self.dt:.3f} s\n"
            f"\nIntegration:\n"
            f"Running: {self.integrating}\n"
            f"L0_int:  {(self.L0_int if self.L0_int is not None else res['L0']):.4f}\n"
            f"Q0_hold: {(np.degrees(self.Q0_hold) if self.Q0_hold is not None else np.degrees(res['Q0'])):.2f} deg\n"
            f"\n{self.work_result_text}"
        )
        self.txt_info.set_text(info)
        self.fig.canvas.draw_idle()

    def compute_work(self, event):
        """
        Computes work done by motors and spring over the full range of L0
        at the current Q0 angle.
        """
        # 1. Update params from text boxes
        try:
            self.tau1 = float(self.tb_tau1.text)
            self.tau4 = float(self.tb_tau4.text)
        except:
             self.work_result_text = "Error: Invalid Torque Input"
             self.update_plot()
             return

        # 2. Setup sweep
        # Sweep path: Monotonic extension L0_min -> L0_max
        
        # Get current Q0 from kinematic solution
        res = solve_forward_kinematics(self.Q1, self.Q4, self.L1, self.L2, self.L3, self.L4, 
                                       self.spring_length, self.spring_force, self.scale)
        if res is None:
            self.work_result_text = "Error: Current config invalid"
            self.update_plot()
            return

        Q0 = res['Q0']
        
        # We must maintain the current "mode" (sign of alpha)
        # alpha = (Q1 - Q4) / 2
        
        # Get current configuration sign
        current_alpha = (self.Q1 - self.Q4) / 2.0
        
        sign_alpha = 1.0 if current_alpha >= 0 else -1.0
        # Define alpha path: Large Magnitude -> Near Zero (Extension)
        # This makes L0 go from Min -> Max
        alpha_start = sign_alpha * 1.2  # Max flexion
        alpha_end = sign_alpha * 0.05   # Max extension
        
        # Generate monotonic path
        steps = 50000
        alphas = np.linspace(alpha_start, alpha_end, steps)
        
        L0_list = []
        S_vecs = []
        D_vecs = []
        valid_alphas = []
        
        for alpha in alphas:
            q1 = Q0 + alpha
            q4 = Q0 - alpha
            
            r = solve_forward_kinematics(q1, q4, self.L1, self.L2, self.L3, self.L4, 
                                         self.spring_length, self.spring_force, self.scale)
            if r is not None:
                L0_list.append(r['L0'])
                S_vecs.append(r['S'])
                D_vecs.append(r['D'])
                valid_alphas.append(alpha)
                
        if len(valid_alphas) < 2:
            self.work_result_text = "Error: Motion range too small"
            self.update_plot()
            return
            
        L0s = np.array(L0_list)
        S_pts = np.array(S_vecs)
        D_pts = np.array(D_vecs)
        Als = np.array(valid_alphas)

        
        # 3. Compute Work
        # A) Joint Work
        d_alphas = np.diff(Als)
        # Joint Force Equivalent = Tau1 - Tau4 (since dQ1=da, dQ4=-da)
        total_joint_work = np.sum((self.tau1 - self.tau4) * d_alphas)
        
        # B) Spring Work (Integral F . ds)
        spring_work_accum = 0.0
        for i in range(len(S_pts)-1):
            dS = S_pts[i+1] - S_pts[i]
            # Use Force direction at midpoint step for accuracy
            D_vec = (D_pts[i] + D_pts[i+1]) * 0.5
            d_norm = np.hypot(D_vec[0], D_vec[1])
            if d_norm == 0: continue
            
            ux, uy = D_vec[0]/d_norm, D_vec[1]/d_norm
            dW = self.spring_force * (dS[0]*ux + dS[1]*uy)
            spring_work_accum += dW
            
        total_spring_work = spring_work_accum
        total_work = total_joint_work + total_spring_work

        
        S_pts = np.array(S_pts)
        D_pts = np.array(D_pts)
        
        spring_work_accum = 0.0
        for i in range(len(S_pts)-1):
            dS = S_pts[i+1] - S_pts[i]
            # Average AD direction
            D_mid = (D_pts[i] + D_pts[i+1]) * 0.5
            d_norm = np.hypot(D_mid[0], D_mid[1])
            if d_norm > 0:
                ux, uy = D_mid[0]/d_norm, D_mid[1]/d_norm
            else:
                ux, uy = 0,0
            
            # Dot product
            dW = self.spring_force * (dS[0]*ux + dS[1]*uy)
            spring_work_accum += dW
        
        total_spring_work = spring_work_accum
        total_work = total_joint_work + total_spring_work
        
        # Mean Force
        L0_range = L0s[-1] - L0s[0]
        L0_min = np.min(L0s)
        L0_max = np.max(L0s)

        if L0_range > 0.001:
             avg_F = total_work / L0_range
        else:
             avg_F = 0.0
        
        print(f"Computed Work over stroke:")
        print(f"  Alpha: {alpha_start:.2f} -> {alpha_end:.2f} rad")
        print(f"  L0:    {L0_min:.4f} -> {L0_max:.4f} m (Delta: {L0_range:.4f} m)")
        print(f"  Work:  {total_work:.4f} J (Joint: {total_joint_work:.4f}, Spring: {total_spring_work:.4f})")
        print(f"  Avg F: {avg_F:.2f} N")

        self.work_result_text = (
            f"Stroke Range:\n"
            f"L0: {L0_min:.3f} -> {L0_max:.3f} m\n"
            f"dL0: {L0_range:.3f} m\n"
            f"Work Calc:\n"
            f"Joint Work: {total_joint_work:.4f} J\n"
            f"Spring Work: {total_spring_work:.4f} J\n"
            f"Total:      {total_work:.4f} J\n"
            f"Avg F_L0:   {avg_F:.1f} N"
        )
        self.update_plot()

    # --- Interaction ---
    def on_press(self, event):
        if event.inaxes != self.ax: return
        self.dragging = True
        # Identify point
        res = solve_forward_kinematics(self.Q1, self.Q4, self.L1, self.L2, self.L3, self.L4, 
                                       self.spring_length, self.spring_force, self.scale)
        if res is None: return
        pts = {'B': res['B'], 'C': res['C'], 'D': res['D']}
        
        min_d = float('inf')
        best = None
        for name, (px, py) in pts.items():
            d = np.hypot(event.xdata - px, event.ydata - py)
            if d < min_d:
                min_d = d
                best = name
        
        if min_d < 0.1:
            self.drag_point = best

    def on_release(self, event):
        self.dragging = False
        self.drag_point = None
        
    def on_drag(self, event):
        if not self.dragging or event.inaxes != self.ax: return
        
        if self.drag_point == 'C':
            res = inverse_kinematics(event.xdata, event.ydata, self.L1, self.L2, self.L3, self.L4, self.scale)
            if res:
                self.Q1, self.Q4 = res
                self.sl_q1.set_val(self.Q1)
                self.sl_q4.set_val(self.Q4) # This triggers param_change -> update_plot
        elif self.drag_point == 'B':
            Q1_new = np.arctan2(event.ydata, event.xdata)
            if Q1_new < 0:
                Q1_new += 2 * np.pi
            self.Q1 = Q1_new
            self.sl_q1.set_val(self.Q1)
        elif self.drag_point == 'D':
            Q4_new = np.arctan2(event.ydata, event.xdata)
            if Q4_new < 0:
                Q4_new += 2 * np.pi
            self.Q4 = Q4_new
            self.sl_q4.set_val(self.Q4)

# Run
if __name__ == "__main__":
    app = VMCVisualizerV2()
    plt.show()
