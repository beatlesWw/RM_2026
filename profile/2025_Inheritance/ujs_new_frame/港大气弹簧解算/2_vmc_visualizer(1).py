"""
VMC (Virtual Model Control) Five-bar Linkage Visualizer
Based on wheel-legged robot VMC kinematics

Run: python vmc_visualizer.py
Dependencies: pip install numpy matplotlib

Geometry:
      B -------- C (endpoint)
     /            \\
    /              \\
   A -------- D
   
   A: Origin (0,0)
   L1: AB length
   L2: BC length (typically equals L3)
   L3: CD length
   L4: AD length (typically equals L1)
   
   Q1: angle of AB with x-axis (joint_motorB_position)
   Q2: angle of BC with x-axis
   Q3: angle of CD with x-axis
   Q4: angle of AD with x-axis (joint_motorF_position)
   
   L0: AC length (virtual leg length)
   Q0: AC angle with x-axis (virtual leg angle)
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, TextBox
from matplotlib.patches import Circle, Arc
import matplotlib.patches as mpatches
import platform
import warnings

# Suppress font warnings
warnings.filterwarnings('ignore', message='.*Glyph.*missing from font.*')

# Set font for Chinese characters support
if platform.system() == 'Windows':
    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'STSong', 'DejaVu Sans']
    plt.rcParams['font.family'] = ['Microsoft YaHei', 'sans-serif']
else:
    plt.rcParams['font.sans-serif'] = ['Arial Unicode MS', 'DejaVu Sans']
    plt.rcParams['font.family'] = ['Arial Unicode MS', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False

class VMCVisualizer:
    def __init__(self):
        # Link parameters (matching C++ code)
        self.L1 = 0.21  # AB (joint_motorB)
        self.L2 = 0.25  # BC
        self.L3 = 0.25  # CD
        self.L4 = 0.21  # AD (joint_motorF)
        
        # Spring parameters (matching Chassis_Task.cpp)
        self.spring_length = 0.04863  # 弹簧力矩长度 (m)
        self.spring_force = 400.0  # 弹簧力 (N)
        
        # Joint angles (radians) - Q1 is motorB, Q4 is motorF
        self.Q1 = 2.0  # AB angle (joint_motorB_position)
        self.Q4 = 1.0  # AD angle (joint_motorF_position)
        
        # Drag state
        self.dragging = False
        self.drag_point = None
        # Scale factor for proportional resizing of the whole leg
        self.scale = 1.0  # 等比缩放因子

        # External joint torques (N*m) 输入框初始值
        self.tau1 = 0.0  # 关节B
        self.tau4 = 0.0  # 关节F
        
        # Create figures: main plot and separate control window
        self.setup_plot()
        self.setup_control_window()
        self.update_plot()

    def _scaled(self):
        """Return scaled link lengths (sL1,sL2,sL3,sL4)."""
        sL1 = self.L1 * self.scale
        sL2 = self.L2 * self.scale
        sL3 = self.L3 * self.scale
        sL4 = self.L4 * self.scale
        return sL1, sL2, sL3, sL4
        
    def setup_plot(self):
        """Setup plot interface"""
        self.fig = plt.figure(figsize=(16, 11))
        self.fig.suptitle('VMC 五连杆可视化工具', fontsize=14, fontweight='bold')
        # Main plot area (large, left)
        self.ax = self.fig.add_axes([0.05, 0.06, 0.62, 0.88])
        self.ax.set_xlim(-0.8, 0.8)
        self.ax.set_ylim(-0.4, 0.9)  # Y轴正方向向下
        self.ax.invert_yaxis()
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m) [向下为正]')
        self.ax.set_title('五连杆机构 (拖动B/C/D点改变姿态)')

        # Info panel (right top)
        self.info_ax = self.fig.add_axes([0.69, 0.56, 0.30, 0.38])
        self.info_ax.axis('off')

        # Mouse events (keep interactions on main figure)
        self.fig.canvas.mpl_connect('button_press_event', self.on_press)
        self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_motion)

    def setup_control_window(self):
        """Create a separate control window containing sliders, text boxes and buttons."""
        self.ctrl_fig = plt.figure(figsize=(6, 9))
        self.ctrl_fig.suptitle('控制面板', fontsize=12, fontweight='bold')

        slider_color = 'lightblue'
        ctrl_x = 0.08
        ctrl_w = 0.84

        # Torque inputs at top
        ax_tau1 = self.ctrl_fig.add_axes([ctrl_x, 0.92, ctrl_w, 0.035])
        self.text_tau1 = TextBox(ax_tau1, 'τ_B (N·m)', initial=str(self.tau1))
        self.text_tau1.on_submit(self.on_tau1_submit)

        ax_tau4 = self.ctrl_fig.add_axes([ctrl_x, 0.88, ctrl_w, 0.035])
        self.text_tau4 = TextBox(ax_tau4, 'τ_F (N·m)', initial=str(self.tau4))
        self.text_tau4.on_submit(self.on_tau4_submit)

        # Sliders stack
        y0 = 0.80
        dy = 0.07

        ax_scale = self.ctrl_fig.add_axes([ctrl_x, y0, ctrl_w, 0.05])
        self.slider_scale = Slider(ax_scale, '整体缩放', 0.5, 2.0, valinit=self.scale, color=slider_color)
        self.slider_scale.on_changed(self.on_slider_change)

        ax_L1 = self.ctrl_fig.add_axes([ctrl_x, y0-dy, ctrl_w, 0.05])
        self.slider_L1 = Slider(ax_L1, 'L1 (大腿)', 0.1, 0.4, valinit=self.L1, color=slider_color)
        self.slider_L1.on_changed(self.on_slider_change)

        ax_L2 = self.ctrl_fig.add_axes([ctrl_x, y0-2*dy, ctrl_w, 0.05])
        self.slider_L2 = Slider(ax_L2, 'L2 (小腿B)', 0.1, 0.4, valinit=self.L2, color=slider_color)
        self.slider_L2.on_changed(self.on_slider_change)

        ax_L3 = self.ctrl_fig.add_axes([ctrl_x, y0-3*dy, ctrl_w, 0.05])
        self.slider_L3 = Slider(ax_L3, 'L3 (小腿F)', 0.1, 0.4, valinit=self.L3, color=slider_color)
        self.slider_L3.on_changed(self.on_slider_change)

        ax_L4 = self.ctrl_fig.add_axes([ctrl_x, y0-4*dy, ctrl_w, 0.05])
        self.slider_L4 = Slider(ax_L4, 'L4 (大腿)', 0.1, 0.4, valinit=self.L4, color=slider_color)
        self.slider_L4.on_changed(self.on_slider_change)

        ax_Q1 = self.ctrl_fig.add_axes([ctrl_x, y0-5*dy, ctrl_w, 0.05])
        self.slider_Q1 = Slider(ax_Q1, 'Q1 (关节B)', 0.0, 2*np.pi, valinit=self.Q1, color='lightgreen')
        self.slider_Q1.on_changed(self.on_slider_change)

        ax_Q4 = self.ctrl_fig.add_axes([ctrl_x, y0-6*dy, ctrl_w, 0.05])
        self.slider_Q4 = Slider(ax_Q4, 'Q4 (关节F)', 0.0, 2*np.pi, valinit=self.Q4, color='lightgreen')
        self.slider_Q4.on_changed(self.on_slider_change)

        ax_spring_len = self.ctrl_fig.add_axes([ctrl_x, y0-7*dy, ctrl_w, 0.05])
        self.slider_spring_length = Slider(ax_spring_len, '弹簧力矩长', 0.01, 0.15, valinit=self.spring_length, color='lightyellow')
        self.slider_spring_length.on_changed(self.on_slider_change)

        ax_spring_force = self.ctrl_fig.add_axes([ctrl_x, y0-8*dy, ctrl_w, 0.05])
        self.slider_spring_force = Slider(ax_spring_force, '弹簧力', 100, 1500, valinit=self.spring_force, color='lightyellow')
        self.slider_spring_force.on_changed(self.on_slider_change)

        # Buttons
        ax_reset = self.ctrl_fig.add_axes([0.08, 0.02, 0.36, 0.06])
        self.btn_reset = Button(ax_reset, '重置')
        self.btn_reset.on_clicked(self.reset)

        ax_compute = self.ctrl_fig.add_axes([0.52, 0.02, 0.36, 0.06])
        self.btn_compute = Button(ax_compute, '计算功')
        self.btn_compute.on_clicked(self.compute_work)
        
    def forward_kinematics(self, Q1, Q4):
        """
        Forward kinematics: Calculate positions from joint angles
        Exactly matching VMC.cpp Forward_kinematic_solution()
        
        Q1 = joint_motorB_position (angle of L1)
        Q4 = joint_motorF_position (angle of L4)
        """
        # A is at origin
        Ax, Ay = 0.0, 0.0
        
        # Adjust Q4 if Q1 < Q4 (matching C++ code)
        Q4_adj = Q4
        if Q1 < Q4:
            Q4_adj = Q4 - 2 * np.pi
        
        # 使用统一的缩放长度
        sL1, sL2, sL3, sL4 = self._scaled()

        # B point position (end of L1)
        Bx = sL1 * np.cos(Q1)
        By = sL1 * np.sin(Q1)

        # D point position (end of L4)
        Dx = sL4 * np.cos(Q4_adj)
        Dy = sL4 * np.sin(Q4_adj)

        # Calculate C point (using exact same method as C++ code, with scaled lengths)
        AE = sL4 * np.cos((Q1 - Q4_adj) / 2.0)
        sin_half = np.sin((Q1 - Q4_adj) / 2.0)
        EC_sq = sL3**2 - (sin_half * sL4)**2
        
        if EC_sq < 0:
            return None  # No solution
            
        EC = np.sqrt(EC_sq)
        AC = AE + EC
        
        Cx = AC * np.cos((Q1 + Q4_adj) / 2.0)
        Cy = AC * np.sin((Q1 + Q4_adj) / 2.0)
        
        # Virtual leg parameters
        L0 = AC  # Leg length
        Q0 = (Q1 + Q4_adj) / 2.0  # Leg angle
        
        # Calculate Q2, Q3
        DCx = Cx - Dx
        DCy = Cy - Dy
        BCx = Cx - Bx
        BCy = Cy - By
        
        Q2 = np.arctan2(BCy, BCx)
        Q3 = np.arctan2(DCy, DCx)
        
        if Q2 < 0:
            Q2 += 2 * np.pi
        if Q3 < 0:
            Q3 += 2 * np.pi
        
        # Leg_angle = Q0 - PI/2 (as in C++ code)
        Leg_angle = Q0 - np.pi / 2
        if Leg_angle < 0:
            Leg_angle += 2 * np.pi
        
        # ========== 弹簧物理模型 ==========
        # 弹簧作用点S: 在DC杆上，距D点spring_length
        # 弹簧力方向: 平行于AD
        # 弹簧力大小: spring_force
        #
        # 弹簧对DC杆绕D点的力矩:
        # τ_DC = F × r × sin(θ)
        # 其中 r = spring_length, θ = AD与DC的夹角 = Q3 - Q4
        # sin(Q3-Q4) = AD_unit × DC_unit (叉乘)
        
        # ========== 方法1: 叉乘法 (原始代码方法) ==========
        # 计算弹簧力补偿 (matching VMC.cpp Forward_kinematic_solution)
        # AC单位向量
        AC_unit_x = Cx / AC
        AC_unit_y = Cy / AC
        # AD单位向量（缩放后）
        AD_unit_x = Dx / sL4 if sL4 != 0 else 0.0
        AD_unit_y = Dy / sL4 if sL4 != 0 else 0.0
        # DC单位向量（缩放后）
        DC_unit_x = DCx / sL3 if sL3 != 0 else 0.0
        DC_unit_y = DCy / sL3 if sL3 != 0 else 0.0
        # AC单位和DC的叉乘 (用于力传递到虚拟腿)
        AC_DC_cross = AC_unit_x * DCy - AC_unit_y * DCx
        # AD单位和DC单位的叉乘 = sin(Q3-Q4)，弹簧力矩的有效分量
        AD_DC_cross = AD_unit_x * DC_unit_y - AD_unit_y * DC_unit_x
        # 前馈力计算: F_L0 = F_spring × spring_length × sin(Q3-Q4) / (AC×DC的投影)
        if abs(AC_DC_cross) > 1e-6:
            feedforward_force_cross = self.spring_force / AC_DC_cross * AD_DC_cross * self.spring_length
        else:
            feedforward_force_cross = 0.0
        
        # 弹簧对DC杆的实际力矩 (绕D点)
        tau_spring_on_DC = self.spring_force * self.spring_length * AD_DC_cross
        
        # ========== 方法2: 虚功原理法 (基于雅可比矩阵) ==========
        # 弹簧力F作用在S点(DC杆上距D点spring_length处)，方向平行于AD
        # 
        # 虚功原理: 弹簧做功 = 末端C点沿虚拟腿方向的功
        # F_spring · δS_AD = F_L0 · δL0
        # 其中 δS_AD 是S点沿AD方向的位移
        #
        # S点位置: S = D + spring_length/L3 × DC
        # S点沿AD方向的位移: δS_AD = (∂S/∂Q3 · AD_unit) × δQ3
        
        # 计算雅可比矩阵 ∂C/∂(Q1,Q4)
        delta = 1e-6
        
        # ∂C/∂Q1
        result_plus = self._calc_C_point(Q1 + delta, Q4_adj)
        result_minus = self._calc_C_point(Q1 - delta, Q4_adj)
        if result_plus and result_minus:
            dCx_dQ1 = (result_plus[0] - result_minus[0]) / (2 * delta)
            dCy_dQ1 = (result_plus[1] - result_minus[1]) / (2 * delta)
        else:
            dCx_dQ1, dCy_dQ1 = 0, 0
        
        # ∂C/∂Q4
        result_plus = self._calc_C_point(Q1, Q4_adj + delta)
        result_minus = self._calc_C_point(Q1, Q4_adj - delta)
        if result_plus and result_minus:
            dCx_dQ4 = (result_plus[0] - result_minus[0]) / (2 * delta)
            dCy_dQ4 = (result_plus[1] - result_minus[1]) / (2 * delta)
        else:
            dCx_dQ4, dCy_dQ4 = 0, 0
        
        # 雅可比矩阵 J = [dCx/dQ1, dCx/dQ4; dCy/dQ1, dCy/dQ4]
        J = np.array([[dCx_dQ1, dCx_dQ4],
                      [dCy_dQ1, dCy_dQ4]])
        det_J = J[0,0]*J[1,1] - J[0,1]*J[1,0]
        
        # ========== 方法2: 雅可比虚功法 (简化) ==========
        # 虚功原理: 弹簧做功 = 末端C点沿虚拟腿方向做功
        #
        # 弹簧力F作用在S点，方向平行于AD
        # 弹簧功率 = F · v_S_AD = F · (dS·AD/dt)
        #
        # 虚拟腿功率 = F_L0 · v_L0 = F_L0 · (dL0/dt)
        #
        # 由于系统只有2个自由度(Q1,Q4)，可以写成:
        # dS·AD/dt = (∂S·AD/∂Q1)·dQ1/dt + (∂S·AD/∂Q4)·dQ4/dt
        # dL0/dt = (∂L0/∂Q1)·dQ1/dt + (∂L0/∂Q4)·dQ4/dt
        #
        # 对于任意运动，虚功守恒要求:
        # F·(∂S·AD/∂Q1) = F_L0·(∂L0/∂Q1)  对Q1
        # F·(∂S·AD/∂Q4) = F_L0·(∂L0/∂Q4)  对Q4
        #
        # 这两个方程应该给出相同的F_L0（如果物理模型正确）
        
        # 计算S点在AD方向的投影对Q1和Q4的偏导数
        S_AD_plus = self._calc_S_dot_AD(Q1 + delta, Q4_adj)
        S_AD_minus = self._calc_S_dot_AD(Q1 - delta, Q4_adj)
        if S_AD_plus is not None and S_AD_minus is not None:
            dS_AD_dQ1 = (S_AD_plus - S_AD_minus) / (2 * delta)
        else:
            dS_AD_dQ1 = 0
        
        S_AD_plus = self._calc_S_dot_AD(Q1, Q4_adj + delta)
        S_AD_minus = self._calc_S_dot_AD(Q1, Q4_adj - delta)
        if S_AD_plus is not None and S_AD_minus is not None:
            dS_AD_dQ4 = (S_AD_plus - S_AD_minus) / (2 * delta)
        else:
            dS_AD_dQ4 = 0
        
        # ∂L0/∂Q1 和 ∂L0/∂Q4
        L0_plus = self._calc_L0(Q1 + delta, Q4_adj)
        L0_minus = self._calc_L0(Q1 - delta, Q4_adj)
        if L0_plus and L0_minus:
            dL0_dQ1 = (L0_plus - L0_minus) / (2 * delta)
        else:
            dL0_dQ1 = 0
            
        L0_plus = self._calc_L0(Q1, Q4_adj + delta)
        L0_minus = self._calc_L0(Q1, Q4_adj - delta)
        if L0_plus and L0_minus:
            dL0_dQ4 = (L0_plus - L0_minus) / (2 * delta)
        else:
            dL0_dQ4 = 0
        
        # 从Q4方向计算F_L0 (与方法3相同)
        if abs(dL0_dQ4) > 1e-10:
            F_L0_from_Q4 = self.spring_force * dS_AD_dQ4 / dL0_dQ4
        else:
            F_L0_from_Q4 = 0
        
        # 从Q1方向计算F_L0 (应该为0，因为弹簧力矩不直接作用于Q1)
        if abs(dL0_dQ1) > 1e-10:
            F_L0_from_Q1 = self.spring_force * dS_AD_dQ1 / dL0_dQ1
        else:
            F_L0_from_Q1 = 0
        
        # 雅可比法结果：使用Q4方向的结果
        F_L0_jacobian = F_L0_from_Q4
        
        # 计算等效的末端力F_C（用于显示）
        # F_L0 是沿AC方向的力，转换为笛卡尔坐标
        F_Cx = F_L0_jacobian * AC_unit_x
        F_Cy = F_L0_jacobian * AC_unit_y
        
        # ========== 方法3: 简化虚功法 (直接计算) ==========
        # 与方法2完全相同：F_spring × dS_AD/dQ4 = F_L0 × dL0/dQ4
        # (方法2和3现在使用相同的计算)
            
        return {
            'A': (Ax, Ay),
            'B': (Bx, By),
            'C': (Cx, Cy),
            'D': (Dx, Dy),
            'L0': L0,
            'Q0': Q0,
            'Q0_raw': Q0,
            'Q1': Q1,
            'Q2': Q2,
            'Q3': Q3,
            'Q4': Q4_adj,
            'Q4_original': Q4,
            'Leg_angle': Leg_angle,
            # 弹簧物理参数
            'tau_spring_on_DC': tau_spring_on_DC,  # 弹簧对DC杆的力矩
            # 方法1: 叉乘法结果
            'feedforward_force_cross': feedforward_force_cross,
            'AC_DC_cross': AC_DC_cross,
            'AD_DC_cross': AD_DC_cross,
            # 方法2&3: 虚功法结果 (基于∂(S·AD)/∂Qi)
            'F_Cx': F_Cx,
            'F_Cy': F_Cy,
            'F_L0_jacobian': F_L0_jacobian,
            'F_L0_from_Q1': F_L0_from_Q1,  # 从Q1方向计算的F_L0（验证用）
            'J': J,
            'det_J': det_J,
            'dS_AD_dQ1': dS_AD_dQ1,
            'dS_AD_dQ4': dS_AD_dQ4,
            'dL0_dQ4': dL0_dQ4,
            'dL0_dQ1': dL0_dQ1,
        }
    
    def _calc_C_point(self, Q1, Q4):
        """辅助函数：计算C点位置（用于雅可比矩阵数值计算）"""
        sL1, sL2, sL3, sL4 = self._scaled()
        AE = sL4 * np.cos((Q1 - Q4) / 2.0)
        sin_half = np.sin((Q1 - Q4) / 2.0)
        EC_sq = sL3**2 - (sin_half * sL4)**2
        if EC_sq < 0:
            return None
        EC = np.sqrt(EC_sq)
        AC = AE + EC
        Cx = AC * np.cos((Q1 + Q4) / 2.0)
        Cy = AC * np.sin((Q1 + Q4) / 2.0)
        return (Cx, Cy)
    
    def _calc_L0(self, Q1, Q4):
        """辅助函数：计算虚拟腿长度L0（用于虚功原理计算）"""
        sL1, sL2, sL3, sL4 = self._scaled()
        AE = sL4 * np.cos((Q1 - Q4) / 2.0)
        sin_half = np.sin((Q1 - Q4) / 2.0)
        EC_sq = sL3**2 - (sin_half * sL4)**2
        if EC_sq < 0:
            return None
        EC = np.sqrt(EC_sq)
        return AE + EC
    
    def _calc_Q3(self, Q1, Q4):
        """辅助函数：计算Q3角度（DC杆与x轴夹角）"""
        C = self._calc_C_point(Q1, Q4)
        if C is None:
            return None
        Cx, Cy = C
        sL1, sL2, sL3, sL4 = self._scaled()
        Dx = sL4 * np.cos(Q4)
        Dy = sL4 * np.sin(Q4)
        DCx = Cx - Dx
        DCy = Cy - Dy
        Q3 = np.arctan2(DCy, DCx)
        if Q3 < 0:
            Q3 += 2 * np.pi
        return Q3
    
    def _calc_S_dot_AD(self, Q1, Q4):
        """辅助函数：计算S点在AD方向的投影（用于虚功计算）
        S点在DC杆上，距D点spring_length
        返回S点位置在AD方向的分量
        """
        C = self._calc_C_point(Q1, Q4)
        if C is None:
            return None
        Cx, Cy = C
        sL1, sL2, sL3, sL4 = self._scaled()
        Dx = sL4 * np.cos(Q4)
        Dy = sL4 * np.sin(Q4)
        DCx = Cx - Dx
        DCy = Cy - Dy
        DC_len = np.sqrt(DCx**2 + DCy**2)
        if DC_len < 1e-10:
            return None
        # S点位置
        Sx = Dx + DCx / DC_len * self.spring_length
        Sy = Dy + DCy / DC_len * self.spring_length
        # AD单位向量
        AD_unit_x = Dx / sL4 if sL4 != 0 else 0.0
        AD_unit_y = Dy / sL4 if sL4 != 0 else 0.0
        # S点在AD方向的投影
        S_dot_AD = Sx * AD_unit_x + Sy * AD_unit_y
        return S_dot_AD
    
    def inverse_kinematics(self, Cx, Cy):
        """Inverse kinematics: Calculate joint angles from endpoint position"""
        # 使用缩放后的长度进行可达性检查
        sL1, sL2, sL3, sL4 = self._scaled()
        L0 = np.sqrt(Cx**2 + Cy**2)
        Q0 = np.arctan2(Cy, Cx)
        
        # Check range
        if L0 > sL1 + sL2 or L0 < abs(sL1 - sL2):
            return None
            
        # Use law of cosines (assuming symmetric structure L1=L4, L2=L3)
        cos_alpha = (sL1**2 + L0**2 - sL2**2) / (2 * sL1 * L0)
        cos_alpha = np.clip(cos_alpha, -1, 1)
        alpha = np.arccos(cos_alpha)
        
        Q1 = Q0 + alpha
        Q4 = Q0 - alpha
        
        # Ensure angles are in reasonable range
        if Q4 < 0:
            Q4 += 2 * np.pi
        if Q1 > np.pi:
            Q1 = 2 * np.pi - Q1
            
        return Q1, Q4
    
    def update_plot(self):
        """更新绘图"""
        self.ax.clear()
        self.ax.set_xlim(-0.8, 0.8)
        self.ax.set_ylim(-0.4, 0.9)  # Y轴正方向向下
        self.ax.invert_yaxis()
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m) [向下为正]')
        self.ax.set_title('五连杆机构 (拖动B/C/D点)')
        
        # 计算各点位置
        result = self.forward_kinematics(self.Q1, self.Q4)
        
        if result is None:
            self.ax.text(0, 0, '无效配置！', ha='center', fontsize=14, color='red')
            self.fig.canvas.draw_idle()
            return
            
        A = result['A']
        B = result['B']
        C = result['C']
        D = result['D']
        
        # 绘制连杆（使用缩放后尺寸显示标签）
        self.ax.plot([A[0], B[0]], [A[1], B[1]], 'r-', linewidth=3, label=f'L1 (大腿)={self.L1*self.scale:.3f}m')
        self.ax.plot([B[0], C[0]], [B[1], C[1]], 'b-', linewidth=3, label=f'L2 (小腿B)={self.L2*self.scale:.3f}m')
        self.ax.plot([C[0], D[0]], [C[1], D[1]], 'g-', linewidth=3, label=f'L3 (小腿F)={self.L3*self.scale:.3f}m')
        self.ax.plot([A[0], D[0]], [A[1], D[1]], 'orange', linewidth=3, label=f'L4 (大腿)={self.L4*self.scale:.3f}m')
        
        # 虚拟腿 (L0) - 品红色虚线
        self.ax.plot([A[0], C[0]], [A[1], C[1]], 'm--', linewidth=2, alpha=0.7, label=f'L0 (虚拟腿)={result["L0"]:.3f}m')
        
        # ====== 绘制弹簧位置 ======
        # 弹簧在DC线段上，距离D点的距离为 spring_length（缩放后）
        # 弹簧力方向平行于AD
        DCx = C[0] - D[0]
        DCy = C[1] - D[1]
        DC_len = np.sqrt(DCx**2 + DCy**2)
        if DC_len > 1e-6:
            # 弹簧作用点S在DC上，距D点spring_length（不随整体缩放改变）
            ratio = self.spring_length / DC_len
            Sx = D[0] + DCx * ratio
            Sy = D[1] + DCy * ratio
            
            # 绘制弹簧作用点
            spring_circle = Circle((Sx, Sy), 0.012, color='brown', zorder=6)
            self.ax.add_patch(spring_circle)
            self.ax.annotate('S(弹簧)', (Sx, Sy), xytext=(-30, -15), textcoords='offset points', 
                           fontsize=9, fontweight='bold', color='brown')
            
            # 绘制弹簧力方向（平行于AD）
            # AD单位向量（缩放后）
            AD_unit_x = D[0] / (self.L4 * self.scale)
            AD_unit_y = D[1] / (self.L4 * self.scale)
            arrow_len = 0.05
            self.ax.annotate('', xy=(Sx + AD_unit_x * arrow_len, Sy + AD_unit_y * arrow_len), 
                           xytext=(Sx, Sy),
                           arrowprops=dict(arrowstyle='->', color='brown', lw=2))
            self.ax.text(Sx + AD_unit_x * arrow_len * 1.5, Sy + AD_unit_y * arrow_len * 1.5, 
                        f'F={self.spring_force:.0f}N\n(//AD)', fontsize=8, color='brown')
        
        # 绘制x轴参考线
        self.ax.axhline(y=0, color='gray', linestyle=':', alpha=0.5, linewidth=1)
        self.ax.plot([0, 0.20], [0, 0], 'gray', linewidth=1.5, alpha=0.7)
        self.ax.annotate('x轴', (0.22, 0), fontsize=10, color='gray')
        
        # 绘制关节点
        points = {'A (原点)': A, 'B': B, 'C (末端)': C, 'D': D}
        colors_pts = {'A (原点)': 'black', 'B': 'red', 'C (末端)': 'blue', 'D': 'green'}
        
        for name, pos in points.items():
            circle = Circle(pos, 0.015, color=colors_pts[name], zorder=5)
            self.ax.add_patch(circle)
            offset = (10, 10) if 'C' in name else (6, 6)
            self.ax.annotate(name.split()[0], pos, xytext=offset, textcoords='offset points', 
                           fontsize=12, fontweight='bold', color=colors_pts[name])
        
        # 绘制角度弧线和标注
        # Q1: AB与x轴的夹角 (关节B电机位置)
        self.draw_angle_arc(A, 0.08, 0, result['Q1'], 'Q1', 'red', result['Q1'])
        
        # Q4: AD与x轴的夹角 (关节F电机位置)
        self.draw_angle_arc(A, 0.12, 0, result['Q4'], 'Q4', 'orange', result['Q4'])
        
        # Q2: BC与x轴的夹角 (在B点)
        self.draw_angle_arc(B, 0.06, 0, result['Q2'], 'Q2', 'blue', result['Q2'])
        
        # Q3: CD与x轴的夹角 (在D点)
        self.draw_angle_arc(D, 0.06, 0, result['Q3'], 'Q3', 'green', result['Q3'])
        
        # Q0: 虚拟腿与x轴的夹角 (在A点)
        self.draw_angle_arc(A, 0.16, 0, result['Q0'], 'Q0', 'magenta', result['Q0'])
        
        self.ax.legend(loc='upper left', fontsize=9)
        
        # Update info panel
        # 计算基于输入关节扭矩的等效力
        dL0_dQ1 = result.get('dL0_dQ1', 0.0)
        dL0_dQ4 = result.get('dL0_dQ4', 0.0)
        denom = dL0_dQ1**2 + dL0_dQ4**2
        if abs(denom) > 1e-12:
            F_L0_from_torques = (self.tau1 * dL0_dQ1 + self.tau4 * dL0_dQ4) / denom
        else:
            # 如果两个偏导数都接近0，尝试用单个关节解（避免因数值问题得到0）
            if abs(dL0_dQ4) > 1e-8:
                F_L0_from_torques = self.tau4 / dL0_dQ4
            elif abs(dL0_dQ1) > 1e-8:
                F_L0_from_torques = self.tau1 / dL0_dQ1
            else:
                F_L0_from_torques = 0.0

        # 末端力（笛卡尔）
        L0 = result.get('L0', 0.0)
        if L0 > 1e-12:
            AC_unit_x = (C[0] - A[0]) / L0
            AC_unit_y = (C[1] - A[1]) / L0
        else:
            AC_unit_x = 0.0
            AC_unit_y = 0.0

        F_Cx_from_torques = F_L0_from_torques * AC_unit_x
        F_Cy_from_torques = F_L0_from_torques * AC_unit_y

        # 残差（检查两个关节方程是否一致）
        res_tau1 = self.tau1 - F_L0_from_torques * dL0_dQ1
        res_tau4 = self.tau4 - F_L0_from_torques * dL0_dQ4

        # 将计算结果加入 result 以便 info 显示
        result['tau1'] = self.tau1
        result['tau4'] = self.tau4
        result['F_L0_from_torques'] = F_L0_from_torques
        result['F_Cx_from_torques'] = F_Cx_from_torques
        result['F_Cy_from_torques'] = F_Cy_from_torques
        result['res_tau1'] = res_tau1
        result['res_tau4'] = res_tau4

        self.update_info(result)
        
        self.fig.canvas.draw_idle()
    
    def draw_angle_arc(self, center, radius, start_angle, end_angle, label, color, angle_value):
        """绘制角度弧线和标注"""
        if abs(end_angle - start_angle) < 0.01:
            return
            
        # 处理负角度
        display_end = end_angle
        if end_angle < 0:
            display_end = end_angle + 2 * np.pi
            
        theta = np.linspace(start_angle, display_end, 50)
        x = center[0] + radius * np.cos(theta)
        y = center[1] + radius * np.sin(theta)
        self.ax.plot(x, y, color=color, linewidth=1.5, alpha=0.7)
        
        # 标注角度值
        mid_angle = (start_angle + display_end) / 2
        label_x = center[0] + (radius + 0.03) * np.cos(mid_angle)
        label_y = center[1] + (radius + 0.03) * np.sin(mid_angle)
        angle_deg = np.degrees(angle_value)
        self.ax.annotate(f'{label}={angle_deg:.1f}°', (label_x, label_y), 
                        fontsize=8, color=color, fontweight='bold')
    
    def update_info(self, result):
        """更新信息面板"""
        self.info_ax.clear()
        self.info_ax.axis('off')
        
        info_text = "====== 虚功分析对比 ======\n\n"
        
        info_text += "[弹簧物理模型]\n"
        info_text += f"  位置: DC杆上距D {self.spring_length*1000*result.get('scale',1):.1f}mm\n"
        info_text += f"  力大小: {self.spring_force:.0f} N\n"
        info_text += "  力方向: 平行于AD\n"
        info_text += f"  AD x DC = sin(Q3-Q4)\n"
        info_text += f"         = {result.get('AD_DC_cross',0.0):.4f}\n"
        info_text += f"  绕D力矩: {result.get('tau_spring_on_DC',0.0):.2f} N*m\n\n"
        
        info_text += "===== 三种方法对比 =====\n\n"
        
        info_text += "[方法1: 叉乘法(原代码)]\n"
        info_text += "  F_L0 = F*r*sin(Q3-Q4)/\n"
        info_text += "         (AC x DC投影)\n"
        info_text += f"  AC x DC: {result.get('AC_DC_cross',0.0):.4f}\n"
        info_text += f"  前馈力: {result.get('feedforward_force_cross',0.0):.2f} N\n\n"
        
        info_text += "[方法2&3: 虚功法]\n"
        info_text += "  F*d(S.AD)/dQ4 = F_L0*dL0/dQ4\n"
        info_text += f"  dS.AD/dQ4: {result.get('dS_AD_dQ4',0.0):.5f}\n"
        info_text += f"  dL0/dQ4: {result.get('dL0_dQ4',0.0):.5f} m/rad\n"
        info_text += f"  前馈力: {result.get('F_L0_jacobian',0.0):.2f} N\n\n"
        
        info_text += "[验证:Q1方向]\n"
        info_text += f"  dS.AD/dQ1: {result.get('dS_AD_dQ1',0.0):.5f}\n"
        info_text += f"  dL0/dQ1: {result.get('dL0_dQ1',0.0):.5f}\n"
        info_text += f"  F_L0(Q1): {result.get('F_L0_from_Q1',0.0):.2f} N\n\n"

        info_text += "[输入关节扭矩]\n"
        info_text += f"  τ_B: {result.get('tau1',0):.3f} N·m\n"
        info_text += f"  τ_F: {result.get('tau4',0):.3f} N·m\n"
        info_text += f"  等效虚腿力 F_L0: {result.get('F_L0_from_torques',0):.3f} N\n"
        info_text += f"  末端等效力 F_Cx,F_Cy: {result.get('F_Cx_from_torques',0):.3f} , {result.get('F_Cy_from_torques',0):.3f} N\n"
        info_text += f"  残差 τ_B - F·dL0/dQ1: {result.get('res_tau1',0):.4f} N·m\n"
        info_text += f"  残差 τ_F - F·dL0/dQ4: {result.get('res_tau4',0):.4f} N·m\n\n"
        
        # 计算误差百分比（使用安全取值）
        F_L0_jacobian_val = result.get('F_L0_jacobian', 0.0)
        feedforward_val = result.get('feedforward_force_cross', 0.0)
        if abs(F_L0_jacobian_val) > 0.01:
            err1 = (feedforward_val - F_L0_jacobian_val) / abs(F_L0_jacobian_val) * 100
        else:
            err1 = 0
        
        info_text += "===== 结果验证 =====\n"
        info_text += f"  叉乘 vs 虚功: {err1:+.2f}%\n\n"
        
        info_text += "[虚拟腿]\n"
        info_text += f"  L0: {result.get('L0',0.0):.4f} m\n"
        info_text += f"  Q0: {np.degrees(result.get('Q0',0.0)):.2f} deg\n\n"
        
        info_text += "拖动B/C/D点改变姿态"
        if 'total_work' in result:
            info_text += f"\n\n[计算结果]\n  关节做功: {result.get('joint_work',0):.4f} J\n"
            info_text += f"  弹簧做功: {result.get('spring_work',0):.4f} J\n"
            info_text += f"  总做功: {result.get('total_work',0.0):.4f} J"
        
        self.info_ax.text(0, 1, info_text, transform=self.info_ax.transAxes,
                         fontsize=9, verticalalignment='top',
                         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    def on_slider_change(self, val):
        """Slider change callback"""
        self.L1 = self.slider_L1.val
        self.L2 = self.slider_L2.val
        self.L3 = self.slider_L3.val
        self.L4 = self.slider_L4.val
        self.Q1 = self.slider_Q1.val
        self.Q4 = self.slider_Q4.val
        # 读取整体缩放因子
        if hasattr(self, 'slider_scale'):
            self.scale = self.slider_scale.val
        self.spring_length = self.slider_spring_length.val
        self.spring_force = self.slider_spring_force.val
        self.update_plot()
    
    def reset(self, event):
        """Reset button callback"""
        self.slider_L1.reset()
        self.slider_L2.reset()
        self.slider_L3.reset()
        self.slider_L4.reset()
        self.slider_Q1.reset()
        self.slider_Q4.reset()
        if hasattr(self, 'slider_scale'):
            self.slider_scale.reset()
        self.slider_spring_length.reset()
        self.slider_spring_force.reset()

    def on_tau1_submit(self, text):
        try:
            self.tau1 = float(text)
        except Exception:
            self.tau1 = 0.0
        self.update_plot()

    def on_tau4_submit(self, text):
        try:
            self.tau4 = float(text)
        except Exception:
            self.tau4 = 0.0
        self.update_plot()

    def compute_work(self, event):
        """Compute total work when virtual leg length L0 varies across its full range
        Path: keep Q0 = (Q1+Q4_adj)/2 fixed and sweep alpha = (Q1-Q4_adj)/2 over allowed range.
        Uses current `tau1` and `tau4` and numerically integrates W = ∫ F_L0 dL0.
        """
        # Wrap the computation in a try/except and ensure proper indentation
        try:
            # show computing status
            tmp = {'A': (0, 0), 'L0': 0.0, 'Q0': 0.0}
            tmp['info_note'] = '计算中...'
            self.update_info(tmp)
            if hasattr(self, 'ctrl_fig'):
                try:
                    self.ctrl_fig.canvas.draw_idle()
                except Exception:
                    pass

            # get current adjusted Q4 as in forward_kinematics
            Q1 = self.Q1
            Q4 = self.Q4
            Q4_adj = Q4 if Q1 >= Q4 else Q4 - 2*np.pi
            Q0 = (Q1 + Q4_adj) / 2.0

            # scaled lengths
            sL3 = self.L3 * self.scale
            sL4 = self.L4 * self.scale

            # alpha range from geometry constraint EC_sq >= 0: |sin(alpha)| <= sL3/sL4
            if sL4 == 0:
                # nothing to do
                joint_work = 0.0
                spring_work = 0.0
                spring_virtual_work = 0.0
                total_work = 0.0
                res = self.forward_kinematics(self.Q1, self.Q4)
                if res is None:
                    err = {'A': (0, 0), 'L0': 0.0, 'Q0': 0.0}
                    err['info_note'] = '当前构型无解，无法计算做功'
                    self.update_info(err)
                    return
                res['spring_virtual_work'] = 0.0
                res['joint_work'] = joint_work
                res['spring_work'] = spring_work
                res['total_work'] = total_work
                self.latest_work = total_work
                self.update_info(res)
                try:
                    self.fig.canvas.draw_idle()
                    if hasattr(self, 'ctrl_fig'):
                        self.ctrl_fig.canvas.draw_idle()
                except Exception:
                    pass
                return

            ratio = min(1.0, abs(sL3 / sL4))
            alpha_max = np.arcsin(ratio)
            alpha_min = -alpha_max

            # sample alphas
            N = 800
            alphas = np.linspace(alpha_min, alpha_max, N)
            L0_vals = []
            F_vals = []

            # storage for spring virtual-leg force
            F_spring_L0_list = []

            for alpha in alphas:
                q1 = Q0 + alpha
                q4 = Q0 - alpha
                L0 = self._calc_L0(q1, q4)
                if L0 is None:
                    L0_vals.append(np.nan)
                    F_vals.append(0.0)
                    F_spring_L0_list.append(0.0)
                    continue

                # numerical derivatives dL0/dQ1 and dL0/dQ4
                delta = 1e-6
                L0_p = self._calc_L0(q1 + delta, q4)
                L0_m = self._calc_L0(q1 - delta, q4)
                if L0_p is None or L0_m is None:
                    dL0_dQ1 = 0.0
                else:
                    dL0_dQ1 = (L0_p - L0_m) / (2*delta)

                L0_p = self._calc_L0(q1, q4 + delta)
                L0_m = self._calc_L0(q1, q4 - delta)
                if L0_p is None or L0_m is None:
                    dL0_dQ4 = 0.0
                else:
                    dL0_dQ4 = (L0_p - L0_m) / (2*delta)

                denom = dL0_dQ1**2 + dL0_dQ4**2
                if abs(denom) > 1e-12:
                    F_L0 = (self.tau1 * dL0_dQ1 + self.tau4 * dL0_dQ4) / denom
                else:
                    # fallback
                    if abs(dL0_dQ4) > 1e-8:
                        F_L0 = self.tau4 / dL0_dQ4
                    elif abs(dL0_dQ1) > 1e-8:
                        F_L0 = self.tau1 / dL0_dQ1
                    else:
                        F_L0 = 0.0

                L0_vals.append(L0)
                F_vals.append(F_L0)

                # 计算弹簧对虚腿的等效力（基于虚功投影）
                S_plus = self._calc_S_dot_AD(q1 + delta, q4)
                S_minus = self._calc_S_dot_AD(q1 - delta, q4)
                if S_plus is None or S_minus is None:
                    dS_dQ1 = 0.0
                else:
                    dS_dQ1 = (S_plus - S_minus) / (2*delta)

                S_plus = self._calc_S_dot_AD(q1, q4 + delta)
                S_minus = self._calc_S_dot_AD(q1, q4 - delta)
                if S_plus is None or S_minus is None:
                    dS_dQ4 = 0.0
                else:
                    dS_dQ4 = (S_plus - S_minus) / (2*delta)

                if abs(denom) > 1e-12:
                    dot = dS_dQ1 * dL0_dQ1 + dS_dQ4 * dL0_dQ4
                    F_L0_spring = self.spring_force * dot / denom
                else:
                    if abs(dL0_dQ4) > 1e-8:
                        F_L0_spring = self.spring_force * dS_dQ4 / dL0_dQ4
                    elif abs(dL0_dQ1) > 1e-8:
                        F_L0_spring = self.spring_force * dS_dQ1 / dL0_dQ1
                    else:
                        F_L0_spring = 0.0

                F_spring_L0_list.append(F_L0_spring)

            L0_vals = np.array(L0_vals)
            F_vals = np.array(F_vals)
            F_spring_L0 = np.array(F_spring_L0_list)

            # also compute S_AD values for spring work
            S_AD_vals = []
            for alpha in alphas:
                q1 = Q0 + alpha
                q4 = Q0 - alpha
                s = self._calc_S_dot_AD(q1, q4)
                S_AD_vals.append(s if s is not None else np.nan)
            S_AD_vals = np.array(S_AD_vals)

            # remove NaNs and sort by L0 to get monotonic path from min->max
            valid = ~np.isnan(L0_vals)
            if valid.sum() < 2:
                joint_work = 0.0
                spring_work = 0.0
                spring_virtual_work = 0.0
            else:
                L0_clean = L0_vals[valid]
                F_clean = F_vals[valid]
                S_clean = S_AD_vals[valid]
                order = np.argsort(L0_clean)
                L0s = L0_clean[order]
                Fs = F_clean[order]
                Ss = S_clean[order]
                Fs_spring = F_spring_L0[valid][order]
                dL = np.diff(L0s)
                if dL.size > 0:
                    F_mid = 0.5*(Fs[:-1] + Fs[1:])
                    joint_work = np.sum(F_mid * dL)
                else:
                    joint_work = 0.0

                if Ss.size >= 2:
                    spring_work = self.spring_force * (Ss[-1] - Ss[0])
                else:
                    spring_work = 0.0

                if dL.size > 0:
                    F_mid_spring = 0.5*(Fs_spring[:-1] + Fs_spring[1:])
                    spring_virtual_work = np.sum(F_mid_spring * dL)
                else:
                    spring_virtual_work = 0.0

            total_work = joint_work + spring_work

            # attach to current kinematic result and refresh info display
            res = self.forward_kinematics(self.Q1, self.Q4)
            if res is None:
                err = {'A': (0, 0), 'L0': 0.0, 'Q0': 0.0}
                err['info_note'] = '当前构型无解，无法计算做功'
                self.update_info(err)
                return

            res['spring_virtual_work'] = spring_virtual_work
            res['joint_work'] = joint_work
            res['spring_work'] = spring_work
            res['total_work'] = total_work
            self.latest_work = total_work
            self.update_info(res)
            try:
                self.fig.canvas.draw_idle()
                if hasattr(self, 'ctrl_fig'):
                    self.ctrl_fig.canvas.draw_idle()
            except Exception:
                pass
            return

        except Exception as e:
            # display exception message in info panel
            err = {'A': (0, 0), 'L0': 0.0, 'Q0': 0.0}
            err['info_note'] = f'计算出错: {e}'
            self.update_info(err)
            try:
                self.fig.canvas.draw_idle()
            except Exception:
                pass
    
    def on_press(self, event):
        """Mouse press event"""
        if event.inaxes != self.ax:
            return
            
        result = self.forward_kinematics(self.Q1, self.Q4)
        if result is None:
            return
            
        C = result['C']
        B = result['B']
        D = result['D']
        
        # Check distance to each draggable point
        dist_C = np.sqrt((event.xdata - C[0])**2 + (event.ydata - C[1])**2)
        dist_B = np.sqrt((event.xdata - B[0])**2 + (event.ydata - B[1])**2)
        dist_D = np.sqrt((event.xdata - D[0])**2 + (event.ydata - D[1])**2)
        
        # Find closest point within threshold
        threshold = 0.05
        min_dist = min(dist_C, dist_B, dist_D)
        
        if min_dist < threshold:
            self.dragging = True
            if min_dist == dist_C:
                self.drag_point = 'C'
            elif min_dist == dist_B:
                self.drag_point = 'B'
            else:
                self.drag_point = 'D'
    
    def on_release(self, event):
        """Mouse release event"""
        self.dragging = False
        self.drag_point = None
    
    def on_motion(self, event):
        """Mouse motion event"""
        if not self.dragging or event.inaxes != self.ax:
            return
            
        if self.drag_point == 'C':
            # Inverse kinematics for endpoint C
            result = self.inverse_kinematics(event.xdata, event.ydata)
            if result is not None:
                Q1_new, Q4_new = result
                # Limit angle range to [0, 2π]
                Q1_new = Q1_new % (2 * np.pi)
                Q4_new = Q4_new % (2 * np.pi)
                
                self.Q1 = Q1_new
                self.Q4 = Q4_new
                
                # Update sliders
                self.slider_Q1.set_val(self.Q1)
                self.slider_Q4.set_val(self.Q4)
                
                self.update_plot()
                
        elif self.drag_point == 'B':
            # Dragging B point directly changes Q1
            Q1_new = np.arctan2(event.ydata, event.xdata)
            if Q1_new < 0:
                Q1_new += 2 * np.pi
            
            self.Q1 = Q1_new
            self.slider_Q1.set_val(self.Q1)
            self.update_plot()
            
        elif self.drag_point == 'D':
            # Dragging D point directly changes Q4
            Q4_new = np.arctan2(event.ydata, event.xdata)
            if Q4_new < 0:
                Q4_new += 2 * np.pi
            
            self.Q4 = Q4_new
            self.slider_Q4.set_val(self.Q4)
            self.update_plot()
    
    def run(self):
        """Run visualization"""
        plt.show()


if __name__ == '__main__':
    visualizer = VMCVisualizer()
    visualizer.run()