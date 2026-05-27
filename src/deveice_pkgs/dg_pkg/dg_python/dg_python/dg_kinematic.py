# dg_python/dg_kinematic.py
# -*- coding: utf-8 -*-
"""
Kinematic utilities for DG under ROS2.
"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict, Tuple, Optional, Iterable
import numpy as np
import re
import math
import os

def _parse_gamma_theta_txt(path: str) -> Tuple[np.poly1d, Dict[str, float], Dict[str, float]]:   # _:内部使用
    """
    Parse gamma_theta_polyfit.txt produced by MATLAB script.
    Returns:
        (poly, meta, params)
        - poly: np.poly1d, such that gamma(theta_deg) = poly(theta_deg)
        - meta: {'order': int, 'R2': float}
        - params: dict of all scalar parameters from set_parameter.m
    """
    order = None
    R2 = None
    coeffs: Optional[np.ndarray] = None
    params: Dict[str, float] = {}

    with open(path, 'r', encoding='utf-8') as f:
        lines = [ln.strip() for ln in f.readlines() if ln.strip()]

    # Extract order & R2
    for ln in lines:
        if ln.lower().startswith('order:'):
            order = int(ln.split(':', 1)[1].strip())
        elif ln.lower().startswith('r2:'):
            R2 = float(ln.split(':', 1)[1].strip())

    # Extract coefficients: the first line that looks like a list of numbers
    number_line = None
    for ln in lines:
        # A line with scientific/float numbers separated by spaces
        if re.fullmatch(r'[+\-]?\d+(\.\d+)?([eE][+\-]?\d+)?(\s+[+\-]?\d+(\.\d+)?([eE][+\-]?\d+)?)*', ln):
            number_line = ln
            break
    if number_line is None:
        raise ValueError("No coefficient line found in txt.")

    coeffs = np.array([float(x) for x in number_line.split()], dtype=float)

    # Extract parameters: lines like "name = value"
    param_pat = re.compile(r'^([A-Za-z_]\w*)\s*=\s*([+\-]?\d+(\.\d+)?([eE][+\-]?\d+)?)$')
    start_params = False
    for ln in lines:
        if ln.lower().startswith('# parameters'):
            start_params = True
            continue
        if start_params:
            m = param_pat.match(ln)
            if m:
                key = m.group(1)
                val = float(m.group(2))
                params[key] = val

    if order is None:
        order = len(coeffs) - 1  # fallback
    if R2 is None:
        R2 = float('nan')

    poly = np.poly1d(coeffs)  # gamma(theta_deg) = poly(theta_deg)
    meta = {'order': order, 'R2': R2}
    return poly, meta, params


@dataclass
class DGKinematic:
    """
    DG kinematic model.

    Notes on units:
    - theta, gamma are in degrees (consistent with MATLAB fitting).
    - If you prefer radians internally, convert explicitly where needed.
    """
    txt_path: str = "gamma_theta_polyfit.txt"
    theta_deg_min: float = 27.0
    theta_deg_max: float = 147.0

    # Loaded from txt
    gamma_theta_poly: np.poly1d = field(init=False)
    meta: Dict[str, float] = field(init=False)
    params: Dict[str, float] = field(init=False)

    def __post_init__(self):
        self.gamma_theta_poly, self.meta, self.params = _parse_gamma_theta_txt(self.txt_path)

    # ---------------------------
    # θ ↔ lin mapping
    # ---------------------------
    def theta_from_lin(self, lin: float) -> float:
        """
        Convert actuator linear position 'lin' to theta (deg).
        """
        lOA = self.params['sl'] - self.params['sl_s']
        theta = np.degrees(2 * np.arccos(lin / (lOA)))
        return theta

    def lin_from_theta(self, theta_deg: float) -> float:
        """
        Convert theta (deg) to actuator 'lin'.
        """
        lOA = self.params['sl'] - self.params['sl_s']
        theta_rad = np.radians(theta_deg)
        lin = lOA * np.cos(theta_rad / 2)
        return lin
    
    # ---------------------------
    # gamma–theta model & helpers
    # ---------------------------
    def gamma_from_theta(self, theta_deg: float) -> float:
        """Compute gamma (deg) from theta (deg) using polynomial fit.
        self.gamma_theta_poly 是一个 numpy.poly1d 多项式对象。
        它是在类初始化时，从 gamma_theta_polyfit.txt 文件中读取的多项式系数创建的：所以它实际上表示一个函数
        """
        return float(self.gamma_theta_poly(theta_deg))

    def theta_from_gamma(self, gamma_deg: float) -> float:
        """
        Invert gamma(theta)=poly(theta) to find theta in [theta_min, theta_max].
        Uses polynomial root finding and chooses the feasible real root.
        """
        # Solve poly(theta) - gamma = 0
        p = np.poly1d(self.gamma_theta_poly) - gamma_deg
        roots = p.roots # 调用 numpy.poly1d 的 roots 属性，一次性求出该多项式的所有根（可能是复数）
        # Pick real roots within range  过滤“几乎是实数”的根：虚部绝对值 < 1e-7 就当作数值误差忽略。只保留这些根的实部，并转成 Python 浮点数。
        real_roots = [float(r.real) for r in roots if abs(r.imag) < 1e-7]
        feasible = [r for r in real_roots if self.theta_deg_min - 1e-6 <= r <= self.theta_deg_max + 1e-6]  # 只留下位于 [theta_min, theta_max]的根。
        if not feasible:
            # Optional: fallback to numeric bracket solve if needed
            raise ValueError(f"No feasible theta found for gamma={gamma_deg:.6f} deg.")
        # If multiple feasible roots, pick the one consistent with monotonic region (here choose median)
        # You can refine based on mechanism monotonicity if known.
        feasible.sort()
        return feasible[len(feasible)//2]

    # ---------------------------
    # Forward / Inverse Kinematics
    # ---------------------------
    def forward_kinematics(self, lin: float) -> Tuple[float, float, float]:
        """
        Forward kinematics:
            input:  lin (actuator)
            output: (n, m, gamma_deg)

        Steps:
            lin -> theta_deg (via your mapping)
            theta_deg -> gamma_deg (via polynomial)
            theta_deg -> n, m    (mechanism geometry)
        """
        theta_deg = self.theta_from_lin(lin)
        gamma_deg = self.gamma_from_theta(theta_deg)
        n, m = self._nm_from_theta(theta_deg)
        return n, m, gamma_deg

    def inverse_from_gamma(self, gamma_deg: float) -> float:
        """
        Inverse kinematics by gamma:
            input:  gamma_deg
            output: lin
        Steps:
            gamma -> theta_deg (poly inverse)
            theta_deg -> lin (via your inverse mapping)
        """
        theta_deg = self.theta_from_gamma(gamma_deg)
        lin = self.lin_from_theta(theta_deg)
        return lin

    def inverse_from_n(self, n: float) -> float:
        """
        Inverse kinematics by n:
            input:  n
            output: lin
        """
        theta_deg = self._theta_from_n(n)
        lin = self.lin_from_theta(theta_deg)
        return lin

    def inverse_from_m(self, m: float) -> float:
        """
        Inverse kinematics by m:
            input:  m
            output: lin
        """
        theta_deg = self._theta_from_m(m)
        lin = self.lin_from_theta(theta_deg)
        return lin

    # ---------------------------
    # Mechanism-specific geometry
    # ---------------------------
    def _nm_from_theta(self, theta_deg: float) -> Tuple[float, float]:
        """
        Compute (n, m) from theta based on your mechanism geometry.
        """
        gamma_deg = self.gamma_from_theta(theta_deg)  # 用多项式求 γ
        lOC = self.params['sl_s']
        lCF = self.params['gl1']

        theta = np.radians(theta_deg)
        gamma = np.radians(gamma_deg)

        n = 2 * lOC * np.sin(theta / 2)
        m = n - 2 * lCF * np.cos(gamma - (np.pi - theta) / 2)
        return n, m

    def _theta_from_n(self, n: float) -> float:
        """
        Inverse mapping n -> theta.
        """
        lOC = self.params['sl_s']       # l_OC
        ratio = n / (2 * lOC)
        ratio = np.clip(ratio, -1.0, 1.0)  # 避免数值超界
        theta_rad = 2 * np.arcsin(ratio)
        return np.degrees(theta_rad)

    def _theta_from_m(self, m: float) -> float:
        """Inverse mapping m -> theta (deg) via numeric solve."""
        sl_s = self.params['sl_s']   # l_OC
        gl1  = self.params['gl1']    # l_CF
        theta_sw = getattr(self, "theta_sw", 95)

        # 定义目标函数 f(theta) = 0
        def f(theta_deg):
            gamma_deg = self.gamma_from_theta(theta_deg)
            theta = np.radians(theta_deg)
            gamma = np.radians(gamma_deg)
            n = 2 * sl_s * np.sin(theta / 2)
            return n - 2 * gl1 * np.cos(gamma - (np.pi - theta) / 2) - m

        # 二分搜索
        t_min, t_max = theta_sw, self.theta_deg_max
        f_min, f_max = f(t_min), f(t_max)
        if f_min * f_max > 0:
            raise ValueError("No sign change in range — m out of reachable domain.")

        for _ in range(100):  # 迭代 60 次，足够收敛
            t_mid = 0.5 * (t_min + t_max)
            f_mid = f(t_mid)
            if f_min * f_mid <= 0:
                t_max, f_max = t_mid, f_mid
            else:
                t_min, f_min = t_mid, f_mid
            if abs(f_mid) < 1e-6:
                break

        return 0.5 * (t_min + t_max)

    # ---------------------------
    # Utilities
    # ---------------------------
    def clamp_theta(self, theta_deg: float) -> float:
        """Clamp theta into [theta_min, theta_max]."""
        return float(np.clip(theta_deg, self.theta_deg_min, self.theta_deg_max))

    def params_dict(self) -> Dict[str, float]:
        """Return a shallow copy of parameters dict."""
        # 安全地返回类中保存的参数字典（self.params）的一个副本。直接返回 self.params 是不安全的,防止外部用户意外篡改类内部状态
        return dict(self.params)


if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))  # 找到脚本文件所在目录
    txt_path = os.path.join(script_dir, "gamma_theta_polyfit.txt")
    kin = DGKinematic(txt_path=txt_path)

    print("==== DG 运动学验证程序 ====")

    # 1️⃣ 测试正运动学：从 lin → (theta, gamma, n, m)
    test_lin = 18
    print(f"\n[正运动学] 输入 lin = {test_lin:.3f}")

    theta_deg = kin.theta_from_lin(test_lin)
    gamma_deg = kin.gamma_from_theta(theta_deg)
    n, m = kin._nm_from_theta(theta_deg)

    print(f"theta = {theta_deg:.3f} deg")
    print(f"gamma = {gamma_deg:.3f} deg")
    print(f"n = {n:.3f}, m = {m:.3f}")

    # 2️⃣ 逆运动学①：由 gamma 求 lin
    lin_from_gamma = kin.inverse_from_gamma(gamma_deg)
    print(f"\n[逆运动学 γ→lin] gamma={gamma_deg:.3f} → lin={lin_from_gamma:.6f}")
    print(f"误差: {abs(lin_from_gamma - test_lin):.6e}")

    # 3️⃣ 逆运动学②：由 n 求 lin
    lin_from_n = kin.inverse_from_n(n)
    print(f"\n[逆运动学 n→lin] n={n:.3f} → lin={lin_from_n:.6f}")
    print(f"误差: {abs(lin_from_n - test_lin):.6e}")

    # 4️⃣ 逆运动学③：由 m 求 lin
    lin_from_m = kin.inverse_from_m(m)
    print(f"\n[逆运动学 m→lin] m={m:.3f} → lin={lin_from_m:.6f}")
    print(f"误差: {abs(lin_from_m - test_lin):.6e}")

    # 5️⃣ 总结
    print("\n==== 验证完成 ====")