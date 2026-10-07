"""车辆参数基础清单：None 表示未确认。

运行时载体是 scripts/policy_node.py 的 VehicleParamsSettings（vehicle.* 参数），
字段与本文件一一对应（tests/test_mpc.py 检查）；确认后的数值填入 YAML 的
vehicle.*，并设置 valid/source。测量方法见 docs/vehicle-params-test-plan.md。
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class VehicleParams:
    """长度 m、角度 rad（左正）、时间 s；drive/steering_action 为归一化请求。"""

    # 几何：有效轴距、重心至后轴距离（正值）。
    wheelbase_m: float | None = None
    rear_axle_from_cg_m: float | None = None

    # 外廓含轮胎和附件，不含避障裕量；前后距离从重心量起，均为正值。
    body_front_extent_from_cg_m: float | None = None
    body_rear_extent_from_cg_m: float | None = None
    # 最大宽度含转向时轮胎外伸；居中矩形若不对称，取 2*max(左右外伸)。
    body_width_m: float | None = None

    # 等效前轮角稳态近似：delta = gain * steering_action + offset。
    # gain 单位为 rad/单位请求，offset 为零请求偏置。
    steering_gain_rad: float | None = None
    steering_offset_rad: float | None = None
    # 当前配置下的前轮角边界，含偏置，左右分别测量。
    steering_min_rad: float | None = None
    steering_max_rad: float | None = None
    # 实际前轮角变化率上限（rad/s），不是归一化命令变化率。
    steering_rate_limit_rad_s: float | None = None

    # 驱动工作范围：-1 <= drive_min < drive_max <= 1。
    drive_min: float | None = None
    drive_max: float | None = None
    # 驱动请求到速度响应的模型说明：方程、系数单位及适用范围。
    drive_response_model: str | None = None
    # 实测零/负驱动行为及制动、倒车切换规则。
    braking_behavior: str | None = None

    # 请求到物理响应开始的延迟；区分观测滤波滞后与完整响应时间。
    steering_delay_s: float | None = None
    drive_delay_s: float | None = None

    # MPC 纵向模型的数值形式（offline/mpc_prediction_model）：
    # m*dv/dt = gain*(drive - drive_deadband) - drag_kg_per_m*v*|v|，
    # drive 不低于死区时 gain = motor_gain_n，低于死区时 gain = brake_gain_n（电调拖刹）。
    mass_kg: float | None = None
    motor_gain_n: float | None = None
    drag_kg_per_m: float | None = None
    drive_deadband: float | None = None
    brake_gain_n: float | None = None
    # 静止起步：请求至少 drive_breakaway 并保持 breakaway_wait_s 后车轮才转（静摩擦）。
    drive_breakaway: float | None = None
    breakaway_wait_s: float | None = None

    # Planning 需要的工作域限值（m/s、m/s²、m）；延迟取上面两个延迟的较大值。
    speed_max_mps: float | None = None
    braking_deceleration_mps2: float | None = None
    safety_margin_m: float | None = None
