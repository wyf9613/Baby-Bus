"""车辆参数基础清单：None 表示未确认，尚未接入控制器。

MPC 额外动力学参数待方程对齐；测量方法见 docs/vehicle-params-test-plan.md。
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
