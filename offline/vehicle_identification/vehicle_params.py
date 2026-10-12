"""前向任务的车辆参数交付：含人工确认、拟合与估算，尚未接入控制器。

数值依据及限制见 docs/vehicle-identification-start.md；MPC 动力学方程仍待对齐。
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class VehicleParams:
    """长度 m、角度 rad（左正）、时间 s；drive/steering_action 为归一化请求。"""

    # 几何：有效轴距、重心至后轴距离（正值）。
    wheelbase_m: float | None = 0.330
    rear_axle_from_cg_m: float | None = 0.132

    # 外廓含轮胎和附件，不含避障裕量；前后距离从重心量起，均为正值。
    body_front_extent_from_cg_m: float | None = 0.297
    body_rear_extent_from_cg_m: float | None = 0.198
    # 最大宽度含转向时轮胎外伸；居中矩形若不对称，取 2*max(左右外伸)。
    body_width_m: float | None = 0.250

    # 等效前轮角稳态近似：delta = gain * steering_action + offset。
    # gain 单位为 rad/单位请求，offset 为零请求偏置；左转角为正。
    # 实测正请求右转，因此 gain 为负：18 deg/action = 0.3142 rad/action。
    steering_gain_rad: float | None = -0.3142
    steering_offset_rad: float | None = 0.0
    # 当前配置下的前轮角边界，含偏置，左右分别测量。
    # 量角器实测：两侧基本对称，0.5 请求约 9°，1.0 请求约 18°。
    steering_min_rad: float | None = -0.3142
    steering_max_rad: float | None = 0.3142
    # 暂用平均转向速率估算：0.3142 / 0.26 ~= 1.21 rad/s；物理速率上限未验证。
    steering_rate_limit_rad_s: float | None = 1.21

    # 前向非零请求工作范围 [drive_min, drive_max]；停车必须另行允许 drive=0。
    # 工作下限 0.3；此前策略 0.3 起步测试未检测到轮速。
    # 遥控实测：起步阈值约 0.35，低速维持请求约 0.3；策略对应值待复核。
    drive_min: float | None = 0.3
    # 用户选定的请求上限；0.7 的 1 s 测试已完成，持续运行速度尚未验证。
    drive_max: float | None = 0.7
    # 局部前向起步模型：drive_action=0.4，静止直行；仅验证到请求后 3 s。
    # t <= 0.26 s 时 v=0；之后 v=1.49*(1-exp(-(t-0.26)/1.33)) m/s。
    # 1.49 m/s 为 3 s 测试的外推稳态值；其他 drive_action 尚需实测。
    drive_response_model: str | None = (
        "Forward local FOPDT at drive_action=0.4: v(t)=0 for t<=0.26 s; "
        "v(t)=1.49*(1-exp(-(t-0.26)/1.33)) m/s for t>0.26 s. "
        "From rest, straight driving; checked for 0<=t<=3 s only. "
        "v_inf is extrapolated from three 3 s runs. Not a mapping for other actions "
        "or a model for zero-drive deceleration."
    )
    # 首次 0.4 测试回零后仍滑行；停车时间/距离为粗略外推，未实测完整停车。
    braking_behavior: str | None = (
        "Continued motion was observed after zero drive from about 1.33 m/s. "
        "The estimated stopping time is 2.5 s and stopping distance is "
        "1.5-1.6 m on the tested surface. Estimates use a linear extrapolation "
        "from the first run's 2 s post-command wheel-speed record; encoder lag "
        "is included. Full physical stopping and active-brake behavior were not "
        "verified. Negative-drive "
        "braking and reverse transitions are outside the forward-only scope."
    )

    # 用户指定的暂用转向延迟；未通过同步角度数据独立测得。
    steering_delay_s: float | None = 0.03
    # 三次日志联合拟合的请求到 wheel-speed 响应的有效延迟；包含编码器观测滞后。
    drive_delay_s: float | None = 0.26
