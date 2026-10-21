"""ROS-free compatibility surface for the simulation adapter.

New consumers should import the owning module directly.
"""

from .contracts import (
    Observation,
)

from .geometry import (
    _path_lengths,
    _path_points,
    _path_project,
    _path_sample,
    _path_tangent,
    finite_number,
    quaternion_inverse,
    quaternion_product,
    quaternion_xyzw,
    roll_pitch_yaw,
    rotate_vector,
    transform_position,
    valid_frame,
    wrap_angle,
)

from .motion import (
    EstimationMotionHistory,
    FirstOrderSampleFilter,
)

from .estimation import (
    EstimationPipeline,
    EstimationSettings,
    _estimation_fit_boundary,
    _estimation_median,
    _estimation_order_boundary,
    _estimation_solve,
)

from .planning import (
    CenterlinePlanner,
    PlanningSettings,
    RoadCenterlinePlanner,
    evaluate_planning_path,
    planning_vehicle_limits,
)

from .control import (
    ControlPID,
    ControlReferenceManager,
    ControlSettings,
    PolicyController,
    PolicyStopRequest,
    RunDistanceLimiter,
    VehicleCalibrationSettings,
    _control_centerline_geometry,
    _control_ordered_geometry,
    _control_poly_eval,
    _control_poly_roots,
    control_path_geometry,
)

from .lattice import (
    ShadowFrame,
    ShadowLatticePlanner,
    ShadowSettings,
    _shadow_at,
    _shadow_collision,
    _shadow_curve,
    _shadow_halton,
    _shadow_lengths,
    _shadow_points,
    _shadow_segment_hit,
)

from .transport import (
    ShadowJobSource,
)

from .execution import PlannerControlSettings, PlannerExecutionManager, calculate_selected_control, priming_motion_history
from .learned import LearnedPolicySettings, FrozenPathPolicy, PolicyExecutionMemory
