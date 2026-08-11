from dataclasses import dataclass, field


@dataclass
class SensorConfig:
    width: float = 80.0
    height: float = 60.0
    depth: float = 80.0

    def to_dict(self):
        return {"width": self.width, "height": self.height, "depth": self.depth}


@dataclass
class ViewpointRecord:
    point: object
    pose: object
    face_index: int
    kind: str
    valid: bool = True
    collision: bool = False


@dataclass
class AppState:
    # Model state
    current_shape: object = None
    current_faces: list = field(default_factory=list)
    # Non-BRep patch records used by trim-aware mesh-grid segmentation.  The
    # legacy current_faces list remains populated for downstream compatibility.
    surface_patches: list = field(default_factory=list)
    last_segmentation_diagnostics: dict = field(default_factory=dict)
    selected_face: object = None
    original_face_normal: object = None

    # Geometry
    face_centers: list = field(default_factory=list)
    face_normals: list = field(default_factory=list)
    center_view_points: list = field(default_factory=list)
    view_points: list = field(default_factory=list)

    # Viewpoints with pose
    center_view_points_with_pose: list = field(default_factory=list)
    view_points_with_pose: list = field(default_factory=list)

    # Optimal
    optimal_viewpoints: list = field(default_factory=list)
    optimal_viewpoints_with_pose: list = field(default_factory=list)

    # Path
    optimal_path: list = field(default_factory=list)
    last_path_length: float = 0.0

    # Speed planning (created only after an ordered path exists)
    speed_plan_result: object = None
    last_speed_csv_path: str = ""
    last_robodk_import: dict = field(default_factory=dict)

    # Scanner mounting calibration. The object is an ExtrinsicConfig instance.
    extrinsic_config: object = None
    extrinsic_config_path: str = ""
    extrinsic_config_sha256: str = ""

    # Collision / sensor
    sensor_size_config: dict = field(default_factory=lambda: {"width": 80, "height": 60, "depth": 80})
    sensor_volumes_list: list = field(default_factory=list)
    face_obbs: list = field(default_factory=list)

    # Visual objects (display handles)
    workpiece_coordinate_system_size: float = 0.0
    workpiece_coordinate_system_objects: list = field(default_factory=list)
    coordinate_systems: list = field(default_factory=list)
    normal_line_objects: list = field(default_factory=list)
    optimal_path_objects: list = field(default_factory=list)
    sensor_volume_objects: list = field(default_factory=list)
    obb_visualizations: list = field(default_factory=list)

    # Reference normal for consistent face orientation
    reference_normal: object = None

    # Visibility flags
    show_model: bool = True
    show_workpiece_coordinate_system: bool = True
    show_face_centers: bool = True
    show_normal_lines: bool = True
    show_all_viewpoints: bool = True
    show_optimal_viewpoints: bool = True
    show_planned_path: bool = True
    show_sensor_volumes: bool = True
    show_obb_boxes: bool = True

    # Process flags
    collision_process_started: bool = False
    collision_process_finished: bool = False
    collision_process_cancelled: bool = False
    collision_detection_executed: bool = False
    sensor_volumes_created: bool = False
    obb_boxes_generated: bool = False

    # Workflow
    workflow_steps: list = field(default_factory=list)

    def reset_all(self):
        self.current_shape = None
        self.current_faces.clear()
        self.surface_patches.clear()
        self.last_segmentation_diagnostics.clear()
        self.selected_face = None
        self.original_face_normal = None
        self.face_centers.clear()
        self.face_normals.clear()
        self.center_view_points.clear()
        self.view_points.clear()
        self.center_view_points_with_pose.clear()
        self.view_points_with_pose.clear()
        self.optimal_viewpoints.clear()
        self.optimal_viewpoints_with_pose.clear()
        self.optimal_path.clear()
        self.last_path_length = 0.0
        self.speed_plan_result = None
        self.last_speed_csv_path = ""
        self.last_robodk_import.clear()
        self.sensor_volumes_list.clear()
        self.face_obbs.clear()
        self.workpiece_coordinate_system_size = 0.0
        self.workpiece_coordinate_system_objects.clear()
        self.coordinate_systems.clear()
        self.normal_line_objects.clear()
        self.optimal_path_objects.clear()
        self.sensor_volume_objects.clear()
        self.obb_visualizations.clear()
        self.reference_normal = None
        self.collision_process_started = False
        self.collision_process_finished = False
        self.collision_process_cancelled = False
        self.collision_detection_executed = False
        self.sensor_volumes_created = False
        self.obb_boxes_generated = False

    def invalidate_after_segmentation(self):
        self.face_centers.clear()
        self.face_normals.clear()
        self.center_view_points.clear()
        self.view_points.clear()
        self.center_view_points_with_pose.clear()
        self.view_points_with_pose.clear()
        self.optimal_viewpoints.clear()
        self.optimal_viewpoints_with_pose.clear()
        self.sensor_volumes_list.clear()
        self.face_obbs.clear()
        self.optimal_path.clear()
        self.last_path_length = 0.0
        self.speed_plan_result = None
        self.last_speed_csv_path = ""
        self.last_robodk_import.clear()
        self.reference_normal = None
        self.collision_process_started = False
        self.collision_process_finished = False
        self.collision_process_cancelled = False
        self.collision_detection_executed = False
        self.sensor_volumes_created = False
        self.obb_boxes_generated = False
