// DO NOT EDIT
// This file is updated by the rostsd_gen script

declare module "rclnodejs" {
  namespace action_msgs {
    namespace msg {
      export type GoalInfo = {
        goal_id: unique_identifier_msgs.msg.UUID,
        stamp: builtin_interfaces.msg.Time
      };
      export type GoalStatus = {
        STATUS_UNKNOWN: 0,
        STATUS_ACCEPTED: 1,
        STATUS_EXECUTING: 2,
        STATUS_CANCELING: 3,
        STATUS_SUCCEEDED: 4,
        STATUS_CANCELED: 5,
        STATUS_ABORTED: 6,
        goal_info: action_msgs.msg.GoalInfo,
        status: number
      };
      export type GoalStatusArray = {
        status_list: action_msgs.msg.GoalStatus[]
      };
    }
  }

  namespace actionlib_msgs {
    namespace msg {
      export type GoalID = {
        stamp: builtin_interfaces.msg.Time,
        id: string
      };
      export type GoalStatus = {
        PENDING: 0,
        ACTIVE: 1,
        PREEMPTED: 2,
        SUCCEEDED: 3,
        ABORTED: 4,
        REJECTED: 5,
        PREEMPTING: 6,
        RECALLING: 7,
        RECALLED: 8,
        LOST: 9,
        goal_id: actionlib_msgs.msg.GoalID,
        status: undefined,
        text: string
      };
      export type GoalStatusArray = {
        header: std_msgs.msg.Header,
        status_list: actionlib_msgs.msg.GoalStatus[]
      };
    }
  }

  namespace builtin_interfaces {
    namespace msg {
      export type Duration = {
        sec: number,
        nanosec: number
      };
      export type Time = {
        sec: number,
        nanosec: number
      };
    }
  }

  namespace composition_interfaces {
  }

  namespace diagnostic_msgs {
    namespace msg {
      export type DiagnosticArray = {
        header: std_msgs.msg.Header,
        status: diagnostic_msgs.msg.DiagnosticStatus[]
      };
      export type DiagnosticStatus = {
        OK: 0,
        WARN: 1,
        ERROR: 2,
        STALE: 3,
        level: number,
        name: string,
        message: string,
        hardware_id: string,
        values: diagnostic_msgs.msg.KeyValue[]
      };
      export type KeyValue = {
        key: string,
        value: string
      };
    }
  }

  namespace geometry_msgs {
    namespace msg {
      export type Accel = {
        linear: geometry_msgs.msg.Vector3,
        angular: geometry_msgs.msg.Vector3
      };
      export type AccelStamped = {
        header: std_msgs.msg.Header,
        accel: geometry_msgs.msg.Accel
      };
      export type AccelWithCovariance = {
        accel: geometry_msgs.msg.Accel,
        covariance: number[]
      };
      export type AccelWithCovarianceStamped = {
        header: std_msgs.msg.Header,
        accel: geometry_msgs.msg.AccelWithCovariance
      };
      export type Inertia = {
        m: number,
        com: geometry_msgs.msg.Vector3,
        ixx: number,
        ixy: number,
        ixz: number,
        iyy: number,
        iyz: number,
        izz: number
      };
      export type InertiaStamped = {
        header: std_msgs.msg.Header,
        inertia: geometry_msgs.msg.Inertia
      };
      export type Point = {
        x: number,
        y: number,
        z: number
      };
      export type Point32 = {
        x: number,
        y: number,
        z: number
      };
      export type PointStamped = {
        header: std_msgs.msg.Header,
        point: geometry_msgs.msg.Point
      };
      export type Polygon = {
        points: geometry_msgs.msg.Point32[]
      };
      export type PolygonStamped = {
        header: std_msgs.msg.Header,
        polygon: geometry_msgs.msg.Polygon
      };
      export type Pose = {
        position: geometry_msgs.msg.Point,
        orientation: geometry_msgs.msg.Quaternion
      };
      export type Pose2D = {
        x: number,
        y: number,
        theta: number
      };
      export type PoseArray = {
        header: std_msgs.msg.Header,
        poses: geometry_msgs.msg.Pose[]
      };
      export type PoseStamped = {
        header: std_msgs.msg.Header,
        pose: geometry_msgs.msg.Pose
      };
      export type PoseWithCovariance = {
        pose: geometry_msgs.msg.Pose,
        covariance: number[]
      };
      export type PoseWithCovarianceStamped = {
        header: std_msgs.msg.Header,
        pose: geometry_msgs.msg.PoseWithCovariance
      };
      export type Quaternion = {
        x: number,
        y: number,
        z: number,
        w: number
      };
      export type QuaternionStamped = {
        header: std_msgs.msg.Header,
        quaternion: geometry_msgs.msg.Quaternion
      };
      export type Transform = {
        translation: geometry_msgs.msg.Vector3,
        rotation: geometry_msgs.msg.Quaternion
      };
      export type TransformStamped = {
        header: std_msgs.msg.Header,
        child_frame_id: string,
        transform: geometry_msgs.msg.Transform
      };
      export type Twist = {
        linear: geometry_msgs.msg.Vector3,
        angular: geometry_msgs.msg.Vector3
      };
      export type TwistStamped = {
        header: std_msgs.msg.Header,
        twist: geometry_msgs.msg.Twist
      };
      export type TwistWithCovariance = {
        twist: geometry_msgs.msg.Twist,
        covariance: number[]
      };
      export type TwistWithCovarianceStamped = {
        header: std_msgs.msg.Header,
        twist: geometry_msgs.msg.TwistWithCovariance
      };
      export type Vector3 = {
        x: number,
        y: number,
        z: number
      };
      export type Vector3Stamped = {
        header: std_msgs.msg.Header,
        vector: geometry_msgs.msg.Vector3
      };
      export type Wrench = {
        force: geometry_msgs.msg.Vector3,
        torque: geometry_msgs.msg.Vector3
      };
      export type WrenchStamped = {
        header: std_msgs.msg.Header,
        wrench: geometry_msgs.msg.Wrench
      };
    }
  }

  namespace lifecycle_msgs {
    namespace msg {
      export type State = {
        PRIMARY_STATE_UNKNOWN: 0,
        PRIMARY_STATE_UNCONFIGURED: 1,
        PRIMARY_STATE_INACTIVE: 2,
        PRIMARY_STATE_ACTIVE: 3,
        PRIMARY_STATE_FINALIZED: 4,
        TRANSITION_STATE_CONFIGURING: 10,
        TRANSITION_STATE_CLEANINGUP: 11,
        TRANSITION_STATE_SHUTTINGDOWN: 12,
        TRANSITION_STATE_ACTIVATING: 13,
        TRANSITION_STATE_DEACTIVATING: 14,
        TRANSITION_STATE_ERRORPROCESSING: 15,
        id: undefined,
        label: string
      };
      export type Transition = {
        TRANSITION_CREATE: 0,
        TRANSITION_CONFIGURE: 1,
        TRANSITION_CLEANUP: 2,
        TRANSITION_ACTIVATE: 3,
        TRANSITION_DEACTIVATE: 4,
        TRANSITION_UNCONFIGURED_SHUTDOWN: 5,
        TRANSITION_INACTIVE_SHUTDOWN: 6,
        TRANSITION_ACTIVE_SHUTDOWN: 7,
        TRANSITION_DESTROY: 8,
        TRANSITION_ON_CONFIGURE_SUCCESS: 10,
        TRANSITION_ON_CONFIGURE_FAILURE: 11,
        TRANSITION_ON_CONFIGURE_ERROR: 12,
        TRANSITION_ON_CLEANUP_SUCCESS: 20,
        TRANSITION_ON_CLEANUP_FAILURE: 21,
        TRANSITION_ON_CLEANUP_ERROR: 22,
        TRANSITION_ON_ACTIVATE_SUCCESS: 30,
        TRANSITION_ON_ACTIVATE_FAILURE: 31,
        TRANSITION_ON_ACTIVATE_ERROR: 32,
        TRANSITION_ON_DEACTIVATE_SUCCESS: 40,
        TRANSITION_ON_DEACTIVATE_FAILURE: 41,
        TRANSITION_ON_DEACTIVATE_ERROR: 42,
        TRANSITION_ON_SHUTDOWN_SUCCESS: 50,
        TRANSITION_ON_SHUTDOWN_FAILURE: 51,
        TRANSITION_ON_SHUTDOWN_ERROR: 52,
        TRANSITION_ON_ERROR_SUCCESS: 60,
        TRANSITION_ON_ERROR_FAILURE: 61,
        TRANSITION_ON_ERROR_ERROR: 62,
        TRANSITION_CALLBACK_SUCCESS: 97,
        TRANSITION_CALLBACK_FAILURE: 98,
        TRANSITION_CALLBACK_ERROR: 99,
        id: undefined,
        label: string
      };
      export type TransitionDescription = {
        transition: lifecycle_msgs.msg.Transition,
        start_state: lifecycle_msgs.msg.State,
        goal_state: lifecycle_msgs.msg.State
      };
      export type TransitionEvent = {
        timestamp: number,
        transition: lifecycle_msgs.msg.Transition,
        start_state: lifecycle_msgs.msg.State,
        goal_state: lifecycle_msgs.msg.State
      };
    }
  }

  namespace logging_demo {
  }

  namespace map_msgs {
    namespace msg {
      export type OccupancyGridUpdate = {
        header: std_msgs.msg.Header,
        x: number,
        y: number,
        width: number,
        height: number,
        data: number[]
      };
      export type PointCloud2Update = {
        ADD: 0,
        DELETE: 1,
        header: std_msgs.msg.Header,
        type: number,
        points: sensor_msgs.msg.PointCloud2
      };
      export type ProjectedMap = {
        map: nav_msgs.msg.OccupancyGrid,
        min_z: number,
        max_z: number
      };
      export type ProjectedMapInfo = {
        frame_id: string,
        x: number,
        y: number,
        width: number,
        height: number,
        min_z: number,
        max_z: number
      };
    }
  }

  namespace nav_msgs {
    namespace msg {
      export type GridCells = {
        header: std_msgs.msg.Header,
        cell_width: number,
        cell_height: number,
        cells: geometry_msgs.msg.Point[]
      };
      export type MapMetaData = {
        map_load_time: builtin_interfaces.msg.Time,
        resolution: number,
        width: number,
        height: number,
        origin: geometry_msgs.msg.Pose
      };
      export type OccupancyGrid = {
        header: std_msgs.msg.Header,
        info: nav_msgs.msg.MapMetaData,
        data: number[]
      };
      export type Odometry = {
        header: std_msgs.msg.Header,
        child_frame_id: string,
        pose: geometry_msgs.msg.PoseWithCovariance,
        twist: geometry_msgs.msg.TwistWithCovariance
      };
      export type Path = {
        header: std_msgs.msg.Header,
        poses: geometry_msgs.msg.PoseStamped[]
      };
    }
  }

  namespace pendulum_msgs {
    namespace msg {
      export type JointCommand = {
        position: number
      };
      export type JointState = {
        position: number,
        velocity: number,
        effort: number
      };
      export type RttestResults = {
        stamp: builtin_interfaces.msg.Time,
        command: pendulum_msgs.msg.JointCommand,
        state: pendulum_msgs.msg.JointState,
        cur_latency: number,
        mean_latency: number,
        min_latency: number,
        max_latency: number,
        minor_pagefaults: number,
        major_pagefaults: number
      };
    }
  }

  namespace rcl_interfaces {
    namespace msg {
      export type FloatingPointRange = {
        from_value: number,
        to_value: number,
        step: number
      };
      export type IntegerRange = {
        from_value: number,
        to_value: number,
        step: number
      };
      export type IntraProcessMessage = {
        publisher_id: number,
        message_sequence: number
      };
      export type ListParametersResult = {
        names: string[],
        prefixes: string[]
      };
      export type Log = {
        DEBUG: 10,
        INFO: 20,
        WARN: 30,
        ERROR: 40,
        FATAL: 50,
        stamp: builtin_interfaces.msg.Time,
        level: undefined,
        name: string,
        msg: string,
        file: string,
        function: string,
        line: number
      };
      export type Parameter = {
        name: string,
        value: rcl_interfaces.msg.ParameterValue
      };
      export type ParameterDescriptor = {
        name: string,
        type: undefined,
        description: string,
        additional_constraints: string,
        read_only: boolean,
        floating_point_range: rcl_interfaces.msg.FloatingPointRange[],
        integer_range: rcl_interfaces.msg.IntegerRange[]
      };
      export type ParameterEvent = {
        stamp: builtin_interfaces.msg.Time,
        node: string,
        new_parameters: rcl_interfaces.msg.Parameter[],
        changed_parameters: rcl_interfaces.msg.Parameter[],
        deleted_parameters: rcl_interfaces.msg.Parameter[]
      };
      export type ParameterEventDescriptors = {
        new_parameters: rcl_interfaces.msg.ParameterDescriptor[],
        changed_parameters: rcl_interfaces.msg.ParameterDescriptor[],
        deleted_parameters: rcl_interfaces.msg.ParameterDescriptor[]
      };
      export type ParameterType = {
        PARAMETER_NOT_SET: 0,
        PARAMETER_BOOL: 1,
        PARAMETER_INTEGER: 2,
        PARAMETER_DOUBLE: 3,
        PARAMETER_STRING: 4,
        PARAMETER_BYTE_ARRAY: 5,
        PARAMETER_BOOL_ARRAY: 6,
        PARAMETER_INTEGER_ARRAY: 7,
        PARAMETER_DOUBLE_ARRAY: 8,
        PARAMETER_STRING_ARRAY: 9      };
      export type ParameterValue = {
        type: undefined,
        bool_value: boolean,
        integer_value: number,
        double_value: number,
        string_value: string,
        byte_array_value: number[],
        bool_array_value: boolean[],
        integer_array_value: number[],
        double_array_value: number[],
        string_array_value: string[]
      };
      export type SetParametersResult = {
        successful: boolean,
        reason: string
      };
    }
  }

  namespace rclnodejs_test_msgs {
    namespace msg {
      export type StaticArrayNonPrimitives = {
        time_value: builtin_interfaces.msg.Time[]
      };
    }
  }

  namespace rosgraph_msgs {
    namespace msg {
      export type Clock = {
        clock: builtin_interfaces.msg.Time
      };
    }
  }

  namespace sensor_msgs {
    namespace msg {
      export type BatteryState = {
        POWER_SUPPLY_STATUS_UNKNOWN: 0,
        POWER_SUPPLY_STATUS_CHARGING: 1,
        POWER_SUPPLY_STATUS_DISCHARGING: 2,
        POWER_SUPPLY_STATUS_NOT_CHARGING: 3,
        POWER_SUPPLY_STATUS_FULL: 4,
        POWER_SUPPLY_HEALTH_UNKNOWN: 0,
        POWER_SUPPLY_HEALTH_GOOD: 1,
        POWER_SUPPLY_HEALTH_OVERHEAT: 2,
        POWER_SUPPLY_HEALTH_DEAD: 3,
        POWER_SUPPLY_HEALTH_OVERVOLTAGE: 4,
        POWER_SUPPLY_HEALTH_UNSPEC_FAILURE: 5,
        POWER_SUPPLY_HEALTH_COLD: 6,
        POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE: 7,
        POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE: 8,
        POWER_SUPPLY_TECHNOLOGY_UNKNOWN: 0,
        POWER_SUPPLY_TECHNOLOGY_NIMH: 1,
        POWER_SUPPLY_TECHNOLOGY_LION: 2,
        POWER_SUPPLY_TECHNOLOGY_LIPO: 3,
        POWER_SUPPLY_TECHNOLOGY_LIFE: 4,
        POWER_SUPPLY_TECHNOLOGY_NICD: 5,
        POWER_SUPPLY_TECHNOLOGY_LIMN: 6,
        header: std_msgs.msg.Header,
        voltage: number,
        temperature: number,
        current: number,
        charge: number,
        capacity: number,
        design_capacity: number,
        percentage: number,
        power_supply_status: undefined,
        power_supply_health: undefined,
        power_supply_technology: undefined,
        present: boolean,
        cell_voltage: number[],
        cell_temperature: number[],
        location: string,
        serial_number: string
      };
      export type CameraInfo = {
        header: std_msgs.msg.Header,
        height: number,
        width: number,
        distortion_model: string,
        d: number[],
        k: number[],
        r: number[],
        p: number[],
        binning_x: number,
        binning_y: number,
        roi: sensor_msgs.msg.RegionOfInterest
      };
      export type ChannelFloat32 = {
        name: string,
        values: number[]
      };
      export type CompressedImage = {
        header: std_msgs.msg.Header,
        format: string,
        data: undefined[]
      };
      export type FluidPressure = {
        header: std_msgs.msg.Header,
        fluid_pressure: number,
        variance: number
      };
      export type Illuminance = {
        header: std_msgs.msg.Header,
        illuminance: number,
        variance: number
      };
      export type Image = {
        header: std_msgs.msg.Header,
        height: number,
        width: number,
        encoding: string,
        is_bigendian: undefined,
        step: number,
        data: undefined[]
      };
      export type Imu = {
        header: std_msgs.msg.Header,
        orientation: geometry_msgs.msg.Quaternion,
        orientation_covariance: number[],
        angular_velocity: geometry_msgs.msg.Vector3,
        angular_velocity_covariance: number[],
        linear_acceleration: geometry_msgs.msg.Vector3,
        linear_acceleration_covariance: number[]
      };
      export type JointState = {
        header: std_msgs.msg.Header,
        name: string[],
        position: number[],
        velocity: number[],
        effort: number[]
      };
      export type Joy = {
        header: std_msgs.msg.Header,
        axes: number[],
        buttons: number[]
      };
      export type JoyFeedback = {
        TYPE_LED: 0,
        TYPE_RUMBLE: 1,
        TYPE_BUZZER: 2,
        type: undefined,
        id: undefined,
        intensity: number
      };
      export type JoyFeedbackArray = {
        array: sensor_msgs.msg.JoyFeedback[]
      };
      export type LaserEcho = {
        echoes: number[]
      };
      export type LaserScan = {
        header: std_msgs.msg.Header,
        angle_min: number,
        angle_max: number,
        angle_increment: number,
        time_increment: number,
        scan_time: number,
        range_min: number,
        range_max: number,
        ranges: number[],
        intensities: number[]
      };
      export type MagneticField = {
        header: std_msgs.msg.Header,
        magnetic_field: geometry_msgs.msg.Vector3,
        magnetic_field_covariance: number[]
      };
      export type MultiDOFJointState = {
        header: std_msgs.msg.Header,
        joint_names: string[],
        transforms: geometry_msgs.msg.Transform[],
        twist: geometry_msgs.msg.Twist[],
        wrench: geometry_msgs.msg.Wrench[]
      };
      export type MultiEchoLaserScan = {
        header: std_msgs.msg.Header,
        angle_min: number,
        angle_max: number,
        angle_increment: number,
        time_increment: number,
        scan_time: number,
        range_min: number,
        range_max: number,
        ranges: sensor_msgs.msg.LaserEcho[],
        intensities: sensor_msgs.msg.LaserEcho[]
      };
      export type NavSatFix = {
        COVARIANCE_TYPE_UNKNOWN: 0,
        COVARIANCE_TYPE_APPROXIMATED: 1,
        COVARIANCE_TYPE_DIAGONAL_KNOWN: 2,
        COVARIANCE_TYPE_KNOWN: 3,
        header: std_msgs.msg.Header,
        status: sensor_msgs.msg.NavSatStatus,
        latitude: number,
        longitude: number,
        altitude: number,
        position_covariance: number[],
        position_covariance_type: undefined
      };
      export type NavSatStatus = {
        STATUS_NO_FIX: -1,
        STATUS_FIX: 0,
        STATUS_SBAS_FIX: 1,
        STATUS_GBAS_FIX: 2,
        SERVICE_GPS: 1,
        SERVICE_GLONASS: 2,
        SERVICE_COMPASS: 4,
        SERVICE_GALILEO: 8,
        status: number,
        service: number
      };
      export type PointCloud = {
        header: std_msgs.msg.Header,
        points: geometry_msgs.msg.Point32[],
        channels: sensor_msgs.msg.ChannelFloat32[]
      };
      export type PointCloud2 = {
        header: std_msgs.msg.Header,
        height: number,
        width: number,
        fields: sensor_msgs.msg.PointField[],
        is_bigendian: boolean,
        point_step: number,
        row_step: number,
        data: undefined[],
        is_dense: boolean
      };
      export type PointField = {
        INT8: 1,
        UINT8: 2,
        INT16: 3,
        UINT16: 4,
        INT32: 5,
        UINT32: 6,
        FLOAT32: 7,
        FLOAT64: 8,
        name: string,
        offset: number,
        datatype: undefined,
        count: number
      };
      export type Range = {
        ULTRASOUND: 0,
        INFRARED: 1,
        header: std_msgs.msg.Header,
        radiation_type: undefined,
        field_of_view: number,
        min_range: number,
        max_range: number,
        range: number
      };
      export type RegionOfInterest = {
        x_offset: number,
        y_offset: number,
        height: number,
        width: number,
        do_rectify: boolean
      };
      export type RelativeHumidity = {
        header: std_msgs.msg.Header,
        relative_humidity: number,
        variance: number
      };
      export type Temperature = {
        header: std_msgs.msg.Header,
        temperature: number,
        variance: number
      };
      export type TimeReference = {
        header: std_msgs.msg.Header,
        time_ref: builtin_interfaces.msg.Time,
        source: string
      };
    }
  }

  namespace shape_msgs {
    namespace msg {
      export type Mesh = {
        triangles: shape_msgs.msg.MeshTriangle[],
        vertices: geometry_msgs.msg.Point[]
      };
      export type MeshTriangle = {
        vertex_indices: number[]
      };
      export type Plane = {
        coef: number[]
      };
      export type SolidPrimitive = {
        BOX: 1,
        SPHERE: 2,
        CYLINDER: 3,
        CONE: 4,
        BOX_X: 0,
        BOX_Y: 1,
        BOX_Z: 2,
        SPHERE_RADIUS: 0,
        CYLINDER_HEIGHT: 0,
        CYLINDER_RADIUS: 1,
        CONE_HEIGHT: 0,
        CONE_RADIUS: 1,
        type: undefined,
        dimensions: number[]
      };
    }
  }

  namespace std_msgs {
    namespace msg {
      export type Bool = {
        data: boolean
      };
      export type Byte = {
        data: number
      };
      export type ByteMultiArray = {
        layout: std_msgs.msg.MultiArrayLayout,
        data: number[]
      };
      export type Char = {
        data: number
      };
      export type ColorRGBA = {
        r: number,
        g: number,
        b: number,
        a: number
      };
      export type Empty = {
      };
      export type Float32 = {
        data: number
      };
      export type Float32MultiArray = {
        layout: std_msgs.msg.MultiArrayLayout,
        data: number[]
      };
      export type Float64 = {
        data: number
      };
      export type Float64MultiArray = {
        layout: std_msgs.msg.MultiArrayLayout,
        data: number[]
      };
      export type Header = {
        stamp: builtin_interfaces.msg.Time,
        frame_id: string
      };
      export type Int16 = {
        data: number
      };
      export type Int16MultiArray = {
        layout: std_msgs.msg.MultiArrayLayout,
        data: number[]
      };
      export type Int32 = {
        data: number
      };
      export type Int32MultiArray = {
        layout: std_msgs.msg.MultiArrayLayout,
        data: number[]
      };
      export type Int64 = {
        data: number
      };
      export type Int64MultiArray = {
        layout: std_msgs.msg.MultiArrayLayout,
        data: number[]
      };
      export type Int8 = {
        data: number
      };
      export type Int8MultiArray = {
        layout: std_msgs.msg.MultiArrayLayout,
        data: number[]
      };
      export type MultiArrayDimension = {
        label: string,
        size: number,
        stride: number
      };
      export type MultiArrayLayout = {
        dim: std_msgs.msg.MultiArrayDimension[],
        data_offset: number
      };
      export type String = {
        data: string
      };
      export type UInt16 = {
        data: number
      };
      export type UInt16MultiArray = {
        layout: std_msgs.msg.MultiArrayLayout,
        data: number[]
      };
      export type UInt32 = {
        data: number
      };
      export type UInt32MultiArray = {
        layout: std_msgs.msg.MultiArrayLayout,
        data: number[]
      };
      export type UInt64 = {
        data: number
      };
      export type UInt64MultiArray = {
        layout: std_msgs.msg.MultiArrayLayout,
        data: number[]
      };
      export type UInt8 = {
        data: undefined
      };
      export type UInt8MultiArray = {
        layout: std_msgs.msg.MultiArrayLayout,
        data: undefined[]
      };
    }
  }

  namespace std_srvs {
  }

  namespace stereo_msgs {
    namespace msg {
      export type DisparityImage = {
        header: std_msgs.msg.Header,
        image: sensor_msgs.msg.Image,
        f: number,
        t: number,
        valid_window: sensor_msgs.msg.RegionOfInterest,
        min_disparity: number,
        max_disparity: number,
        delta_d: number
      };
    }
  }

  namespace trajectory_msgs {
    namespace msg {
      export type JointTrajectory = {
        header: std_msgs.msg.Header,
        joint_names: string[],
        points: trajectory_msgs.msg.JointTrajectoryPoint[]
      };
      export type JointTrajectoryPoint = {
        positions: number[],
        velocities: number[],
        accelerations: number[],
        effort: number[],
        time_from_start: builtin_interfaces.msg.Duration
      };
      export type MultiDOFJointTrajectory = {
        header: std_msgs.msg.Header,
        joint_names: string[],
        points: trajectory_msgs.msg.MultiDOFJointTrajectoryPoint[]
      };
      export type MultiDOFJointTrajectoryPoint = {
        transforms: geometry_msgs.msg.Transform[],
        velocities: geometry_msgs.msg.Twist[],
        accelerations: geometry_msgs.msg.Twist[],
        time_from_start: builtin_interfaces.msg.Duration
      };
    }
  }

  namespace unique_identifier_msgs {
    namespace msg {
      export type UUID = {
        uuid: undefined[]
      };
    }
  }

  namespace visualization_msgs {
    namespace msg {
      export type ImageMarker = {
        CIRCLE: 0,
        LINE_STRIP: 1,
        LINE_LIST: 2,
        POLYGON: 3,
        POINTS: 4,
        ADD: 0,
        REMOVE: 1,
        header: std_msgs.msg.Header,
        ns: string,
        id: number,
        type: number,
        action: number,
        position: geometry_msgs.msg.Point,
        scale: number,
        outline_color: std_msgs.msg.ColorRGBA,
        filled: undefined,
        fill_color: std_msgs.msg.ColorRGBA,
        lifetime: builtin_interfaces.msg.Duration,
        points: geometry_msgs.msg.Point[],
        outline_colors: std_msgs.msg.ColorRGBA[]
      };
      export type InteractiveMarker = {
        header: std_msgs.msg.Header,
        pose: geometry_msgs.msg.Pose,
        name: string,
        description: string,
        scale: number,
        menu_entries: visualization_msgs.msg.MenuEntry[],
        controls: visualization_msgs.msg.InteractiveMarkerControl[]
      };
      export type InteractiveMarkerControl = {
        INHERIT: 0,
        FIXED: 1,
        VIEW_FACING: 2,
        NONE: 0,
        MENU: 1,
        BUTTON: 2,
        MOVE_AXIS: 3,
        MOVE_PLANE: 4,
        ROTATE_AXIS: 5,
        MOVE_ROTATE: 6,
        MOVE_3D: 7,
        ROTATE_3D: 8,
        MOVE_ROTATE_3D: 9,
        name: string,
        orientation: geometry_msgs.msg.Quaternion,
        orientation_mode: undefined,
        interaction_mode: undefined,
        always_visible: boolean,
        markers: visualization_msgs.msg.Marker[],
        independent_marker_orientation: boolean,
        description: string
      };
      export type InteractiveMarkerFeedback = {
        KEEP_ALIVE: 0,
        POSE_UPDATE: 1,
        MENU_SELECT: 2,
        BUTTON_CLICK: 3,
        MOUSE_DOWN: 4,
        MOUSE_UP: 5,
        header: std_msgs.msg.Header,
        client_id: string,
        marker_name: string,
        control_name: string,
        event_type: undefined,
        pose: geometry_msgs.msg.Pose,
        menu_entry_id: number,
        mouse_point: geometry_msgs.msg.Point,
        mouse_point_valid: boolean
      };
      export type InteractiveMarkerInit = {
        server_id: string,
        seq_num: number,
        markers: visualization_msgs.msg.InteractiveMarker[]
      };
      export type InteractiveMarkerPose = {
        header: std_msgs.msg.Header,
        pose: geometry_msgs.msg.Pose,
        name: string
      };
      export type InteractiveMarkerUpdate = {
        KEEP_ALIVE: 0,
        UPDATE: 1,
        server_id: string,
        seq_num: number,
        type: undefined,
        markers: visualization_msgs.msg.InteractiveMarker[],
        poses: visualization_msgs.msg.InteractiveMarkerPose[],
        erases: string[]
      };
      export type Marker = {
        ARROW: 0,
        CUBE: 1,
        SPHERE: 2,
        CYLINDER: 3,
        LINE_STRIP: 4,
        LINE_LIST: 5,
        CUBE_LIST: 6,
        SPHERE_LIST: 7,
        POINTS: 8,
        TEXT_VIEW_FACING: 9,
        MESH_RESOURCE: 10,
        TRIANGLE_LIST: 11,
        ADD: 0,
        MODIFY: 0,
        DELETE: 2,
        DELETEALL: 3,
        header: std_msgs.msg.Header,
        ns: string,
        id: number,
        type: number,
        action: number,
        pose: geometry_msgs.msg.Pose,
        scale: geometry_msgs.msg.Vector3,
        color: std_msgs.msg.ColorRGBA,
        lifetime: builtin_interfaces.msg.Duration,
        frame_locked: boolean,
        points: geometry_msgs.msg.Point[],
        colors: std_msgs.msg.ColorRGBA[],
        text: string,
        mesh_resource: string,
        mesh_use_embedded_materials: boolean
      };
      export type MarkerArray = {
        markers: visualization_msgs.msg.Marker[]
      };
      export type MenuEntry = {
        FEEDBACK: 0,
        ROSRUN: 1,
        ROSLAUNCH: 2,
        id: number,
        parent_id: number,
        title: string,
        command: string,
        command_type: undefined
      };
    }
  }

}
