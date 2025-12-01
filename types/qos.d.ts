declare module 'rclnodejs' {
  /**
   * Middleware quality of service
   */

  class QoS {
    /**
     * Create a QoS.
     *
     * @param history - The history value default = RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT
     * @param depth - The depth value, default = 0.
     * @param reliability - The reliability value, default = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT
     * @param durability - The durability value, default = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT
     * @param liveliness - The liveliness value, default = RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT
     * @param avoidRosNameSpaceConventions - The avoidRosNameSpaceConventions value, default = false.
     */
    constructor(
      history?: QoS.HistoryPolicy,
      depth?: number,
      reliability?: QoS.ReliabilityPolicy,
      durability?: QoS.DurabilityPolicy,
      liveliness?: QoS.LivelinessPolicy,
      avoidRosNameSpaceConventions?: boolean
    );

    /**
     * History value.
     */
    history: number;

    /**
     * The depth value.
     */
    depth: number;

    /**
     * Get the reliability value.
     */
    reliability(): QoS.ReliabilityPolicy;

    /**
     * Get the durability value.
     */
    durability: QoS.DurabilityPolicy;

    /**
     * Get the liveliness value.
     */
    liveliness: QoS.LivelinessPolicy;

    /**
     * Get the avoidRosNameSpaceConventions value.
     */
    avoidRosNameSpaceConventions: boolean;
  }

  namespace QoS {
    /**
     * Default profileref.
     */
    export const profileDefault = 'qos_profile_default';

    /**
     * Default system profileref.
     */
    export const profileSystemDefault = 'qos_profile_system_default';

    /**
     * Sensor data profileref.
     */
    export const profileSensorData = 'qos_profile_sensor_data';

    /**
     *  Default services profileref.
     */
    export const profileServicesDefault = 'qos_profile_services_default';

    /**
     * Parameters profileref.
     */
    export const profileParameters = 'qos_profile_parameters';

    /**
     * The parameter events profileref.
     */
    export const profileParameterEvents = 'qos_profile_parameter_events';

    /**
     * The action status profileref.
     */
    export const profileActionStatusDefault =
      'qos_profile_action_status_default';

    /**
     * A named policy reference.
     */
    export type ProfileRef = string;

    /**
     * HistoryPolicy
     */
    export enum HistoryPolicy {
      RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT = 0,
      RMW_QOS_POLICY_HISTORY_KEEP_LAST = 1,
      RMW_QOS_POLICY_HISTORY_KEEP_ALL = 2,
    }

    /**
     * ReliabilityPolicy
     */
    export enum ReliabilityPolicy {
      RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT = 0,
      RMW_QOS_POLICY_RELIABILITY_RELIABLE = 1,
      RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT = 2,
    }

    /**
     * DurabilityPolicy
     */
    enum DurabilityPolicy {
      RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT = 0,
      RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL = 1,
      RMW_QOS_POLICY_DURABILITY_VOLATILE = 2,
    }

    /**
     * LivelinessPolicy
     */
    enum LivelinessPolicy {
      RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT = 0,
      RMW_QOS_POLICY_LIVELINESS_AUTOMATIC = 1,
      RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC = 3,
      RMW_QOS_POLICY_LIVELINESS_UNKNOWN = 4,
      RMW_QOS_POLICY_LIVELINESS_BEST_AVAILABLE = 5,
    }
  }
}
