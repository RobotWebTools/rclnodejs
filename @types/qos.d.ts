
declare module "rclnodejs" {

  /** Class representing middleware quality of service */
  class QoS {
    /**
     * Create a QoS.
     * @param {HistoryPolicy} [history=RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT] - The history value.
     * @param {number} [depth=0] - The depth value.
     * @param {ReliabilityPolicy} [reliability=RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT] - The reliability value.
     * @param {DurabilityPolicy} [durability=RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT] - The durability value.
     * @param {boolean} [avoidRosNameSpaceConventions=false] - The avoidRosNameSpaceConventions value.
     */
    constructor(history: QoS.HistoryPolicy,
      depth: number,
      reliability: QoS.ReliabilityPolicy,
      durability: QoS.DurabilityPolicy,
      avoidRosNameSpaceConventions: boolean);

    /**
     * History value.
     * @name QoS#get:history
     */
    history: number;


    /**
     * The depth value.
     * @name QoS#get:depth
     */
    depth: number;


    /**
     * Get the reliability value.
     * @name QoS#get:reliability
     */
    reliability(): QoS.ReliabilityPolicy;


    /**
     * Get the durability value.
     * @name QoS#get:durability
     */
    durability: QoS.DurabilityPolicy;

    /**
     * Get the avoidRosNameSpaceConventions value.
     * @name QoS#get:avoidRosNameSpaceConventions
     */
    avoidRosNameSpaceConventions: boolean;

  }


  namespace QoS {

   /**
     * Default profileref.
     * @name QoS#static get:profileDefault
     */
    export const profileDefault = "qos_profile_default";

    /**
       * Default system profileref.
       * @name QoS#static get:profileSystemDefault
       */
    export const profileSystemDefault = "qos_profile_system_default";

    /**
     * Sensor data profileref.
     * @name QoS#static get:profileSensorData
     */
    export const profileSensorData = "qos_profile_sensor_data";

    /**
     *  Default services profileref.
     * @name QoS#static get:profileServicesDefault
     */
    export const profileServicesDefault = "qos_profile_services_default";

    /**
     * Parameters profileref.
     * @name QoS#static get:profileParameters
     */
    export const profileParameters = "qos_profile_parameters";

    /**
     * The parameter events profileref.
     * @name QoS#static get:profileParameterEvents
     */
    export const profileParameterEvents = "qos_profile_parameter_events";


    export type ProfileRef = string;
    
    /**
     * Enum for HistoryPolicy
     * @readonly
     * @enum {number}
     */
    export enum HistoryPolicy {
      /** @member {number} */
      RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT = 0,
      /** @member {number} */
      RMW_QOS_POLICY_HISTORY_KEEP_LAST = 1,
      /** @member {number} */
      RMW_QOS_POLICY_HISTORY_KEEP_ALL = 2
    }

    /**
     * Enum for ReliabilityPolicy
     * @readonly
     * @enum {number}
     */
    export enum ReliabilityPolicy {
      /** @member {number} */
      RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT = 0,
      /** @member {number} */
      RMW_QOS_POLICY_RELIABILITY_RELIABLE = 1,
      /** @member {number} */
      RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT = 2
    }

    /**
     * Enum for DurabilityPolicy
     * @readonly
     * @enum {number}
     */
    enum DurabilityPolicy {
      /** @member {number} */
      RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT = 0,
      /** @member {number} */
      RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL = 1,
      /** @member {number} */
      RMW_QOS_POLICY_DURABILITY_VOLATILE = 2
    }
  }

}

