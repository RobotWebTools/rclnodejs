declare module 'rclnodejs' {
  interface TopicEndpointInfo {
    node_name: string;
    node_namespace: string;
    topic_type: string;
    topic_type_hash: object;
    endpoint_type: number;
    endpoint_gid: number[];
    qos_profile: object;
  }

  interface ServiceEndpointInfo {
    node_name: string;
    node_namespace: string;
    service_type: string;
    service_type_hash: object;
    endpoint_type: number;
    endpoint_count: number;
    endpoint_gids: number[][];
    qos_profiles: object[];
  }

  class ActionEndpointInfo {
    readonly goalServiceInfo: ServiceEndpointInfo;
    readonly cancelServiceInfo: ServiceEndpointInfo;
    readonly resultServiceInfo: ServiceEndpointInfo;
    readonly feedbackTopicInfo: TopicEndpointInfo;
    readonly statusTopicInfo: TopicEndpointInfo;
    readonly nodeName: string;
    readonly nodeNamespace: string;
    readonly actionType: string;
    readonly endpointType: number;
  }
}
