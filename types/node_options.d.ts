// eslint camelcase: ["error", {ignoreImports: true}]

declare module 'rclnodejs' {
  /**
   * NodeOptions specify configuration choices during the
   * node instantiation process.
   * @class
   */
  class NodeOptions {
    /**
     * Create a new instance with default property values.
     */
    constructor();

    /**
     * A flag controlling the startup of a node's parameter-service.
     * When true a node will start it's parameter-service during initialization.
     * Default value = true;
     * @returns {boolean} -
     */
    startParameterServices: boolean;

    /**
     * An array of Parameters that serve as overrides for a node's default
     * parameters. Default = empty array [].
     */
    parameterOverrides: Array<Parameter>;

    /**
     * Instructs a node if it should declare parameters from it's
     * parameter-overrides.
     */
    automaticallyDeclareParametersFromOverrides: boolean;

    /**
     * An instance configured with default values.
     */
    static defaultOptions: NodeOptions;
  }
}
