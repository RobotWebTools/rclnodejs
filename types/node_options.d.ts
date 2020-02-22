// eslint camelcase: ["error", {ignoreImports: true}]

import { Parameters } from 'rclnodejs'; 


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
     * When true  
     * Default value = true;
     * @returns {boolean} - 
     */
    startParameterServices: boolean;

    /**
     * An array of Parameters that serve as overrides for a node's default
     * parameters. Default = empty array [].
     */
    parameterOverrides: Array<Parameters.Parameter>;

    /**
     * True indicates that a node shold declare declare parameters from
     * it's parameter-overrides 
     */
    automaticallyDeclareParametersFromOverrides: boolean;
    

    /**
     * Returns an instance configured with default options.
     */
    static defaultOptions: NodeOptions; 
  }
}
