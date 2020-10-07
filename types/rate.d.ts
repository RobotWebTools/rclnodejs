declare module 'rclnodejs' {
  /**
   * A timer that runs at a regular frequency (hz).
   *
   * A client calls Rate#sleep() to block until the end of the current cycle.
   * This makes Rate useful for looping at a regular frequency (hz). Rate#sleep()
   * avoids blocking the JS event-loop by returning a Promise that the caller
   * should block on, e.g. use 'await rate.sleep()'.
   *
   * Note that Rate.sleep() does not prevent rclnodejs from invoking callbacks
   * such as a subscription or client if the entity's node is spinning. Thus
   * if your intent is to use rate to synchronize when callbacks are invoked
   * then use a spinOnce() just after rate.sleep() as in the example below.
   *
   * Rate runs within it's own private rcl context. This enables it to be
   * available immediately after construction. That is, unlike Timer, Rate
   * does not require a spin or spinOnce to be active.
   *
   * @example
   * async function run() {
   *   await rclnodejs.init();
   *   const node = rclnodejs.createNode('mynode');
   *   const rate = node.createRate(1); // 1 hz
   *   while (true) {
   *     doSomeStuff();
   *     await rate.sleep();
   *     rclnodejs.spinOnce(node);
   *   }
   * }
   */
  export interface Rate {
    /**
     * Get the frequency in hertz (hz) of this timer.
     *
     * @returns hertz
     */
    readonly frequency: number;

    /**
     * Returns a Promise that when waited on, will block the sender
     * until the end of the current timer cycle.
     *
     * If this Rate has been cancelled, calling this sleep() will
     * result in an error.
     *
     * Note that the Node event-loop is not blocked when a rate is sleep()'ing.
     *
     * @example
     * (async () => {
     *   await rate.sleep();
     * })();
     *
     * @returns A promise
     */
    sleep(): Promise<void>;

    /**
     * Permanently stops the rate's timer.
     */
    cancel(): void;

    /**
     * Determine if this rate has been cancelled.
     *
     * @returns True when cancel() has been called; False otherwise.
     */
    isCanceled(): boolean;
  }
}
