declare module 'rclnodejs' {
  /**
   * A wrapper that provides RxJS Observable support for ROS 2 subscriptions.
   * This class wraps a standard Subscription and emits messages through an Observable.
   */
  class ObservableSubscription<T = any> {
    /**
     * Get the RxJS Observable for this subscription.
     * Use this to pipe operators and subscribe to messages.
     */
    readonly observable: import('rxjs').Observable<T>;

    /**
     * Get the underlying ROS 2 subscription.
     */
    readonly subscription: Subscription;

    /**
     * Get the topic name.
     */
    readonly topic: string;

    /**
     * Check if this observable subscription has been destroyed.
     */
    readonly isDestroyed: boolean;

    /**
     * Complete the observable and clean up resources.
     * After calling this, no more messages will be emitted.
     */
    complete(): void;

    /**
     * Alias for complete() for consistency with RxJS naming.
     */
    destroy(): void;
  }
}
