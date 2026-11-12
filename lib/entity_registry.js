const WAITABLE_COLLECTIONS = new Set([
  'subscriptions',
  'clients',
  'services',
  'timers',
  'guards',
  'events',
  'actionClients',
  'actionServers',
]);

const DESTRUCTION_ORDER = [
  'actionClients',
  'actionServers',
  'parameterClients',
  'parameterWatchers',
  'parameterEventHandlers',
  'timers',
  'events',
  'publishers',
  'subscriptions',
  'clients',
  'services',
  'guards',
];

class EntityRegistry {
  constructor(node) {
    this._node = node;
    this._destroyingAll = false;
    this.collections = {
      publishers: [],
      subscriptions: [],
      clients: [],
      services: [],
      timers: [],
      guards: [],
      events: [],
      actionClients: [],
      actionServers: [],
      parameterClients: [],
      parameterWatchers: [],
      parameterEventHandlers: [],
    };
  }

  register(kind, entity) {
    this.collections[kind].push(entity);
    const events = entity.events || [];
    this.collections.events.push(...events);
    if (WAITABLE_COLLECTIONS.has(kind) || events.length > 0) {
      this._node.syncHandles();
    }
    return entity;
  }

  _remove(kind, entity) {
    const entities = this.collections[kind];
    const index = entities.indexOf(entity);
    if (index > -1) {
      entities.splice(index, 1);
    }
  }

  destroy(kind, entity) {
    this._remove(kind, entity);
    if (entity.isDestroyed && entity.isDestroyed()) return;

    const events = [...(entity.events || [])];
    for (const event of events) {
      this._remove('events', event);
    }
    if (
      !this._destroyingAll &&
      (WAITABLE_COLLECTIONS.has(kind) || events.length > 0)
    ) {
      this._node.syncHandles();
    }

    for (const event of events) {
      this._release(event);
    }
    if (entity.events) {
      entity.events = [];
    }
    this._release(entity);
  }

  destroyAll() {
    this._destroyingAll = true;
    try {
      for (const kind of DESTRUCTION_ORDER) {
        for (const entity of [...this.collections[kind]]) {
          this.destroy(kind, entity);
        }
      }
    } finally {
      this._destroyingAll = false;
    }
  }

  _release(entity) {
    if (entity._destroy) {
      entity._destroy();
    } else if (entity.handle) {
      entity.handle.release();
    } else {
      entity.destroy();
    }
  }
}

export default EntityRegistry;
