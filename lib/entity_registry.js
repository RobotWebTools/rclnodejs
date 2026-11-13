// Copyright (c) 2026, The Robot Web Tools Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
    if (entity.finalizeDestruction) {
      entity.finalizeDestruction();
    } else if (entity.handle) {
      entity.handle.release();
    } else {
      entity.destroy();
    }
  }
}

export default EntityRegistry;
