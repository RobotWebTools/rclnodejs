/**
 * Module dependencies.
 */

const debug = require('debug')('array-index');

const get = Symbol('get');
const set = Symbol('set');
const length = Symbol('length');

/**
 * JavaScript Array "length" is bound to an unsigned 32-bit int.
 * See: http://stackoverflow.com/a/6155063/376773
 */

const MAX_LENGTH = Math.pow(2, 32);

/**
 * Module exports.
 */

module.exports = ArrayIndex;
ArrayIndex.get = get;
ArrayIndex.set = set;

/**
 * Subclass this.
 */

function ArrayIndex(_length) {
  Object.defineProperty(this, 'length', {
    get: getLength,
    set: setLength,
    enumerable: false,
    configurable: true,
  });

  this[length] = 0;

  if (arguments.length > 0) {
    setLength.call(this, _length);
  }
}

/**
 * You overwrite the "get" Symbol in your subclass.
 */

ArrayIndex.prototype[ArrayIndex.get] = function () {
  throw new Error('you must implement the `ArrayIndex.get` Symbol');
};

/**
 * You overwrite the "set" Symbol in your subclass.
 */

ArrayIndex.prototype[ArrayIndex.set] = function () {
  throw new Error('you must implement the `ArrayIndex.set` Symbol');
};

/**
 * Converts this array class into a real JavaScript Array. Note that this
 * is a "flattened" array and your defined getters and setters won't be invoked
 * when you interact with the returned Array. This function will call the
 * getter on every array index of the object.
 *
 * @return {Array} The flattened array
 * @api public
 */

ArrayIndex.prototype.toArray = function toArray() {
  const arrayLength = this.length;
  const array = new Array(arrayLength);
  for (let index = 0; index < arrayLength; index++) {
    array[index] = this[index];
  }
  return array;
};

/**
 * Basic support for `JSON.stringify()`.
 */

ArrayIndex.prototype.toJSON = function toJSON() {
  return this.toArray();
};

/**
 * toString() override. Use Array.prototype.toString().
 */

ArrayIndex.prototype.toString = function toString() {
  const array = this.toArray();
  return array.toString.apply(array, arguments);
};

/**
 * inspect() override. For the REPL.
 */

ArrayIndex.prototype.inspect = function inspect() {
  const array = this.toArray();
  Object.keys(this).forEach(function (key) {
    array[key] = this[key];
  }, this);
  return array;
};

/**
 * Getter for the "length" property.
 * Returns the value of the "length" Symbol.
 */

function getLength() {
  debug('getting "length": %o', this[length]);
  return this[length];
}

/**
 * Setter for the "length" property.
 * Calls "ensureLength()", then sets the "length" Symbol.
 */

function setLength(value) {
  debug('setting "length": %o', value);
  return (this[length] = ensureLength(this, value));
}

/**
 * Ensures that getters/setters from 0 up to "_newLength" have been defined
 * on `Object.getPrototypeOf(this)`.
 *
 * @api private
 */

function ensureLength(self, _newLength) {
  let newLength;
  if (_newLength > MAX_LENGTH) {
    newLength = MAX_LENGTH;
  } else {
    newLength = _newLength | 0;
  }
  const proto = Object.getPrototypeOf(self);
  const current = proto[length] | 0;
  const count = newLength - current;
  if (count > 0) {
    const descriptors = {};
    debug('creating a descriptor object with %o entries', count);
    for (let index = current; index < newLength; index++) {
      descriptors[index] = setup(index);
    }
    debug('calling `Object.defineProperties()` with %o entries', count);
    Object.defineProperties(proto, descriptors);
    proto[length] = newLength;
  }
  return newLength;
}

/**
 * Returns a property descriptor for the given "index", with "get" and "set"
 * functions created within the closure.
 *
 * @api private
 */

function setup(index) {
  function get() {
    return this[ArrayIndex.get](index);
  }
  function set(value) {
    return this[ArrayIndex.set](index, value);
  }
  return {
    enumerable: true,
    configurable: true,
    get: get,
    set: set,
  };
}
