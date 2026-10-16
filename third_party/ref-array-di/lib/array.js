
/**
 * Module dependencies.
 */

var assert = require('assert')
var debug = require('debug')('ref:array')
var ArrayIndex = require('array-index')
var isArray = Array.isArray

/**
 * The Array "type" constructor.
 * The returned constructor's API is highly influenced by the WebGL
 * TypedArray API.
 */

module.exports = function (_ref) {

var Array_ = function Array (_type, _length) {
  debug('defining new array "type"')
  var type = _ref.coerceType(_type)
  var fixedLength = _length | 0

  /**
   * This is the ArrayType "constructor" that gets returned.
   */

  function ArrayType (data, length) {
    if (!(this instanceof ArrayType)) {
      return new ArrayType(data, length)
    }
    debug('creating new array instance')
    ArrayIndex.call(this)
    var item_size = ArrayType.BYTES_PER_ELEMENT
    if (0 === arguments.length) {
      // new IntArray()
      // use the "fixedLength" if provided, otherwise throw an Error
      if (fixedLength > 0) {
        this.length = fixedLength
        this.buffer = Buffer.alloc(this.length * item_size)
      } else {
        throw new Error('A "length", "array" or "buffer" must be passed as the first argument')
      }
    } else if ('number' == typeof data) {
      // new IntArray(69)
      this.length = data
      this.buffer = Buffer.alloc(this.length * item_size)
    } else if (isArray(data)) {
      // new IntArray([ 1, 2, 3, 4, 5 ], {len})
      // use optional "length" if provided, otherwise use "fixedLength, otherwise
      // use the Array's .length
      var len = 0
      if (null != length) {
        len = length
      } else if (fixedLength > 0) {
        len = fixedLength
      } else {
        len = data.length
      }
      if (data.length < len) {
        throw new Error('array length must be at least ' + len + ', got ' + data.length)
      }
      this.length = len
      this.buffer = Buffer.alloc(len * item_size)
      for (var i = 0; i < len; i++) {
        this[i] = data[i]
      }
    } else if (Buffer.isBuffer(data) || _ref.PointerBuffer.isPointerBuffer(data)) {
      // new IntArray(Buffer(8))
      var len = 0
      if (null != length) {
        len = length
      } else if (fixedLength > 0) {
        len = fixedLength
      } else {
        len = data.length / item_size | 0
      }
      var expectedLength = item_size * len
      this.length = len
      if (data.length != expectedLength) {
        if (data.length < expectedLength) {
          throw new Error('buffer length must be at least ' + expectedLength + ', got ' + data.length)
        } else {
          debug('resizing buffer from %d to %d', data.length, expectedLength)
          data = data.slice(0, expectedLength)
        }
      }
      this.buffer = data
    }
  }

  // make array instances inherit from our `ArrayIndex.prototype`
  ArrayType.prototype = Object.create(ArrayIndex.prototype, {
    constructor: {
      value: ArrayType,
      enumerable: false,
      writable: true,
      configurable: true
    },
    // "buffer" is the backing buffer instance
    buffer: {
      value: _ref.NULL,
      enumerable: true,
      writable: true,
      configurable: true
    },
    // "node-ffi" calls this when passed an array instance to an ffi'd function
    ref: {
      value: ref,
      enumerable: true,
      writable: true,
      configurable: true
    },
    // "slice" implementation
    slice: {
      value: slice,
      enumerable: true,
      writable: true,
      configurable: true
    }
  })

  // part of the "array-index" interface
  ArrayType.prototype[ArrayIndex.get] = getter
  ArrayType.prototype[ArrayIndex.set] = setter

  // save down the "fixedLength" if specified. "ref-struct" needs this value
  if (fixedLength > 0) {
    ArrayType.fixedLength = fixedLength
  }

  // keep a reference to the base "type"
  ArrayType.type = type
  ArrayType.BYTES_PER_ELEMENT = type.indirection == 1 ? type.size : _ref.sizeof.pointer
  assert(ArrayType.BYTES_PER_ELEMENT > 0)

  // the ref "type" interface
  if (fixedLength > 0) {
    // this "type" is probably going in a ref-struct or being used manually
    ArrayType.size = ArrayType.BYTES_PER_ELEMENT * fixedLength
    ArrayType.alignment = type.alignment
    ArrayType.indirection = 1
    ArrayType.get = get
    ArrayType.set = set
  } else {
    // this "type" is probably an argument/return value for a node-ffi function
    ArrayType.size = _ref.sizeof.pointer
    ArrayType.alignment = _ref.alignof.pointer
    ArrayType.indirection = 1
    ArrayType.get = getRef
    ArrayType.set = setRef
  }

  // untilZeros() function
  ArrayType.untilZeros = untilZeros

  ArrayType.toString = function() { return '[ArrayType]';}

  return ArrayType
}

/**
 * The "get" function of the Array "type" interface.
 * Most likely invoked when accessing within a "ref-struct" type.
 */

function get (buffer, offset) {
  debug('Array "type" getter for buffer at offset', offset)
  if (offset > 0) {
    buffer = buffer.slice(offset)
  }
  return new this(buffer)
}

/**
 * The "set" function of the Array "type" interface.
 * Most likely invoked when setting within a "ref-struct" type.
 */

function set (buffer, offset, value) {
  debug('Array "type" setter for buffer at offset', buffer, offset, value)
  var array = this.get(buffer, offset)
  var isInstance = value instanceof this
  if (isInstance || isArray(value)) {
    for (var i = 0; i < value.length; i++) {
      array[i] = value[i]
    }
  } else {
    throw new Error('not sure how to set into Array: ' + value)
  }
}

/**
 * Reads a pointer from the given offset and returns a new "array" instance of
 * this type.
 * Most likely invoked when getting an array instance back as a return value from
 * an FFI'd function.
 */

function getRef (buffer, offset) {
  debug('Array reference "type" getter for buffer at offset', offset)
  return new this(buffer.readPointer(offset))
}

/**
 * Most likely invoked when passing an array instance as an argument to an FFI'd
 * function.
 */

function setRef (buffer, offset, value) {
  debug('Array reference "type" setter for buffer at offset', offset)
  var ptr
  if (value instanceof this) {
    ptr = value.buffer
  } else {
    ptr = new this(value).buffer
  }
  _ref.writePointer(buffer, offset, ptr)
}

/**
 * Returns a reference to the backing buffer of this Array instance.
 *
 * i.e. if the array represents `int[]` (a.k.a. `int *`),
 *      then the returned Buffer represents `int (*)[]` (a.k.a. `int **`)
 */

function ref () {
  debug('ref()')
  var type = this.constructor
  var origSize = this.buffer.length
  var r = _ref.ref(this.buffer)
  r.type = Object.create(_ref.types.CString)
  r.type.get = function (buf, offset) {
    return new type(_ref.readPointer(buf, offset | 0, origSize))
  }
  r.type.set = function () {
    assert(0, 'implement!!!')
  }
  return r
}

/**
 * The "getter" implementation for the "array-index" interface.
 */

function getter (index) {
  debug('getting array[%d]', index)
  var size = this.constructor.BYTES_PER_ELEMENT
  var baseType = this.constructor.type
  var offset = size * index
  var end = offset + size
  var buffer = this.buffer
  if (buffer.length < end) {
    debug('reinterpreting buffer from %d to %d', buffer.length, end)
    buffer = _ref.reinterpret(buffer, end)
  }
  return _ref.get(buffer, offset, baseType)
}

/**
 * The "setter" implementation for  the "array-index" interface.
 */

function setter (index, value) {
  debug('setting array[%d]', index)
  var size = this.constructor.BYTES_PER_ELEMENT
  var baseType = this.constructor.type
  var offset = size * index
  var end = offset + size
  var buffer = this.buffer
  if (buffer.length < end) {
    debug('reinterpreting buffer from %d to %d', buffer.length, end)
    buffer = _ref.reinterpret(buffer, end)
  }
  // TODO: DRY with getter()

  _ref.set(buffer, offset, value, baseType)
  return value
}

/**
 * The "slice" implementation.
 */

function slice (start, end) {
  var data

  if (end) {
    debug('slicing array from %d to %d', start, end)
    data = this.buffer.slice(start*this.constructor.BYTES_PER_ELEMENT, end*this.constructor.BYTES_PER_ELEMENT)
  } else {
    debug('slicing array from %d', start)
    data = this.buffer.slice(start*this.constructor.BYTES_PER_ELEMENT)
  }

  return new this.constructor(data)
}

/**
 * Accepts a Buffer instance that should be an already-populated with data for the
 * ArrayType. The "length" of the Array is determined by searching through the
 * buffer's contents until an aligned NULL pointer is encountered.
 *
 * @param {Buffer} buffer the null-terminated buffer to convert into an Array
 * @api public
 */

function untilZeros (buffer) {
  return new this(_ref.reinterpretUntilZeros(buffer, this.type.size))
}

return Array_;

}
