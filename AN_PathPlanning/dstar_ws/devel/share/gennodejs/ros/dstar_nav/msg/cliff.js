// Auto-generated. Do not edit!

// (in-package dstar_nav.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class cliff {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.coordinate = null;
      this.vectors = null;
    }
    else {
      if (initObj.hasOwnProperty('coordinate')) {
        this.coordinate = initObj.coordinate
      }
      else {
        this.coordinate = [];
      }
      if (initObj.hasOwnProperty('vectors')) {
        this.vectors = initObj.vectors
      }
      else {
        this.vectors = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type cliff
    // Serialize message field [coordinate]
    bufferOffset = _arraySerializer.float32(obj.coordinate, buffer, bufferOffset, null);
    // Serialize message field [vectors]
    bufferOffset = _arraySerializer.float32(obj.vectors, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type cliff
    let len;
    let data = new cliff(null);
    // Deserialize message field [coordinate]
    data.coordinate = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [vectors]
    data.vectors = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.coordinate.length;
    length += 4 * object.vectors.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dstar_nav/cliff';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bfa410a2b0e06309593d5c5ca9ddf49e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] coordinate
    float32[] vectors
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new cliff(null);
    if (msg.coordinate !== undefined) {
      resolved.coordinate = msg.coordinate;
    }
    else {
      resolved.coordinate = []
    }

    if (msg.vectors !== undefined) {
      resolved.vectors = msg.vectors;
    }
    else {
      resolved.vectors = []
    }

    return resolved;
    }
};

module.exports = cliff;
