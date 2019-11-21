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

class envData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.setMap = null;
      this.cliff = null;
      this.x = null;
      this.y = null;
      this.z = null;
      this.val = null;
      this.vectors = null;
    }
    else {
      if (initObj.hasOwnProperty('setMap')) {
        this.setMap = initObj.setMap
      }
      else {
        this.setMap = false;
      }
      if (initObj.hasOwnProperty('cliff')) {
        this.cliff = initObj.cliff
      }
      else {
        this.cliff = false;
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0;
      }
      if (initObj.hasOwnProperty('z')) {
        this.z = initObj.z
      }
      else {
        this.z = 0;
      }
      if (initObj.hasOwnProperty('val')) {
        this.val = initObj.val
      }
      else {
        this.val = 0.0;
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
    // Serializes a message object of type envData
    // Serialize message field [setMap]
    bufferOffset = _serializer.bool(obj.setMap, buffer, bufferOffset);
    // Serialize message field [cliff]
    bufferOffset = _serializer.bool(obj.cliff, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.int16(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.int16(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.int16(obj.z, buffer, bufferOffset);
    // Serialize message field [val]
    bufferOffset = _serializer.float32(obj.val, buffer, bufferOffset);
    // Serialize message field [vectors]
    bufferOffset = _arraySerializer.float32(obj.vectors, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type envData
    let len;
    let data = new envData(null);
    // Deserialize message field [setMap]
    data.setMap = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [cliff]
    data.cliff = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [val]
    data.val = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vectors]
    data.vectors = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.vectors.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dstar_nav/envData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1e43a5c8c54c87cddb3fbc5f19377b83';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool setMap
    bool cliff
    int16 x
    int16 y
    int16 z
    float32 val
    float32[] vectors
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new envData(null);
    if (msg.setMap !== undefined) {
      resolved.setMap = msg.setMap;
    }
    else {
      resolved.setMap = false
    }

    if (msg.cliff !== undefined) {
      resolved.cliff = msg.cliff;
    }
    else {
      resolved.cliff = false
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0
    }

    if (msg.z !== undefined) {
      resolved.z = msg.z;
    }
    else {
      resolved.z = 0
    }

    if (msg.val !== undefined) {
      resolved.val = msg.val;
    }
    else {
      resolved.val = 0.0
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

module.exports = envData;
