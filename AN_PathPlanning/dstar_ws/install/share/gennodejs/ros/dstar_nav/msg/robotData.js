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

class robotData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robPos = null;
      this.goalPos = null;
      this.robOrient = null;
      this.proxVec = null;
      this.sense3D = null;
      this.proxDist = null;
    }
    else {
      if (initObj.hasOwnProperty('robPos')) {
        this.robPos = initObj.robPos
      }
      else {
        this.robPos = [];
      }
      if (initObj.hasOwnProperty('goalPos')) {
        this.goalPos = initObj.goalPos
      }
      else {
        this.goalPos = [];
      }
      if (initObj.hasOwnProperty('robOrient')) {
        this.robOrient = initObj.robOrient
      }
      else {
        this.robOrient = [];
      }
      if (initObj.hasOwnProperty('proxVec')) {
        this.proxVec = initObj.proxVec
      }
      else {
        this.proxVec = [];
      }
      if (initObj.hasOwnProperty('sense3D')) {
        this.sense3D = initObj.sense3D
      }
      else {
        this.sense3D = [];
      }
      if (initObj.hasOwnProperty('proxDist')) {
        this.proxDist = initObj.proxDist
      }
      else {
        this.proxDist = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type robotData
    // Serialize message field [robPos]
    bufferOffset = _arraySerializer.float32(obj.robPos, buffer, bufferOffset, null);
    // Serialize message field [goalPos]
    bufferOffset = _arraySerializer.float32(obj.goalPos, buffer, bufferOffset, null);
    // Serialize message field [robOrient]
    bufferOffset = _arraySerializer.float32(obj.robOrient, buffer, bufferOffset, null);
    // Serialize message field [proxVec]
    bufferOffset = _arraySerializer.float32(obj.proxVec, buffer, bufferOffset, null);
    // Serialize message field [sense3D]
    bufferOffset = _arraySerializer.float32(obj.sense3D, buffer, bufferOffset, null);
    // Serialize message field [proxDist]
    bufferOffset = _serializer.float32(obj.proxDist, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type robotData
    let len;
    let data = new robotData(null);
    // Deserialize message field [robPos]
    data.robPos = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [goalPos]
    data.goalPos = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [robOrient]
    data.robOrient = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [proxVec]
    data.proxVec = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [sense3D]
    data.sense3D = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [proxDist]
    data.proxDist = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.robPos.length;
    length += 4 * object.goalPos.length;
    length += 4 * object.robOrient.length;
    length += 4 * object.proxVec.length;
    length += 4 * object.sense3D.length;
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dstar_nav/robotData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e646730a5027be23477dd7883ffccb5d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] robPos
    float32[] goalPos
    float32[] robOrient
    float32[] proxVec
    float32[] sense3D
    float32 proxDist
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new robotData(null);
    if (msg.robPos !== undefined) {
      resolved.robPos = msg.robPos;
    }
    else {
      resolved.robPos = []
    }

    if (msg.goalPos !== undefined) {
      resolved.goalPos = msg.goalPos;
    }
    else {
      resolved.goalPos = []
    }

    if (msg.robOrient !== undefined) {
      resolved.robOrient = msg.robOrient;
    }
    else {
      resolved.robOrient = []
    }

    if (msg.proxVec !== undefined) {
      resolved.proxVec = msg.proxVec;
    }
    else {
      resolved.proxVec = []
    }

    if (msg.sense3D !== undefined) {
      resolved.sense3D = msg.sense3D;
    }
    else {
      resolved.sense3D = []
    }

    if (msg.proxDist !== undefined) {
      resolved.proxDist = msg.proxDist;
    }
    else {
      resolved.proxDist = 0.0
    }

    return resolved;
    }
};

module.exports = robotData;
