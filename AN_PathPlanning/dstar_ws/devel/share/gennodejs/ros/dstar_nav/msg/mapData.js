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

class mapData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.map = null;
      this.mapDim = null;
    }
    else {
      if (initObj.hasOwnProperty('map')) {
        this.map = initObj.map
      }
      else {
        this.map = [];
      }
      if (initObj.hasOwnProperty('mapDim')) {
        this.mapDim = initObj.mapDim
      }
      else {
        this.mapDim = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type mapData
    // Serialize message field [map]
    bufferOffset = _arraySerializer.float32(obj.map, buffer, bufferOffset, null);
    // Serialize message field [mapDim]
    bufferOffset = _arraySerializer.uint8(obj.mapDim, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type mapData
    let len;
    let data = new mapData(null);
    // Deserialize message field [map]
    data.map = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [mapDim]
    data.mapDim = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.map.length;
    length += object.mapDim.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dstar_nav/mapData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '73e74e4094b68133ffe3c8aeb805d77d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] map 
    uint8[] mapDim
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new mapData(null);
    if (msg.map !== undefined) {
      resolved.map = msg.map;
    }
    else {
      resolved.map = []
    }

    if (msg.mapDim !== undefined) {
      resolved.mapDim = msg.mapDim;
    }
    else {
      resolved.mapDim = []
    }

    return resolved;
    }
};

module.exports = mapData;
