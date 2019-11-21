// Auto-generated. Do not edit!

// (in-package dstar_nav.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class edgeRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.point1 = null;
      this.point2 = null;
    }
    else {
      if (initObj.hasOwnProperty('point1')) {
        this.point1 = initObj.point1
      }
      else {
        this.point1 = [];
      }
      if (initObj.hasOwnProperty('point2')) {
        this.point2 = initObj.point2
      }
      else {
        this.point2 = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type edgeRequest
    // Serialize message field [point1]
    bufferOffset = _arraySerializer.float32(obj.point1, buffer, bufferOffset, null);
    // Serialize message field [point2]
    bufferOffset = _arraySerializer.float32(obj.point2, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type edgeRequest
    let len;
    let data = new edgeRequest(null);
    // Deserialize message field [point1]
    data.point1 = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [point2]
    data.point2 = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.point1.length;
    length += 4 * object.point2.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dstar_nav/edgeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd21ddc5b335d396229c93135240e91ac';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] point1
    float32[] point2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new edgeRequest(null);
    if (msg.point1 !== undefined) {
      resolved.point1 = msg.point1;
    }
    else {
      resolved.point1 = []
    }

    if (msg.point2 !== undefined) {
      resolved.point2 = msg.point2;
    }
    else {
      resolved.point2 = []
    }

    return resolved;
    }
};

class edgeResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.weight = null;
    }
    else {
      if (initObj.hasOwnProperty('weight')) {
        this.weight = initObj.weight
      }
      else {
        this.weight = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type edgeResponse
    // Serialize message field [weight]
    bufferOffset = _serializer.float32(obj.weight, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type edgeResponse
    let len;
    let data = new edgeResponse(null);
    // Deserialize message field [weight]
    data.weight = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dstar_nav/edgeResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ead5cc9b5c68ccc4aabb4fdc1ec2e8db';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 weight
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new edgeResponse(null);
    if (msg.weight !== undefined) {
      resolved.weight = msg.weight;
    }
    else {
      resolved.weight = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: edgeRequest,
  Response: edgeResponse,
  md5sum() { return 'acc72f313e3078f8990785d2053a66d5'; },
  datatype() { return 'dstar_nav/edge'; }
};
