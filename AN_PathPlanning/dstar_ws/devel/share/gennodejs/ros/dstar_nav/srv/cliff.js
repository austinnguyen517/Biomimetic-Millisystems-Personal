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

class cliffRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pos = null;
    }
    else {
      if (initObj.hasOwnProperty('pos')) {
        this.pos = initObj.pos
      }
      else {
        this.pos = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type cliffRequest
    // Serialize message field [pos]
    bufferOffset = _arraySerializer.float32(obj.pos, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type cliffRequest
    let len;
    let data = new cliffRequest(null);
    // Deserialize message field [pos]
    data.pos = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.pos.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dstar_nav/cliffRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2ddbdf76cbefd59cd9a914780fdf2963';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] pos
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new cliffRequest(null);
    if (msg.pos !== undefined) {
      resolved.pos = msg.pos;
    }
    else {
      resolved.pos = []
    }

    return resolved;
    }
};

class cliffResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.vectors = null;
    }
    else {
      if (initObj.hasOwnProperty('vectors')) {
        this.vectors = initObj.vectors
      }
      else {
        this.vectors = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type cliffResponse
    // Serialize message field [vectors]
    bufferOffset = _arraySerializer.float32(obj.vectors, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type cliffResponse
    let len;
    let data = new cliffResponse(null);
    // Deserialize message field [vectors]
    data.vectors = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.vectors.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dstar_nav/cliffResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd4566730a8e1f6231837d96dc42fdfac';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] vectors
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new cliffResponse(null);
    if (msg.vectors !== undefined) {
      resolved.vectors = msg.vectors;
    }
    else {
      resolved.vectors = []
    }

    return resolved;
    }
};

module.exports = {
  Request: cliffRequest,
  Response: cliffResponse,
  md5sum() { return 'a9018bdf22282cd0592a32946602fd53'; },
  datatype() { return 'dstar_nav/cliff'; }
};
