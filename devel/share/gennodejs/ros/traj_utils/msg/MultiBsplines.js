// Auto-generated. Do not edit!

// (in-package traj_utils.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Bspline = require('./Bspline.js');

//-----------------------------------------------------------

class MultiBsplines {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.drone_id_from = null;
      this.traj = null;
    }
    else {
      if (initObj.hasOwnProperty('drone_id_from')) {
        this.drone_id_from = initObj.drone_id_from
      }
      else {
        this.drone_id_from = 0;
      }
      if (initObj.hasOwnProperty('traj')) {
        this.traj = initObj.traj
      }
      else {
        this.traj = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MultiBsplines
    // Serialize message field [drone_id_from]
    bufferOffset = _serializer.int32(obj.drone_id_from, buffer, bufferOffset);
    // Serialize message field [traj]
    // Serialize the length for message field [traj]
    bufferOffset = _serializer.uint32(obj.traj.length, buffer, bufferOffset);
    obj.traj.forEach((val) => {
      bufferOffset = Bspline.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MultiBsplines
    let len;
    let data = new MultiBsplines(null);
    // Deserialize message field [drone_id_from]
    data.drone_id_from = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [traj]
    // Deserialize array length for message field [traj]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.traj = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.traj[i] = Bspline.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.traj.forEach((val) => {
      length += Bspline.getMessageSize(val);
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'traj_utils/MultiBsplines';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7f45adafc838893dace82d2af415aae3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 drone_id_from
    
    Bspline[] traj
    
    
    ================================================================================
    MSG: traj_utils/Bspline
    int32 drone_id
    
    int32 order
    int64 traj_id
    time start_time
    
    float64[] knots
    geometry_msgs/Point[] pos_pts
    
    float64[] yaw_pts
    float64 yaw_dt
    
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MultiBsplines(null);
    if (msg.drone_id_from !== undefined) {
      resolved.drone_id_from = msg.drone_id_from;
    }
    else {
      resolved.drone_id_from = 0
    }

    if (msg.traj !== undefined) {
      resolved.traj = new Array(msg.traj.length);
      for (let i = 0; i < resolved.traj.length; ++i) {
        resolved.traj[i] = Bspline.Resolve(msg.traj[i]);
      }
    }
    else {
      resolved.traj = []
    }

    return resolved;
    }
};

module.exports = MultiBsplines;
