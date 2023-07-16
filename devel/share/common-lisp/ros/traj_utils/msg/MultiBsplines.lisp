; Auto-generated. Do not edit!


(cl:in-package traj_utils-msg)


;//! \htmlinclude MultiBsplines.msg.html

(cl:defclass <MultiBsplines> (roslisp-msg-protocol:ros-message)
  ((drone_id_from
    :reader drone_id_from
    :initarg :drone_id_from
    :type cl:integer
    :initform 0)
   (traj
    :reader traj
    :initarg :traj
    :type (cl:vector traj_utils-msg:Bspline)
   :initform (cl:make-array 0 :element-type 'traj_utils-msg:Bspline :initial-element (cl:make-instance 'traj_utils-msg:Bspline))))
)

(cl:defclass MultiBsplines (<MultiBsplines>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MultiBsplines>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MultiBsplines)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traj_utils-msg:<MultiBsplines> is deprecated: use traj_utils-msg:MultiBsplines instead.")))

(cl:ensure-generic-function 'drone_id_from-val :lambda-list '(m))
(cl:defmethod drone_id_from-val ((m <MultiBsplines>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:drone_id_from-val is deprecated.  Use traj_utils-msg:drone_id_from instead.")
  (drone_id_from m))

(cl:ensure-generic-function 'traj-val :lambda-list '(m))
(cl:defmethod traj-val ((m <MultiBsplines>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:traj-val is deprecated.  Use traj_utils-msg:traj instead.")
  (traj m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MultiBsplines>) ostream)
  "Serializes a message object of type '<MultiBsplines>"
  (cl:let* ((signed (cl:slot-value msg 'drone_id_from)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'traj))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'traj))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MultiBsplines>) istream)
  "Deserializes a message object of type '<MultiBsplines>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'drone_id_from) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'traj) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'traj)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'traj_utils-msg:Bspline))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MultiBsplines>)))
  "Returns string type for a message object of type '<MultiBsplines>"
  "traj_utils/MultiBsplines")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MultiBsplines)))
  "Returns string type for a message object of type 'MultiBsplines"
  "traj_utils/MultiBsplines")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MultiBsplines>)))
  "Returns md5sum for a message object of type '<MultiBsplines>"
  "7f45adafc838893dace82d2af415aae3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MultiBsplines)))
  "Returns md5sum for a message object of type 'MultiBsplines"
  "7f45adafc838893dace82d2af415aae3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MultiBsplines>)))
  "Returns full string definition for message of type '<MultiBsplines>"
  (cl:format cl:nil "int32 drone_id_from~%~%Bspline[] traj~%~%~%================================================================================~%MSG: traj_utils/Bspline~%int32 drone_id~%~%int32 order~%int64 traj_id~%time start_time~%~%float64[] knots~%geometry_msgs/Point[] pos_pts~%~%float64[] yaw_pts~%float64 yaw_dt~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MultiBsplines)))
  "Returns full string definition for message of type 'MultiBsplines"
  (cl:format cl:nil "int32 drone_id_from~%~%Bspline[] traj~%~%~%================================================================================~%MSG: traj_utils/Bspline~%int32 drone_id~%~%int32 order~%int64 traj_id~%time start_time~%~%float64[] knots~%geometry_msgs/Point[] pos_pts~%~%float64[] yaw_pts~%float64 yaw_dt~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MultiBsplines>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'traj) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MultiBsplines>))
  "Converts a ROS message object to a list"
  (cl:list 'MultiBsplines
    (cl:cons ':drone_id_from (drone_id_from msg))
    (cl:cons ':traj (traj msg))
))
