; Auto-generated. Do not edit!


(cl:in-package dstar_nav-srv)


;//! \htmlinclude edge-request.msg.html

(cl:defclass <edge-request> (roslisp-msg-protocol:ros-message)
  ((point1
    :reader point1
    :initarg :point1
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (point2
    :reader point2
    :initarg :point2
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass edge-request (<edge-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <edge-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'edge-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dstar_nav-srv:<edge-request> is deprecated: use dstar_nav-srv:edge-request instead.")))

(cl:ensure-generic-function 'point1-val :lambda-list '(m))
(cl:defmethod point1-val ((m <edge-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dstar_nav-srv:point1-val is deprecated.  Use dstar_nav-srv:point1 instead.")
  (point1 m))

(cl:ensure-generic-function 'point2-val :lambda-list '(m))
(cl:defmethod point2-val ((m <edge-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dstar_nav-srv:point2-val is deprecated.  Use dstar_nav-srv:point2 instead.")
  (point2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <edge-request>) ostream)
  "Serializes a message object of type '<edge-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'point1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'point1))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'point2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'point2))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <edge-request>) istream)
  "Deserializes a message object of type '<edge-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'point1) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'point1)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'point2) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'point2)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<edge-request>)))
  "Returns string type for a service object of type '<edge-request>"
  "dstar_nav/edgeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'edge-request)))
  "Returns string type for a service object of type 'edge-request"
  "dstar_nav/edgeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<edge-request>)))
  "Returns md5sum for a message object of type '<edge-request>"
  "acc72f313e3078f8990785d2053a66d5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'edge-request)))
  "Returns md5sum for a message object of type 'edge-request"
  "acc72f313e3078f8990785d2053a66d5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<edge-request>)))
  "Returns full string definition for message of type '<edge-request>"
  (cl:format cl:nil "float32[] point1~%float32[] point2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'edge-request)))
  "Returns full string definition for message of type 'edge-request"
  (cl:format cl:nil "float32[] point1~%float32[] point2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <edge-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'point1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'point2) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <edge-request>))
  "Converts a ROS message object to a list"
  (cl:list 'edge-request
    (cl:cons ':point1 (point1 msg))
    (cl:cons ':point2 (point2 msg))
))
;//! \htmlinclude edge-response.msg.html

(cl:defclass <edge-response> (roslisp-msg-protocol:ros-message)
  ((weight
    :reader weight
    :initarg :weight
    :type cl:float
    :initform 0.0))
)

(cl:defclass edge-response (<edge-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <edge-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'edge-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dstar_nav-srv:<edge-response> is deprecated: use dstar_nav-srv:edge-response instead.")))

(cl:ensure-generic-function 'weight-val :lambda-list '(m))
(cl:defmethod weight-val ((m <edge-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dstar_nav-srv:weight-val is deprecated.  Use dstar_nav-srv:weight instead.")
  (weight m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <edge-response>) ostream)
  "Serializes a message object of type '<edge-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'weight))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <edge-response>) istream)
  "Deserializes a message object of type '<edge-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'weight) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<edge-response>)))
  "Returns string type for a service object of type '<edge-response>"
  "dstar_nav/edgeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'edge-response)))
  "Returns string type for a service object of type 'edge-response"
  "dstar_nav/edgeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<edge-response>)))
  "Returns md5sum for a message object of type '<edge-response>"
  "acc72f313e3078f8990785d2053a66d5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'edge-response)))
  "Returns md5sum for a message object of type 'edge-response"
  "acc72f313e3078f8990785d2053a66d5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<edge-response>)))
  "Returns full string definition for message of type '<edge-response>"
  (cl:format cl:nil "float32 weight~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'edge-response)))
  "Returns full string definition for message of type 'edge-response"
  (cl:format cl:nil "float32 weight~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <edge-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <edge-response>))
  "Converts a ROS message object to a list"
  (cl:list 'edge-response
    (cl:cons ':weight (weight msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'edge)))
  'edge-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'edge)))
  'edge-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'edge)))
  "Returns string type for a service object of type '<edge>"
  "dstar_nav/edge")