; Auto-generated. Do not edit!


(cl:in-package dstar_nav-srv)


;//! \htmlinclude cliff-request.msg.html

(cl:defclass <cliff-request> (roslisp-msg-protocol:ros-message)
  ((pos
    :reader pos
    :initarg :pos
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass cliff-request (<cliff-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cliff-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cliff-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dstar_nav-srv:<cliff-request> is deprecated: use dstar_nav-srv:cliff-request instead.")))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <cliff-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dstar_nav-srv:pos-val is deprecated.  Use dstar_nav-srv:pos instead.")
  (pos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cliff-request>) ostream)
  "Serializes a message object of type '<cliff-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'pos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'pos))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cliff-request>) istream)
  "Deserializes a message object of type '<cliff-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pos) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'pos)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cliff-request>)))
  "Returns string type for a service object of type '<cliff-request>"
  "dstar_nav/cliffRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cliff-request)))
  "Returns string type for a service object of type 'cliff-request"
  "dstar_nav/cliffRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cliff-request>)))
  "Returns md5sum for a message object of type '<cliff-request>"
  "a9018bdf22282cd0592a32946602fd53")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cliff-request)))
  "Returns md5sum for a message object of type 'cliff-request"
  "a9018bdf22282cd0592a32946602fd53")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cliff-request>)))
  "Returns full string definition for message of type '<cliff-request>"
  (cl:format cl:nil "float32[] pos~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cliff-request)))
  "Returns full string definition for message of type 'cliff-request"
  (cl:format cl:nil "float32[] pos~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cliff-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cliff-request>))
  "Converts a ROS message object to a list"
  (cl:list 'cliff-request
    (cl:cons ':pos (pos msg))
))
;//! \htmlinclude cliff-response.msg.html

(cl:defclass <cliff-response> (roslisp-msg-protocol:ros-message)
  ((vectors
    :reader vectors
    :initarg :vectors
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass cliff-response (<cliff-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cliff-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cliff-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dstar_nav-srv:<cliff-response> is deprecated: use dstar_nav-srv:cliff-response instead.")))

(cl:ensure-generic-function 'vectors-val :lambda-list '(m))
(cl:defmethod vectors-val ((m <cliff-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dstar_nav-srv:vectors-val is deprecated.  Use dstar_nav-srv:vectors instead.")
  (vectors m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cliff-response>) ostream)
  "Serializes a message object of type '<cliff-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'vectors))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'vectors))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cliff-response>) istream)
  "Deserializes a message object of type '<cliff-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'vectors) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'vectors)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cliff-response>)))
  "Returns string type for a service object of type '<cliff-response>"
  "dstar_nav/cliffResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cliff-response)))
  "Returns string type for a service object of type 'cliff-response"
  "dstar_nav/cliffResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cliff-response>)))
  "Returns md5sum for a message object of type '<cliff-response>"
  "a9018bdf22282cd0592a32946602fd53")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cliff-response)))
  "Returns md5sum for a message object of type 'cliff-response"
  "a9018bdf22282cd0592a32946602fd53")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cliff-response>)))
  "Returns full string definition for message of type '<cliff-response>"
  (cl:format cl:nil "float32[] vectors~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cliff-response)))
  "Returns full string definition for message of type 'cliff-response"
  (cl:format cl:nil "float32[] vectors~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cliff-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'vectors) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cliff-response>))
  "Converts a ROS message object to a list"
  (cl:list 'cliff-response
    (cl:cons ':vectors (vectors msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'cliff)))
  'cliff-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'cliff)))
  'cliff-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cliff)))
  "Returns string type for a service object of type '<cliff>"
  "dstar_nav/cliff")