; Auto-generated. Do not edit!


(cl:in-package dstar_nav-msg)


;//! \htmlinclude cliff.msg.html

(cl:defclass <cliff> (roslisp-msg-protocol:ros-message)
  ((coordinate
    :reader coordinate
    :initarg :coordinate
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (vectors
    :reader vectors
    :initarg :vectors
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass cliff (<cliff>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cliff>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cliff)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dstar_nav-msg:<cliff> is deprecated: use dstar_nav-msg:cliff instead.")))

(cl:ensure-generic-function 'coordinate-val :lambda-list '(m))
(cl:defmethod coordinate-val ((m <cliff>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dstar_nav-msg:coordinate-val is deprecated.  Use dstar_nav-msg:coordinate instead.")
  (coordinate m))

(cl:ensure-generic-function 'vectors-val :lambda-list '(m))
(cl:defmethod vectors-val ((m <cliff>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dstar_nav-msg:vectors-val is deprecated.  Use dstar_nav-msg:vectors instead.")
  (vectors m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cliff>) ostream)
  "Serializes a message object of type '<cliff>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'coordinate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'coordinate))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cliff>) istream)
  "Deserializes a message object of type '<cliff>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'coordinate) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'coordinate)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cliff>)))
  "Returns string type for a message object of type '<cliff>"
  "dstar_nav/cliff")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cliff)))
  "Returns string type for a message object of type 'cliff"
  "dstar_nav/cliff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cliff>)))
  "Returns md5sum for a message object of type '<cliff>"
  "bfa410a2b0e06309593d5c5ca9ddf49e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cliff)))
  "Returns md5sum for a message object of type 'cliff"
  "bfa410a2b0e06309593d5c5ca9ddf49e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cliff>)))
  "Returns full string definition for message of type '<cliff>"
  (cl:format cl:nil "float32[] coordinate~%float32[] vectors~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cliff)))
  "Returns full string definition for message of type 'cliff"
  (cl:format cl:nil "float32[] coordinate~%float32[] vectors~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cliff>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'coordinate) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'vectors) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cliff>))
  "Converts a ROS message object to a list"
  (cl:list 'cliff
    (cl:cons ':coordinate (coordinate msg))
    (cl:cons ':vectors (vectors msg))
))
