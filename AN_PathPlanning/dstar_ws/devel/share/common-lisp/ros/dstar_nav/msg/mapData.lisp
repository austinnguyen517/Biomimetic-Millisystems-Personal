; Auto-generated. Do not edit!


(cl:in-package dstar_nav-msg)


;//! \htmlinclude mapData.msg.html

(cl:defclass <mapData> (roslisp-msg-protocol:ros-message)
  ((map
    :reader map
    :initarg :map
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (mapDim
    :reader mapDim
    :initarg :mapDim
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass mapData (<mapData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mapData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mapData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dstar_nav-msg:<mapData> is deprecated: use dstar_nav-msg:mapData instead.")))

(cl:ensure-generic-function 'map-val :lambda-list '(m))
(cl:defmethod map-val ((m <mapData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dstar_nav-msg:map-val is deprecated.  Use dstar_nav-msg:map instead.")
  (map m))

(cl:ensure-generic-function 'mapDim-val :lambda-list '(m))
(cl:defmethod mapDim-val ((m <mapData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dstar_nav-msg:mapDim-val is deprecated.  Use dstar_nav-msg:mapDim instead.")
  (mapDim m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mapData>) ostream)
  "Serializes a message object of type '<mapData>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'map))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'map))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'mapDim))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'mapDim))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mapData>) istream)
  "Deserializes a message object of type '<mapData>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'map) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'map)))
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
  (cl:setf (cl:slot-value msg 'mapDim) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'mapDim)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mapData>)))
  "Returns string type for a message object of type '<mapData>"
  "dstar_nav/mapData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mapData)))
  "Returns string type for a message object of type 'mapData"
  "dstar_nav/mapData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mapData>)))
  "Returns md5sum for a message object of type '<mapData>"
  "73e74e4094b68133ffe3c8aeb805d77d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mapData)))
  "Returns md5sum for a message object of type 'mapData"
  "73e74e4094b68133ffe3c8aeb805d77d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mapData>)))
  "Returns full string definition for message of type '<mapData>"
  (cl:format cl:nil "float32[] map ~%uint8[] mapDim~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mapData)))
  "Returns full string definition for message of type 'mapData"
  (cl:format cl:nil "float32[] map ~%uint8[] mapDim~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mapData>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'map) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'mapDim) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mapData>))
  "Converts a ROS message object to a list"
  (cl:list 'mapData
    (cl:cons ':map (map msg))
    (cl:cons ':mapDim (mapDim msg))
))
