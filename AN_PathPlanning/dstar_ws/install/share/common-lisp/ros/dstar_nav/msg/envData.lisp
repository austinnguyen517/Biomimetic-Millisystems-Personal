; Auto-generated. Do not edit!


(cl:in-package dstar_nav-msg)


;//! \htmlinclude envData.msg.html

(cl:defclass <envData> (roslisp-msg-protocol:ros-message)
  ((setMap
    :reader setMap
    :initarg :setMap
    :type cl:boolean
    :initform cl:nil)
   (cliff
    :reader cliff
    :initarg :cliff
    :type cl:boolean
    :initform cl:nil)
   (x
    :reader x
    :initarg :x
    :type cl:fixnum
    :initform 0)
   (y
    :reader y
    :initarg :y
    :type cl:fixnum
    :initform 0)
   (z
    :reader z
    :initarg :z
    :type cl:fixnum
    :initform 0)
   (val
    :reader val
    :initarg :val
    :type cl:float
    :initform 0.0)
   (vectors
    :reader vectors
    :initarg :vectors
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass envData (<envData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <envData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'envData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dstar_nav-msg:<envData> is deprecated: use dstar_nav-msg:envData instead.")))

(cl:ensure-generic-function 'setMap-val :lambda-list '(m))
(cl:defmethod setMap-val ((m <envData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dstar_nav-msg:setMap-val is deprecated.  Use dstar_nav-msg:setMap instead.")
  (setMap m))

(cl:ensure-generic-function 'cliff-val :lambda-list '(m))
(cl:defmethod cliff-val ((m <envData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dstar_nav-msg:cliff-val is deprecated.  Use dstar_nav-msg:cliff instead.")
  (cliff m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <envData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dstar_nav-msg:x-val is deprecated.  Use dstar_nav-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <envData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dstar_nav-msg:y-val is deprecated.  Use dstar_nav-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <envData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dstar_nav-msg:z-val is deprecated.  Use dstar_nav-msg:z instead.")
  (z m))

(cl:ensure-generic-function 'val-val :lambda-list '(m))
(cl:defmethod val-val ((m <envData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dstar_nav-msg:val-val is deprecated.  Use dstar_nav-msg:val instead.")
  (val m))

(cl:ensure-generic-function 'vectors-val :lambda-list '(m))
(cl:defmethod vectors-val ((m <envData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dstar_nav-msg:vectors-val is deprecated.  Use dstar_nav-msg:vectors instead.")
  (vectors m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <envData>) ostream)
  "Serializes a message object of type '<envData>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'setMap) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cliff) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'z)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'val))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <envData>) istream)
  "Deserializes a message object of type '<envData>"
    (cl:setf (cl:slot-value msg 'setMap) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'cliff) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'z) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'val) (roslisp-utils:decode-single-float-bits bits)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<envData>)))
  "Returns string type for a message object of type '<envData>"
  "dstar_nav/envData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'envData)))
  "Returns string type for a message object of type 'envData"
  "dstar_nav/envData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<envData>)))
  "Returns md5sum for a message object of type '<envData>"
  "1e43a5c8c54c87cddb3fbc5f19377b83")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'envData)))
  "Returns md5sum for a message object of type 'envData"
  "1e43a5c8c54c87cddb3fbc5f19377b83")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<envData>)))
  "Returns full string definition for message of type '<envData>"
  (cl:format cl:nil "bool setMap~%bool cliff~%int16 x~%int16 y~%int16 z~%float32 val~%float32[] vectors~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'envData)))
  "Returns full string definition for message of type 'envData"
  (cl:format cl:nil "bool setMap~%bool cliff~%int16 x~%int16 y~%int16 z~%float32 val~%float32[] vectors~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <envData>))
  (cl:+ 0
     1
     1
     2
     2
     2
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'vectors) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <envData>))
  "Converts a ROS message object to a list"
  (cl:list 'envData
    (cl:cons ':setMap (setMap msg))
    (cl:cons ':cliff (cliff msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':val (val msg))
    (cl:cons ':vectors (vectors msg))
))
