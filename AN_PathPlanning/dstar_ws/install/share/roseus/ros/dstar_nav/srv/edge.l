;; Auto-generated. Do not edit!


(when (boundp 'dstar_nav::edge)
  (if (not (find-package "DSTAR_NAV"))
    (make-package "DSTAR_NAV"))
  (shadow 'edge (find-package "DSTAR_NAV")))
(unless (find-package "DSTAR_NAV::EDGE")
  (make-package "DSTAR_NAV::EDGE"))
(unless (find-package "DSTAR_NAV::EDGEREQUEST")
  (make-package "DSTAR_NAV::EDGEREQUEST"))
(unless (find-package "DSTAR_NAV::EDGERESPONSE")
  (make-package "DSTAR_NAV::EDGERESPONSE"))

(in-package "ROS")





(defclass dstar_nav::edgeRequest
  :super ros::object
  :slots (_point1 _point2 ))

(defmethod dstar_nav::edgeRequest
  (:init
   (&key
    ((:point1 __point1) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:point2 __point2) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _point1 __point1)
   (setq _point2 __point2)
   self)
  (:point1
   (&optional __point1)
   (if __point1 (setq _point1 __point1)) _point1)
  (:point2
   (&optional __point2)
   (if __point2 (setq _point2 __point2)) _point2)
  (:serialization-length
   ()
   (+
    ;; float32[] _point1
    (* 4    (length _point1)) 4
    ;; float32[] _point2
    (* 4    (length _point2)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _point1
     (write-long (length _point1) s)
     (dotimes (i (length _point1))
       (sys::poke (elt _point1 i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _point2
     (write-long (length _point2) s)
     (dotimes (i (length _point2))
       (sys::poke (elt _point2 i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _point1
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _point1 (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _point1 i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _point2
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _point2 (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _point2 i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(defclass dstar_nav::edgeResponse
  :super ros::object
  :slots (_weight ))

(defmethod dstar_nav::edgeResponse
  (:init
   (&key
    ((:weight __weight) 0.0)
    )
   (send-super :init)
   (setq _weight (float __weight))
   self)
  (:weight
   (&optional __weight)
   (if __weight (setq _weight __weight)) _weight)
  (:serialization-length
   ()
   (+
    ;; float32 _weight
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _weight
       (sys::poke _weight (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _weight
     (setq _weight (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass dstar_nav::edge
  :super ros::object
  :slots ())

(setf (get dstar_nav::edge :md5sum-) "acc72f313e3078f8990785d2053a66d5")
(setf (get dstar_nav::edge :datatype-) "dstar_nav/edge")
(setf (get dstar_nav::edge :request) dstar_nav::edgeRequest)
(setf (get dstar_nav::edge :response) dstar_nav::edgeResponse)

(defmethod dstar_nav::edgeRequest
  (:response () (instance dstar_nav::edgeResponse :init)))

(setf (get dstar_nav::edgeRequest :md5sum-) "acc72f313e3078f8990785d2053a66d5")
(setf (get dstar_nav::edgeRequest :datatype-) "dstar_nav/edgeRequest")
(setf (get dstar_nav::edgeRequest :definition-)
      "float32[] point1
float32[] point2
---
float32 weight
")

(setf (get dstar_nav::edgeResponse :md5sum-) "acc72f313e3078f8990785d2053a66d5")
(setf (get dstar_nav::edgeResponse :datatype-) "dstar_nav/edgeResponse")
(setf (get dstar_nav::edgeResponse :definition-)
      "float32[] point1
float32[] point2
---
float32 weight
")



(provide :dstar_nav/edge "acc72f313e3078f8990785d2053a66d5")


