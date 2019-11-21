;; Auto-generated. Do not edit!


(when (boundp 'dstar_nav::robotData)
  (if (not (find-package "DSTAR_NAV"))
    (make-package "DSTAR_NAV"))
  (shadow 'robotData (find-package "DSTAR_NAV")))
(unless (find-package "DSTAR_NAV::ROBOTDATA")
  (make-package "DSTAR_NAV::ROBOTDATA"))

(in-package "ROS")
;;//! \htmlinclude robotData.msg.html


(defclass dstar_nav::robotData
  :super ros::object
  :slots (_robPos _goalPos _robOrient _proxVec _sense3D _proxDist ))

(defmethod dstar_nav::robotData
  (:init
   (&key
    ((:robPos __robPos) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:goalPos __goalPos) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:robOrient __robOrient) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:proxVec __proxVec) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:sense3D __sense3D) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:proxDist __proxDist) 0.0)
    )
   (send-super :init)
   (setq _robPos __robPos)
   (setq _goalPos __goalPos)
   (setq _robOrient __robOrient)
   (setq _proxVec __proxVec)
   (setq _sense3D __sense3D)
   (setq _proxDist (float __proxDist))
   self)
  (:robPos
   (&optional __robPos)
   (if __robPos (setq _robPos __robPos)) _robPos)
  (:goalPos
   (&optional __goalPos)
   (if __goalPos (setq _goalPos __goalPos)) _goalPos)
  (:robOrient
   (&optional __robOrient)
   (if __robOrient (setq _robOrient __robOrient)) _robOrient)
  (:proxVec
   (&optional __proxVec)
   (if __proxVec (setq _proxVec __proxVec)) _proxVec)
  (:sense3D
   (&optional __sense3D)
   (if __sense3D (setq _sense3D __sense3D)) _sense3D)
  (:proxDist
   (&optional __proxDist)
   (if __proxDist (setq _proxDist __proxDist)) _proxDist)
  (:serialization-length
   ()
   (+
    ;; float32[] _robPos
    (* 4    (length _robPos)) 4
    ;; float32[] _goalPos
    (* 4    (length _goalPos)) 4
    ;; float32[] _robOrient
    (* 4    (length _robOrient)) 4
    ;; float32[] _proxVec
    (* 4    (length _proxVec)) 4
    ;; float32[] _sense3D
    (* 4    (length _sense3D)) 4
    ;; float32 _proxDist
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _robPos
     (write-long (length _robPos) s)
     (dotimes (i (length _robPos))
       (sys::poke (elt _robPos i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _goalPos
     (write-long (length _goalPos) s)
     (dotimes (i (length _goalPos))
       (sys::poke (elt _goalPos i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _robOrient
     (write-long (length _robOrient) s)
     (dotimes (i (length _robOrient))
       (sys::poke (elt _robOrient i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _proxVec
     (write-long (length _proxVec) s)
     (dotimes (i (length _proxVec))
       (sys::poke (elt _proxVec i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _sense3D
     (write-long (length _sense3D) s)
     (dotimes (i (length _sense3D))
       (sys::poke (elt _sense3D i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _proxDist
       (sys::poke _proxDist (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _robPos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _robPos (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _robPos i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _goalPos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _goalPos (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _goalPos i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _robOrient
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _robOrient (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _robOrient i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _proxVec
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _proxVec (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _proxVec i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _sense3D
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _sense3D (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _sense3D i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32 _proxDist
     (setq _proxDist (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get dstar_nav::robotData :md5sum-) "e646730a5027be23477dd7883ffccb5d")
(setf (get dstar_nav::robotData :datatype-) "dstar_nav/robotData")
(setf (get dstar_nav::robotData :definition-)
      "float32[] robPos
float32[] goalPos
float32[] robOrient
float32[] proxVec
float32[] sense3D
float32 proxDist

")



(provide :dstar_nav/robotData "e646730a5027be23477dd7883ffccb5d")


