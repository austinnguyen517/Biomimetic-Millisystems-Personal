;; Auto-generated. Do not edit!


(when (boundp 'dstar_nav::cliff)
  (if (not (find-package "DSTAR_NAV"))
    (make-package "DSTAR_NAV"))
  (shadow 'cliff (find-package "DSTAR_NAV")))
(unless (find-package "DSTAR_NAV::CLIFF")
  (make-package "DSTAR_NAV::CLIFF"))
(unless (find-package "DSTAR_NAV::CLIFFREQUEST")
  (make-package "DSTAR_NAV::CLIFFREQUEST"))
(unless (find-package "DSTAR_NAV::CLIFFRESPONSE")
  (make-package "DSTAR_NAV::CLIFFRESPONSE"))

(in-package "ROS")





(defclass dstar_nav::cliffRequest
  :super ros::object
  :slots (_pos ))

(defmethod dstar_nav::cliffRequest
  (:init
   (&key
    ((:pos __pos) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _pos __pos)
   self)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:serialization-length
   ()
   (+
    ;; float32[] _pos
    (* 4    (length _pos)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _pos
     (write-long (length _pos) s)
     (dotimes (i (length _pos))
       (sys::poke (elt _pos i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _pos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pos (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _pos i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(defclass dstar_nav::cliffResponse
  :super ros::object
  :slots (_vectors ))

(defmethod dstar_nav::cliffResponse
  (:init
   (&key
    ((:vectors __vectors) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _vectors __vectors)
   self)
  (:vectors
   (&optional __vectors)
   (if __vectors (setq _vectors __vectors)) _vectors)
  (:serialization-length
   ()
   (+
    ;; float32[] _vectors
    (* 4    (length _vectors)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _vectors
     (write-long (length _vectors) s)
     (dotimes (i (length _vectors))
       (sys::poke (elt _vectors i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _vectors
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _vectors (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _vectors i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(defclass dstar_nav::cliff
  :super ros::object
  :slots ())

(setf (get dstar_nav::cliff :md5sum-) "a9018bdf22282cd0592a32946602fd53")
(setf (get dstar_nav::cliff :datatype-) "dstar_nav/cliff")
(setf (get dstar_nav::cliff :request) dstar_nav::cliffRequest)
(setf (get dstar_nav::cliff :response) dstar_nav::cliffResponse)

(defmethod dstar_nav::cliffRequest
  (:response () (instance dstar_nav::cliffResponse :init)))

(setf (get dstar_nav::cliffRequest :md5sum-) "a9018bdf22282cd0592a32946602fd53")
(setf (get dstar_nav::cliffRequest :datatype-) "dstar_nav/cliffRequest")
(setf (get dstar_nav::cliffRequest :definition-)
      "float32[] pos
---
float32[] vectors
")

(setf (get dstar_nav::cliffResponse :md5sum-) "a9018bdf22282cd0592a32946602fd53")
(setf (get dstar_nav::cliffResponse :datatype-) "dstar_nav/cliffResponse")
(setf (get dstar_nav::cliffResponse :definition-)
      "float32[] pos
---
float32[] vectors
")



(provide :dstar_nav/cliff "a9018bdf22282cd0592a32946602fd53")


