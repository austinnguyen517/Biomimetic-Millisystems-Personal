;; Auto-generated. Do not edit!


(when (boundp 'dstar_nav::cliff)
  (if (not (find-package "DSTAR_NAV"))
    (make-package "DSTAR_NAV"))
  (shadow 'cliff (find-package "DSTAR_NAV")))
(unless (find-package "DSTAR_NAV::CLIFF")
  (make-package "DSTAR_NAV::CLIFF"))

(in-package "ROS")
;;//! \htmlinclude cliff.msg.html


(defclass dstar_nav::cliff
  :super ros::object
  :slots (_coordinate _vectors ))

(defmethod dstar_nav::cliff
  (:init
   (&key
    ((:coordinate __coordinate) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:vectors __vectors) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _coordinate __coordinate)
   (setq _vectors __vectors)
   self)
  (:coordinate
   (&optional __coordinate)
   (if __coordinate (setq _coordinate __coordinate)) _coordinate)
  (:vectors
   (&optional __vectors)
   (if __vectors (setq _vectors __vectors)) _vectors)
  (:serialization-length
   ()
   (+
    ;; float32[] _coordinate
    (* 4    (length _coordinate)) 4
    ;; float32[] _vectors
    (* 4    (length _vectors)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _coordinate
     (write-long (length _coordinate) s)
     (dotimes (i (length _coordinate))
       (sys::poke (elt _coordinate i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _vectors
     (write-long (length _vectors) s)
     (dotimes (i (length _vectors))
       (sys::poke (elt _vectors i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _coordinate
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _coordinate (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _coordinate i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
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

(setf (get dstar_nav::cliff :md5sum-) "bfa410a2b0e06309593d5c5ca9ddf49e")
(setf (get dstar_nav::cliff :datatype-) "dstar_nav/cliff")
(setf (get dstar_nav::cliff :definition-)
      "float32[] coordinate
float32[] vectors
")



(provide :dstar_nav/cliff "bfa410a2b0e06309593d5c5ca9ddf49e")


