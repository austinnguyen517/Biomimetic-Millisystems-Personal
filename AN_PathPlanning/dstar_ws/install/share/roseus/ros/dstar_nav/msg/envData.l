;; Auto-generated. Do not edit!


(when (boundp 'dstar_nav::envData)
  (if (not (find-package "DSTAR_NAV"))
    (make-package "DSTAR_NAV"))
  (shadow 'envData (find-package "DSTAR_NAV")))
(unless (find-package "DSTAR_NAV::ENVDATA")
  (make-package "DSTAR_NAV::ENVDATA"))

(in-package "ROS")
;;//! \htmlinclude envData.msg.html


(defclass dstar_nav::envData
  :super ros::object
  :slots (_setMap _cliff _x _y _z _val _vectors ))

(defmethod dstar_nav::envData
  (:init
   (&key
    ((:setMap __setMap) nil)
    ((:cliff __cliff) nil)
    ((:x __x) 0)
    ((:y __y) 0)
    ((:z __z) 0)
    ((:val __val) 0.0)
    ((:vectors __vectors) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _setMap __setMap)
   (setq _cliff __cliff)
   (setq _x (round __x))
   (setq _y (round __y))
   (setq _z (round __z))
   (setq _val (float __val))
   (setq _vectors __vectors)
   self)
  (:setMap
   (&optional __setMap)
   (if __setMap (setq _setMap __setMap)) _setMap)
  (:cliff
   (&optional __cliff)
   (if __cliff (setq _cliff __cliff)) _cliff)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:val
   (&optional __val)
   (if __val (setq _val __val)) _val)
  (:vectors
   (&optional __vectors)
   (if __vectors (setq _vectors __vectors)) _vectors)
  (:serialization-length
   ()
   (+
    ;; bool _setMap
    1
    ;; bool _cliff
    1
    ;; int16 _x
    2
    ;; int16 _y
    2
    ;; int16 _z
    2
    ;; float32 _val
    4
    ;; float32[] _vectors
    (* 4    (length _vectors)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _setMap
       (if _setMap (write-byte -1 s) (write-byte 0 s))
     ;; bool _cliff
       (if _cliff (write-byte -1 s) (write-byte 0 s))
     ;; int16 _x
       (write-word _x s)
     ;; int16 _y
       (write-word _y s)
     ;; int16 _z
       (write-word _z s)
     ;; float32 _val
       (sys::poke _val (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32[] _vectors
     (write-long (length _vectors) s)
     (dotimes (i (length _vectors))
       (sys::poke (elt _vectors i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _setMap
     (setq _setMap (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _cliff
     (setq _cliff (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int16 _x
     (setq _x (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _y
     (setq _y (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _z
     (setq _z (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _val
     (setq _val (sys::peek buf ptr- :float)) (incf ptr- 4)
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

(setf (get dstar_nav::envData :md5sum-) "1e43a5c8c54c87cddb3fbc5f19377b83")
(setf (get dstar_nav::envData :datatype-) "dstar_nav/envData")
(setf (get dstar_nav::envData :definition-)
      "bool setMap
bool cliff
int16 x
int16 y
int16 z
float32 val
float32[] vectors

")



(provide :dstar_nav/envData "1e43a5c8c54c87cddb3fbc5f19377b83")


