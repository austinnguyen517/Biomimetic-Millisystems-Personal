;; Auto-generated. Do not edit!


(when (boundp 'dstar_nav::mapData)
  (if (not (find-package "DSTAR_NAV"))
    (make-package "DSTAR_NAV"))
  (shadow 'mapData (find-package "DSTAR_NAV")))
(unless (find-package "DSTAR_NAV::MAPDATA")
  (make-package "DSTAR_NAV::MAPDATA"))

(in-package "ROS")
;;//! \htmlinclude mapData.msg.html


(defclass dstar_nav::mapData
  :super ros::object
  :slots (_map _mapDim ))

(defmethod dstar_nav::mapData
  (:init
   (&key
    ((:map __map) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:mapDim __mapDim) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _map __map)
   (setq _mapDim __mapDim)
   self)
  (:map
   (&optional __map)
   (if __map (setq _map __map)) _map)
  (:mapDim
   (&optional __mapDim)
   (if __mapDim (setq _mapDim __mapDim)) _mapDim)
  (:serialization-length
   ()
   (+
    ;; float32[] _map
    (* 4    (length _map)) 4
    ;; uint8[] _mapDim
    (* 1    (length _mapDim)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _map
     (write-long (length _map) s)
     (dotimes (i (length _map))
       (sys::poke (elt _map i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; uint8[] _mapDim
     (write-long (length _mapDim) s)
     (princ _mapDim s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _map
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _map (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _map i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; uint8[] _mapDim
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _mapDim (make-array n :element-type :char))
     (replace _mapDim buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get dstar_nav::mapData :md5sum-) "73e74e4094b68133ffe3c8aeb805d77d")
(setf (get dstar_nav::mapData :datatype-) "dstar_nav/mapData")
(setf (get dstar_nav::mapData :definition-)
      "float32[] map 
uint8[] mapDim
")



(provide :dstar_nav/mapData "73e74e4094b68133ffe3c8aeb805d77d")


