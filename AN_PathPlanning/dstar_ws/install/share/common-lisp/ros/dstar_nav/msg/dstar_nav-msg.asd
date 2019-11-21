
(cl:in-package :asdf)

(defsystem "dstar_nav-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "envData" :depends-on ("_package_envData"))
    (:file "_package_envData" :depends-on ("_package"))
    (:file "mapData" :depends-on ("_package_mapData"))
    (:file "_package_mapData" :depends-on ("_package"))
    (:file "robotData" :depends-on ("_package_robotData"))
    (:file "_package_robotData" :depends-on ("_package"))
  ))