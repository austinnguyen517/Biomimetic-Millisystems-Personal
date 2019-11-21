
(cl:in-package :asdf)

(defsystem "dstar_nav-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "cliff" :depends-on ("_package_cliff"))
    (:file "_package_cliff" :depends-on ("_package"))
    (:file "edge" :depends-on ("_package_edge"))
    (:file "_package_edge" :depends-on ("_package"))
  ))