
(cl:in-package :asdf)

(defsystem "traj_utils-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Bspline" :depends-on ("_package_Bspline"))
    (:file "_package_Bspline" :depends-on ("_package"))
    (:file "DataDisp" :depends-on ("_package_DataDisp"))
    (:file "_package_DataDisp" :depends-on ("_package"))
    (:file "MultiBsplines" :depends-on ("_package_MultiBsplines"))
    (:file "_package_MultiBsplines" :depends-on ("_package"))
  ))