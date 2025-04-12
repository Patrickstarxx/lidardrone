
(cl:in-package :asdf)

(defsystem "msg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "NAV_WYPT_MODE" :depends-on ("_package_NAV_WYPT_MODE"))
    (:file "_package_NAV_WYPT_MODE" :depends-on ("_package"))
    (:file "NAV_WYPT_TYPE_SWITCH" :depends-on ("_package_NAV_WYPT_TYPE_SWITCH"))
    (:file "_package_NAV_WYPT_TYPE_SWITCH" :depends-on ("_package"))
  ))