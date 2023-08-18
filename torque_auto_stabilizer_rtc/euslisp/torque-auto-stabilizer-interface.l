(require"package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(ros::load-ros-manifest "torque_auto_stabilizer_rtc")
(instance rtm-ros-robot-interface :define-all-ROSBridge-srv-methods :ros-pkg-name "torque_auto_stabilizer_rtc")

(def-set-get-param-method
  'torque_auto_stabilizer_rtc::Openhrp_TorqueAutoStabilizerService_TorqueAutoStabilizerParam
  :raw-set-torque-auto-stabilizer-param :get-torque-auto-stabilizer-param :get-torque-auto-stabilizer-param-arguments
  :torqueautostabilizerservice_settorqueautostabilizerparam :torqueautostabilizerservice_gettorqueautostabilizerparam)

(defmethod rtm-ros-robot-interface
  (:start-ast
   ()
   "Start AutoStabilizer Mode."
   (send self :torqueautostabilizerservice_startautostabilizer)
   )
  (:stop-ast
   ()
   "Stop AutoStabilizer Mode."
   (send self :torqueautostabilizerservice_stopautostabilizer)
   )
)


