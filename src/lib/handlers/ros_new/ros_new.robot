RobotName: # The name of the robot
ros_new

Type: # Robot type
ros_new

InitHandler: # Robot default init handler with default argument values
ros_new.RosInitHandler(robotPixelWidth=200, robotPhysicalWidth=.5)

PoseHandler: # Robot default pose handler with default argument values
ros_new.RosPoseHandler(modelName="ego")

SensorHandler: # Robot default sensors handler with default argument values
ros_new.RosSensorHandler()

ActuatorHandler: # Robot default actuator handler wit hdefault argument values
ros_new.RosActuatorHandler()

MotionControlHandler: # Robot default motion control handler with default argument values
share.MotionControl.HeatControllerHandler()

DriveHandler: # Robot default drive handler with deafult argument values
ros_new.RosDriveHandler(d=.3)

LocomotionCommandHandler: # Robot default locomotion command handler with default argument values
ros_new.RosLocomotionCommandHandler(velocityTopic='/ego/mobile_base/commands/velocity')
