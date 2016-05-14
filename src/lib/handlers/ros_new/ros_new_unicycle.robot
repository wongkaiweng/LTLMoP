RobotName: # The name of the robot
ros_new

Type: # Robot type
ros_new

InitHandler: # Robot default init handler with default argument values
ros_new.RosInitHandler(robotPixelWidth=200, robotPhysicalWidth=.2)

PoseHandler: # Robot default pose handler with default argument values
ros_new.RosPoseHandler(modelName="ego")

SensorHandler: # Robot default sensors handler with default argument values
ros_new.RosSensorHandler()

ActuatorHandler: # Robot default actuator handler wit hdefault argument values
ros_new.RosActuatorHandler()

MotionControlHandler: # Robot default motion control handler with default argument values
share.MotionControl.ReasynsFastHandler(scalingPixelsToMeters=1.,fname='reasyns_primitives')

DriveHandler: # Robot default drive handler with deafult argument values
share.Drive.UnicycleDriveHandler(multiplier=50.0,maxspeed=999.0)

LocomotionCommandHandler: # Robot default locomotion command handler with default argument values
ros_new.RosLocomotionCommandHandler(velocityTopic='/ego/mobile_base/commands/velocity')
