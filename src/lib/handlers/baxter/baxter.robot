RobotName: # Robot Name
fozzie

Type: # Robot type
baxter

DriveHandler: # Robot default drive handler with default argument values
share.Drive.BipedalDriveHandler()

InitHandler: # Robot default init handler with default argument values
baxter.BaxterInitHandler()

LocomotionCommandHandler: # Robot default locomotion command handler with default argument values
baxter.BaxterLocomotionCommandHandler()

MotionControlHandler: # Robot default motion control handler with default argument values
share.MotionControl.VectorControllerHandler()

PoseHandler: # Robot default pose handler with default argument values
share.Pose.ViconPoseHandler(host='10.0.0.102',port=800,x_VICON_name="baxter:baxter <t-X>",y_VICON_name="baxter:baxter <t-Y>",theta_VICON_name="baxter:baxter <a-Z>")

SensorHandler: # Robot default sensor handler with default argument values
baxter.BaxterSensorHandler()

ActuatorHandler: # Robot default actuator handler with default argument values
baxter.BaxterActuatorHandler(limb='both', rate=100.0, mode='velocity')
