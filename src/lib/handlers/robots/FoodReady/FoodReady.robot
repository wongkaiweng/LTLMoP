RobotName: # Robot Name
Food

Type: # Robot type
FoodReady

ActuatorHandler: # Robot default actuator handler with default argument values
FoodReadyActuator()

DriveHandler: # Robot default drive handler with default argument values
differentialDrive(d=0.65)

InitHandler: # Robot default init handler with default argument values
FoodReadyInit(NetduinoIP="192.168.5.100",ListenerPort = 12000)

LocomotionCommandHandler: # Robot locomotion command actuator handler with default argument values
FoodReadyLocomotionCommand(speed = 1.0)

MotionControlHandler: # Robot default motion control handler with default argument values
vectorController()

PoseHandler: # Robot default pose handler with default argument values
basicSimPose()

SensorHandler: # Robot default sensor handler with default argument values
FoodReadySensor()

CalibrationMatrix:
array([[  3.3333,       0, 0],
       [       0, -3.3333, 0],
       [       0,       0, 1]])



