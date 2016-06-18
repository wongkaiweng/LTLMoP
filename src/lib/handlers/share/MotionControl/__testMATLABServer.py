import matlab.engine
print matlab.engine.find_matlab()
import __MATLABPythonInterface_direct as MATLABPythonInterface
import sys, os
import argparse
import time

# Climb the tree to find out where we are
p = os.path.abspath(__file__)
t = ""
while t != "src":
    (p, t) = os.path.split(p)
    if p == "":
        print "I have no idea where I am; this is ridiculous"
        sys.exit(1)
sys.path.append(os.path.join(p,"src","lib"))
import logging
import globalConfig

pose = {'rob1':[231.25, 159.75], 'rob2':[533, 565.25], 'rob3':[531.8, 146.4]}
next_regIndices = {'rob1':[4], 'rob2':[0], 'rob3':[3]} # 415
idx = 0


if __name__ == "__main__":
    # get arguments
    parser = argparse.ArgumentParser(description="test MATLAB Server.")
    parser.add_argument('robot_no', type=str, help='robot number')
    args = parser.parse_args()

    # manager1
    session = matlab.engine.connect_matlab(matlab.engine.find_matlab()[0])
    MATLABPythonInterface.init_singleRobot(session, 'rob'+args.robot_no, pose['rob'+args.robot_no], next_regIndices['rob'+args.robot_no][idx])

    # make sure the others are also launched
    time.sleep(5)

    # manager2
    #session2 = matlab.engine.connect_matlab(matlab.engine.find_matlab()[0])
    #MATLABPythonInterface.init_singleRobot(session1, 'rob2', pose['rob2'], next_regIndices['rob2'][idx])

    # manager3
    #session3 = matlab.engine.connect_matlab(matlab.engine.find_matlab()[0])
    #MATLABPythonInterface.init_singleRobot(session1,'rob3', pose['rob3'], next_regIndices['rob3'][idx])

    # calculate velocity
    for x in range(20):
        vx, vy, regionChanges, currentLoc = MATLABPythonInterface.getSingleMATLABVelocity(session, 'rob'+args.robot_no, \
                                    pose['rob'+args.robot_no], next_regIndices['rob'+args.robot_no][idx])
        logging.debug("rob1-- vx:{vx}, vy:{vy}, regionChanges:{regionChanges}, currentLoc:{currentLoc}".format(\
                                vx=vx, vy=vy, regionChanges=regionChanges, currentLoc=currentLoc))

    # vx, vy, regionChanges, currentLoc = session2.get_velocity("rob2", pose['rob2'], next_regIndices['rob2'][idx])
    # print "rob12-- vx:{vx}, vy:{vy}, regionChanges:{regionChanges}, currentLoc:{currentLoc}".format(\
    #                         vx=vx, vy=vy, regionChanges=regionChanges, currentLoc=currentLoc)

    # vx, vy, regionChanges, currentLoc = session3.get_velocity("rob3", pose['rob3'], next_regIndices['rob3'][idx])
    # print "rob3-- vx:{vx}, vy:{vy}, regionChanges:{regionChanges}, currentLoc:{currentLoc}".format(\
    #                         vx=vx, vy=vy, regionChanges=regionChanges, currentLoc=currentLoc)

