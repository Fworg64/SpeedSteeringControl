import sys
import math
import subprocess

robotpos = [2, 1]
testangles = [0, math.pi/4, math.pi/2, 3*math.pi/4, math.pi, -math.pi, -3*math.pi/4, -math.pi/2, -math.pi/4]
testpos = [[2, 0], [2.5, 0], [3, 1], [3, 1.5], [2, 1.5], [1.5, 1.5], [1, 1], [1, .5]]

testno =0;

for angleR in testangles:
  for pos in testpos:
    for angleT in testangles:
      #result = subprocess.check_output(["octave", "--eval","turnPlotter(%f, %f, %f, %f, %f, %f)" % (robotpos[0], robotpos[1], angleR, pos[0], pos[1], angleT)])
      #print result
      #child = subprocess.Popen(["python", "output2test.py"], stdin = subprocess.PIPE)
      #child.communicate(result)
      testno = testno +1
      print "TEST(WaypointControllerHelperTests, ableToReflectWaypoint%d)\n{" % testno
      print "  pose waypoint = {.x = %f, .y = %f, .theta = %f};" % (robotpos[0], robotpos[1], angleR)
      print "  pose initialPose = {.x = %f, .y = %f, .theta = %f};" % (pos[0], pos[1], angleT)
      print "  pose returnPose;\n"
      
      print "  std::vector<maneuver> myMans;"
      print "  myMans = waypoint2maneuvers(initialPose, waypoint);\n"
      print "  EXPECT_TRUE(myMans.size() ==2);"
     
      print "  returnPose = endOfManeuver(initialPose, myMans.at(0));"
      print "  returnPose = endOfManeuver(returnPose, myMans.at(1));"

      print "  EXPECT_NEAR(returnPose.x, waypoint.x, WAYPOINT2MANEUVERTOL);"
      print "  EXPECT_NEAR(returnPose.y, waypoint.y, WAYPOINT2MANEUVERTOL);"
      print "  EXPECT_NEAR(anglediff(returnPose.theta, waypoint.theta),0.0f, WAYPOINT2MANEUVERTOL);"
      print "}\n"