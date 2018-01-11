import sys
import math
import subprocess

def printf(format, *args):
    sys.stdout.write(format % args)
#^^ I bet I get to go to a special ring in python hell for this

#this file is for turning forward oneTurn tests
#into inverseOneTurn tests by taking input from
#a bunch of unit tests through octave to produce
#a bunch of unit tests for the inverse
#(it swaps the initial and final poses

#to use it, pipe in the unit tests
#an example might be
#echo unittests.c | python forward2inversetest.py 
#user:>  octave --eval "turnPlotter(1,0,0,2,.5,-pi/3)" | python output2test.py


for line in sys.stdin:
  if " = {" in line:
    daline = line.split("{")
    moreline = daline[1].split(",")

    if (daline[0] == "  pose initialPose = "):
        initialX = float(moreline[0].split(" = ")[1])
        initialY = float(moreline[1].split(" = ")[1])
        if "M_PI_2" in moreline[2].split(" = ")[1][:-3]:
            if "-" in moreline[2].split(" = ")[1][:-3]:
              initialTh = -math.pi/2
            else:
              initialTh = math.pi/2
        elif "M_PI" in moreline[2].split(" = ")[1][:-3]:
            if "-" in moreline[2].split(" = ")[1][:-3]:
              initialTh = -math.pi
            else:
              initialTh = math.pi
        else:
            initialTh =float(moreline[2].split(" = ")[1][:-3])
    if (daline[0] == "  pose finalDestination = "):
        finalX = float(moreline[0].split(" = ")[1])
        finalY = float(moreline[1].split(" = ")[1])
        if "M_PI_2" in moreline[2].split(" = ")[1][:-3]:
            if "-" in moreline[2].split(" = ")[1][:-3]:
              finalTh = -math.pi/2
            else:
              finalTh = math.pi/2
        elif "M_PI" in moreline[2].split(" = ")[1][:-3]:
            if "-" in moreline[2].split(" = ")[1][:-3]:
              finalTh = -math.pi
            else:
              finalTh = math.pi
        else:
            finalTh =float(moreline[2].split(" = ")[1][:-3])
    if (daline[0] == "  maneuver expected1 = "):
        radius1 = float(moreline[0].split(" = ")[1])
        xc1 = float(moreline[1].split(" = ")[1])
        yc1 =float(moreline[2].split(" = ")[1])
        distance1 =float(moreline[3].split(" = ")[1][:-3])
    if (daline[0] == "  maneuver expected2 = "):
        radius2 = float(moreline[0].split(" = ")[1])
        xc2 = float(moreline[1].split(" = ")[1])
        yc2 =float(moreline[2].split(" = ")[1])
        distance2 =float(moreline[3].split(" = ")[1][:-3])

  if "std::vector<maneuver> myMans;" in line:
    result = subprocess.check_output(["octave", "--eval","turnPlotter(%f, %f, %f, %f, %f, %f)" % (finalX, finalY, finalTh, initialX, initialY, initialTh)])
    #print result
    child = subprocess.Popen(["python", "output2test.py"], stdin = subprocess.PIPE)
    child.communicate(result)
    #print newresult


#print initialX, initialY, initialTh
#print finalX, finalY, finalTh
#print radius1, xc1, yc1, distance1
#print radius2, xc2, yc2, distance2



#print "TEST(WaypointControllerHelperTests, waypoint2maneuversTestX)"
#print "{"
#printf("  pose initialPose = {.x = %f, .y = %f, .theta = %f };\n", initialX, initialY, initialTh)
#printf("  pose finalDestination = {.x = %f, .y = %f, .theta = %f };\n", finalX, finalY, finalTh)
#printf("  maneuver expected1 = {.radius = %f, .xc = %f, .yc = %f, .distance = %f};\n", radius1, xc1, yc1,  distance1 )
#printf("  maneuver expected2 = {.radius = %f, .xc = %f, .yc = %f, .distance = %f};\n\n", radius2, xc2, yc2, distance2 )

#print "  std::vector<maneuver> myMans;"
#print ""
#print "  myMans = waypoint2maneuvers(initialPose, finalDestination);"

#print """  EXPECT_TRUE(myMans.size() ==2);\n  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);\n  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);\n\n  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);\n  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);\n\n  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);\n  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);\n\n  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);\n  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);\n
#}\n"""

