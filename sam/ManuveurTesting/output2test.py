import sys
def printf(format, *args):
    sys.stdout.write(format % args)
#^^ I bet I get to go to a special ring in python hell for this

#this file is for generating unit tests based on
#output from the octave/MATLAB script turnPlotter

#to use it, pipe in the output from the turnPlotter
#and it will output a formatted unit test

#an example might be
#user:>  octave --eval "turnPlotter(1,0,0,2,.5,-pi/3)" | python output2test.py


for line in sys.stdin:
    daline = line.split(" = ")

    if len(daline) == 2:
        name = daline[0]
        value = float(daline[1])
    
        if name == "xi":
            initialX = value
        if name == "yi":
            initialY = value
        if name == "thi":
            initialTh = value
        
        if name == "wpx":
            finalX = value
        if name == "wpy":
            finalY = value
        if name == "wpth":
            finalTh = value
        if name == "distance1":
            distance1 = value
        if name == "radius1":
            radius1 = value
        if name == "xc1":
            xc1 = value
        if name == "yc1":
            yc1 = value
        
        if name == "distance2":
            distance2 = value
        if name == "radius2":
            radius2 = value
        if name == "xc2":
            xc2 = value
        if name == "yc2":
            yc2 = value


print "TEST(WaypointControllerHelperTests, waypoint2maneuversTestX)"
print "{"
printf("  pose initialPose = {.x = %f, .y = %f, .theta = %f };\n", initialX, initialY, initialTh)
printf("  pose finalDestination = {.x = %f, .y = %f, .theta = %f };\n", finalX, finalY, finalTh)
printf("  maneuver expected1 = {.radius = %f, .xc = %f, .yc = %f, .distance = %f};\n", radius1, xc1, yc1,  distance1 )
printf("  maneuver expected2 = {.radius = %f, .xc = %f, .yc = %f, .distance = %f};\n\n", radius2, xc2, yc2, distance2 )

print "  std::vector<maneuver> myMans;"
print ""
print "  myMans = waypoint2maneuvers(initialPose, finalDestination);"

print """  EXPECT_TRUE(myMans.size() ==2);\n  EXPECT_NEAR(myMans.at(0).distance, expected1.distance, WAYPOINT2MANEUVERTOL);\n  EXPECT_NEAR(myMans.at(1).distance, expected2.distance, WAYPOINT2MANEUVERTOL);\n\n  EXPECT_NEAR(myMans.at(0).radius, expected1.radius, WAYPOINT2MANEUVERTOL);\n  EXPECT_NEAR(myMans.at(1).radius, expected2.radius, WAYPOINT2MANEUVERTOL);\n\n  EXPECT_NEAR(myMans.at(0).xc, expected1.xc, WAYPOINT2MANEUVERTOL);\n  EXPECT_NEAR(myMans.at(1).xc, expected2.xc, WAYPOINT2MANEUVERTOL);\n\n  EXPECT_NEAR(myMans.at(0).yc, expected1.yc, WAYPOINT2MANEUVERTOL);\n  EXPECT_NEAR(myMans.at(1).yc, expected2.yc, WAYPOINT2MANEUVERTOL);\n
}\n"""



