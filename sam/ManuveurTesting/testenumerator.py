import fileinput

testno =0
for line in fileinput.input("2turntests", inplace=True):
    if ("TEST" in line):
      testno = testno +1
      print "TEST(WaypointControllerHelperTests2, waypoint2maneuversTest%d)\n" % (testno),
    else:
      print "%s" % (line),
