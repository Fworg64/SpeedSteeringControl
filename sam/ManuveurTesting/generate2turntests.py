import sys
import math
import subprocess

robotpos = [2, 1]
testangles = [0, math.pi/4, math.pi/2, 3*math.pi/4, math.pi, -math.pi, -3*math.pi/4, -math.pi/2, -math.pi/4]
testpos = [[2, 0], [2.5, 0], [3, 1], [3, 1.5], [2, 1.5], [1.5, 1.5], [1, 1], [1, .5]]

for angleR in testangles:
  for pos in testpos:
    for angleT in testangles:
      result = subprocess.check_output(["octave", "--eval","turnPlotter(%f, %f, %f, %f, %f, %f)" % (robotpos[0], robotpos[1], angleR, pos[0], pos[1], angleT)])
      #print result
      child = subprocess.Popen(["python", "output2test.py"], stdin = subprocess.PIPE)
      child.communicate(result)