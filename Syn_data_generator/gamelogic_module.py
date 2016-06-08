# This module generates groundtruth data of the trajectory and store them in the file "syn_groundtruth.csv"

import bge
import csv
import mathutils

cont = bge.logic.getCurrentController()
own = cont.owner
path = bge.logic.expandPath("//")

if own['counter'] == 0 :
    bge.logic.setLogicTicRate(200)
    file = open(path+"syn_groundtruth.csv", 'w')
    file.close()

orientation = own.localOrientation.to_quaternion()
rotVel = own.localAngularVelocity
#rotVel = own.localOrientation.to_euler()

position = own.worldPosition
linVel = own.localLinearVelocity

rotMat = own.localOrientation


file = open(path+"syn_groundtruth.csv", 'a')
spamwriter = csv.writer(file, delimiter='\t', quotechar='|',quoting=csv.QUOTE_MINIMAL, lineterminator='\n')
spamwriter.writerow([linVel[0], linVel[1], linVel[2], rotVel[0], rotVel[1], rotVel[2], position[0], position[1], position[2], orientation[1], orientation[2], orientation[3], orientation[0], rotMat[0][0], rotMat[0][1], rotMat[0][2], rotMat[1][0], rotMat[1][1], rotMat[1][2], rotMat[2][0], rotMat[2][1], rotMat[2][2]])
file.close()

# variables defined here will only be set once when the
# module is first imported. Set object specific vars
# inside the function if you intend to use the module
# with multiple objects.

own['counter'] += 1
print(own['timer'])
mat =  mathutils.Vector((0.0,0.0,9.81))
print(rotMat[1])

if own['counter'] > 12001 :
    bge.logic.endGame()
    
# dont call main(bge.logic.getCurrentController()), the py controller will
