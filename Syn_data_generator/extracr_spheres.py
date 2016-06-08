import bge
import csv
import mathutils

scene = bge.logic.getCurrentScene()
cont = bge.logic.getCurrentController()
own = cont.owner
path = bge.logic.expandPath("//")

print(own.worldPosition)

if own['newparse']==0:
    file = open(path+"featurePos.csv", 'w')
    file.close()

for object in scene.objects :
    if own['newparse']==0 and object.name.startswith('Sphere'):
        locPos = object.worldPosition
        file = open(path+"featurePos.csv", 'a')
        spamwriter = csv.writer(file, delimiter='\t', quotechar='|',quoting=csv.QUOTE_MINIMAL, lineterminator='\n')
        spamwriter.writerow([locPos[0], locPos[1], locPos[2]])
        file.close()

own['newparse'] = 1