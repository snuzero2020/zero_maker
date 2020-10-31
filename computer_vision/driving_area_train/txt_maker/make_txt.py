import os
import glob

maindir = "/home/ayoung/catkin_ws/src/zero_maker/computer_vision/test_data/"
os.chdir(maindir)

ftxt = open("test.txt", "w")
imgnm = sorted(glob.glob(maindir+"img/*"))
binnm = sorted(glob.glob(maindir+"bin/*"))

assert len(imgnm) == len(binnm)

for i in range(len(imgnm)):
    ftxt.write(imgnm[i]+" "+binnm[i]+"\n")