import os.path

cupath = "/home/snuzero/VISION/LDLN/lane_detection_data/CUlane"
train_path = "/list/train.txt"
train_dir = "/laneseg_label_w16"
test_path = "/list/test.txt"
test_dir = "/laneseg_label_w16_test"

ftrain = open(cupath+train_path, "r")
lst_train = ftrain.readlines()
print(len(lst_train))
print(cupath+lst_train[0])
print(os.path.isfile(str(cupath+lst_train[0]).rstrip()))
print(cupath+lst_train[0])
print(os.path.isfile(str(cupath+lst_train[0]).rstrip()))
ftrain.close()
num_train = 0

ftrain_new = open(cupath+train_path[:-4]+"_new"+train_path[-4:], "w")
for term in lst_train:
    term = term.rstrip()
    if os.path.isfile(cupath+term) and os.path.isfile(cupath+train_dir+term[:-4]+".png"):
        ftrain_new.write(cupath+term+" ")
        ftrain_new.write(cupath+train_dir+term[:-4]+".png"+" ")
        ftrain_new.write(cupath+train_dir+term[:-4]+".png"+" ")
        ftrain_new.write("\n")
        num_train += 1
ftrain_new.close()
print(num_train)

ftest = open(cupath+test_path, "r")
lst_test = ftest.readlines()
print(len(lst_test))
ftest.close()
num_test = 0

ftest_new = open(cupath+test_path[:-4]+"_new"+test_path[-4:], "w")
for term in lst_test:
    term = term.rstrip()
    if os.path.isfile(cupath+term) and os.path.isfile(cupath+test_dir+term[:-4]+".png"):
        ftest_new.write(cupath+term+" ")
        ftest_new.write(cupath+test_dir+term[:-4]+".png"+" ")
        ftest_new.write(cupath+test_dir+term[:-4]+".png"+" ")
        ftest_new.write("\n")
        num_test += 1
ftest_new.close()
print(num_test)
