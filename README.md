# zero_maker

DOPE 실행방법

1. Complete install part in DOPE github (https://github.com/NVlabs/Deep_Object_Pose)

2. Download Domain Randomized Data from ZERO google drive (https://drive.google.com/file/d/1J3zATK30D2143jmZgTV44vQyIMz_ASUs/view?usp=sharing) or from SNUZERO > 2020 > 메이커 > DATASET

3. Run training code 

$ python train.py --data path/to/DATA --object turtlebot_burger --outf turtlebot_burger --gpuids 0

if your data path is (dope package)/scripts/turtlebot_burger, you would write down 'turtlebot_burger' for path/to/DATA

when error occurs, please check your library or CUDA version!!
