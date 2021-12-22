This is the no-ros version of jingpang's LearnVIORB (https://github.com/jingpang/LearnVIORB). Both the **monocular and stereo visual inertial odometry** are avaliable, please see Examples/ROS/ORB_VIO/srcnoros_vio.cc(Mono), Examples/ROS/ORB_VIO/stereo_noros_vio.cc(Stereo) for detail.

## NOTE: Since I have moved forward to other projects, this repository will not be maintained anymore. Thanks for your interest!


## Execute the following  command to get it start on the Euroc Dataset.
### For Mono：

V1_02_medium

./build/ORBVIO_euroc  ~/Documents/github/develop_ORBSLAM/LearnVIORBnorosgai2/Vocabulary/ORBvoc.bin ~/Documents/github/develop_ORBSLAM/LearnVIORBnorosgai2/config/euroc.yaml  ~/Documents/dataset/EURCO/V1_02_medium/mav0/imu0/data.csv  
~/Documents/dataset/EURCO/V1_02_medium/mav0/cam0/data.csv  
~/Documents/dataset/EURCO/V1_02_medium/mav0/cam0/data V1_02_medium

V1_03_difficult

./build/ORBVIO_euroc  ~/Documents/github/develop_ORBSLAM/LearnVIORBnorosgai2/Vocabulary/ORBvoc.bin ~/zuojiaxing/Documents/github/develop_ORBSLAM/LearnVIORBnorosgai2/config/euroc.yaml  ~/Documents/dataset/EURCO/V1_03_difficult/mav0/imu0/data.csv  
~/Documents/dataset/EURCO/V1_03_difficult/mav0/cam0/data.csv  
~/Documents/dataset/EURCO/V1_03_difficult/mav0/cam0/data V1_03_difficult

### For Stereo：

V1_02_medium

./build/StereoORBVIO_euroc ~/Documents/github/develop_ORBSLAM/LearnVIORBnorosgai2/Vocabulary/ORBvoc.bin ~/Documents/github/develop_ORBSLAM/LearnVIORBnorosgai2/config/Stereoeuroc.yaml  ~/Documents/dataset/EURCO/V1_02_medium/mav0/imu0/data.csv  
~/Documents/dataset/EURCO/V1_02_medium/mav0/cam0/data.csv  
~/Documents/dataset/EURCO/V1_02_medium/mav0/cam0/data  
~/Documents/dataset/EURCO/V1_02_medium/mav0/cam1/data V1_02_medium


V1_03_difficult

./build/StereoORBVIO_euroc ~/Documents/github/develop_ORBSLAM/LearnVIORBnorosgai2/Vocabulary/ORBvoc.bin ~/Documents/github/develop_ORBSLAM/LearnVIORBnorosgai2/config/Stereoeuroc.yaml  ~/Documents/dataset/EURCO/V1_03_difficult/mav0/imu0/data.csv  
~/Documents/dataset/EURCO/V1_03_difficult/mav0/cam0/data.csv  
~/Documents/dataset/EURCO/V1_03_difficult/mav0/cam0/data  
~/Documents/dataset/EURCO/V1_03_difficult/mav0/cam1/data V1_03_difficult

# NOTE:
The results of this project is shown in ![resultOnV102.png](https://github.com/ZuoJiaxing/LearnVIORBnorosgai2/blob/master/resultOnV102.png?raw=true) and 
![resultOnV103.png](https://github.com/ZuoJiaxing/LearnVIORBnorosgai2/blob/master/resultOnV103.png?raw=true), which is carried out on the V102 medium and v103 difficult dataset. I only adjust some functions to make the code suitable for no-ros application, add the initilization of stereo with IMU, and some edges of G2O related to stereo-inertial. Not bug-free. Not real-time.

###################################################################################

An implementation of [Visual Inertial ORBSLAM](https://arxiv.org/abs/1610.05949) based on [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)

Not bug-free. Not real-time. Just try the basic ideas of Visual Inertial SLAM in above paper. Welcome to improve it together!

Build with `build.sh`. Modify the path in `config/euroc.yaml`. 

Tested on [EuRoc](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) ROS bag data with ROS launch file `Examples/ROS/ORB_VIO/launch/testeuroc.launch`. Files in `pyplotscripts` can be used to visualize some results.

Tested on sensors: [UI-1221-LE](https://en.ids-imaging.com/store/ui-1221le.html) and [3DM-GX3-25](http://www.microstrain.com/inertial/3dm-gx3-25), see video on [Youtube (real-time)](https://youtu.be/AUWBpSj-XtA) or [YouKu](http://v.youku.com/v_show/id_XMTkxMjgzMzMwOA).

