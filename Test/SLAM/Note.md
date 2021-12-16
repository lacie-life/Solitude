## TODO
-Run the existing binocular ORB+IMU (almost done, single thread)
-Sliding window on the back end
-When the Ref of Point comes out of the window, should it be changed to ref or marg or fixed?
-New key frame strategy, NeedNewKeyFrame() changed to similar to ORB
- Multithreading
- Delete redundant KF (unknown bug...)
- Under PureVision, after LocalBA, the pose of mpLastFrame should be updated according to the pose change of the previous key frame.
- Programs with IMU have not considered the static and weak state. (Still state, MH01 uses 30KF, DeleteRedundant can run. Weak has not been considered)
- check why backend estimated pose will jump in some special cases.
- If DeleteRedundant is placed before LocalBA, there will be a seg fault. Can't find why... (Frame.mFeaturesLeft can't even call .size()?)
- Debug the WEAK mode in IMU
- If the key frames overlap too much, the problem of back-end optimization may be a little morbid
- Segment fault that appears irregularly
- More stable optical flow

## WEAK mode processing
-IMU volume reinitialization
-Fixed zero offset during optimization
-How to operate the back end during WEAK? Back end with IMU optimization during WEAK is easy to fly away
-Detect frequent loss of vision
-Adjust the key frame strategy when recovering from WEAK
-Temporary conclusion:
1. The reset effect of the whole system is good

## NOTE
-The cost of OpenCV's GFTT call is about 10+ms per call, please use it with caution
-Inv depth's back-end optimization always has some inexplicable problems, the pose is very unstable?
-The front-end optimized pose shows obvious jumps about 100 frames in front of MH_05


## DONE.
-Front-end optical flow
-check "v is nullptr in Local ba without imu"(done.)
-check why backend estimated pose will jump in some special cases. (done. Reason: some map points on the backend have not updated the world coordinates)
-check Why is nRefMatches less than inliers of currentFrame in Tracker? (done. normal)
-Consider add inv depth prior in estimation (done.)


## Log
### 7.8
-Fixed the multi-threaded backend with IMU optimization

### 7.6
-Added multi-threaded backend
-Change the FAST+ built-in optical flow back to GFTT+cv optical flow. Only mention new feature points at key frames
-Corrected the calculation of IMMATURE judged GOOD
-Fixed the problem of too large lock in Viewer

### 6.27
-Reorganized the entire process of binocular light flow to make it more tidy
-Added an optimization based on XYZ coordinates in the back end. Seems more stable than inverse depth parameterization?

### 6.21
-add temporal map point in non-keyframe

### 6.19
-Added the practice of rejecting some tracking points based on RANSAC at LK
-Added some temporary map points on the front end to enhance stability

### 6.14
-adjust some thresholds in IMU related optimization

### 6.13
-Modified IMU initialization strategy
-Test the entire process in test/testTrackerAll.cpp

### 6.12
-add clahe into frame creation to balance the grayscale histogram.
-add a grid in LK tracking and making features will not be too large.
-fix the bugs caused by not updating the world position of map points in backend optimization. The pose will not jump now.
-Tune the process with IMU and fix some bugs. Still have problems
-Adjust the DeleteRedundant position and place it before DeleteKF(0). When the window includes 30KF, it can handle the still scene of MH01

### 6.10
-add LK tracker, derived from ORB Tracker, see cv/src/TrackerLK.cpp
-There are so many points ...

### 6.9
-add good feature to track and opencv's optical flow

### 6.8
-The method of calculating the parallax of the optical flow between the left and right eyes has been added, but the effect is not very good for the close range. Slightly faster than the original orb.

### 6.7
-fixbug "v is nullptr in Local ba without imu": Delete the expired mpRefKF from the mObservations of MapPoint when CleanMapPoint
-Added calculation of optical flow part, see cv/Align.cpp, cv/LKFlow.cpp, test see test/testLKFlow.cpp
-For the optical flow parameters, see the constants in align.h, reducing the patch will increase the speed significantly.

### 6.6
-Merge the shared_ptr related modifications to the master
-In the computeError() of EdgeProjectPoseOnly, if invz<0, _error cannot be set to zero, otherwise it will be treated as an inlier in poseoptimization
-Test the pure vision solution, see test/testPureVision.cpp
-wj-Add NeedNewKeyFrame() logic
-In the OptimizePose of wj-Tracker, no points with Observations less than 1 (TBD) are not added

### 6.5
-Changed many memory-related things to shared_ptr and weak_ptr, so you don’t have to worry about memory leaks anymore!

### 6.4
-wj:
-Frame::SetDelete, after modifying the shared_ptr, should be considered according to the situation

### 6.3
-xiang:
-Under 16.04 or the same version of linux, there is a sentence setZero in g2o/core/jacobian_workspace.cpp, which needs to add dimension. Otherwise, it is easy to cause a seg fault when opening up the Jacobian space.

### 6.2
-fix many Eigen::aligned_allocator problem
-rewrite the test program in test/xxx
-add stereo imu initialization code
-wj:
-Add the constraint of g=9.81 in imuinitialization, and add an estimation step.
-Added to the back-end pure visual LocalBA: delete the observation of outlier
-The success condition of imuinitialization: the modulus length of g estimated without constraint is between 9.6 and 10.0, and the modulus length of the error of the accelerometer zero bias estimated before and after the constraint g=9.81 is less than 0.2m/s^2
Note: Look at the conditions of the log and think about it. . TBD
-In V101, about 31 KF can meet the conditions (starting from the 6th. The first 6s did not move, and it was skipped during the test), and MH01 requires more than 50 KF. The estimated zero offset may still have errors, and continue to optimize afterwards.
-After successful imuinitialization, align the w system with the direction of gravity and set mgWrold=[0,0,9.81], so that the gravity is considered to be a true value and no optimization update is required.


### 5.31
-Modified the memory management problem in testViewer.

### 5.30
-Added the process of adding new map points on the backend, but it has not been tested

### 5.29
-fix many things in tracker and ORBExtractor, ORBMatcher by testing stereo init.


### 5.27
-Added Viewer test, see test/testViewer.cpp
-Due to the problem of pangolin/OpenGL, an error will occur when the openGL thread is newly opened under osx, so testViewer calls the visualization in the main thread

### 5.21
-Ready to start testing the binocular initialization part of the code with IMU

### 5.20
-Added Local BA test with IMU, see test/testLocalBAIMU
-Under osx, the g2o optimizer will generate double free problems when destructuring and deleting vertices. The reason is unknown. There is no problem under Ubuntu.

### 5.17
-The backend LocalBA passed the test, see test/testLocalBA

### 5.12
-Added two back-end BAs, to be tested

### 5.10
-Add some g2otypes, use P+R, V, Ba, Bg for basic representation
-Start adding Tracker content

### 4.27 The data structure is basically completed, and feature extraction is added (to be tested)
### 2017.4.26 Adjust the architecture and add front-end algorithms