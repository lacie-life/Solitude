
# Dependency
If you are using ubuntu, just type "./install_dep.sh" to install all the dependencies except pangolin.

- Pangolin (for visualization): https://github.com/stevenlovegrove/Pangolin 
- Eigen3: sudo apt-get install libeigen3-dev
- g2o: sudo apt-get install libcxsparse-dev libqt4-dev libcholmod3.0.6 libsuitesparse-dev qt4-qmake 
- OpenCV: sudo apt-get install libopencv-dev
- glog (for logging): sudo apt-get install libgoogle-glog-dev

# Compile
run "./generate.sh" to compile all the things, or follow the steps in generate.sh

# Examples
You can put stereo or stereo-imu data into ygz-stereo, for example the EUROC dataset 
(http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). We provide the pure stereo vision and stereo-inertial vision for EUROC. Run the pure vision examples by typing:

```
bin/EurocStereo ./examples/EurocStereo.yaml
```

to run the pure vision mode. Don't forget to specify the dataset directory in the yaml config file first. Also, to run visual-inertial mode, type: 
```
bin/EurocStereoVIO ./examples/EurocStereoVIO.yaml
```

to run the stereo VIO case.



