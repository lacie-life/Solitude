# Master Thesis

## SLAM TODO

- [ ] Review VIO based 
    - [ ] IMU preintegration
    - [ ] GTSAM fusion
    - [ ] rpg_svo_pro_open framework
    - [ ] FAST feature in CUDA
    - [x] ORB_SLAM2
- [ ] Build a Stereo VIO SLAM based on ORB_SLAM2 => kms_slam
    - [ ] Run VIO ORB_SLAM2
    - [ ] Modify IMU preintegration of VIO ORB_SLAM2 (based on Forster C, Carlone L, Dellaert F, et al. On-Manifold Preintegration for Real-Time Visual--Inertial Odometry. IEEE Transactions on Robotics, 2017, 33(1): 1-21. PDF.)
- [ ] Add Fast feature and KTL tracking to kms_slam
    - [ ] Faster than FAST ![Paper](http://rpg.ifi.uzh.ch/docs/IROS20_Nagy.pdf) 
- [ ] Using factor graph in globally bundle adjusted map
    - [ ] Ref rpg_svo_pro_open
    - [ ] iSAM2 ![Paper](http://frank.dellaert.com/pub/Kaess12ijrr.pdf)
- [ ] Run QradricSLAM (object detect by Yolo) ![Paper](https://arxiv.org/abs/1804.04011)
- [ ] Add Qradric feature to kms_slam mapping

## 4D Reconstruction TODO

- [ ] Run QradricSLAM (object detect by Yolo) ![Paper](https://arxiv.org/abs/1804.04011)
- [ ] GTSAM Factor Graph
- [ ] Review 4D Reconstruction
- [ ] Encode Point Cloud by Quadric feature


