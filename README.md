# My Researchs

## SLAM TODO

- [ ] Review VIO based 
    - [x] IMU preintegration
    - [ ] GTSAM fusion
        - [ ] Understand factor graphs
        - [ ] Understach custom factor
    - [ ] rpg_svo_pro_open framework
    - [x] FAST feature in CUDA
    - [x] ORB_SLAM2 [Source code](https://github.com/raulmur/ORB_SLAM2)
- [ ] Review Data Accociation Method
    - [ ] QuadricFeature
- [ ] Build a Stereo VIO SLAM based on ORB_SLAM2 => kms_slam
    - [x] Run VIO ORB_SLAM2
    - [x] Modify IMU preintegration of VIO ORB_SLAM2 (based on [Paper](https://arxiv.org/abs/1512.02363))
    - [x] Test in ErUoC Dataset
    - [ ] Test IMU preintegration of VIO ORB_SLAM2 using GTSAM
- [ ] Add Fast feature and KTL tracking to kms_slam
    - [ ] Faster than FAST [Paper](http://rpg.ifi.uzh.ch/docs/IROS20_Nagy.pdf) 
- [ ] Using factor graph in globally bundle adjusted map
    - [ ] Ref rpg_svo_pro_open
    - [ ] iSAM2 [Paper](http://frank.dellaert.com/pub/Kaess12ijrr.pdf)
- [ ] Run QuadricSLAM (object detect by Yolo) [Paper](https://arxiv.org/abs/1804.04011)
- [ ] Add Quadric feature to kms_slam mapping
- [ ] MSCKF in Multicamera 
    - [ ] Hardware
    - [ ] open-VINS
    - [ ] Software

## 4D Reconstruction TODO

- [ ] Run QuadricSLAM (object detect by Yolo) [Paper](https://arxiv.org/abs/1804.04011)
- [ ] GTSAM Factor Graph
- [ ] Review 4D Reconstruction
- [ ] Encode Point Cloud by Quadric feature
- [ ] Livox Mid-40 test
- [ ] LOAM test

## Place Recognition TODO

- [x] Review Place Recognition
- [x] Test NetVLAD
- [ ] Loopclosing use NetVLAD



