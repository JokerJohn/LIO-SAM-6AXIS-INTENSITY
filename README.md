# LIO-SAM-6AXIS-INTENSITY
[LIO-SAM-6AXIS](https://github.com/JokerJohn/LIO_SAM_6AXIS) with intensity image-based  loop optimization. 

This repo integrates [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) and [Imaging_lidar_place_recognition](https://github.com/TixiaoShan/imaging_lidar_place_recognition) to achieve better mapping and localization result for SLAM system. Thanks for their hard work！

- Contains all the functions of the LIO-SAM-6AXIS.
- support the intensity loop detection method with ICP refine to get the robust loop constriant and add them to the factor graph finally.

# some instructions

The system also has the following problems

- Effective for high-beam lidars, but poor for low-beam lidars such as VLP16

- Effective for indoor scenes, many closed-loop constraints found are better than the original ICP method, and the effect is poor in outdoor open scenes.

- It must be ensured that the number of point clouds is equal to the number of pixels of the intensity image.

- In the outdoor scene, the resolution of the intensity image is too low, and it is difficult to realize the scene recognition stably

We analyze why it is effective in indoor scenarios

1. There is no extrinsinc error when the lidar is projected to the intensity image compared to the camera.
2. Each pixel of the intensity image corresponds to a lidar point, there is an accurate depth, and the feature matching effect is good after 3D PNP Ransanc.

![image-20220716205706914](README/image-20220716205706914.png)

（red line segments are for visual loop constraint from intensity image）

![image-20220716211813361](README/image-20220716211813361.png)
