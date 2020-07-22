### Quadric SLAM Based on ORB SLAM2
### under construction

### add online detection thread(quadric_slam/Detecting.cc) using darknet yolov3(fit bugs about memory leak and variable name conflict)
## save bbox in SaveDetectBox folder for offline model with KITTI dataset seq00 seq07 seq05

### Useage(add the fifth parameter for Detection(offline is 0 online is 1)  
 ## offline detect model:(need to change read offline_box_folder in Frame::ReadOfflineBox() )
   ./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER 0 

   ./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER 0
 ## online detect model:(need to change save online_detect_folder in Tracking::SaveOnlineDetectBox() )
  ./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER 1 

  ./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER 1

### add Data association for Quadric Landmark

### add project the Quadric landmark to all associated observations in QuadricLandmark::QuadricProject() 
### add project all Quadric landmarks in map to KeyFrame in LocalKeyFrames in  QuadricLandmark::QuadricProjectToCurrent(KeyFrame *mCurrentKF)

### save data for Quadric Init offline Debug
## need to change mkdir folders in Tracking::Tracking() 
## need to change mkdir folders in QuadricLandmark::QuadricInit()

## add object optimization without dynamic (need to be checked) 

### add one condition for KeyFrame in Tracking::NeedNewKeyFrame() const bool c1d

##  finish quadric initization function 

##  add save and load map function(mappoints and keyframes) need change Examples/Monocular/mono_kitt.cc
