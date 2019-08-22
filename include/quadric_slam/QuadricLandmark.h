#ifndef QUADRICLANDMARK_H
#define QUADRICLANDMARK_H

#include <quadric_slam/g2o_Object.h>
#include <quadric_slam/build_quadric.hpp>
#include <vector>

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"
#include"MapPoint.h"

#include<opencv2/core/core.hpp>
#include<mutex>


namespace ORB_SLAM2
{
class Frame;
class KeyFrame;
class MapPoint;

class Detection
{
public:
    double x,y,w,h,prob;    //x,y左上角坐标，width,height,probability置信度
    int mnClassId;  //semantic class id
    //int mnId;   //bbox global id
    int mnLocalId;   //单帧中的每个bbox 的id,
    //int mnFrameID;   //releated Frame ID

    //std::vector<Detection*> mvpBoxTracking; //to remove

    //key-value
    //连续帧关联的detections，frame*->detection[size_t]
    std::map<Frame*,size_t> mDetTracking;
    int nTrackings;   // tracking_box' count

    //应投影在检测框内的KP mappoint
    std::vector<MapPoint*> mvpRelatedMP;
    std::vector<cv::KeyPoint*> mvpRealtedKP;

    std::map<Frame*,size_t> GetTrackings(); 
    int Trackings();
    void AddTracking(Frame* pKF,size_t idx);
    void EraseTracking(Frame* pKF);

    //construction todo:构造其他参数
    Detection(Vector6d raw_2d_objs,int local_id);
    //return (x_min y_min x_max y_max) bbox representation
    inline Eigen::Vector4d toBbox();
    //todo: data association
    //void BoxTacking();

    std::mutex mMutexDetTrackings;

};

/*
enum DETECT_RESULT
{
    NO_QUADRIC,
    NEW_QUADRIC,
    UPDATE_QUADRIC
};*/

class QuadricLandmark
{
public:
    QuadricLandmark();  // todo:add construct function

    unsigned long int mnId;
    static unsigned long int nNextId;   //可理解为lastID

    bool mbIsInitialized;
    bool mbIsOptimized;

    g2o::Quadric mQuadricMeas; // quadric_value
    g2o::VertexQuadric *mpQuadricVertex; //to remove
    double mMeasQuality; // [0,1] the higher, the better

    //used in optimization
    //keyframes co-observing this landmark and according landmark index
    std::map<KeyFrame*,size_t> mObservations;   //attention: to add mutex lock
    int nObs;   // observations' count
    std::map<KeyFrame*,size_t> GetObservations();
    int Observations();
    void AddObservation(KeyFrame* pKF,size_t idx);
    void EraseObservation(KeyFrame* pKF);

    //to initial the quadric with muti frames constrain-> BoxTracking
    //to modify
    void QuadricInit(std::map<Frame*,size_t> DetTracking);

    std::mutex mMutexQuadrics;
};

} //namespace ORBSLAM2
#endif //QuadricLandmark.h