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
class Map;

class Detection
{
public:
    double x,y,w,h,prob;    //x,y左上角坐标，width,height,probability置信度
    int mnClassId;  //semantic class id
    //int mnId;   //bbox global id
    int mnLocalId;   //单帧中的每个bbox 的id,
    //int mnFrameID;   //releated Frame ID

    //std::vector<Detection*> mvpBoxTracking; //to remove

    //应投影在检测框内的KP mappoint
    //std::vector<MapPoint*> mvpRelatedMP;
    //std::vector<cv::KeyPoint*> mvpRealtedKP;

    //construction todo:构造其他参数
    Detection(Vector6d raw_2d_objs,int local_id);
    //return (x_min y_min x_max y_max) bbox representation
    inline Eigen::Vector4d toBbox();
    //todo: data association

    //std::mutex mMutexDetTrackings;

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
    QuadricLandmark(Detection* pDet,Map* pMap);  // todo:add construct function

    unsigned long int mnId; //unique id
    static unsigned long int nNextId;   //可理解为lastID
    unsigned long int GetIncrementedIndex();

    //------local members------
    //当前检测框，用来查找和bbox关联的keyPoints与MapPoints  //仅使用检测框，关联会不够鲁棒
    Detection* mpCurrDetection;
    int mnLocalId;       //在关键帧中的局部id=detection->mnLocalId
    //应投影在检测框内的KP mappoint
    std::vector<MapPoint*> mvpRelatedMP;
    std::vector<cv::KeyPoint*> mvpRealtedKP;
    Eigen::Vector4d mBox;   //当前bbox参数 x_min y_min x_max y_max

    bool mbIsInitialized;   //landmark state
    bool mbIsOptimized;     //是否已优化
    bool mbIsAssociated;    //是否已关联
    bool mbIsCandidate;     //是否为候选landmark

    g2o::Quadric mQuadricMeas; // quadric_value
    g2o::VertexQuadric *mpQuadricVertex; //to remove
    double mMeasQuality; // [0,1] the higher, the better

    //associated landmark may be itself
    QuadricLandmark* mpAssocaitedLandmark;

    //used in optimization
    //keyframes co-observing this landmark and according landmark index
    void AddObservation(KeyFrame* pKF,size_t idx);
    void EraseObservation(KeyFrame* pKF);
    std::map<KeyFrame*,size_t> GetObservations();
    int Observations();

    bool IsInKeyFrame(KeyFrame* pKF);
    int GetIndexInKeyframe(KeyFrame* pKF);
    bool IsInBox(KeyPoint* pKP);    //check whether keypoint is located in bbox

    std::set<MapPoint*> GetAssociatedMP();
    void AddAssociatedMP(MapPoint* pMP);
    bool CheckIsValidObject(int own_points_thre);

    long unsigned int mnBALocalForKF;   //for local mappping
    long unsigned int mnAssociationRefidInTracking;

    //to initial the quadric landmark
    //to modify
    void QuadricInit(std::map<KeyFrame*,size_t> Observations);

    std::mutex mMutexQuadrics;
    static std::mutex mGlobalMutex;

protected:
    Map* mpMap;
    std::map<KeyFrame*,size_t> mObservations;   //attention: to add mutex lock
    int nObs;   // observations' count
    std::set<MapPoint*> msAssociatedMP;  //与quadric关联的mappoints

};

} //namespace ORBSLAM2
#endif //QuadricLandmark.h