#ifndef QUADRICLANDMARK_H
#define QUADRICLANDMARK_H

/*
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/StdVector>*/
//#include <quadric_slam/build_quadric.hpp>
#include <quadric_slam/g2o_Object.h>
#include <vector>

#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"
#include "MapPoint.h"
#include <unordered_map>
#include <opencv2/core/core.hpp>
#include <mutex>



namespace ORB_SLAM2
{
class Frame;
class KeyFrame;
class MapPoint;
class Map;
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
    QuadricLandmark(Map *pMap, bool whether_update_index = false);

    float bboxOverlapratio(const cv::Rect &rect1, const cv::Rect &rect2);

    inline Eigen::Vector4d RecttoVector4d(cv::Rect &bbox_2d);
    void SetReferenceKeyFrame(KeyFrame *reKF);
    KeyFrame *GetReferenceKeyFrame();
    KeyFrame *GetLatestKeyFrame();
    bool isBad();
    void SetBadFlag();
    bool isGood;

   g2o::Quadric pose_Twc_latesKF;

   g2o::Quadric pose_Twc_afterba;
    
    unsigned long int mnId;           //unique id
    static unsigned long int nNextId; //可理解为lastID
    static long int GetIncrementedIndex();
    //------local members------
    //当前检测框，用来查找和bbox关联的keyPoints与MapPoints  //仅使用检测框，关联会不够鲁棒
    int mnLocalId; //在关键帧中的局部id
    //应投影在检测框内的KP mappoint
    std::vector<MapPoint *> mvpRelatedMP;
    std::vector<cv::KeyPoint *> mvpRealtedKP;
    Eigen::Vector4d bbox_vec;   //当前bbox参数 center width height  for optimize
    //added by yxqc
    cv::Rect bbox_2d;                  // x y w h  Rect type
    cv::Rect bbox_2d_tight;
    double prop;                       // detection confidence
    double class_id;
    std::vector<int> associate_points; //feature points in bbox

    bool mbIsInitialized; //landmark state
    bool mbIsOptimized;   //是否已优化
    bool mbIsAssociated;  //是否已关联
    bool mbIsCandidate;   //是否为候选landmark

    bool become_candidate;
    g2o::Quadric mQuadricMeas; // quadric_value
    g2o::Quadric mQuadricLocalMeas;
    g2o::VertexQuadric *mpQuadricVertex; //to remove
    double mMeasQuality;                 // [0,1] the higher, the better
    Vector4d conic_bbox;  //projection
    Vector4d conic;
    //associated landmark may be itself
    QuadricLandmark *mpAssocaitedLandmark;
    QuadricLandmark *mpLocalObject; //local object

    //used in optimization
    //keyframes co-observing this landmark and according landmark index
    void AddObservation(KeyFrame *pKF, size_t idx);
    void EraseObservation(KeyFrame *pKF);
    std::unordered_map<KeyFrame *, size_t> GetObservations();
    int Observations();

    std::vector<KeyFrame *> GetObserveFrames();
    std::vector<KeyFrame *> GetObserveFramesSequential();
    std::vector<KeyFrame *> observed_frames;
    bool IsInKeyFrame(KeyFrame *pKF);
    int GetIndexInKeyFrame(KeyFrame *pKF);

    bool CheckIsValidObjectByKeyPoints(int own_keypoints_thre);

    std::set<MapPoint *> GetAssociatedMP();

    void AddAssociatedMP(MapPoint *pMP);
    bool CheckIsValidObject(int own_points_thre);

    long unsigned int mnBALocalForKF; //for local mappping
    int point_quadric_BA_counter = -1;


    std::vector<MapPoint*>used_points_in_BA;//points with enough observation will be used  in  BA
    std::vector<MapPoint*>used_points_in_BA_filtered;//actually BA used points,after removing some for away points


    long unsigned int mnAssociationRefidInTracking;
    long unsigned int mnAssociationInitialRefidInTracking;
    
   
    void SetWorldPos(const g2o::Quadric &Pos);
    g2o::Quadric GetWorldPos();   
    g2o::SE3Quat GetWorldPosInv();
    g2o::Quadric GetWorldPosBA();

 
    void QuadricInit();
    void QuadricProject();
    void QuadricProjectToCurrent(KeyFrame* mCKF);
    void SaveTrajectory();
    void SaveProjection(std::vector<Eigen::Matrix<double, 3, 4>,Eigen::aligned_allocator<Eigen::Matrix<double, 3, 4>>>&vProjMats);
    void SaveQuadric();
    void SavePlanes(std::vector<Eigen::Vector4d>&vplanes);
    void SaveInitDetectBox(std::vector<Eigen::Vector4d>&vBoxes);
    void SaveMapPoints();

    static std::mutex mGlobalMutex;

    std::vector<MapPoint *> GetUniqueMapPoints();
    int NumUniqueMapPoints();
    void AddUniqueMapPoint(MapPoint *pMP, int obs_num); // called by pointAddobjectObservations
    void EraseUniqueMapPoint(MapPoint *pMP, int obs_num);

    std::vector<MapPoint *> GetPotentialMapPoints();
    void AddPotentialMapPoint(MapPoint *pMP);
    int largest_point_observations; // largest point observations. better use a heap....
    int pointOwnedThreshold;        // some threshold to whether use points in optimization.  set in optimizer.

    bool check_whether_valid_object(int own_point_thre = 15);

    void MergeIntoLandmark(QuadricLandmark *otherLocalObject); // merge the points here. this should already be a landmark. observation should be added elsewhere
    void SetAsLandmark();
    int left_right_to_car;   //on the left or right of a car left =1 ,right=23 undecided =0  initial =-1;
    string QuadricLandmarkFolder; //added by yxqc save data for offline init


protected:
    g2o::Quadric pose_Twc;
    g2o::Quadric pose_Tcw;
    Map *mpMap;
    std::unordered_map<KeyFrame *, size_t> mObservations; //attention: to add mutex lock
    vector<pair<KeyFrame*, int>> mObs;  //added by yxqc  sort  the obsrevations by ascending order of KeyFrame frameID
    vector<pair<KeyFrame*, int>> allObs; 

    int nObs;                                             // observations' count
    std::set<MapPoint *> msAssociatedMP_unique;       // uniquedly owned  by this object;
    std::set<MapPoint*> msAssociatedMP_potential;
    KeyFrame *moRefKF;                                    //Reference KeyFrame first frame see it
    KeyFrame *mLastKF;                                    //Latest frame see it
    bool mbBad;
    std::mutex mMutexPos;
    //std::mutex mMutexFeatures;
    std::mutex mMutexQuadrics;
};

/*class QuadricLandmark
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
    std::unordered_map<KeyFrame*,size_t> GetObservations();
    int Observations();

    bool IsInKeyFrame(KeyFrame* pKF);
    int GetIndexInKeyframe(KeyFrame* pKF);
    bool IsInBox(cv::KeyPoint* pKP);    //check whether keypoint is located in bbox

    std::set<MapPoint*> GetAssociatedMP();
    void AddAssociatedMP(MapPoint* pMP);
    bool CheckIsValidObject(int own_points_thre);

    long unsigned int mnBALocalForKF;   //for local mappping
    long unsigned int mnAssociationRefidInTracking;

    //to initial the quadric landmark
    //to modify
    void QuadricInit(std::_map<KeyFrame*,size_t> Observations);

    std::mutex mMutexQuadrics;
    static std::mutex mGlobalMutex;

protected:
    Map* mpMap;
    std::map<KeyFrame*,size_t> mObservations;   //attention: to add mutex lock
    int nObs;   // observations' count
    std::set<MapPoint*> msAssociatedMP;  //与quadric关联的mappoints

};
*/
} // namespace ORB_SLAM2
#endif //QuadricLandmark.h
