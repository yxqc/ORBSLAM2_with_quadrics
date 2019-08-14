//TODO if necessary
#include "QuadricLandmark.h"
#include "Converter.h"
#include <iostream>

#define QUALITY_THRESHOLD 0.5

namespace ORB_SLAM2
{
class Converter;

/****************
 * Detection*****
 * **************/
Detection::Detection(Vector6d raw_2d_objs,int local_id)
{
    x = raw_2d_objs[0];
    y = raw_2d_objs[1];
    w = raw_2d_objs[2];
    h = raw_2d_objs[3];
    prob = raw_2d_objs[4];
    mnClassId = raw_2d_objs[5]; //double to int
    mnLocalId=local_id;
    nTrackings=0;
}
inline Eigen::Vector4d Detection::toBbox()
{
    Vector4d res;
    res[0] = x;
    res[1] = y;
    res[2] = x + w;
    res[3] = y + h;
    return res;
}

void Detection::AddTracking(Frame *pF, size_t idx)
{
    unique_lock<mutex> lock(mMutexDetTrackings);
    if (mDetTracking.count(pF)) //if already containing the KF -> return
        return;
    mDetTracking[pF] = idx;

    /*
     * 单目只有一个观测，针对keypoint，对quadric暂时无意义
    if(pF->mvuRight[idx]>=0)
        nTrackings+=2;
    else
        nTrackings++;
    */
    nTrackings++;
}

void Detection::EraseTracking(Frame *pF)
{
    unique_lock<mutex> lock(mMutexDetTrackings);
    if (mDetTracking.count(pF))
    {
        //int idx = mDetTracking[pF];
        nTrackings--;
        mDetTracking.erase(pF);
    }
}

std::map<Frame *, size_t> Detection::GetTrackings()
{
    unique_lock<mutex> lock(mMutexDetTrackings);
    return mDetTracking;
}

int Detection::Trackings()
{
    unique_lock<mutex> lock(mMutexDetTrackings);
    return nTrackings;
}


/******************
 * QuadricLandmark*
 * ****************/

long unsigned int QuadricLandmark::nNextId=0;
//todo: construct
QuadricLandmark::QuadricLandmark()
{
    nObs=0;
    mMeasQuality=0.6;
    mnId=nNextId++;
    mbIsInitialized = false;
    mbIsInitialized = false;
    
    //待补充
}

void QuadricLandmark::AddObservation(KeyFrame *pKF, size_t idx)
{
    unique_lock<mutex> lock(mMutexQuadrics);
    if (mObservations.count(pKF)) //if already containing the KF -> return
        return;
    mObservations[pKF] = idx;

    /*
     * 单目只有一个观测，针对keypoint，对quadric暂时无意义
    if(pKF->mvuRight[idx]>=0)
        nObs+=2;
    else
        nObs++;
    */
    nObs++;
}

void QuadricLandmark::EraseObservation(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexQuadrics);
    if (mObservations.count(pKF))
    {
        int idx = mObservations[pKF];
        nObs--;

        mObservations.erase(pKF);
    }
}

std::map<KeyFrame *, size_t> QuadricLandmark::GetObservations()
{
    unique_lock<mutex> lock(mMutexQuadrics);
    return mObservations;
}

int QuadricLandmark::Observations()
{
    unique_lock<mutex> lock(mMutexQuadrics);
    return nObs;
}

void QuadricLandmark::QuadricInit(std::map<Frame*,size_t> DetTracking)
{
    //如果bbox框过少，或者quadric已经初始化，则返回
    if (DetTracking.size() < 10 || mbIsInitialized == true)
    {
        std::cout<<"bbox过少，或quadric已初始化"<<std::endl;
        return;
    }

    //begin init
    //提取bbox
    std::vector<Eigen::Vector4d,
        Eigen::aligned_allocator<Eigen::Vector4d>> vBoxes;
    for (auto det=DetTracking.begin();det!=DetTracking.end();det++)
    {
        Eigen::Vector4d box;
        int idx=det->second;
        box=det->first->mvpDetctions[idx]->toBox();
    }

    //提取各帧对应projection matrix R|t,提取Kalib
    std::vector<Eigen::Matrix<double, 3, 4>,
        Eigen::aligned_allocator<Eigen::Matrix<double, 3, 4>>> vProjMats;

    Eigen::Matrix3d Kalib;
    Kalib = Converter::toMatrix3d(DetTracking.begin()->first->mK);

    for (auto det = DetTracking.begin(); det != DetTracking.end(); det++)
    {
        Eigen::Matrix<double, 3, 4> temp;
        temp = Converter::toProjMat(det->first->mTcw);
        vProjMats.push_back(temp);
    }
    assert(vProjMats.size() == vBoxes.size());

    //开始计算
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
        lines;
    ComputeLineMat(vBoxes, lines);
    assert(lines.size() / 4 == vProjMats.size());

    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>
        planes;
    ComputePlanesMat(Kalib, vProjMats, lines, planes);

    std::vector<Eigen::Matrix<double, 1, 10>,
                Eigen::aligned_allocator<Eigen::Matrix<double, 1, 10>>>
        planes_parameter;

    ComputePlanesParameters(planes, planes_parameter);

    Eigen::Matrix3d rotation;
    Eigen::Vector3d shape;
    Eigen::Vector3d translation;
    Eigen::Matrix4d constrained_quadric;

    ComputeDualQuadric(planes_parameter, rotation, shape, translation,
                       constrained_quadric);
    
    if(mMeasQuality > QUALITY_THRESHOLD)
    {
        mQuadricMeas=g2o::Quadric(rotation, translation, shape);
        //mQuadricVertex = new g2o::VertexQuadric();//放入构造函数
        mpQuadricVertex->setEstimate(mQuadricMeas); 
        mbIsInitialized=true;
    }
}

} // namespace ORB_SLAM2