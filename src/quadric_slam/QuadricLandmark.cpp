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
}
inline Eigen::Vector4d Detection::toBbox()
{
    Vector4d res;
    res[0] = x - w/2;
    res[1] = y - h/2;
    res[2] = x + w/2;
    res[3] = y + h/2;
    return res;
}


/******************
 * QuadricLandmark*
 * ****************/

long unsigned int QuadricLandmark::nNextId=0;
mutex QuadricLandmark::mGlobalMutex;
//todo: construct
QuadricLandmark::QuadricLandmark(Detection* pDet,Map* pMap):mpCurrDetection(pDet),mpMap(pMap)
{
    nObs=0;
    mMeasQuality=0.6;
    mnId=nNextId++;
    mbIsInitialized = false;
    mbIsInitialized = false;
    mbIsAssociated=false;

    mpAssocaitedLandmark=nullptr;
    mnLocalId=pDet->mnLocalId;
    mBox=pDet->toBbox();

}

unsigned long int QuadricLandmark::GetIncrementedIndex()
{
	nNextId++;
	return nNextId;
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
        //int idx = mObservations[pKF];
        mObservations.erase(pKF);
        nObs--;
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

bool QuadricLandmark::IsInKeyFrame(KeyFrame *pKF)
{
	unique_lock<mutex> lock(mMutexQuadrics);
	return (mObservations.count(pKF));
}

int QuadricLandmark::GetIndexInKeyframe(KeyFrame *pKF)
{
	unique_lock<mutex> lock(mMutexQuadrics);
	if (mObservations.count(pKF))
		return mObservations[pKF];
	else
		return -1;
}

bool QuadricLandmark::IsInBox(KeyPoint* pKP)
{
    float x=pKP->pt.x;
    float y=pKP->pt.y;

    if(x>mBox[0]&&x<mBox[2]&&y>mBox[1]&&y<mBox[3])
        return true;
    else
        return false;
}

std::set<MapPoint*> QuadricLandmark::GetAssociatedMP()
{
    unique_lock<mutex> lock(mMutexQuadrics);
    return msAssociatedMP;
}

void QuadricLandmark::AddAssociatedMP(MapPoint* pMP)
{
    unique_lock<mutex> lock(mMutexQuadrics);
    msAssociatedMP.insert(pMP);
}

 bool QuadricLandmark::CheckIsValidObject(int own_points_thre=15)
 {
    if((int)msAssociatedMP.size()>own_points_thre)
        mbIsCandidate=true;
    else
        mbIsCandidate=false;
    return mbIsCandidate;
 }


void QuadricLandmark::QuadricInit(std::map<KeyFrame*,size_t> Observations)
{
    //如果bbox框过少，或者quadric已经初始化，则返回
    if (Observations.size() < 3 || mbIsInitialized == true)
    {
        std::cout<<"bbox过少，或quadric已初始化"<<std::endl;
        return;
    }

    //begin init
    std::cout<<"begin quadriclandmark init..."<<endl;
    //提取bbox
    std::vector<Eigen::Vector4d,
        Eigen::aligned_allocator<Eigen::Vector4d>> vBoxes;
    for (auto det=Observations.begin();det!=Observations.end();det++)
    {
        Eigen::Vector4d box;
        int idx=det->second;
        box=det->first->mvpQuadricLandmarks[idx]->mpCurrDetection->toBbox();
    }

    //提取Kalib 计算projection matrix K*R|t 3*4
    std::vector<Eigen::Matrix<double, 3, 4>,
        Eigen::aligned_allocator<Eigen::Matrix<double, 3, 4>>> vProjMats;

    Eigen::Matrix3d Kalib;
    Kalib = Converter::toMatrix3d(Observations.begin()->first->mK);
    for (auto det = Observations.begin(); det != Observations.end(); det++)
    {
        Eigen::Matrix<double, 3, 4> temp;
        temp = Kalib*Converter::toProjMat(det->first->GetPose());
        vProjMats.push_back(temp);
    }
    assert(vProjMats.size() == vBoxes.size());

    //开始计算
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> lines;
    Box2Line(vBoxes,lines);
    assert(lines.size() / 4 == vProjMats.size());

    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>
        planes;
    ComputePlanesMat(vProjMats, lines, planes);

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