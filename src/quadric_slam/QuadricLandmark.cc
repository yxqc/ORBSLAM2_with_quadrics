//TODO if necessary
#include "QuadricLandmark.h"
#include "Converter.h"
#include <fstream>
#include <Eigen/Dense>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

#define QUALITY_THRESHOLD 0.5

namespace ORB_SLAM2
{

bool compare_map(const pair<KeyFrame*, int>&a, const pair<KeyFrame*, int>&b) {
	  return a.first->mnFrameId < b.first->mnFrameId;
}
using namespace Eigen;
class Converter;

/******************
 * QuadricLandmark*
 * ****************/
mutex QuadricLandmark::mGlobalMutex;

long unsigned int QuadricLandmark::nNextId = 0;


//todo: construct
QuadricLandmark::QuadricLandmark(Map *pMap, bool whether_update_index) : mbBad(false), mpMap(pMap)
{
    if (whether_update_index)
        mnId = nNextId++;
    else
        mnId = -1; //没有变成从local object变成landmark，index为-1
    nObs = 0;
    mMeasQuality = 0.6;

    mbIsInitialized = false;
    mbIsCandidate = false;
    mbIsOptimized = false;
    mbIsAssociated = false;

    moRefKF = nullptr;
    mLastKF = nullptr;
    mpAssocaitedLandmark = nullptr;

    mnLocalId = -1;
    pointOwnedThreshold = 0;
    largest_point_observations = 0;

    mnAssociationRefidInTracking = -1;
    mnAssociationInitialRefidInTracking = -1;

    left_right_to_car = -1;
    isGood = false;
    //mBox = pDet->toBbox();
}

float QuadricLandmark::bboxOverlapratio(const cv::Rect &rect1, const cv::Rect &rect2)
{

    int overlap_area = (rect1 & rect2).area();

    return (float)overlap_area / ((float)(rect1.area() + rect2.area() - overlap_area));
}

bool QuadricLandmark::isBad()
{
    unique_lock<mutex> lock(mMutexQuadrics);
    unique_lock<mutex> lock2(mMutexPos);
    return mbBad;
}

bool QuadricLandmark::IsInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexQuadrics);
    return (mObservations.count(pKF));
}

void QuadricLandmark::SetReferenceKeyFrame(KeyFrame *reKF)
{

    moRefKF = reKF;
}

KeyFrame *QuadricLandmark::GetReferenceKeyFrame()
{
    return moRefKF;
}
KeyFrame *QuadricLandmark::GetLatestKeyFrame()
{

    unique_lock<mutex> lock(mMutexQuadrics);
    return mLastKF;
}

inline Eigen::Vector4d QuadricLandmark::RecttoVector4d(cv::Rect &Bbox)
{
    Vector4d res;
    res[0] = Bbox.x;
    res[1] = Bbox.y;
    res[2] = Bbox.x + Bbox.width;
    res[3] = Bbox.y + Bbox.height;
    return res;
}
long int QuadricLandmark::GetIncrementedIndex()
{
    nNextId++;
    return nNextId;
}

void QuadricLandmark::AddObservation(KeyFrame *pKF, size_t idx)
{
    unique_lock<mutex> lock(mMutexQuadrics);
    if (mLastKF == nullptr)
        mLastKF = pKF;
    if (pKF->mnId >= mLastKF->mnId)
        mLastKF = pKF;
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
    observed_frames.push_back(pKF);
}

void QuadricLandmark::EraseObservation(KeyFrame *pKF)
{
    bool bBad = false;
    unique_lock<mutex> lock(mMutexQuadrics);
    if (mObservations.count(pKF))
    {
        //int idx = mObservations[pKF];
        mObservations.erase(pKF);
        if (moRefKF == pKF)
        {
            long unsigned int min_kf_id = mLastKF->mnId + 1;
            KeyFrame *smallest_kf = nullptr;
            for (unordered_map<KeyFrame *, size_t>::iterator mit = mObservations.begin(), mend = mObservations.end(); mit != mend; mit++)
            {
                if (mit->first->mnId < min_kf_id)
                    smallest_kf = mit->first;

                moRefKF = smallest_kf;
            }
            if (mLastKF == pKF)
            {
                observed_frames.pop_back();
                mLastKF = observed_frames.back();
            }
            nObs--;
            if (nObs <= 0)
                bBad = true;
        }
    }
    if (bBad)
        SetBadFlag();
}

void QuadricLandmark::SetBadFlag()
{
    unordered_map<KeyFrame *, size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexQuadrics);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad = true;
        obs = mObservations;
        mObservations.clear();
        observed_frames.clear();
    }
    for (unordered_map<KeyFrame *, size_t>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++)
    {
        KeyFrame *pKF = mit->first;
        pKF->EraseQuadricLandmarkMatch(mit->second);
    }
    mpMap->EraseQuadricLandmark(this);
}

std::unordered_map<KeyFrame *, size_t> QuadricLandmark::GetObservations()
{
    unique_lock<mutex> lock(mMutexQuadrics);
    return mObservations;
}

int QuadricLandmark::Observations() //?mMutexQuadrics
{
    unique_lock<mutex> lock(mMutexQuadrics);
    return nObs;
}

int QuadricLandmark::GetIndexInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexQuadrics);
    if (mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

/*bool QuadricLandmark::IsInBox(cv::KeyPoint *pKP)
{
    float x = pKP->pt.x;
    float y = pKP->pt.y;

    if (x > mBox[0] && x < mBox[2] && y > mBox[1] && y < mBox[3])
        return true;
    else
        return false;
}

std::set<MapPoint *> QuadricLandmark::GetAssociatedMP()
{
    unique_lock<mutex> lock(mMutexQuadrics);
    return msAssociatedMP;
}

void QuadricLandmark::AddAssociatedMP(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexQuadrics);
    msAssociatedMP.insert(pMP);
}*/
bool QuadricLandmark::CheckIsValidObjectByKeyPoints(int own_keypoints_thre)
{
    if (associate_points.size() > own_keypoints_thre)
        become_candidate = true;
    else
        become_candidate = false;
    return become_candidate;
}

bool QuadricLandmark::CheckIsValidObject(int own_points_thre = 15)
{
    if ((int)msAssociatedMP_potential.size() > own_points_thre)
        mbIsCandidate = true;
    else
        mbIsCandidate = false;
    return mbIsCandidate;
}

vector<MapPoint *> QuadricLandmark::GetUniqueMapPoints()
{
    unique_lock<mutex> lock(mMutexQuadrics);
    return vector<MapPoint *>(msAssociatedMP_unique.begin(), msAssociatedMP_unique.end());
}

int QuadricLandmark::NumUniqueMapPoints()
{
    unique_lock<mutex> lock(mMutexQuadrics);
    return msAssociatedMP_unique.size();
}

void QuadricLandmark::AddUniqueMapPoint(MapPoint *pMP, int obs_num)
{
    unique_lock<mutex> lock(mMutexQuadrics);
    msAssociatedMP_unique.insert(pMP);
    if (obs_num > largest_point_observations) // exact way.
        largest_point_observations = obs_num;
}

void QuadricLandmark::EraseUniqueMapPoint(MapPoint *pMP, int obs_num)
{
    unique_lock<mutex> lock(mMutexQuadrics);
    msAssociatedMP_unique.erase(pMP);
    if (obs_num == largest_point_observations)
        largest_point_observations -= 1; // HACK only an approximate way. otherwise need to keep sorted the observations (heap). complicated, not worth it.
}

vector<MapPoint *> QuadricLandmark::GetPotentialMapPoints()
{
    unique_lock<mutex> lock(mMutexQuadrics);
    return vector<MapPoint *>(msAssociatedMP_potential.begin(), msAssociatedMP_potential.end());
}

void QuadricLandmark::AddPotentialMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexQuadrics);
    msAssociatedMP_potential.insert(pMP);
}

void QuadricLandmark::SetWorldPos(const g2o::Quadric &Pos)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    pose_Twc = Pos;
    pose_Tcw = pose_Twc;
    pose_Tcw.pose = pose_Twc.pose.inverse();
}

g2o::Quadric QuadricLandmark::GetWorldPos()
{
    unique_lock<mutex> lock(mMutexPos);
    //if(is_dynamic)
    //return pose_Twc_latesKF;
    return pose_Twc;
}

g2o::SE3Quat QuadricLandmark::GetWorldPosInv()
{
    unique_lock<mutex> lock(mMutexPos);
    return pose_Tcw.pose;
}

g2o::Quadric QuadricLandmark::GetWorldPosBA()
{
    unique_lock<mutex> lock(mMutexPos);
    return pose_Twc_afterba;
}

void QuadricLandmark::SetAsLandmark()
{
    for (set<MapPoint *>::iterator mit = msAssociatedMP_potential.begin(); mit != msAssociatedMP_potential.end(); mit++)
    {

        (*mit)->AddObjectObservation(this); // add into actual object landmark observation. make sure this->already_associated=true;
    }
    cout << "set as landmark" << endl;
}

void QuadricLandmark::MergeIntoLandmark(QuadricLandmark *otherLocalObject)
{
    //NOTE other object is just local. not add to actual point-object observation yet. therefore no need to delete object-point observation.
    for (set<MapPoint *>::iterator mit = otherLocalObject->msAssociatedMP_potential.begin(); mit != otherLocalObject->msAssociatedMP_potential.end(); mit++)
    {
        (*mit)->AddObjectObservation(this);
    }
    cout << "merge into landmark" << endl;
}

void QuadricLandmark::SaveTrajectory()
{
    char temp[256];
    std::string KeyFrameId, KeyFramePose;

    for (auto det =mObs.begin(); det !=mObs.end(); det++)
    {
        sprintf(temp, "%04d", (int)det->first->mnFrameId);
        KeyFrameId.append(temp);
        
    }

    KeyFramePose = QuadricLandmarkFolder + "CameraWorldPose_" + KeyFrameId + ".txt";
    ofstream file;
    file.open(KeyFramePose.c_str());

    for (auto det =mObs.begin(); det !=mObs.end(); det++)
    {
        for (size_t i = 0; i < 4; i++)
            for (size_t j = 0; j < 4; j++)
                file << det->first->GetPoseInverse().at<float>(i, j) << " ";
        file << endl;
    }

}

void QuadricLandmark::SaveProjection(std::vector<Eigen::Matrix<double, 3, 4>,Eigen::aligned_allocator<Eigen::Matrix<double, 3, 4>>>&vProjMats)
{
    char temp[256]; 
    std::string KeyFrameId, projection;

    for (auto det =mObs.begin(); det !=mObs.end(); det++)
    {
        sprintf(temp, "%04d", (int)det->first->mnFrameId);

        KeyFrameId.append(temp);
    }
    projection = QuadricLandmarkFolder + "initprojection_" + KeyFrameId + ".txt";
    ofstream file;
    file.open(projection.c_str());
    for (auto p =vProjMats.begin(); p !=vProjMats.end(); p++)
    {
        for (size_t i = 0; i < 3; i++)
            for (size_t j = 0; j < 4; j++)
              file <<(*p)(i,j)<<" ";
        file<<endl;
        cout << "SaveProjection" << (*p)<< endl;
    }
    
}
void QuadricLandmark::SaveMapPoints()
{
    //std::string saved_dir = QuadricLandmarkFolder;
    char temp[256], object[256]; 
    std::string KeyFrameId, MapWorldPose;

    for (auto det =mObs.begin(); det !=mObs.end(); det++)
    {
        sprintf(temp, "%04d", (int)det->first->mnFrameId);
        KeyFrameId.append(temp);
    }

    sprintf(object, "%d", (int)mObs.begin()->second);
    MapWorldPose = QuadricLandmarkFolder + "MapWorldPose_"+KeyFrameId + "_LocalId_" + object + ".txt";
    ofstream file;
    file.open(MapWorldPose.c_str());
    std::vector<MapPoint *> mapPoints_own =mpAssocaitedLandmark->GetUniqueMapPoints();
    for (auto mapPoint : mapPoints_own)
    {
        for (size_t i = 0; i < 3; i++)
            file << mapPoint->GetWorldPos().at<float>(i, 0) << " ";

        file << endl;

        cout << "3d=" << mapPoint->GetWorldPos() << endl;
    }

}
void QuadricLandmark::SavePlanes(std::vector<Eigen::Vector4d>& vPlanes)
{
    ofstream file;
    char temp[256];
    std::string KeyFrameId, planes;
    for (auto det =mObs.begin(); det !=mObs.end(); det++)
    {
        sprintf(temp, "%04d", (int)det->first->mnFrameId);
        KeyFrameId.append(temp);
    }
    planes = QuadricLandmarkFolder+ "Planes_"+KeyFrameId + ".txt";
    file.open(planes.c_str());
    for(auto p=vPlanes.begin();p!=vPlanes.end();p++){
        file <<(*p)[0]<<" "<<(*p)[1]<<" "<<(*p)[2]<<" "<<(*p)[3]<<endl;
        cout << "Saveplanes" <<(*p)[0]<< endl;
    }
}

void QuadricLandmark::SaveQuadric()
{
    ofstream file;
    char temp[256] ;
    std::string KeyFrameId, QuadricParam;
    for (auto det =mObs.begin(); det !=mObs.end(); det++)
    {
        sprintf(temp, "%04d", (int)det->first->mnFrameId);
        KeyFrameId.append(temp);
    }
    QuadricParam = QuadricLandmarkFolder +"Quadric_"+ KeyFrameId + ".txt";
    file.open(QuadricParam.c_str());
    for (size_t i = 0; i < 3; i++)
        file << mQuadricMeas.scale(i, 0) << " ";

    cv::Mat QuadricPose = Converter::toCvMat(mQuadricMeas.pose);
    for (size_t i = 0; i < 4; i++)
for (size_t j = 0; j < 4; j++)
            file << QuadricPose.at<float>(j, i) << " ";
    file << endl;
    cout << "Quadric Pose" << QuadricPose << endl;
}

void QuadricLandmark::SaveInitDetectBox(std::vector<Eigen::Vector4d>&vBoxes)
{
    ofstream file;
    char temp[256],object[256];
    std::string KeyFrameId, InitDetectBox;
    for (auto det =mObs.begin(); det !=mObs.end(); det++)
    {
        sprintf(temp, "%04d", (int)det->first->mnFrameId);
        KeyFrameId.append(temp);
    }
    sprintf(object, "%d", (int)mObs.begin()->second);
    InitDetectBox= QuadricLandmarkFolder + "init_"+ KeyFrameId + "_LocalId_" + object + ".txt";
    file.open(InitDetectBox.c_str());
    for(auto box=vBoxes.begin();box!=vBoxes.end();box++){
        for (size_t i = 0; i < 4; i++)
        {
            file<<(*box)[i]<<" ";
        }
        file << endl;
    }
}

void QuadricLandmark::QuadricInit()

{
    //如果观测帧过少，或者quadric已经初始化，s则返回
    if (mObservations.size() < 1)
    {
        std::cout <<" 观测帧数过少 "<< std::endl;
        return;
    }
    /*
    if (mbIsInitialized == true)  
    {
       std::cout<<"quadric已初始化"<<std::endl;
       return;
    }*/

    mnId = GetIncrementedIndex();

    umask(0);
    QuadricLandmarkFolder = "../seq00_1frames_50/SaveQuadricLandmark/" +to_string(mnId)+"/";
    mkdir(QuadricLandmarkFolder.c_str(), 0777); 
   
    for (auto &det : mObservations)
    {  
         mObs.push_back(det);
    }
    stable_sort(mObs.begin(), mObs.end(), compare_map); 

    //begin init
    std::cout << "begin quadriclandmark init..." << endl;
    //提取bbox
    std::vector<Eigen::Vector4d> vBoxes;
    for (auto det = mObs.begin(); det != mObs.end(); det++)
    {
        Eigen::Vector4d box;
        int idx = det->second;
        cv::Rect bbox_2d= det->first->mvpLocalObjects[idx]->bbox_2d;
        box = det->first->mvpLocalObjects[idx]->RecttoVector4d(bbox_2d);
        vBoxes.push_back(box);
    }

    //SaveInitDetectBox(vBoxes);

    //提取kalib 计算projection matrix K*R|t 3*4
    std::vector<Eigen::Matrix<double, 3, 4>,
                Eigen::aligned_allocator<Eigen::Matrix<double, 3, 4>>>
        vProjMats;

    Eigen::Matrix3d kalib; //随意哪一帧的K均可
    kalib = Converter::toMatrix3d(mObs.begin()->first->mK);
    SaveTrajectory();
    for (auto det = mObs.begin(); det != mObs.end(); det++)
    {
        Eigen::Matrix<double, 3, 4> temp;
        cout << "Init_Twc" << det->first->GetPoseInverse() << " " << det->first->mnFrameId << endl;
        temp = kalib * Converter::toProjMat(det->first->GetPose());
        cout << "Init_projection " << det->first->mnFrameId << " " << temp << endl;
        vProjMats.push_back(temp);
    }
    assert(vProjMats.size() == vBoxes.size());

    //boxes to Lines
    std::vector<Eigen::Vector3d> vLines;
    for (auto box : vBoxes)
    {
        vLines.push_back(Eigen::Vector3d(1, 0, -box(0)));
        vLines.push_back(Eigen::Vector3d(0, 1, -box(1)));
        vLines.push_back(Eigen::Vector3d(1, 0, -box(2)));
        vLines.push_back(Eigen::Vector3d(0, 1, -box(3)));
    }
    assert(vLines.size() == 4 * vProjMats.size());

    //lines to Planes
    std::vector<Eigen::Vector4d> vPlanes;
    for (std::size_t i = 0; i < vLines.size(); i++)
    {
        vPlanes.push_back(vProjMats[i / 4].transpose() * vLines[i]);
    }

    assert(vPlanes.size() == 4 * vProjMats.size());
    SavePlanes(vPlanes);
    //SaveProjection(vProjMats);
    SaveMapPoints();


    //计算约束方程系数
    std::vector<Eigen::Matrix<double, 1, 10>> vPlanes_parameter;
    
    for (auto pPlane = vPlanes.begin(); pPlane != vPlanes.end(); pPlane++)
    {
        Eigen::Matrix<double, 10, 1> temp;

        temp << pow((*pPlane)(0, 0), 2), 2 * (*pPlane)(0, 0) * (*pPlane)(1, 0),
            2 * (*pPlane)(0, 0) * (*pPlane)(2, 0), 2 * (*pPlane)(0, 0) * (*pPlane)(3, 0),
            pow((*pPlane)(1, 0), 2), 2 * (*pPlane)(1, 0) * (*pPlane)(2, 0),
            2 * (*pPlane)(1, 0) * (*pPlane)(3, 0), pow((*pPlane)(2, 0), 2),
            2 * (*pPlane)(2, 0) * (*pPlane)(3, 0), pow((*pPlane)(3, 0), 2);
        vPlanes_parameter.push_back(temp);
    }
    assert(vPlanes_parameter.size() == vPlanes.size());
    //构建SVD分解求解未知数q
    Eigen::MatrixXd svd_A(vPlanes_parameter.size(), 10);
    for (size_t i = 0; i < vPlanes_parameter.size(); i++)
    {
        //map类相比copy是复用原数据的存储，param1：指向矩阵/向量的指针，param2：尺寸
        svd_A.row(i) = Eigen::VectorXd::Map(&vPlanes_parameter[i][0],
                                            vPlanes_parameter[i].size());
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(svd_A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::FullPivLU<Eigen::MatrixXd> lu_comp(svd_A);
    cout << "rows cols" << svd_A.rows() << " " << svd_A.cols() << endl;
    cout << "rank" << lu_comp.rank() << endl;
    cout << "single value is " << endl
         << svd.singularValues() << endl;
    //note: make sure size of q_solution is correct!!
    Eigen::Matrix<double, 10, 1> q_solution = svd.matrixV().col(9);
    Eigen::FullPivLU<Eigen::MatrixXd> lu(svd.matrixV());
    cout << "svd rows cols" << svd.matrixV().rows() << " " << svd.matrixV().cols() << endl;
    cout << " svd rank" << lu.rank() << endl;

    Eigen::FullPivLU<Eigen::MatrixXd> q_lu(q_solution);
    cout << "q_sol rows cols" << q_solution.rows() << "  " << q_solution.cols() << endl;
    cout << "q_sol rank" << q_lu.rank() << endl;
    std::cout << "q_solution" << q_solution << std::endl;
    if (mMeasQuality > QUALITY_THRESHOLD)
    {

        mQuadricMeas = g2o::Quadric();
        mQuadricMeas.fromVector10d(q_solution);
     
        cout << "pose" << mQuadricMeas.pose << "scale" << mQuadricMeas.scale << endl;
        SaveQuadric();
        //mQuadricVertex = new g2o::VertexQuadric();//放入构造函数
        ///mpQuadricVertex->setEstimate(mQuadricMeas); //not optimized commited 
        KeyFrame *refframe = this->GetReferenceKeyFrame();
        mQuadricLocalMeas = mQuadricMeas.transform_to(Converter::toSE3Quat(refframe->GetPoseInverse())); //input:Twc
        mbIsInitialized = true;
    }
}

void QuadricLandmark::QuadricProjectToCurrent(KeyFrame *mCurrentKF)
{
    Eigen::Matrix3d kalib;
    kalib = Converter::toMatrix3d(mCurrentKF->mK);
    Eigen::Matrix<double,3,4>pro;
    pro =kalib * Converter::toProjMat(mCurrentKF->GetPose());

    cout << "Project Quadric to Frame ID: " << mCurrentKF->mnFrameId <<" projection: "<<pro<<endl;
    conic_bbox = mQuadricMeas.projectOntoImageRect(Converter::toSE3Quat(mCurrentKF->GetPose()), kalib);
    cv::Rect ConicBbox = cv::Rect(conic_bbox(0, 0), conic_bbox(1, 0), conic_bbox(2, 0) - conic_bbox(0, 0), conic_bbox(3, 0) - conic_bbox(1, 0));
    cout << "current_conic_bbox" << conic_bbox(0, 0) << " " << conic_bbox(1, 0) << " " << conic_bbox(2, 0) << " " << conic_bbox(3, 0) << endl;
    for (QuadricLandmark *mPO : mCurrentKF->mvpLocalObjects)
    {

        cout << " " << mPO->bbox_2d << " " << mPO->mnLocalId << endl;
        cout << "overlap" << bboxOverlapratio(ConicBbox, mPO->bbox_2d) << endl;
    }

    for (auto det = mObs.begin(); det != mObs.end(); det++)
    {
        Eigen::Vector4d box;
        int idx = det->second;
        cv::Rect bbox_2d = det->first->mvpLocalObjects[idx]->bbox_2d;
        box = det->first->mvpLocalObjects[idx]->RecttoVector4d(bbox_2d);
        cout << "Generate Quadric Frame ID: " << det->first->mnFrameId << " " << idx << " detect_box: " << bbox_2d << box << endl;
    }
}

void QuadricLandmark::QuadricProject()
{
    string AssociateDetectFolder = QuadricLandmarkFolder + "DetectBoxAssocaite/";
    string AssociateProjectionFolder = QuadricLandmarkFolder + "ProjectionAssociate/";
    mkdir(AssociateDetectFolder.c_str(),0777);
    mkdir(AssociateProjectionFolder.c_str(),0777);

    Eigen::Matrix3d kalib;
    kalib = Converter::toMatrix3d(mObservations.begin()->first->mK);
    std::vector<Eigen::Matrix<double, 3, 4>,
                Eigen::aligned_allocator<Eigen::Matrix<double, 3, 4>>>
        vProjMats;

    std::vector<Eigen::Vector4d> vDetectBox;

    for (auto &det : mObservations)
    {  
         allObs.push_back(det);  //allObs will be duplicate,because it's the attribute of the quadric,must after sort then use unique 
    }
    stable_sort(allObs.begin(), allObs.end(), compare_map); 
    vector<pair<KeyFrame*, int>>::iterator iter = unique(allObs.begin(),allObs.end());
    allObs.erase(iter,allObs.end());


    char dest[256];   // for change fileName 
    strcpy(dest,"");
    for (auto det =  allObs.begin(); det != allObs.end(); det++){
        cout << "All Frame ID: " << det->first->mnFrameId << endl;
        char suffix[256];
        sprintf(suffix, "%04d", (int)det->first->mnFrameId);
        strcat(dest,suffix);
        cout <<"dest"<<dest << endl;

        Eigen::Matrix<double,3,4> proj;
        proj = kalib*Converter::toProjMat(det->first->GetPose());
        vProjMats.push_back(proj);

        conic_bbox = mQuadricMeas.projectOntoImageRect(Converter::toSE3Quat(det->first->GetPose()), kalib);
        cout << "conic_bbox" << conic_bbox(0, 0) << " " << conic_bbox(1, 0) << " " << conic_bbox(2, 0) << " " << conic_bbox(3, 0) << endl;

        Eigen::Vector4d box;
        int idx = det->second;
        cv::Rect bbox_2d = det->first->mvpLocalObjects[idx]->bbox_2d;
        box = det->first->mvpLocalObjects[idx]->RecttoVector4d(bbox_2d);
        vDetectBox.push_back(box);
        cout << "box" << box(0, 0) << " " << box(1, 0) << " " << box(2, 0) << " " << box(3, 0) << endl;
    }

    string temp = dest;
    string projectName = AssociateProjectionFolder + "Projection_"+temp+".txt";
    string detectName = AssociateDetectFolder + "Detect_"+temp+".txt";
    ofstream projectionFile,detectFile;

    projectionFile.open(projectName);
    detectFile.open(detectName);
    for (auto p = vProjMats.begin(); p != vProjMats.end(); p++)
    {
        for (size_t i = 0; i < 3; i++)
            for (size_t j = 0; j < 4; j++)
              projectionFile <<(*p)(i,j)<<" ";
        projectionFile << endl;
    }

    for(auto box = vDetectBox.begin(); box !=vDetectBox.end(); box++)
    {
        for (size_t i = 0; i < 4; i++)
        {
            detectFile<<(*box)[i]<<" ";
        }
        detectFile << endl;
    }

    projectionFile.close();
    detectFile.close();

}

} // namespace ORB_SLAM2
