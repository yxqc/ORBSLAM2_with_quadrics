//TODO if necessary
#include "quadric_slam/QuadricLandmark.h"
#include "quadric_slam/Convhull_3d.h"
#include "quadric_slam/Dbscan.h"
#include "Converter.h"
#include <fstream>
#include <Eigen/Dense>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#define INIT_IOU_THRESHOLD 0.5
#define MINIMIUM_POINTS 10
#define EPSILON (0.1*0.1)
#define CONVHULL_3D_ENABLE

extern "C"{
  #include "quadric_slam/Convhull_3d.h"
}

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
QuadricLandmark::QuadricLandmark(Map* pMap,bool whether_update_index) : mbBad(false), mpMap(pMap)
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
    mbIsInitializedOneObs = false;

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
}
QuadricLandmark::~QuadricLandmark()
{

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


void QuadricLandmark::CandidatePlanes(std::vector<Eigen::Vector4d>& vplanes,std::vector<Eigen::Vector4d>& convhull3d_normals,std::vector<Eigen::Vector4d>& candidate_planes)
{
  for (int i = 4; i < 8;i++){
    candidate_planes.push_back(vplanes[i]);
  }
  for (int  i = 0; i < convhull3d_normals.size(); i++){
    candidate_planes.push_back(convhull3d_normals[i]);
  }
  for(int i = 0 ;i<candidate_planes.size();i++){

    cout<<"candidate_planes"<<candidate_planes[i]<<endl;
  }
}

float QuadricLandmark::CheckInitialSucess()
{
    Eigen::Matrix3d kalib;
    kalib = Converter::toMatrix3d(mObservations.begin()->first->mK);
    std::vector<Eigen::Matrix<double, 3, 4>,
                Eigen::aligned_allocator<Eigen::Matrix<double, 3, 4>>>
        vProjMats;    

    float aver_2d_iou = 0.0;
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
        cv::Rect rect_conic_bbox = cv::Rect (conic_bbox(0,0),conic_bbox(1,0),conic_bbox(2,0)-conic_bbox(0,0),conic_bbox(3,0)-conic_bbox(1,0));
        cout<< " rect_conic_bbox "<< rect_conic_bbox << endl;
 
        Eigen::Vector4d box;
        int idx = det->second;
        cv::Rect bbox_2d = det->first->mvpLocalObjects[idx]->bbox_2d;

        float iou =  bboxOverlapratio(rect_conic_bbox,bbox_2d);
        cout<<"2d_IoU" << iou<<endl;
        aver_2d_iou += iou;
    }

    return aver_2d_iou/allObs.size();
 

} 
   
void QuadricLandmark::GetVertices(std::vector<ch_vertex>dbscan_inliners,ch_vertex** out_vertices)
{
  
  int inliner_numbers = dbscan_inliners.size();
  (*out_vertices) = new ch_vertex[inliner_numbers];
  for (int i = 0 ;i<inliner_numbers;i++){
    (*out_vertices)[i].v[0] = dbscan_inliners[i].x;
    (*out_vertices)[i].v[1] = dbscan_inliners[i].y;
    (*out_vertices)[i].v[2] = dbscan_inliners[i].z;

   }
   
}
void QuadricLandmark::ClusterConvhull(std::vector<Eigen::Vector4d>& convhull3d_normals)
{
    std::vector<MapPoint *> mapPoints_own =mpAssocaitedLandmark->GetUniqueMapPoints();
    std::vector<DBSCAN_Point> dbscan_points;
    for (auto mapPoint : mapPoints_own)
    {
        cout << "3d=" << mapPoint->GetWorldPos() << endl;
        DBSCAN_Point dbscan_point;
        dbscan_point.x = mapPoint->GetWorldPos().at<float>(0,0);
        dbscan_point.y = mapPoint->GetWorldPos().at<float>(1,0);
        dbscan_point.z = mapPoint->GetWorldPos().at<float>(2,0);
        dbscan_point.clusterID = UNCLASSIFIED;
        dbscan_points.push_back(dbscan_point);

    }
    DBSCAN ds(MINIMIUM_POINTS,EPSILON,dbscan_points);
    ds.run();
    vector<ch_vertex>dbscan_inliners;
    for (int i = 0;i<ds.getTotalPointSize();i++){
       cout<<"dbscan"<<ds.m_points[i].x<<" "<<ds.m_points[i].y<<" "<<ds.m_points[i].z<<" "<<ds.m_points[i].clusterID<<endl;
       if(ds.m_points[i].clusterID == 1)
        {
           ch_vertex inliner;
           inliner.x = ds.m_points[i].x;
           inliner.y = ds.m_points[i].y;
           inliner.z = ds.m_points[i].z;
           dbscan_inliners.push_back(inliner);
        }
    }


    ch_vertex* vertices = NULL;
    GetVertices(dbscan_inliners,&vertices); 
    int* out_faces = NULL;
    int nOut_faces;
    BuildConvhull3d(vertices,dbscan_inliners.size(),&out_faces,&nOut_faces); 
    convhull3d_normals = Convhull3dNormals(vertices,dbscan_inliners.size(),out_faces,nOut_faces);
    delete (vertices);
    delete (out_faces);
}
     

void QuadricLandmark::QuadricInitOneObs()
{
    
    umask(0);
    QuadricLandmarkFolderOneObs = "../kitti_raw/seq09_1frames_30/SaveQuadricLandmark/" +to_string(mnId)+"/";
    mkdir(QuadricLandmarkFolderOneObs.c_str(), 0777); 
    std::vector<Eigen::Matrix<double,4,1>,Eigen::aligned_allocator<Eigen::Matrix<double,4,1>>> vBoxes;
    Eigen::Matrix<double,4,1> box;
    
    int idx = mObservations.begin()->second;
    cv::Rect bbox_2d= mObservations.begin()->first->mvpLocalObjects[idx]->bbox_2d;
    box = mObservations.begin()->first->mvpLocalObjects[idx]->RecttoVector4d(bbox_2d);
    vBoxes.push_back(box);
    
    std::vector<Eigen::Matrix<double, 3, 4>,Eigen::aligned_allocator<Eigen::Matrix<double, 3, 4>>>vProjMats;

    Eigen::Matrix3d kalib;
    kalib = Converter::toMatrix3d(mObservations.begin()->first->mK);
    
    for (auto det = mObservations.begin(); det != mObservations.end(); det++)
    {
        Eigen::Matrix<double, 3, 4> temp;
        cout << "OneObs_Init_Twc" << det->first->GetPoseInverse() << " " << det->first->mnFrameId << endl;
        temp = kalib * Converter::toProjMat(det->first->GetPose());
        cout << "OneObs_Init_projection " << det->first->mnFrameId << " " << temp << endl;
        vProjMats.push_back(temp);
    }
    assert(vProjMats.size() == vBoxes.size());

   
    std::vector<Eigen::Vector3d> vLines;
    
    for (auto box:vBoxes)
    {
        vLines.push_back(Eigen::Vector3d(1, 0, -box(0)));
        vLines.push_back(Eigen::Vector3d(0, 1, -box(1)));
        vLines.push_back(Eigen::Vector3d(1, 0, -box(2)));
        vLines.push_back(Eigen::Vector3d(0, 1, -box(3)));
    }
    assert(vLines.size() == 4 * vProjMats.size());
    
    //lines to Planes
    std::vector<Eigen::Matrix<double,4,1>,Eigen::aligned_allocator<Eigen::Matrix<double,4,1>>> vPlanes;
    for (std::size_t i = 0; i < vLines.size(); i++)
    {
        vPlanes.push_back(vProjMats[i / 4].transpose() * vLines[i]);
    }
    assert(vPlanes.size() == 4 * vProjMats.size());

    mbIsInitializedOneObs = true;

    SaveOneObsPlaneTwcMap(vPlanes);

}

void QuadricLandmark::SaveOneObsPlaneTwcMap(std::vector<Eigen::Matrix<double,4,1>,Eigen::aligned_allocator<Eigen::Matrix<double,4,1>>>& vPlanes)
{
    ofstream OneObsPlane,OneObsTwc,OneObsMap;
   
    char mnFrameId[256],object[256];
   
    sprintf(mnFrameId, "%04d", (int)mObservations.begin()->first->mnFrameId); 
    sprintf(object, "%d", (int)mObservations.begin()->second);
    
    string plane = QuadricLandmarkFolderOneObs+"Planes_"+mnFrameId+".txt";
    OneObsPlane.open(plane.c_str());
   
    for(auto p=vPlanes.begin();p!=vPlanes.end();p++){
        OneObsPlane<<(*p)[0]<<" "<<(*p)[1]<<" "<<(*p)[2]<<" "<<(*p)[3]<<endl;
    }
    string Twc = QuadricLandmarkFolderOneObs+"CameraWorldPose_"+mnFrameId+".txt";
    OneObsTwc.open(Twc.c_str());

    for (size_t i = 0; i < 4; i++)
      for (size_t j = 0; j < 4; j++)
         OneObsTwc<< mObservations.begin()->first->GetPoseInverse().at<float>(i, j) << " ";
 
    string Map = QuadricLandmarkFolderOneObs+"MapWorldPose_"+mnFrameId+"_LocalId_" +object+".txt";
    OneObsMap.open(Map.c_str());  

    std::vector<MapPoint *> mapPoints_own =mpAssocaitedLandmark->GetUniqueMapPoints();
    for (auto mapPoint : mapPoints_own)
    {
        for (size_t i = 0; i < 3; i++)
            OneObsMap << mapPoint->GetWorldPos().at<float>(i, 0) << " ";

        OneObsMap << endl;
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


void QuadricLandmark::SaveQuadric()
{
  
    char temp[256];
    for (auto det =mObs.begin(); det !=mObs.end(); det++)
    {
        std::string key_frame_id,QuadricParam;
        ofstream file;
        sprintf(temp, "%04d", (int)det->first->mnFrameId);
        cout<<"temp"<<temp<<endl;
        key_frame_id.append(temp);
        cout<<"key_frame_id"<<key_frame_id<<endl;
        QuadricParam = "../kitti_raw/seq09_3frames_30/Offline/Quadric_"+ key_frame_id + ".txt";
        file.open(QuadricParam.c_str(),ios::app);
        
        mQuadricLocalMeas = mQuadricMeas.transform_to(Converter::toSE3Quat(det->first->GetPoseInverse())); //input:Twc

        for (size_t i = 0; i < 3; i++)
          file << mQuadricLocalMeas.scale(i, 0) << " ";
        for (size_t i = 0; i < 3; i++)
          file << mQuadricLocalMeas.pose.translation()(i,0) << " ";
         
        cv::Mat QuadricPose = Converter::toCvMat(mQuadricLocalMeas.pose);
        cv::Mat QuadricR = QuadricPose(cv::Rect(0,0,3,3));
        cout << "QuadricR" << QuadricR << endl;
        vector<float> quater = Converter::toQuaternion(QuadricR);
        for (size_t i = 0; i < quater.size() ; i++)
             file << quater[i] << " ";
        file << endl;

        cout << "Quadric Pose" << QuadricPose << endl;
    }
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

void QuadricLandmark::QuadricInit()
{
    //如果观测帧过少，或者quadric已经初始化，s则返回
    if (mObservations.size() < 3)
    {
        std::cout <<" 观测帧数过少"<< std::endl;
        return;
    }
    umask(0);
    QuadricLandmarkFolder = "../kitti_raw/seq09_3frames_30/SaveQuadricLandmark/" +to_string(mnId)+"/";
    mkdir(QuadricLandmarkFolder.c_str(), 0777); 
   

    for (auto &det : mObservations)
    {  
         mObs.push_back(det);
    }
    stable_sort(mObs.begin(), mObs.end(), compare_map); 

    //begin init
    std::cout << "begin to init quadriclandmark..." << endl;
   
    std::vector<Eigen::Vector4d> vBoxes;
    for (auto det = mObs.begin(); det != mObs.end(); det++)
    {
        Eigen::Vector4d box;
        int idx = det->second;
        cv::Rect bbox_2d= det->first->mvpLocalObjects[idx]->bbox_2d;
        box = det->first->mvpLocalObjects[idx]->RecttoVector4d(bbox_2d);
        vBoxes.push_back(box);
    }


    //提取kalib 计算projection matrix K*R|t 3*4
    std::vector<Eigen::Matrix<double, 3, 4>,
                Eigen::aligned_allocator<Eigen::Matrix<double, 3, 4>>>
        vProjMats;

    SaveTrajectory();
    Eigen::Matrix3d kalib; //随意哪一帧的K均可
    kalib = Converter::toMatrix3d(mObs.begin()->first->mK);
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
    SaveMapPoints();
    std::vector<Vector4d> convhull3d_normals;
  
    //begin init

    std::cout << "begin to cluster and construct convehull_3d..." << std::endl;
    ClusterConvhull(convhull3d_normals);
    vector<Vector4d> candidate_planes;
    CandidatePlanes(vPlanes,convhull3d_normals,candidate_planes);

    //计算约束方程系数

    std::vector<Eigen::Matrix<double, 1, 10>> vPlanes_parameter;
    
    for (auto pPlane = candidate_planes.begin(); pPlane != candidate_planes.end(); pPlane++)
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
   
    mQuadricMeas = g2o::Quadric();
    mQuadricMeas.fromVector10d(q_solution);
     
    cout << "pose" << mQuadricMeas.pose << "scale" << mQuadricMeas.scale << endl;
    SaveQuadric();

   // if(0.7 > INIT_IOU_THRESHOLD){
    if(CheckInitialSucess() > INIT_IOU_THRESHOLD){
      mbIsInitialized = true;
      std:cout<<"initialized quadric translation"<<mQuadricMeas.pose.translation()<<endl;  
      double quadric_cam_dist = std::min(std::max(mQuadricMeas.pose.translation()(2),10.0),30.0);
      mMeasQuality = (60.0-quadric_cam_dist)/40.0;
      std::cout<<"mMeasQuality "<<mMeasQuality<<endl;
    }

      
    
    /*
    if (mMeasQuality > INIT_IOU_THRESHOLD)
    {

        mQuadricMeas = g2o::Quadric();
        mQuadricMeas.fromVector10d(q_solution);
     
        cout << "pose" << mQuadricMeas.pose << "scale" << mQuadricMeas.scale << endl;
        /
        //mQuadricVertex = new g2o::VertexQuadric();//放入构造函数
        ///mpQuadricVertex->setEstimate(mQuadricMeas); //not optimized commited 
        KeyFrame *refframe = this->GetReferenceKeyFrame();
        mQuadricLocalMeas = mQuadricMeas.transform_to(Converter::toSE3Quat(refframe->GetPoseInverse())); //input:Twc
        mbIsInitialized = true;
    }
    */
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
//        cout << "overlap" << bboxOverlapratio(ConicBbox, mPO->bbox_2d) << endl;
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
void QuadricLandmark::QuadricProjectOneObs()
{
    string AssociateDetectFolder = QuadricLandmarkFolderOneObs + "DetectBoxAssocaite/";
    string AssociateProjectionFolder = QuadricLandmarkFolderOneObs + "ProjectionAssociate/";
    mkdir(AssociateDetectFolder.c_str(),0777);
    mkdir(AssociateProjectionFolder.c_str(),0777);
    
    Eigen::Matrix3d kalib;
    kalib = Converter::toMatrix3d(mObservations.begin()->first->mK);
    std::vector<Eigen::Matrix<double, 3, 4>,
                Eigen::aligned_allocator<Eigen::Matrix<double, 3, 4>>>
        vProjMats;


    std::vector<Eigen::Matrix<double,4,1>,
                Eigen::aligned_allocator<Eigen::Matrix<double,4,1 >>> 
        vDetectBox;

    for (auto &det : mObservations)
    {  
         allObsfromOne.push_back(det);  //allObs will be duplicate,because it's the attribute of the quadric,must after sort then use unique 
    }
    stable_sort(allObsfromOne.begin(), allObsfromOne.end(), compare_map); 
    vector<pair<KeyFrame*, int>>::iterator iter = unique(allObsfromOne.begin(),allObsfromOne.end());
    allObsfromOne.erase(iter,allObsfromOne.end());


    char dest[256];   // for change fileName 
    strcpy(dest,"");
    for (auto det =  allObsfromOne.begin(); det != allObsfromOne.end(); det++){
        cout << "OneObs_All Frame ID: " << det->first->mnFrameId << endl;
        char suffix[256];
        sprintf(suffix, "%04d", (int)det->first->mnFrameId);
        strcat(dest,suffix);
        cout <<"OneObs_dest"<<dest << endl;

        Eigen::Matrix<double,3,4> proj;

        proj = kalib*Converter::toProjMat(det->first->GetPose());
        vProjMats.push_back(proj);


        //conic_bbox = mQuadricMeas.projectOntoImageRect(Converter::toSE3Quat(det->first->GetPose()), kalib);
        //cout << "conic_bbox" << conic_bbox(0, 0) << " " << conic_bbox(1, 0) << " " << conic_bbox(2, 0) << " " << conic_bbox(3, 0) << endl;
       
        Eigen::Matrix<double,4,1> box;
        int idx = det->second;
        cv::Rect bbox_2d = det->first->mvpLocalObjects[idx]->bbox_2d;
        box = det->first->mvpLocalObjects[idx]->RecttoVector4d(bbox_2d);
        vDetectBox.push_back(box);
        cout << "OneObs_box" << box(0, 0) << " " << box(1, 0) << " " << box(2, 0) << " " << box(3, 0) << endl;
      
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
