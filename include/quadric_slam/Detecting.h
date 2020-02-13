#ifndef DETECTING_H
#define DETECTING_H

#include "Tracking.h"
#include "Frame.h"
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <mutex>
#include <iostream>
#include <mutex>
#include <vector>
#include <eigen3/Eigen/Dense>
#include<eigen3/Eigen/StdVector>
#include <queue>
#include <condition_variable>
#include "image_opencv.h"

extern "C"
{
#include "darknet.h"
}

using namespace std;
using namespace cv;

namespace ORB_SLAM2
{
class Tracking;
class Frame;
typedef Eigen::Matrix<double,6,1> Vector6d;
extern std::mutex mut;
extern std::queue<Frame*> data_queue;
extern std::condition_variable data_cond;

class Detecting
{
public:
    Detecting(const int &nImage);
    //Detection(const Vector6d )
    std::vector<Vector6d,Eigen::aligned_allocator<Vector6d>>* raw_2d_objs;
    //std::vector<Vector4d,Eigen::aligned_allocator<Vector4d>> Bbox;
    void Run();
    void Filter(det_box* box,int online_num,int w,int h,std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &boxes);
    void SetTracker(Tracking *pTracker);
    void InsertFrame(Frame *pFrame);
    void DetectBbox(network* net);
    bool CheckNewFrame();
    void RequestReset();
   /* inline Eigen::Vector4d toBbox(){
        
    }
    */
  float bboxOverlapratio(const cv::Rect &rect1, const cv::Rect &rect2)

  {

    int overlap_area = (rect1 & rect2).area();

    return (float)overlap_area / ((float)(rect1.area() + rect2.area() - overlap_area));
  }
    
private:
    vector<string> left, right;
    int online_num,total_frames;
    std::list<Frame *> NewFrames;
    void Reset();
    det_box* dets;
protected:
    Tracking *mpTracker;
    bool mbResetRequested;
    std::mutex mMutexReset;
};
} // namespace ORB_SLAM
#endif
