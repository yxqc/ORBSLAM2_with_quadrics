#include "Detecting.h"
#include <unistd.h>
#include <mutex>
#include <vector>
#include <mcheck.h>
#include <eigen3/Eigen/Dense>
/*extern "C"{
#include<stdlib.h>
}*/
using namespace std;

namespace ORB_SLAM2
{

std::mutex mut;
std::queue<Frame *> data_queue;
static int nums = 0;
std::condition_variable data_cond;
Detecting::Detecting(const int &nImages)
{
  total_frames = nImages;
}

void Detecting::Filter(det_box *dets, int online_num, int img_width, int img_height, std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &boxes)
{
  int close_bound_margin = 10;

  std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>> temp_boxes;

  for (int i = 0; i < online_num; i++)
  {
    Vector6d temp_box;
    double x_min = (double)dets[i].x_min;
    double y_min = (double)dets[i].y_min;
    double class_id = (double)(dets[i].class_id);
    double confidence = (double)(dets[i].p);
    double width = (double)(dets[i].w);
    double height = (double)(dets[i].h);
    if (width + height < 40)
      continue;
    if ((x_min < close_bound_margin) || (x_min + width > img_width - close_bound_margin) || (y_min < close_bound_margin) || (y_min + height > img_height - close_bound_margin))
      continue;
    temp_box << x_min, y_min, width, height, class_id, confidence;
    temp_boxes.push_back(temp_box);
  }

  for (int i = 0; i < temp_boxes.size(); i++)
  {
    if (temp_boxes[i][5] > 0)
    {
      cv::Rect box1 = cv::Rect((int)temp_boxes[i][0], (int)temp_boxes[i][1], (int)temp_boxes[i][2], (int)temp_boxes[i][3]);
      for (int j = i + 1; j < temp_boxes.size(); j++)
      {
        if (temp_boxes[j][5] > 0)
        {
          cv::Rect box2 = cv::Rect((int)temp_boxes[j][0], (int)temp_boxes[j][1], (int)temp_boxes[j][2], (int)temp_boxes[j][3]);
          float iou_ratio = bboxOverlapratio(box1, box2);
          if (iou_ratio > 0.5)
          {
            if (temp_boxes[i][5] > temp_boxes[j][5])

              temp_boxes[j][5] = -1;
            else
              temp_boxes[i][5] = -1;
          }
          else
            continue;
        }
        else
          continue;
      }
    }
    else
      continue;
  }
  for (int i = 0; i < temp_boxes.size(); i++)
  {
    if (temp_boxes[i][5] < 0)
      continue;
    Vector6d box;
    box << temp_boxes[i][0], temp_boxes[i][1], temp_boxes[i][2], temp_boxes[i][3], temp_boxes[i][4], temp_boxes[i][5];
    boxes.push_back(box);
  }
}

void Detecting::Run()
{

  network *net = get_load_network("/home/autolab/ORBSLAM2_with_quadric/Thirdparty/darknet/cfg/yolov3.cfg", "/home/autolab/ORBSLAM2_with_quadric/Thirdparty/darknet/yolov3.weights");

  while (1)
  {
    if (CheckNewFrame())
    {
      DetectBbox(net);
    }
    if (nums == total_frames)
      break;
  }
   
}



void Detecting::DetectBbox(network *net)
{
  for (list<Frame *>::iterator sit = NewFrames.begin(); sit != NewFrames.end();)
  {
    Frame *cur = *sit;
    cv::Mat img = cur->mLeftImg;
    image im = mat_to_image(img);
    det_box *dets = yolo_detect(im, net, 0.6, &online_num);

    std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>> boxes;
    Filter(dets, online_num, im.w, im.h, boxes);
    cur->mDetectingBoxes = boxes;
    cur->mDetectNums = boxes.size();
    std::lock_guard<std::mutex> lk(mut);
    data_queue.push(cur);
    data_cond.notify_one();

    NewFrames.erase(sit++);
    nums++;
    usleep(100000);
  }
}

void Detecting::SetTracker(Tracking *pTracker)
{
  mpTracker = pTracker;
}

void Detecting::InsertFrame(Frame *cur_frame)
{
  NewFrames.push_back(cur_frame);
}

bool Detecting::CheckNewFrame()
{
  return (!NewFrames.empty());
}

} //namespace ORB_SLAM2
