# 新增接口
+ 接口描述
头文件：include/darknet.h
库文件：libdarknet.a、libdarknet.so
```c
det_box *yolo_detect(char *cfgfile,char *weightfile,image im,float thresh)
```
+ 输入值
```
char *cfgfile,
char *weightfile,
image im,
float thresh
```
+ 返回数据格式
```c
//返回det_box数组，单位数据结构如下：
typedef struct{
    float classs_id, x_min, y_min, w,h,p；
} det_box;
```

### change log 运行前务必更改路径
+ darknet.h增加yolo_detect函数描述，写入src/demo.c中
+ cfg/coco.data中names必须更改为data/coco.names文件所在的绝对路径 
+ src/demo.c中yolo_detect函数默认config如下，将datacfg更改为cfg/coco.data文件所在的绝对路径
```c
    //configure
    char *datacfg="~/quadric_slam_temp/Thirdparty/Darknet/cfg/coco.data";//绝对路径
    float hier_thresh=0.5;
    char *name_list = option_find_str(options, "names", "data/names.list");
```
### usage
1. CMakeList链接静态库
2. include头文件
```c
#include "image_opencv.h"   //mat2image
extern "C" {
#include "darknet.h"
}

###add for quadric_slam_temp CMakeLists.txt

#CMakeList.txt
cmake_minimum_required( VERSION 2.8 )
project(testAPI)

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# opencv 
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

#set path
include_directories( "~/quadric_slam_temp/Thirdparty/Darknet/include")
include_directories( "~/quadric_slam_temp/Thirdparty/Darknet/src") #for image.h
link_directories("~/quadric_slam_temp/Thirdparty/Darknet")

#add_library 设定image_opencv.cpp的绝对路径
add_library(image_opencv SHARED ~/quadric_slam_temp/Thirdparty/src/image_opencv.cpp)
target_link_libraries( ~/quadric_slam_temp/Thirdparty/Darknet/libdarknet.so )

## YOLO
+ 官网：https://pjreddie.com/darknet/yolo/
+ github：https://github.com/pjreddie/darknet



