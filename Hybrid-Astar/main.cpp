/*
 * @Author: your name
 * @Date: 2021-12-09 11:13:17
 * @LastEditTime: 2021-12-09 18:01:25
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /hybrid_astar_planner/main.cpp
 */
#include <glog/logging.h>

#include <chrono>
#include <ctime>
#include <string>
#include <vector>

#include "hybrid_astar.h"
#include "param_reader.h"

int foo = 30;             //定义尺寸
int fooResize = 2 * foo;  //最终显示放大镜的正方形区域的边长，用作下面if的判断
using namespace cv;
Mat org;        //原图
Mat img;        //输出图片
Mat imgROI;     //感兴趣区域
Mat roiResize;  //感兴趣区域ROI的两倍
Mat tranPart;
void on_mouse(int event, int x, int y, int flags, void* ustc) {
  std::cout << "点击位置" << x << ", " << y << std::endl;
  //形参x,y是鼠标点击的位置
  Mat img_temp = img.clone();  //临时变量，存放原图
  //if的作用，x，y是鼠标坐标，限定不能太靠近左上角x > fooResize和 y>fooResize，目的是能让放大镜显示出来。
  //同时限定不能太靠近右下角x<img_temp.cols - fooResize和 y < img_temp.rows - fooResize
  if (x > fooResize && x < img_temp.cols - fooResize && y > fooResize && y < img_temp.rows - fooResize) {
    //感兴趣区域范围
    imgROI = img_temp(Rect(x - foo, y - foo, 2 * foo, 2 * foo));  //以(x,y)为中心，左右为foo做正方形
    //tranPart是最后显示放大的区域，这里
    tranPart = img_temp(Range(y - 2 * foo, y + 2 * foo), Range(x - 2 * foo, x + 2 * foo));  //Range表示纵、横坐标的范围
    //感兴趣区域放大两倍，需要和tranPart的尺寸搭配。可调，但是注意不能超过图片本身范围。
    resize(imgROI, roiResize, Size(2 * imgROI.cols, 2 * imgROI.rows));  //输出到roiResize这个Mat对象中
    roiResize.copyTo(tranPart);                                         //复制到tranPart这个Mat对象中
                                                                        //标注鼠标的位置
    circle(img_temp, Point(x, y), 10, Scalar(0, 0, 255), 1, 8, 0);      //画一个简单的圆
    imshow("img", img_temp);                                            //显示窗口
  }
}

int main(int argc, char** argv) {
  if (argc != 2) return -1;
  ParamReader reader;
  if (reader.loadParam(argv[1])) {
    std::string map_info_path, car_info_path;
    std::vector<double> start_and_goal;

    if (!reader.getValue("map_info_path", map_info_path)) {
      LOG(FATAL) << "map_info_path must be set!";
      return -1;
    }
    if (!reader.getValue("car_info_path", car_info_path)) {
      LOG(FATAL) << "car_info_path must be set!";
      return -1;
    }
    if (!reader.getValueVec("start_and_goal", start_and_goal)) {
      LOG(FATAL) << "start_and_goal must be set!";
      return -1;
    }
    CHECK(start_and_goal.size() == 6) << "start_and_goal must has 6 components!";
    double time_tolerance = 30, step_size = 1.0, turning_penalty_factor = 5.0;
    double goal_tolerance_dist = 0.5, goal_tolerance_theta = 0.1;
    bool enable_backward = false;
    reader.getValue("enable_backward", enable_backward);
    reader.getValue("time_tolerance", time_tolerance);
    reader.getValue("step_size", step_size);
    reader.getValue("turning_penalty_factor", turning_penalty_factor);
    reader.getValue("goal_tolerance_dist", goal_tolerance_dist);
    reader.getValue("goal_tolerance_theta", goal_tolerance_theta);
    GridMap* map = new GridMap();
    Car* car = new Car();
    map->load(map_info_path);
    car->load(car_info_path);

    HybridAStar* planner = new HybridAStar();
    planner->initialize(car, map, enable_backward, time_tolerance, step_size,
                        turning_penalty_factor, goal_tolerance_dist, goal_tolerance_theta);
    Pose2D start(start_and_goal[0], start_and_goal[1], start_and_goal[2]);
    Pose2D goal(start_and_goal[3], start_and_goal[4], start_and_goal[5]);

    std::vector<Pose2D> solution = {};
    auto ts = std::chrono::system_clock::now();
    if (planner->plan(start, goal, solution)) {
      LOG(ERROR) << "hybrid astar path size: " << solution.size();
      auto te = std::chrono::system_clock::now();
      std::chrono::duration<double> dt = te - ts;
      LOG(INFO) << "succeed to find the path! cost time " << dt.count() << "s.";
      cv::Mat img = map->drawResult(*car, solution);
      cv::namedWindow("img", cv::WINDOW_NORMAL);
      cv::imwrite("./plan_res.jpg", img);
      cv::imshow("img", img);
      cv::setMouseCallback("img", on_mouse, 0);
      LOG(INFO) << "test";
      cv::waitKey(0);
    } else {
      auto te = std::chrono::system_clock::now();
      std::chrono::duration<double> dt = te - ts;
      LOG(INFO) << "failed to find the path! cost time " << dt.count() << "s.";
      std::vector<Pose2D> sg = {start, goal};
      cv::Mat wtf = map->drawResult(*car, sg);
      cv::imshow("wtf", wtf);
      cv::waitKey(0);
    }

    delete car;
    delete map;
    delete planner;
  }
  return 0;
}