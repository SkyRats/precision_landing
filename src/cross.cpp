#include <iostream>
using namespace std;
#include <opencv2/opencv.hpp>
using namespace cv;
#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "precision_landing/H_info.h"
#include "std_msgs/Bool.h"

#define ANGLE_THRESH 0.01
#define PI 3.14159265

#define vp vector<Point>
#define vpf vector<Point2f>

// Set true for debugging purposes, showing internals of algorithm
#define DEBUG false

// Sorts points based on y coordinate
struct comparison
{
    bool operator()(Point2f pt1, Point2f pt2) { return (pt1.y > pt2.y); }
} comparing;

class HDetector
{
private:
    vpf edge_pts = {Point2f(0, 0), Point2f(0, 0), Point2f(0, 0), Point2f(0, 0)};
    Rect bounds;
    float area_ratio;
    void order_points();
    Mat four_points_transform(Mat image);
    float angle(Point2f v1, Point2f v2, Point2f relative = Point2f(0, 0));
    bool angle_check(vpf pts);
    ros::NodeHandle n;
    void image_cb(const sensor_msgs::ImageConstPtr &img);
    void runnin_state_cb(std_msgs::Bool data);
    ros::Publisher h_pub;
    ros::Publisher img_debug_pub;
    ros::Subscriber h_sub_image;
    ros::Subscriber h_sub_runner;
    bool runnin;
    // int lower[4] = {30, 80, 0};
    // int upper[4] = {60, 255, 255};

public:
    Mat warped;
    HDetector();
    bool detect(Mat frame);
    float getArea();
    void setArea(vpf contour, Mat frame);
    int getCenter_X();
    int getCenter_Y();
    // int *getLower();
    // int *getUpper();
};

HDetector::HDetector()
{
    this->h_pub = this->n.advertise<precision_landing::H_info>("/precision_landing/detection", 0);
    this->img_debug_pub = this->n.advertise<sensor_msgs::Image>("/precision_landing/debug/image_raw", 0);
    this->h_sub_image = this->n.subscribe("/uav1/bluefox_optflow/image_raw", 5, &HDetector::image_cb, this);
    this->h_sub_runner = this->n.subscribe("/precision_landing/set_running_state", 10, &HDetector::runnin_state_cb, this);
}

void HDetector::runnin_state_cb(std_msgs::Bool data)
{
    this->runnin = data.data;
}

void HDetector::image_cb(const sensor_msgs::ImageConstPtr &img)
{
    if (this->runnin)
    {

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        precision_landing::H_info msg;
        if (this->detect(cv_ptr->image))
        {
            msg.detected = true;
            msg.center_x = this->getCenter_X();
            msg.center_y = this->getCenter_Y();
            msg.area_ratio = this->getArea();
            cout << "Centro em x: " << msg.center_x << endl;
            cout << "Centro em y: " << msg.center_y << endl;
            cout << "Area ratio " << msg.area_ratio << endl;

            this->h_pub.publish(msg);
        }
    }
}

float HDetector::getArea()
{
    return this->area_ratio;
}

void HDetector::setArea(vpf contour, Mat frame)
{
    this->area_ratio = contourArea(contour, false) / (frame.cols * frame.rows);
}

int HDetector::getCenter_X()
{
    return (this->bounds.x + this->bounds.width / 2);
}

int HDetector::getCenter_Y()
{
    return (this->bounds.y + this->bounds.height / 2);
}

// Takes an image 'frame' and detects whether it contains the letter H
bool HDetector::detect(Mat frame)
{
    bool detected = false;

    // if(DEBUG) Mat frame2 = frame;

    cvtColor(frame, frame, COLOR_BGR2HSV);
    // Blur and threshold remove noise from image
    medianBlur(frame, frame, 11);

    // inRange(frame, Scalar(this->getLower()), Scalar(this->getUpper()), frame);
    inRange(frame, Scalar(30, 80, 0), Scalar(60, 255, 255), frame);
    Mat hsv_channels[3];
    split(frame, hsv_channels);
    Mat gray_frame = hsv_channels[2];

    vector<vp> contours;
    vector<vp> hierarchy;

    findContours(frame, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    // int colors = [
    //     [ 0, 0, 255 ],
    //     [ 0, 255, 0 ],
    //     [ 255, 0, 0 ],
    //     [ 0, 255, 255 ],
    //     [ 255, 0, 255 ],
    //     [ 255, 255, 0 ],
    // ];
    int max_w = 0,
        max_h = 0, max_x = 0, max_y = 0;

    for (vp cnt : contours)
    {
        Rect rect = boundingRect(cnt);
        if (rect.width * rect.height > max_w * max_h)
        {
            max_w = rect.width;
            max_h = rect.height;
            max_x = rect.x;
            max_y = rect.y;
        }
        rectangle(frame, Point(rect.x, rect.y), Point(rect.x + rect.width, rect.y + rect.height), Scalar(0, 255, 0), 1);
    }

    rectangle(frame, Point(max_x, max_y),
              Point(max_x + max_w, max_y + max_h), Scalar(0, 0, 255), 2);
    circle(frame, Point((max_x + max_w / 2), (max_y + max_h / 2)),
           1, (0, 0, 255), 3);
    imshow("frame", frame);
    waitKey(3); // Wait for a keystroke in the window

    if (DEBUG)
    {
        imshow("Processed", frame);
        waitKey(3); // Wait for a keystroke in the window
    }

    if (false)
    {
        detected = true;
    }

    return detected;
}

// For testing
int main(int argc, char **arvg)
{
    ROS_INFO("Running cross detection node!");
    ros::init(argc, arvg, "cross_node");
    HDetector *detector = new HDetector();
    ros::spin();
}
