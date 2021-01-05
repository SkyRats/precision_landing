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

class HDetector {
    private:
        vpf edge_pts = { Point2f(0,0), Point2f(0,0), Point2f(0,0), Point2f(0,0) };
        Rect hBoundingRect;
        bool runnin;
        float area_ratio;
        ros::NodeHandle n;
        ros::Publisher h_info_pub;
        ros::Publisher img_debug_pub;
        ros::Subscriber h_image_sub;
        ros::Subscriber running_state_sub;
        void remove_noise_from_image(Mat frame);
        void determine_perspective_points(vpf approx);
        void order_points();
        Mat edge_perspective_transform(Mat image);
        float angle_between_vectors(Point2f v1, Point2f v2, Point2f origin = Point2f(0,0) );
        bool angle_check(vpf pts);
        void image_detection_callback(const sensor_msgs::ImageConstPtr& img);
        void runnin_state_cb(std_msgs::Bool data);
    public:
        Mat warped; 
        bool detect (Mat frame);
        HDetector();
        float getArea();
        void setArea(vpf contour, Mat frame);
        int getCenter_X();
        int getCenter_Y();
};

HDetector::HDetector(){
    this->h_info_pub = this->n.advertise<precision_landing::H_info>("/precision_landing/detection", 0);
    this->img_debug_pub = this->n.advertise<sensor_msgs::Image>("/precision_landing/debug/image_raw", 0);
    this->h_image_sub = this->n.subscribe("/iris_fpv_cam/usb_cam/image_raw", 5, &HDetector::image_detection_callback, this);
    this->running_state_sub = this->n.subscribe("/precision_landing/set_running_state", 10, &HDetector::runnin_state_cb, this);
}

// Takes an image 'frame' and detects whether it contains the letter H
bool HDetector::detect (Mat frame){
    bool detected = false;

    cvtColor(frame, frame, CV_BGR2GRAY);

    remove_noise_from_image(frame)

    vector<vp> contour;
    findContours(frame, contour, RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    for(vp cnt : contour){

        int peri = arcLength(cnt, true);
        vpf approx;
        approxPolyDP(cnt, approx, 0.02*peri, true);
        
        if (approx.size() == 12){
            
            this->hBoundingRect = boundingRect(approx); 
            
            determine_perspective_points(approx);

            Mat perspective = edge_perspective_transform(frame);
            vpf transformed;
            perspectiveTransform(approx, transformed, perspective);

            if (angle_check(approx)){
                detected = true;
                this->setArea(approx, frame);
            }

        }
    }
    return detected;
}

void remove_noise_from_image(Mat frame){
    threshold(frame, frame, 115, 255, 1);
    adaptiveThreshold(frame, frame, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 9, 20.0);
    adaptiveThreshold(frame, frame, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 3, 0.0);
}

void determine_perspective_points(vpf approx){
    float a1 = angle_between_vectors(approx[0] - approx[1], Point2f(0,1));
    float a2 = angle_between_vectors(approx[1] - approx[2], Point2f(0,1));

    /* If the sides of the H are very close to parallel to its hBoundingRect,
        use the bounding rect vertices for warp */
    if( a1 < 0.1 || a2 < 0.1
        || abs(a1 - PI) < 0.1 || abs(a1 - PI) < 0.1 ){

        this->edge_pts = {
            Point2f (hBoundingRect.x, hBoundingRect.y) ,
            Point2f (hBoundingRect.x + hBoundingRect.width, hBoundingRect.y) , 
            Point2f (hBoundingRect.x, hBoundingRect.y + hBoundingRect.height) ,
            Point2f (hBoundingRect.x + hBoundingRect.width, hBoundingRect.y + hBoundingRect.height)
        };
    
    /* If they are far, use the vertices that are closest to the bounding
        rect sides */
    }else{

        for(Point2f vertex : approx){

            // Close on left side of bound
            if( abs(vertex.x - hBoundingRect.x) <= 1) this->edge_pts[0] = vertex;
            // On right side
            else if( abs(vertex.x - (hBoundingRect.x + hBoundingRect.width) ) <= 1) this->edge_pts[1] = vertex;

            // On top
            else if( abs(vertex.y - hBoundingRect.y) <= 1) this->edge_pts[2] = vertex;
            //On bottom
            else if( abs(vertex.y - (hBoundingRect.y + hBoundingRect.height) ) <= 1) this->edge_pts[3] = vertex;

        }

    }
}

/* Checks if all sides of a 12 sided shape 'pts' are perpendicular and have the 
right orientation, using ANGLE_THRESH */
bool HDetector::angle_check(vpf pts){
    
    int bitmasks[8] = {2145,195,390,780,1560,3120};
    int current_bm = 0;

    float a = 0;
    for (int i = 0; i < 12; i++){
        
        a = angle_between_vectors(pts[(12+i+1)%12], pts[(12+i-1)%12], pts[(12+i)%12]);
        if ( abs( abs(a) - PI/2 ) < ANGLE_THRESH)
            return false;
        else
            current_bm = current_bm | (a > 0) << i;
    }

    for(int bm : bitmasks){
        if(current_bm == bm) 
            return true;
    }
    return false;

}

// Determines angle between vectors 'v1' and 'v2' using 'origin' as origin
float HDetector::angle_between_vectors(Point2f v1, Point2f v2, Point2f origin){
    
    float v1Lenght, v2Length; 
    
    v1Length = sqrt((v1.x - origin.x)*(v1.x - origin.x) + (v1.y - origin.y)*(v1.y - origin.y));
    v2Length = sqrt((v2.x - origin.x)*(v2.x - origin.x) + (v2.y - origin.y)*(v2.y - origin.y));
    // Takes dot product and divides by vector lengths to get cos of the angle
    float a = ((v1.x - origin.x)*(v2.y - origin.y) - (v1.y - origin.y)*(v2.x -origin.x))/(v1Length*v2Length);

    return asin(a);
}

/* Takes an image as argument and returns warped perspective, moving edge_pts to
the edge of the frame */
Mat HDetector::edge_perspective_transform(Mat image){

    order_points();

    Point tl = this->edge_pts[0];
    Point tr = this->edge_pts[1];
    Point br = this->edge_pts[2];
    Point bl = this->edge_pts[3];

    float widthA =  sqrt( abs ( pow( br.x - bl.x, 2.0) - pow( br.y - bl.y, 2.0 ) ) );  
    float widthB =  sqrt( abs ( pow( tr.x - tl.x, 2.0) - pow( tr.y - tl.y, 2.0 ) ) );
    // Requires explicit reference due to cv::max
    float maxWidth = std::max(widthA, widthB);

    float heightA =  sqrt( abs ( pow( br.y - tr.y, 2.0 ) - pow( br.x - tr.x, 2.0) ) ); 
    float heightB =  sqrt( abs ( pow( tl.y - bl.y, 2.0 ) - pow( tl.x - bl.x, 2.0) ) );

    float maxHeight = std::max(heightA, heightB);

    vpf dst = {
        Point2f(0.0, 0.0) ,
        Point2f(maxWidth, 0.0) ,
        Point2f(maxWidth, maxHeight),
        Point2f(0.0, maxHeight)
    };

    Mat M = getPerspectiveTransform(this->edge_pts, dst);
    
    warpPerspective(image, this->warped, M, Size(maxWidth, maxHeight));

    return M;
}

// Sorts points based on y coordinate
struct comparison {
    bool operator() (Point2f pt1, Point2f pt2) { return (pt1.y > pt2.y);}
} hasGreaterY;

/* Order points in edge_pts so that the first exit is the top-left, the second 
top-right, the third bottom-right, and the fourth bottom-left */   
void HDetector::order_points(){

    sort(this->edge_pts.begin(), this->edge_pts.end(), hasGreaterY);
    Point2f p1, p2;

    /* Switch the position of the first and second points
        if the second is to the right of the second */
    if(this->edge_pts[0].x > this->edge_pts[1].x ){
        p1 = this->edge_pts[1];
        this->edge_pts[1] = this->edge_pts[0];
        this->edge_pts[0] = p1;
    }

    /* Same as above for the third and fourth */
    if(this->edge_pts[2].x < this->edge_pts[3].x){
        p2 = this->edge_pts[3];
        this->edge_pts[3] = this->edge_pts[2];
        this->edge_pts[2] = p2;
    }

}

void HDetector::runnin_state_cb(std_msgs::Bool data){
    this->runnin = data.data;
}

void HDetector::image_detection_callback(const sensor_msgs::ImageConstPtr& img){
    if(this->runnin){
        cv_bridge::CvImagePtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        
        }catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        precision_landing::H_info msg;

        if (this->detect(cv_ptr->image)){
            msg.detected = true;
            msg.center_x = this->getCenter_X();
            msg.center_y = this->getCenter_Y();
            msg.area_ratio = this->getArea();
            this->h_info_pub.publish(msg);
        }
        
    }
}

float HDetector::getArea(){
    return this->area_ratio;
}

void HDetector::setArea(vpf contour, Mat frame){
    this->area_ratio = contourArea(contour, false)/(frame.cols*frame.rows);
}

int HDetector::getCenter_X(){
    return (this->hBoundingRect.x + this->hBoundingRect.width/2);
}

int HDetector::getCenter_Y(){
    return (this->hBoundingRect.y + this->hBoundingRect.height/2);
}


// For testing
int main(int argc, char** arvg){
    ROS_INFO("Running H detection node!");
    ros::init(argc, arvg, "h_node");
    HDetector* detector = new HDetector();
    ros::spin();
}

