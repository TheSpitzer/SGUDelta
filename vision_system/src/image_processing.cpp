#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

static const std::string OPENCV_WINDOW = "Raw Image";
//static const std::string OPENCV_WINDOW2 = "Binary Image";
static const std::string OPENCV_WINDOW3 = "Processed Image";

using namespace cv;

class ImageProcessing
{

protected:
	ros::NodeHandle nh_;
	ros::Publisher coord_pub_;
	ros::Publisher vel_pub_;
	ros::Subscriber vel_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	Mat img_in_;
	Mat img_hsv_;
	Mat img_hue_;
	Mat img_sat_;
	Mat img_bin_;
	Mat img_out_;

public:
	ImageProcessing()
	  : it_(nh_)
	{
		//Subscribe to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/PS3Eye/image_raw", 1, &ImageProcessing::imageCb, this);
		image_pub_ = it_.advertise("/image_processing/output_video", 1);

		namedWindow(OPENCV_WINDOW);
//		namedWindow(OPENCV_WINDOW2);
		namedWindow(OPENCV_WINDOW3);
	}

	~ImageProcessing()
	{
		destroyWindow(OPENCV_WINDOW);
//		destroyWindow(OPENCV_WINDOW2);
		destroyWindow(OPENCV_WINDOW3);
	}

	void velCb(geometry_msgs::Vector3Stamped v)
	{
//		vel_pub_ = nh_.advertise<geometry_msgs::Twist>("coordinate", 100);
//		vel_ = nh_.subscribe("/rpm", 100, &ImageProcessing::velCb, this);
		geometry_msgs::Twist velocity;
		velocity.angular.x = v.vector.x;
		vel_pub_.publish(velocity);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		//Convert ROS Input Image Message to IplImage
		cv_bridge::CvImagePtr cv_input_;
		try
		{
		  cv_input_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
		  ROS_ERROR("cv_bridge exception: %s", e.what());
		  return;
		}
		
		//Convert IplImage to cv::Mat
		//img_in_ = cv::Mat(cv_input_).clone();
		img_in_ = cv_input_->image;

		//Output image = Input Image (Segmentation)
		img_out_ = img_in_.clone();

		//Convert Input Image to HSV
		cv::cvtColor (img_in_, img_hsv_, CV_BGR2HSV);

		//Zero Matrices
		img_hue_ = Mat::zeros(img_hsv_.rows, img_hsv_.cols, CV_8U);
    		img_sat_ = Mat::zeros(img_hsv_.rows, img_hsv_.cols, CV_8U);
    		img_bin_ = Mat::zeros(img_hsv_.rows, img_hsv_.cols, CV_8U);
		
		//HSV Channel 0 -> img_hue_ & HSV Channel 1 -> img_sat_
		int from_to[] = { 0,0, 1,1};
   		Mat img_split[] = { img_hue_, img_sat_};
    		mixChannels(&img_hsv_, 3,img_split,2,from_to,2);

		for(int ii = 0; ii < img_out_.rows; ii++)
		{
			for(int jj = 0; jj < img_out_.cols; jj++)
			{
				//The output pixel is white if input pixel hue is darker and saturation is reasonable
				if(img_hue_.at<uchar>(ii,jj) > 0 && img_hue_.at<uchar>(ii,jj) < 10 && img_sat_.at<uchar>(ii,jj) > 160)
				{
					img_bin_.at<uchar>(ii,jj) = 255;
				}
				else
				{
					img_bin_.at<uchar>(ii,jj) = 0;
/*					//Clear pixel BGR output channel
					img_out_.at<uchar>(ii,jj*3+0) = 0;
					img_out_.at<uchar>(ii,jj*3+1) = 0;
					img_out_.at<uchar>(ii,jj*3+2) = 0;*/
				}
			}
		}

		//strel_size is the size of the structuring element
		Size strel_size;
		strel_size.width = 3;
		strel_size.height = 3;

		//Create an elliptical structuring element
		Mat strel = getStructuringElement(MORPH_ELLIPSE,strel_size);

		//Apply an opening morphological operation using the structuring element to the image three times
		morphologyEx(img_bin_,img_bin_,MORPH_OPEN,strel,Point(-1,-1),3);

		//Invert the Image
		bitwise_not(img_bin_,img_bin_);

		//Blur the Image to Improve Detection
		GaussianBlur(img_bin_, img_bin_, Size(11,11),2,2);

		//Vector Circles will hold the position and radius of the detected circles
		vector<Vec3f> circles;

		//Detect Circles
		HoughCircles(img_bin_, circles, CV_HOUGH_GRADIENT, 1, 80, 140, 15, 5, 100);

		for( size_t i = 0; i < circles.size(); i++ )
    		{
         		// round the floats to an int
         		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
         		int radius = cvRound(circles[i][2]);
         		// draw the circle center
         		circle( img_out_, center, 3, Scalar(0,255,0), -1, 8, 0 );
        	 	// draw the circle outline
         		circle( img_out_, center, radius+1, Scalar(0,0,255), 3, 8, 0 );
         		// Debugging Output
//         		ROS_INFO("x: %d y: %d r: %d",center.x,center.y, radius);
			// Publish the data
	 	/*	coord_pub_ = it_.advertise<std_msgs::String>("Coordinate", 100);
		coord_pub_ = nh_.advertise<std_msgs::Float32>("coordinate", 100);
		std_msgs::Float32 msg;
		msg.data = center.x;
		msg.data = center.y;
			
		coord_pub_.publish(msg);
		*/
		coord_pub_ = nh_.advertise<geometry_msgs::Twist>("coordinate", 100);
		geometry_msgs::Twist msg;
		msg.linear.x = center.x;
		msg.linear.y = center.y;	
		coord_pub_.publish(msg);
    }

		//Update GUI Window
		imshow(OPENCV_WINDOW, img_in_);
//		imshow(OPENCV_WINDOW2, img_bin_);
		imshow(OPENCV_WINDOW3, img_out_);
		
		waitKey(3);

		//Convert cv::Mat to IplImage
//		cv_bridge::CvImage img_out_;
//		img_out_.encoding = sensor_msgs::image_encodings::BGR8;
//		img_out_.image = img_hsv_;

		//Publish Output Image
//		image_pub_.publish(img_out_.toImageMsg());

	}
};

int main(int argc, char** argv)
{
	//Initialize ROS Node
	ros::init(argc, argv, "image_processing");

//	ros::Subscriber rpm_ = nh_.subscribe("/rpm", 100, velCb);
	//Start Node
	ImageProcessing ip;


	//Keep the ROS Running
	ros::spin();

return 0;
}
