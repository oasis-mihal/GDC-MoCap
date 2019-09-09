// OpenCVPractice.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/objdetect.hpp"
#include "opencv2/features2d.hpp"
#include <iostream>
#include <fstream>
#include <String>

using namespace cv;
using namespace std;

int max_value = 255;
#define Scale 5.0

int low_H = 0, low_S = 0, low_V = 0;
int high_H = max_value, high_S = max_value, high_V = max_value;
const String window_detection_name = "Red Channel";

struct Joint {
	string name;
	int parentIndex = -1;
	Point3d offset;

	bool isRoot = false;
	int numChannels;
	vector<double> channels;

};

#pragma region Trackbars
static void on_low_H_thresh_trackbar(int, void *)
{
	low_H = min(high_H - 1, low_H);
	setTrackbarPos("Low H", window_detection_name, low_H);
}
static void on_high_H_thresh_trackbar(int, void *)
{
	high_H = max(high_H, low_H + 1);
	setTrackbarPos("High H", window_detection_name, high_H);
}
static void on_low_S_thresh_trackbar(int, void *)
{
	low_S = min(high_S - 1, low_S);
	setTrackbarPos("Low S", window_detection_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void *)
{
	high_S = max(high_S, low_S + 1);
	setTrackbarPos("High S", window_detection_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void *)
{
	low_V = min(high_V - 1, low_V);
	setTrackbarPos("Low V", window_detection_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void *)
{
	high_V = max(high_V, low_V + 1);
	setTrackbarPos("High V", window_detection_name, high_V);
}
#pragma endregion

float dot(Point a, Point b) {
	return a.x * b.x + a.y * b.y;
}

float mag(Point a) {
	return sqrt(float(a.x * a.x + a.y * a.y));
}

float cross(Point a, Point b) {
	float x = 0;
	float y = 0;
	float z = a.x * b.y - a.y * b.x;
	return z;
}

float vecAngle(Point a, Point b) {
	if (mag(a) * mag(b) < 0.1) {
		return 0.0;
	}
	float dotAB = dot(a, b);
	float magA = mag(a);
	float magB = mag(b);



	float angle = acos(dot(a, b) / (mag(a) * mag(b))) * 180.0 / 3.1416;
	//Check if the rotation is clockwise or ccw
	if (cross(a, b) < 0) {
		angle *= -1;
	}

	
	
	if (isnan(angle)) {
		angle = 0.0;
	}
	if (angle > 360.0) {
		angle = 360.0;
	}
	if (angle < -360.0) {
		angle = 0.0;
	}
	return angle;
}

void OutputData(vector<vector<Point>> points, vector<string> structure, vector<Joint> joints) {
	ofstream fin;
	fin.open("../x64/Debug/MoCap.bvh");
	for (int line = 0; line < structure.size(); line++) {
		fin << structure.at(line) << endl;
	}

	fin << "MOTION" << endl
		<< "Frames:\t" << (int)points.size() << endl
		<< "Frame Time: 0.033333" << endl;

	Point root;

	for (int i = 0; i < points.size(); i++) {
		root = points.at(i).at(0);
		fin << points.at(i).at(0).x / Scale << "\t" << points.at(i).at(0).y / Scale << "\t" << 0.0 << "\t" << "-3.41	 14.78	-164.35";
		for (int j = 2; j < points.at(i).size(); j++) {
			//Point newPoint = points.at(i).at(j);// -root;
			float relativeAngle = 0;
			Joint previousJoint = joints.at(j);
			if (previousJoint.name == "Site ") {
				continue;
			}
			if (previousJoint.isRoot == false) {
				int previousJointIndex = previousJoint.parentIndex;
				previousJoint = joints.at(previousJoint.parentIndex);
				if (previousJoint.isRoot == false) {
					//Dot poduct of two 2D vectors will give the z angle
					relativeAngle = vecAngle(points.at(i).at(j) - points.at(i).at(previousJointIndex), Point((int)joints.at(j).offset.x, (int)joints.at(j).offset.y));
				}
				
				
			}
			
			fin << "\t" << relativeAngle << "\t" << 0.0 << "\t" << 0.0;
		}
		//TODO: Impliment the last point as well
		fin << "\t" << 0.0 << "\t" << 0.0 << "\t" << 0.0;
		fin << endl;
	}
	fin.close();

}



Point3d ParseOffset(string str) {
	int start = str.find("OFFSET");

	int xs = str.find("\t", start);
	int xe = str.find("\t", xs + 1);
	double x = stod(str.substr(xs, xe - xs));

	int ye = str.find("\t", xe + 1);
	double y = stod(str.substr(xe, ye - xe));

	int ze = str.find("\t", ye + 1);
	double z = stod(str.substr(ye, ze - ye));
	

	return Point3d(x * Scale, y * Scale, z * Scale);

}

#define MaxRadius 10
#define NumPoints 18


vector<Joint> ParseJoints(vector<string> data) {
	vector<Joint> joints;
	int lastParent = -1;
	for (int i = 0; i < data.size(); i++) {
		string line = data.at(i);
		Joint joint = Joint();
		int pos = data.at(i).find("JOINT");
		
		if (pos == string::npos) {
			pos = data.at(i).find("ROOT");
			if (pos != string::npos) {
				joint.isRoot = true;
			}
		}
		if (pos == string::npos) {
			pos = data.at(i).find("End Site");
		}
		if (pos == string::npos) {
			if (line.find("}") != string::npos) {
				lastParent = joints.at(lastParent).parentIndex;
			}
			continue;
		}
		joint.parentIndex = lastParent;
		lastParent = joints.size();
		
		joint.name = data.at(i).substr(line.find(" ") + 1);
		joint.offset = ParseOffset(data.at(i + 2));
		joints.push_back(joint);


		
		//keyPoints.at(0).push_back(ParseOffset(pos, line));
		
	}
	return joints;
}

int main(int argc, char* argv) {
	VideoCapture camera = VideoCapture(0);

	Mat frame;

	namedWindow(window_detection_name, WINDOW_NORMAL);
	// Trackbars to set thresholds for HSV values
	createTrackbar("Low H", window_detection_name, &low_H, 255, on_low_H_thresh_trackbar);
	createTrackbar("High H", window_detection_name, &high_H, 255, on_high_H_thresh_trackbar);
	createTrackbar("Low S", window_detection_name, &low_S, 255, on_low_S_thresh_trackbar);
	createTrackbar("High S", window_detection_name, &high_S, 255, on_high_S_thresh_trackbar);
	createTrackbar("Low V", window_detection_name, &low_V, 255, on_low_V_thresh_trackbar);
	createTrackbar("High V", window_detection_name, &high_V, 255, on_high_V_thresh_trackbar);

	setTrackbarPos("Low H", window_detection_name, 0);
	setTrackbarPos("High H", window_detection_name, 115);
	setTrackbarPos("Low S", window_detection_name, 0);
	setTrackbarPos("High S", window_detection_name, 70);
	setTrackbarPos("Low V", window_detection_name, 255);
	setTrackbarPos("High V", window_detection_name, 255);

	vector<vector<Point>> keyPoints;

	vector<Joint> joints;
	

	keyPoints.push_back(vector<Point>());
	//keyPoints.at(0).resize(NumPoints);

	//Load data from file
	vector<std::string> structureData;
	fstream file;
	file.open("../x64/Debug/MoCap-Structure.bvh");
	if (file.fail()) {
		cout << "File failed to open" << endl;
		waitKey(0);
	}
	Point parent = Point();
	while (!file.fail() && !file.eof()) {
		std::string line;
		getline(file, line);
		structureData.push_back(line);
		
		
	}
	joints = ParseJoints(structureData);

	keyPoints.at(0).push_back(Point(joints.at(0).offset.x, joints.at(0).offset.y));
	for (int i = 1; i < joints.size(); i++) {
		Point absoluteOffset = Point(joints.at(i).offset.x, joints.at(i).offset.y);
		Joint previousJoint = joints.at(i);
		while (previousJoint.isRoot == false) {
			previousJoint = joints.at(previousJoint.parentIndex);
			absoluteOffset += Point(previousJoint.offset.x, previousJoint.offset.y);
		}
		keyPoints.at(0).push_back(absoluteOffset);
	}

	
	bool isFirst = true;
	bool isFirst_ = true;

	bool colorCheckMode = true;


	while (camera.isOpened()) {


		camera.read(frame);
		imshow("Frame", frame);
		Point screenOffset = Point(frame.cols/2, frame.rows/2);

		vector<Mat> channels;
		split(frame, channels);


		Mat image = frame;
		Mat output;
		frame.copyTo(output);

		cvtColor(image, image, COLOR_BGR2HSV);
		//inRange(image, Vec3b(21, 88, 123), Vec3b(31, 216, 255), image);
		inRange(image, Vec3b(low_H, low_S, low_V), Vec3b(high_H, high_S, high_V), image);
		medianBlur(image, image, 9);
		dilate(image, image, Mat::ones(3, 3, image.type()));

		imshow("Red Channel", image);
		if (colorCheckMode && waitKey(1) == -1) {
			continue;
		}
		else {
			colorCheckMode = false;
		}
		//continue;


		//cvtColor(channels[2], channels[2], COLOR_BGR2GRAY);
		//threshold(channels[2], channels[2], 127, 255.0, THRESH_BINARY);
		//channels[2].convertTo(channels[2], CV_8UC1);


		vector <vector<Point>> contours;

		findContours(image, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

		keyPoints.push_back(keyPoints.back());

		//Show the previous keypoints
		for (int i = 0; i < keyPoints.back().size(); i++){
			circle(output, Point(keyPoints.back().at(i).x, -keyPoints.back().at(i).y) + screenOffset, 3, Scalar(0, 0, 255), 3);
		}

		//Show Bone relations
		//Assume 0 is root
		for (int i = 1; i < keyPoints.back().size(); i++) {
			int parentIndex = joints.at(i).parentIndex;
			line(output, Point(keyPoints.back().at(i).x, -keyPoints.back().at(i).y) + screenOffset, Point(keyPoints.back().at(parentIndex).x, -keyPoints.back().at(parentIndex).y) + screenOffset, Scalar(0, 0, 255), 1);
		}

		for (int i = 0; i < contours.size(); i++) {// && contours.size() >= NumPoints
			vector<Point> hull;
			convexHull(contours.at(i), hull);

			double hullArea = fabs(contourArea(hull));
			polylines(output, hull, true, Scalar(0, 255, 0));

			if (hullArea > 20) {
				Rect bounds = boundingRect(hull);
				Point keyPoint = Point(bounds.x + bounds.width / 2.0, (bounds.y + bounds.height / 2.0)) - screenOffset;
				keyPoint.y *= -1.0;
				circle(output, Point(bounds.x + bounds.width / 2.0,bounds.y + bounds.height / 2.0), 3, Scalar(255, 255, 0), 3);
				
				
				int closestPoint = -1;
				double closestDist = std::numeric_limits<double>().max();
				for (int j = 0; j < keyPoints.at(keyPoints.size() - 1).size(); j++) {
					double currentDist = norm(keyPoints.back().at(j) - keyPoint);
					if (currentDist < closestDist && currentDist < MaxRadius) {
						closestPoint = j;
						closestDist = currentDist;
					}
				}
				if(closestPoint != -1)
					keyPoints.back().at(closestPoint) = (keyPoint);
				
				
				
				
				
			}
			//if (!isFirst) {
				//polylines(output, keyPoints.at(0), true, Scalar(0, 255, 255));
				//OutputData(keyPoints, structureData);
				
			//}
		}
		isFirst = false;

		imshow("Output", output);
		if (waitKey(33) > 0) {
			if (!isFirst) {
				OutputData(keyPoints, structureData, joints);

			}
			break;
		}
	}
	
	return 0;
}



