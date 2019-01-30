/*
 * main.cpp
 *
 *  Created on: Jan 26, 2019
 *      Author: Bao
 */

#include <iostream>
#include <glob.h>

#include "LaneDetector.h"


inline void mouse_callback(int  event, int  x, int  y, int  flag, void *param)
{
    if (event == EVENT_MOUSEMOVE) {
        cout << "(" << x << ", " << y << ")" << endl;
    }
}


inline void getFiles(const string& pttrn, vector<string>& fileList) {
	glob_t buf;
	glob(pttrn.c_str(), GLOB_TILDE, NULL, &buf);
	for (int i = 0; i < buf.gl_pathc; ++i) {
		fileList.push_back(buf.gl_pathv[i]);
	}
	if (buf.gl_pathc > 0)
		globfree(&buf);
}



int main() {
	using namespace std;

	LaneDetector ld(0, 0);
	vector<string> chessboard_images;
	getFiles("camera_cal/*.jpg", chessboard_images);

	ld.cameraCalib(chessboard_images);
	Mat img = imread("camera_cal/calibration1.jpg");
	Mat undist;
	Mat undist_v;
	ld.undistImage(img, undist, undist_v);
	imshow("dist", img);
	imshow("undist", undist);
	waitKey();

//	// initialize
//	VideoCapture cap("project_video.mp4");
//
//	if (!cap.isOpened()) return -1;
//	Mat first;
//	cap >> first;
//	LaneDetector ld(first.rows, first.cols);
//
//
//	/* camera calibration */
//
//	vector<string> chessboard_images;
//	getFiles("camera_cal/*.jpg", chessboard_images);
//
//	ld.cameraCalib(chessboard_images);
//
//	/* Process  */
////	VideoWriter out("output_images/out_project_video.avi",
////			VideoWriter::fourcc('M','J','P','G'), 30, first.size());
//	VideoWriter outMonitor("output_images/out_project_video_monitor.avi",
//			VideoWriter::fourcc('M','J','P','G'), 30,
//			Size(int(first.cols*3/2), int(first.rows*2/2)));
//
//	int countFrame = 0;
//	for (;;) {
//		Mat frame;
//		cap >> frame;
//		cout << countFrame << endl;
//		countFrame++;
//		if (frame.empty()) break;
//		Mat overlay, monitor;
//		ld.detect(frame, overlay, monitor);
////		out.write(overlay);
//		outMonitor.write(monitor);
//	}
////	out.release();
//	outMonitor.release();
//
//	return 0;
}























