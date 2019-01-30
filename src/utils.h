/*
 * utils.cpp
 *
 *  Created on: Jan 27, 2019
 *      Author: Bao
 */

#ifndef SRC_UTILS_H_
#define SRC_UTILS_H_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdarg.h>

using namespace std;
using namespace cv;


void monitor(vector<Mat> imgs, OutputArray clg, int ncols=3, double rx=0.5, double ry=0.5) {
	int num_images = imgs.size();
	int nrows = num_images / ncols + ((num_images % ncols == 0)?0:1);
	int height = int(imgs[0].rows*ry);
	int width = int(imgs[0].cols*rx);
	Mat collage(Size(width*ncols, height*nrows), CV_8UC3, Scalar(0));
	for (int i = 0; i < imgs.size(); i++) {
		Mat imgResize;
		resize(imgs[i], imgResize, Size(), rx, ry, INTER_LINEAR);
		imgResize.copyTo(collage(Rect(i%ncols*width, i/ncols*height, width, height)));
	}
	collage.copyTo(clg);
}


#endif /* SRC_UTILS_H_ */
