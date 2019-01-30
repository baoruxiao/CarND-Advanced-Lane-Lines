/*
 * LaneDetector.cpp
 *
 *  Created on: Jan 26, 2019
 *      Author: Bao
 */

#include "LaneDetector.h"
#include "PolynomialRegressional.h"
#include "utils.h"

using namespace cv;

#define CV_CALIB_FIX_K4 2048;
#define CV_CALIB_FIX_K5 4096;

void LaneDetector::cameraCalib(const vector<string>& chessboard_imgs, int nx, int ny) {
	int num_imgs = chessboard_imgs.size();

	vector<Point3f> objpts;
	for (int i = 0; i < ny; i++) {
		for (int j = 0; j < nx; j++) {
			objpts.push_back(Point3f(float(j), float(i), 0));
		}
	}

	vector<vector<Point3f> > objpoints = {};
	vector<vector<Point2f> > imgpoints = {};
	for (int i = 0; i < num_imgs; i++) {
		Mat img = imread(chessboard_imgs[i]);
		if (!img.data) {
			cout << "Cannot load image." << endl;
			return;
		}
		Mat gray;
		cvtColor(img, gray, COLOR_BGR2GRAY);
		vector<Point2f> imgpts;
		bool ret = findChessboardCorners(gray, Size(nx, ny), imgpts);
		if (!ret) {
			cout << "failed to extract chessboard corners, skipped." << endl;
		} else {
			objpoints.push_back(objpts);
			imgpoints.push_back(imgpts);
		}
	}

	// calibrate camera
	int flag = 0;
	flag |= CV_CALIB_FIX_K4;
	flag |= CV_CALIB_FIX_K5;
	Mat img = imread(chessboard_imgs[0]);
	calibrateCamera(objpoints, imgpoints, img.size(), cam.cameraMat,
			cam.distCoeff, cam.rvecs, cam.tvecs);
}


void LaneDetector::undistImage(InputArray img, OutputArray undist, OutputArray undist_v) {
		undistort(img, undist, cam.cameraMat, cam.distCoeff, cam.cameraMat);
		Mat undistVis;
		undist.copyTo(undistVis);
		vector<vector<Point2i> > poly(1);
		for (auto x : src) poly[0].push_back(x);
		polylines(undistVis, InputArrayOfArrays(poly),  true, Scalar(0, 0, 255), 3);
		undistVis.copyTo(undist_v);
}

void LaneDetector::thresholding(InputArray img, OutputArray threshed, OutputArray threshedCol) {
	Mat hls, r_channel, r_binary,  s_channel, s_binary;
	vector<Mat> bgr(3);
	split(img, bgr);
	r_channel = bgr[2];
	threshold(r_channel, r_binary, 200, 255, THRESH_BINARY);

	vector<Mat> chs(3);
	cvtColor(img, hls, COLOR_BGR2HLS);
	split(hls, chs);
	s_channel = chs[2];
	threshold(s_channel, s_binary, 170, 255, THRESH_BINARY);

	Mat gray, sobelx, abs_sobelx, scaled_sobel, sxbinary_low, sxbinary_high, sxbinary;
	cvtColor(img, gray, COLOR_BGR2GRAY);
	Sobel(gray, sobelx, CV_64F, 1, 0);
	abs_sobelx = cv::abs(sobelx);
	scaled_sobel = 255*abs_sobelx/(*max_element(abs_sobelx.begin<double>(),
			abs_sobelx.end<double>()));
	threshold(scaled_sobel, sxbinary_low, 20, 255, THRESH_BINARY);
	threshold(scaled_sobel, sxbinary_high, 100, 255, THRESH_BINARY_INV);
	sxbinary_low.convertTo(sxbinary_low, CV_8U);
	sxbinary_high.convertTo(sxbinary_high, CV_8U);
	sxbinary = sxbinary_low & sxbinary_high;

	Mat combinedCol;
	Mat combined = sxbinary | s_binary | r_binary;
	merge(vector<Mat>{sxbinary, s_binary, r_binary}, combinedCol);
	combined.copyTo(threshed);
	combinedCol.copyTo(threshedCol);
}


void LaneDetector::projectForward(InputArray img, OutputArray warped) {
	Mtrans = getPerspectiveTransform(src, dst);
	Minv = getPerspectiveTransform(dst, src);
	warpPerspective(img, warped, Mtrans, img.size(), INTER_LINEAR);
}


void LaneDetector::slidingWindow(InputArray img, OutputArray detected, OutputArray debug) {
	Mat imgM = img.getMat();
	int nwindows = 9;
	int margin = 100;
	int minpix = 50;
	int window_height = (int)imgM.rows/nwindows;

	Mat bottomHalf = imgM(Rect(0, imgM.rows/2, imgM.cols, imgM.rows/2));
	bottomHalf.convertTo(bottomHalf, CV_32F);
	vector<float> histogram;
	for (int i = 0; i < bottomHalf.cols; i++) {
		double column_sum = 0;
		for (int j = 0; j < bottomHalf.rows; j++) {
			column_sum += bottomHalf.at<float>(j, i);
		}
		histogram.push_back(column_sum);
	}

	vector<float>& left_coeff = leftLine.best_fit;
	vector<float>& right_coeff = rightLine.best_fit;
	vector<int>& left_xfitted = leftLine.recent_xfitted;
	vector<int>& right_xfitted = rightLine.recent_xfitted;
	vector<Point2i> left_lane_inds = {};
	vector<Point2i> right_lane_inds = {};
	Mat imgMRGB;
	merge(vector<Mat>{imgM, imgM, imgM}, imgMRGB);
	Mat det(imgMRGB.size(), CV_8UC3, Scalar(0));
	if (leftLine.detected & rightLine.detected) {
		for (int row=0; row < imgM.rows; row++) {
			int left_col = left_xfitted[row];
			int right_col = right_xfitted[row];
			int win_xleft_low = std::min(std::max(left_col - margin, 0), imgM.cols - 1);
			int win_xleft_high = std::min(std::max(left_col + margin, 0), imgM.cols - 1);;
			int win_xright_low = std::min(std::max(right_col - margin, 0), imgM.cols - 1);
			int win_xright_high = std::min(std::max(right_col + margin, 0), imgM.cols - 1);
			imgMRGB.at<Vec3b>(row, win_xleft_low) = Vec3b(0, 255, 0);
			imgMRGB.at<Vec3b>(row, win_xleft_high) = Vec3b(0, 255, 0);
			imgMRGB.at<Vec3b>(row, win_xright_low) = Vec3b(0, 255, 0);
			imgMRGB.at<Vec3b>(row, win_xright_high) = Vec3b(0, 255, 0);
			for (int col=win_xleft_low; col < win_xleft_high; col++) {
				if (int(imgM.at<uchar>(row, col)) > 0) {
					left_lane_inds.push_back(Point2i(col, row));
					imgMRGB.at<Vec3b>(row, col) = Vec3b(0, 0, 255);
					det.at<Vec3b>(row, col) = Vec3b(0, 0, 255);
				}
			}
			for (int col=win_xright_low; col < win_xright_high; col++) {
				if (int(imgM.at<uchar>(row, col)) > 0) {
					right_lane_inds.push_back(Point2i(col, row));
					imgMRGB.at<Vec3b>(row, col) = Vec3b(255, 0, 0);
					det.at<Vec3b>(row, col) = Vec3b(255, 0, 0);
				}
			}
		}
	} else {
		int mid_point = bottomHalf.cols/2;
		auto leftx_peak = max_element(histogram.begin(), histogram.begin() + mid_point);
		auto rightx_peak = max_element(histogram.begin() + mid_point, histogram.end());
		int leftx_base = distance(histogram.begin(), leftx_peak);
		int rightx_base = distance(histogram.begin(), rightx_peak);

		int leftx_current = leftx_base;
		int rightx_current = rightx_base;

		for (int i = 0; i < nwindows; i++) {
			vector<Point2i> good_left_inds = {};
			vector<Point2i> good_right_inds = {};

			int win_y_low = imgM.rows - (i + 1)*window_height;
			int win_y_high = imgM.rows - i * window_height;

			int win_xleft_low = std::min(std::max(leftx_current - margin, 0), imgM.cols - 1);
			int win_xleft_high = std::min(std::max(leftx_current + margin, 0), imgM.cols - 1);;
			int win_xright_low = std::min(std::max(rightx_current - margin, 0), imgM.cols - 1);
			int win_xright_high = std::min(std::max(rightx_current + margin, 0), imgM.cols - 1);

			rectangle(imgMRGB, Point2i(win_xleft_low, win_y_low), Point2i(win_xleft_high, win_y_high),
					Scalar(0, 255, 0), 2);
			rectangle(imgMRGB, Point2i(win_xright_low, win_y_low), Point2i(win_xright_high, win_y_high),
				Scalar(0, 255, 0), 2);
			for (int row=win_y_low; row < win_y_high; row++) {
				for (int col=win_xleft_low; col < win_xleft_high; col++) {
					if (int(imgM.at<uchar>(row, col)) > 0) {
						good_left_inds.push_back(Point2i(col, row));
						imgMRGB.at<Vec3b>(row, col) = Vec3b(0, 0, 255);
					};
				}
				for (int col=win_xright_low; col < win_xright_high; col++) {
					if (int(imgM.at<uchar>(row, col)) > 0) {
						good_right_inds.push_back(Point2i(col, row));
						imgMRGB.at<Vec3b>(row, col) = Vec3b(255, 0, 0);
					}
				}
			}
			left_lane_inds.insert(left_lane_inds.end(), good_left_inds.begin(), good_left_inds.end());
			right_lane_inds.insert(right_lane_inds.end(), good_right_inds.begin(), good_right_inds.end());

			if (good_left_inds.size() > minpix) {
				int tmp_sum = 0;
				for (auto good_left_in : good_left_inds) {
					tmp_sum += good_left_in.x;
				}
				leftx_current = tmp_sum / good_left_inds.size();
			}
			if (good_right_inds.size() > minpix) {
				int tmp_sum = 0;
				for (auto good_right_in : good_right_inds) {
					tmp_sum += good_right_in.x;
				}
				rightx_current = tmp_sum / good_right_inds.size();
			}
		}

	}
	vector<float> left_lane_indx = {}; vector<float> left_lane_indy = {};
	vector<float> right_lane_indx = {}; vector<float> right_lane_indy = {};
	for (auto item : left_lane_inds) {
		left_lane_indx.push_back((float)item.x); left_lane_indy.push_back((float)item.y);
	}
	for (auto item : right_lane_inds) {
		right_lane_indx.push_back((float)item.x); right_lane_indy.push_back((float)item.y);
	}
	leftLine.detected = true;
	rightLine.detected = true;
	// fitting second order poly
	PolynomialRegression<float> poly;
	left_coeff.clear();
	right_coeff.clear();
	left_xfitted.clear();
	right_xfitted.clear();
	poly.fitIt(left_lane_indy, left_lane_indx, 2, left_coeff);
	poly.fitIt(right_lane_indy, right_lane_indx, 2, right_coeff);
	for (int y = 0; y < imgM.rows; y++) {
		float x = left_coeff[0] + left_coeff[1]*float(y) +
				left_coeff[2]*float(y)*float(y);
		int checked_x = std::min(std::max(int(x), 0), imgM.cols - 1);
		left_xfitted.push_back(checked_x);
		imgMRGB.at<Vec3b>(int(y), checked_x) = Vec3b(0, 255, 0);
	}
	for (int y = 0; y < imgM.rows; y++) {
		float x = right_coeff[0] + right_coeff[1]*float(y) +
				right_coeff[2]*float(y)*float(y);
		int checked_x = std::min(std::max(int(x), 0), imgM.cols - 1);
		right_xfitted.push_back(checked_x);
		imgMRGB.at<Vec3b>(int(y), checked_x) = Vec3b(0, 255, 0);
	}
	// overlay lane

	for (int y = 0; y < imgM.rows; y++) {
		for (int x = left_xfitted[y]; x < right_xfitted[y]; x++) {
			if (det.at<Vec3b>(y, x) == Vec3b(0, 0, 0))
				det.at<Vec3b>(y, x) = Vec3b(0, 255, 0);
		}
	}

	imgMRGB.copyTo(debug);
	det.copyTo(detected);
}


void LaneDetector::compCurveRadius() {
	float ym_per_pix = float(30/720);
	vector<float>& left_fit = leftLine.best_fit;
	vector<float>& right_fit = rightLine.best_fit;

	leftLine.radius_of_curvature = pow(1 + pow(2*left_fit[2]*719*ym_per_pix + left_fit[1],2),1.5) / fabs(2*left_fit[2]);
	rightLine.radius_of_curvature = pow(1 + pow(2*right_fit[2]*719*ym_per_pix + right_fit[1],2),1.5) / fabs(2*right_fit[2]);
}


void LaneDetector::compDistToCtr() {
	float xm_per_pix = float(3.7/700);
	int rightx_bottom = rightLine.recent_xfitted[719];
	int leftx_bottom = leftLine.recent_xfitted[719];
	float lane_ctr = float(leftx_bottom) + float(rightx_bottom - leftx_bottom) / 2;
	vector<Point2f> warpLaneCtr;
	warpLaneCtr.push_back(Point2f(lane_ctr, (float)imgHeight));
	vector<Point2f> realLaneCtr;
	perspectiveTransform(warpLaneCtr, realLaneCtr, Minv);
	float img_ctr = float(imgWidth / 2);
	float lane_ctr_real = realLaneCtr[0].x;
	distToCtr = (lane_ctr_real - img_ctr) * xm_per_pix;
}


void LaneDetector::projectBackward(InputArray img, InputArray det,
		OutputArray overlay, OutputArray wrpdbck) {
	Mat image = img.getMat();
	Mat warpedback;
	warpPerspective(det, warpedback, Minv, det.size(), INTER_LINEAR);
	Mat ovrly(image.size(), CV_8UC3, Scalar(0));
	for (int i = 0; i < image.rows; i++) {
		for (int j = 0; j < image.cols; j++) {
			if (warpedback.at<Vec3b>(i, j) != Vec3b(0, 0, 0)) {
				ovrly.at<Vec3b>(i, j) = 0.7 * image.at<Vec3b>(i, j) + 0.3 * warpedback.at<Vec3b>(i, j);
			} else {
				ovrly.at<Vec3b>(i, j) = image.at<Vec3b>(i, j);
			}
		}
	}
	stringstream ss1;
	ss1 << "Radius of curvature: " <<
			fixed << setprecision(0) << (leftLine.radius_of_curvature + rightLine.radius_of_curvature)/2 << "(m)";
	stringstream ss2;
	ss2 << "Vehicle is " << fixed << setprecision(1);
	if (distToCtr > 0) {
		ss2 << distToCtr << "(m)" << " left of center";
	}  else {
		ss2 << abs(distToCtr) << "(m)" << " right of center";
	}
	putText(ovrly, ss1.str(), Point(10, 30), 0, 1, Scalar(255, 255, 255), 1, 1);
	putText(ovrly, ss2.str(), Point(10, 60), 0, 1, Scalar(255, 255, 255), 1, 1);
	ovrly.copyTo(overlay);
	warpedback.copyTo(wrpdbck);
}


void LaneDetector::detect(InputArray img, OutputArray out, OutputArray debug) {
	Mat undist, undist_v;
	undistImage(img, undist, undist_v);

	Mat threshed, threshedCol;
	thresholding(undist, threshed, threshedCol);

	Mat warped;
	projectForward(threshed, warped);

	Mat detected, debug_detected;
	slidingWindow(warped, detected, debug_detected);
	compCurveRadius();
	compDistToCtr();

	Mat overlay;
	Mat warpedback;
	projectBackward(img, detected, overlay, warpedback);

	Mat image = img.getMat();
	Mat collage;
	monitor({image, undist_v, threshedCol, debug_detected, warpedback, overlay}, collage);

	overlay.copyTo(out);
	collage.copyTo(debug);
}














