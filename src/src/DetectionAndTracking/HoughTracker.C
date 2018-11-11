/*
 * Copyright 2018 MBARI
 *
 * Licensed under the GNU LESSER GENERAL PUBLIC LICENSE, Version 3.0
 * (the "License"); you may not use this file except in compliance 
 * with the License. You may obtain a copy of the License at
 *
 * http://www.gnu.org/copyleft/lesser.html
 *
 * Unless required by applicable law or agreed to in writing, so its Features are
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * This is a program to automate detection and tracking of events in underwater 
 * video. This is based on modified version from Dirk Walther's 
 * work that originated at the 2002 Workshop  Neuromorphic Engineering 
 * in Telluride, CO, USA. 
 * 
 * This code requires the The iLab Neuromorphic Vision C++ Toolkit developed
 * by the University of Southern California (USC) and the iLab at USC. 
 * See http://iLab.usc.edu for information about this project.
 *  
 * This work would not be possible without the generous support of the 
 * David and Lucile Packard Foundation
 */ 

/*!@file HoughTracker.C a class for using Hough transform tracking algorithm */

#include "DetectionAndTracking/HoughTracker.H"
#include "DetectionAndTracking/DetectionParameters.H"
#include "Media/MbariResultViewer.H"

#include <csignal>
#include <vector>

#define STEP_WIDTH 1
#define SHIFT_TO_CENTER
#define GRABCUT_ROUNDS 5
//#define DEBUG

using namespace std;
using namespace cv;

// ######################################################################
HoughTracker::HoughTracker() { }

// ######################################################################
HoughTracker::HoughTracker(const Image< PixRGB<byte> > &img, BitObject &bo) {
	reset(img, bo, DEFAULT_FORGET_CONSTANT);
}

// ######################################################################
HoughTracker::~HoughTracker() {
	free();
}

// ######################################################################
void HoughTracker::free() {
	itsFeatures.clear();
	itsFerns.clear();
}

// ######################################################################
void HoughTracker::reset(const Image< PixRGB<byte> > &img, BitObject &bo, const float forgetConstant) {
	Rectangle region = bo.getBoundingBox();
	Point2D<int> center = bo.getCentroid();
	LINFO("Resetting HoughTracker region top %d left %d width %d height %d", \
region.left(), region.top(), region.width(), region.height());
	free();
	DetectionParameters dp = DetectionParametersSingleton::instance()->itsParameters;

	int baseSize = 12;
	itsObject = Rect(region.left(), region.top(), region.width(), region.height());
	itsImgRect = Rect(baseSize / 2, baseSize / 2, img.getDims().w() - baseSize, img.getDims().h() - baseSize);
	Mat frame = Mat(img.getHeight(), img.getWidth(), CV_8UC3, (char *) img.getArrayPtr()).clone();
	itsFeatures.setImage(frame);

	float opacity = 1.0F;
	byte foreground(GC_FGD);
	Image<byte> mask(img.getDims(), ZEROS); // initialize as background
	bo.drawShape(mask, foreground, opacity); // initialize shape as foreground
	//Mat backProject = Mat(mask.getHeight(), mask.getWidth(), CV_8UC1, (char *) mask.getArrayPtr()).clone();

	Mat backProject(img.getDims().h(), img.getDims().w(), CV_8UC1, Scalar(GC_BGD));
	rectangle(backProject, Point(itsObject.x-10, itsObject.y-10), Point(itsObject.x+itsObject.width+10, itsObject.y+itsObject.height+10), Scalar(GC_PR_BGD), -1);
	rectangle(backProject, Point(itsObject.x, itsObject.y), Point(itsObject.x+itsObject.width, itsObject.y+itsObject.height), Scalar(GC_FGD), -1);
	itsFerns.initialize(20, Size(baseSize, baseSize), 8, itsFeatures.getNumChannels());
	itsMaxObject = intersect(itsImgRect, squarify(itsObject, DEFAULT_SCALE_INCREASE));
	Point objCenter(center.i, center.j);

	LINFO("Initial position: %d,%d %dx%d", itsObject.x, itsObject.y, itsObject.width, itsObject.height);
	Rect updateRegion = intersect(itsMaxObject + Size(10, 10) - Point(5, 5), itsImgRect);

	try {
		run(updateRegion, objCenter, backProject, forgetConstant);
		itsSearchWindow = itsMaxObject + Size(10, 10) - Point(5, 5);
		LINFO(" Start tracking");
	}
	catch (...) {
		LINFO("Exception occurred");
	}
}

// ######################################################################
bool HoughTracker::update(nub::soft_ref <MbariResultViewer> &rv,
						  const uint frameNum,
						  Image< PixRGB<byte> > &img,
						  const Image<byte> &occlusionImg,
						  Rectangle &region,
						  Image<byte>& binaryImg,
						  const int evtNum,
						  const float forgetConstant) {
	float backProjectRadius = 0.5;
	float backProjectminProb = 0.5;
	double minVal, maxVal = 6.0f;
	Point minLoc;
	Mat frame = Mat(img.getHeight(), img.getWidth(), CV_8UC3, (char *) img.getArrayPtr()).clone();
	itsFeatures.setImage(frame);
	Mat result(frame.rows, frame.cols, CV_32FC1, Scalar(0.0));
	Mat backProject(frame.rows, frame.cols, CV_8UC1, Scalar(GC_BGD));
	Point center;
	DetectionParameters dp = DetectionParametersSingleton::instance()->itsParameters;

	int baseSize = 12;
	itsObject = Rect(region.left(), region.top(), region.width(), region.height());
	itsImgRect = Rect(baseSize / 2, baseSize / 2, img.getDims().w() - baseSize, img.getDims().h() - baseSize);
	itsMaxObject = intersect(itsImgRect, squarify(itsObject, DEFAULT_SCALE_INCREASE));
	LINFO("Reset position: %d,%d %dx%d", itsObject.x, itsObject.y, itsObject.width, itsObject.height);
	Rect updateRegion = intersect(itsMaxObject + Size(10, 10) - Point(5, 5), itsImgRect);
	itsSearchWindow = itsMaxObject + Size(10, 10) - Point(5, 5);

	try {
		LINFO("Evaluate");
		itsFerns.evaluate(itsFeatures, intersect(itsSearchWindow, itsImgRect), result, STEP_WIDTH, 0.5f);
		Mat out = result;

		normalize(out, out, 255, 0, NORM_MINMAX);
		minMaxLoc(result, &minVal, &maxVal, &minLoc, &itsMaxLoc);
		LINFO("Locate: maximum is at (%d/%d: %f)", itsMaxLoc.x, itsMaxLoc.y, maxVal);

		if (maxVal < 3.0f) {
			LINFO("Max val too small: %f", maxVal);
			return false;
		}

		center = Point(itsMaxLoc.x, itsMaxLoc.y);

		setCenter(itsMaxObject, center);
		setCenter(itsObject, center);
		setCenter(itsSearchWindow, center);

		LINFO("Backproject");
		backProject = Scalar(GC_BGD);
		rectangle(backProject, Point(itsMaxObject.x, itsMaxObject.y),
				  Point(itsMaxObject.x + itsMaxObject.width, itsMaxObject.y + itsMaxObject.height),
				  Scalar(GC_PR_BGD), -1);

		int cnt = itsFerns.backProject(itsFeatures, backProject, intersect(itsMaxObject, itsImgRect), itsMaxLoc,
									   backProjectRadius, STEP_WIDTH, backProjectminProb);
		showSegmentation(rv, backProject, "BackProject", frameNum, evtNum);

		if (cnt > 0) {
			LINFO("Segment");
			Mat subframe(frame, intersect(itsSearchWindow, itsImgRect));
			Mat subbackProject(backProject, intersect(itsSearchWindow, itsImgRect));

			Mat fgmdl, bgmdl;
			grabCut(subframe, subbackProject, itsObject, fgmdl, bgmdl, GRABCUT_ROUNDS, GC_INIT_WITH_MASK);
			backProject = maskOcclusion(occlusionImg, backProject);
			showSegmentation(rv, subbackProject, "Segmentation", frameNum, evtNum);

#ifdef SHIFT_TO_CENTER
			center = centerOfMass(backProject);
			setCenter(itsObject, center);
			setCenter(itsMaxObject, center);
			setCenter(itsSearchWindow, center);
#endif
		}

		if (cnt > 0) {
			Rect updateRegion = intersect(itsMaxObject + Size(10, 10) - Point(5, 5), itsImgRect);
			run(updateRegion, center, backProject, forgetConstant);
		}

		Rect bbox = getBoundingBox(backProject);
		if (bbox.width >= 0 && bbox.height >= 0) {
			LINFO("Found object %dx%d", bbox.width, bbox.height);
			binaryImg = makeBinarySegmentation(backProject, frameNum, evtNum);
			return true;
		}
		return false;
	}
	catch (...) {
		LINFO("Exception occurred");
		return false;
	}
}

bool HoughTracker::run(const Rect &ROI, const Point &center, const Mat &mask, const float forgetConstant) {
	int numPos = 0;
	int numNeg = 0;

	//try {
	for (int x = ROI.x; x < ROI.x + ROI.width; x += STEP_WIDTH)
		for (int y = ROI.y; y < ROI.y + ROI.height; y += STEP_WIDTH) {
			if ((mask.at < unsigned
			char > (y, x) == GC_FGD) || (mask.at < unsigned
			char > (y, x) == GC_PR_FGD))
			{
				itsFerns.update(itsFeatures, Point(x, y), 1, center);
				numPos++;
			}
			else if (mask.at < unsigned
			char > (y, x) == GC_BGD)
			{
				itsFerns.update(itsFeatures, Point(x, y), 0, center);
				numNeg++;
			}
		}
	itsFerns.forget(forgetConstant);
	LINFO("Updated %d points (%d+, %d-)", numPos + numNeg, numPos, numNeg);
	//} catch(VoteTooLargeException){
	//  	LINFO("Voting exception");
	//}
}

Rect  HoughTracker::getBoundingBox(const Mat &backProject) {
	Point min(backProject.cols, backProject.rows);
	Point max(0, 0);

	for (int x = 0; x < backProject.cols; x++)
		for (int y = 0; y < backProject.rows; y++) {
			if ((backProject.at < unsigned
			char > (y, x) == GC_FGD) || (backProject.at < unsigned
			char > (y, x) == GC_PR_FGD))
			{
				if (x < min.x) min.x = x;
				if (y < min.y) min.y = y;
				if (x > max.x) max.x = x;
				if (y > max.y) max.y = y;
			}
		}

	return Rect(min.x, min.y, max.x - min.x, max.y - min.y);
}

Point HoughTracker::centerOfMass(const Mat &mask) {
	float c_x = 0.0f;
	float c_y = 0.0f;
	float c_n = 0.0f;

	for (int x = 0; x < mask.cols; x++)
		for (int y = 0; y < mask.rows; y++)
			if ((mask.at < unsigned
	char > (y, x) == GC_FGD) || (mask.at < unsigned
	char > (y, x) == GC_PR_FGD))
	{
		c_x += x;
		c_y += y;
		c_n += 1.0f;
	}

	return Point(static_cast<int>(round(c_x / c_n)), static_cast<int>(round(c_y / c_n)));
}

Image<byte> HoughTracker::makeBinarySegmentation(const Mat &backProject, const uint frameNum, const int evtNum) {
	DetectionParameters dp = DetectionParametersSingleton::instance()->itsParameters;
	Mat display(backProject.rows, backProject.cols, CV_8U, Scalar(1));

	for (int x = 0; x < backProject.cols; x++)
		for (int y = 0; y < backProject.rows; y++) {
			switch (backProject.at < unsigned
			char > (y, x))
			{
				case GC_BGD:
					display.at < unsigned char > (y, x) = 0;
				break;
				case GC_FGD:
					display.at < unsigned char > (y, x) = 1;
				break;
				case GC_PR_BGD:
					display.at < unsigned char > (y, x) = 0;
				break;
				case GC_PR_FGD:
					display.at < unsigned char > (y, x) = 1;
				break;
				default:
					display.at < unsigned char > (y, x) = 0;
			}
		}

	Image <byte> output((const byte *) display.data, display.cols, display.rows);
	return output;
}

Mat HoughTracker::maskOcclusion(const Image<byte> &occlusionImg, const Mat &backProject) {
	Mat backProjectO(backProject.rows, backProject.cols, CV_8UC1, Scalar(GC_BGD));
	if (occlusionImg.getWidth() == backProject.cols && occlusionImg.getHeight() == backProject.rows) {
		for (int x = 0; x < backProjectO.cols; x++)
			for (int y = 0; y < backProjectO.rows; y++)
				if (occlusionImg.getVal(x, y) == 0)
					backProjectO.at < unsigned char > (y, x) = GC_PR_BGD; //set masked occlusion as possible background pixel
				else
					backProjectO.at < unsigned char > (y, x) = backProject.at < unsigned char > (y, x);
	} else {
		LFATAL("invalid sized occlusion mask; size is %dx%d but should be same size as input frame %dx%d",
			   occlusionImg.getWidth(), occlusionImg.getHeight(), backProject.cols, backProject.rows);
	}
	return backProjectO;
}


void HoughTracker::showSegmentation(nub::soft_ref<MbariResultViewer> &rv, \
								const Mat &backProject, \
								const string title, \
								const uint frameNum, \
								const int evtNum) {
#ifdef DEBUG
	Mat display(backProject.rows, backProject.cols, CV_8UC3, Scalar(0, 0, 0));

	for (int x = 0; x < backProject.cols; x++)
		for (int y = 0; y < backProject.rows; y++) {
			switch (backProject.at < unsigned char > (y, x))
			{
				case cv::GC_BGD:
				    display.at<unsigned char>(y,x*3+0) = 255;
				    display.at<unsigned char>(y,x*3+1) = 0;
				    display.at<unsigned char>(y,x*3+2) = 0;
					break;
				case cv::GC_FGD:
				    display.at<unsigned char>(y,x*3+0) = 0;
				    display.at<unsigned char>(y,x*3+1) = 0;
				    display.at<unsigned char>(y,x*3+2) = 255;
					break;
				case cv::GC_PR_BGD:
				    display.at<unsigned char>(y,x*3+0) = 255;
				    display.at<unsigned char>(y,x*3+1) = 128;
				    display.at<unsigned char>(y,x*3+2) = 128;
					break;
				case cv::GC_PR_FGD:
				    display.at<unsigned char>(y,x*3+0) = 128;
				    display.at<unsigned char>(y,x*3+1) = 128;
				    display.at<unsigned char>(y,x*3+2) = 255;
					break;
				default:
				    display.at<unsigned char>(y,x*3+0) = 0;
				    display.at<unsigned char>(y,x*3+1) = 128;
				    display.at<unsigned char>(y,x*3+2) = 0;
			}
		}

	//TODO: FIX this - this isn't compiling - getting error
	Image< PixRGB<byte> > output((const PixRGB<byte> *)display.data, display.cols, display.rows);
	rv->display(output, frameNum, title, evtNum);
#endif
}

// #############################################################################
// Instantiations
/*#define INSTANTIATE(T) \
template Image<T> HoughTracker::makeBinarySegmentation(const Mat &backProject, const uint frameNum, const int evtNum); \
template HoughTracker::HoughTracker(const Image< PixRGB<T> > &img, BitObject &bo); \
template Mat HoughTracker::maskOcclusion(const Image<T> &occlusionImg, const Mat &backProject); \
template void HoughTracker::reset(Image< PixRGB<T> > &img, BitObject &bo, const float forgetConstant); \
template bool HoughTracker::update(nub::soft_ref <MbariResultViewer> &rv,\
							  const uint frameNum,\
							  Image< PixRGB<T> > &img,\
							  const Image< T > &occlusionImg,\
							  Rectangle &region,\
							  Image< T > &binaryImg,\
							  const int evtNum,\
							  const float forgetConstant); \
INSTANTIATE(byte);*/
