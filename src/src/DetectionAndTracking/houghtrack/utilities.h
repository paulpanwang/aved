/******************************************************************************
 * Author: Martin Godec
 *         godec@icg.tugraz.at
 ******************************************************************************/

#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <set>
#include <vector>
#include <string>
#include <opencv/cv.h>

#define  sign(s)	((s > 0 ) ? 1 : ((s<0) ? -1 : 0))
#define  round(v)   ((int) (v+0.5))

inline float sigmoid(float x)
{
	return 1.0f/(1.0f+exp(-x));
}

inline double sigmoid(double x)
{
	return 1.0/(1.0+exp(-x));
}

//! Returns a random number in [0, 1]
double randDouble();

//! Returns a random number in [-1, 1]
double randnDouble();

//! Returns a random number with gaussian probability distribution around zero
double randGauss( double std_dev );

//! Returns an integer in [from:from + range]
int randIntFromRange(const int from, const int range);

std::string createFilename(std::string prefix, int idx, std::string suffix, int numLength = 4);

void setCenter(cv::Rect& rect, const cv::Point& center);

cv::Point getCenter(const cv::Rect& rect);

cv::Rect intersect(const cv::Rect& rectA, const cv::Rect& rectB);

void writeResult(std::vector<cv::Rect>& trackedPositions, std::string seq_name);


#endif // UTILITIES_H_
