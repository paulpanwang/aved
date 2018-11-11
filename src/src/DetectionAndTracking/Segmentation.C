/*
 * Copyright 2018 MBARI
 *
 * Licensed under the GNU LESSER GENERAL PUBLIC LICENSE, Version 3.0
 * (the "License"); you may not use this file except in compliance 
 * with the License. You may obtain a copy of the License at
 *
 * http://www.gnu.org/copyleft/lesser.html
 *
 * Unless required by applicable law or agreed to in writing, software
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
#include <cstdio>
#include <cstdlib>
#include <utility>
#include <string>
#include <sstream>

#include "DetectionAndTracking/MbariFunctions.H"
#include "DetectionAndTracking/segment/image.h"
#include "DetectionAndTracking/segment/misc.h"
#include "DetectionAndTracking/segment/pnmfile.h"
#include "DetectionAndTracking/segment/segment-image.h"
#include "DetectionAndTracking/Segmentation.H"
#include "Image/CutPaste.H"
#include "Image/MathOps.H"
#include "Image/MorphOps.H"
#include "Image/Kernels.H"
#include "Raster/Raster.H"
#include "Raster/PngWriter.H"

using namespace std;

Segmentation::Segmentation() {
}

Segmentation::~Segmentation() {
}


// ######################################################################
// Returns neighborhood to search for adaptive segmentation. If not between 1 and 20
// defaults to 5
// ######################################################################
int Segmentation::getNeighborhoodSize(const vector< float > & v) {
    float n = v.at(0);
       if ( n >= 1.0F && n <= 20.F )
           return (int) n;
       return 5;
}

// ######################################################################
// Returns neighborhood to search for adaptive segmentation. If not between 0 and 20
// defaults to 7
// ######################################################################
int Segmentation::getOffset(const vector< float > & v) {
    float n = v.at(1);
       if ( n >= 0.F && n <= 20.F )
           return (int) n;
       return 7;
}

// ######################################################################
// Returns sigma for graph segmentation. If not between 0 and 1 defaults to 0.5
// ######################################################################
float Segmentation::getSigma(const vector< float > & v) {
    float sigma = v.at(0);
    if (sigma <= 1.0f && sigma > 0.0f)
        return sigma;
    return 0.50;
}
// ######################################################################
// Returns K for graph segmentation. If not greater than 0 defaults to 500
// ######################################################################
int Segmentation::getK(const vector< float > & v) {
    float k = v.at(1);
    if ( k > 0.F)
        return (int) k;
    return 500;
}
// ######################################################################
// Returns minimum size for graph segmentation. If not greater than 0 defaults to 50
// ######################################################################
int Segmentation::getMinSize(const vector< float > & v) {
    float minSize = v.at(2);
    if ( minSize > 0.F)
        return (int) minSize;
    return 50;
}

// ######################################################################

Image< PixRGB<byte> > Segmentation::runGraph(const float sigma, const int k, const int min_size,
        const Image < PixRGB<byte> >&input) {
  LDEBUG("processing with sigma: %f k: %d minsize: %d ",sigma,k,min_size);

    image<rgb> *im = new image<rgb > (input.getWidth(), input.getHeight());

    for (int x = 0; x < input.getWidth(); x++)
        for (int y = 0; y < input.getHeight(); y++) {
            rgb val;
            PixRGB<byte> val2 = input.getVal(x, y);
            val.r = val2.red();
            val.g = val2.green();
            val.b = val2.blue();
            imRef(im, x, y) = val;
        }

    // run segmentation
    image <rgb> *seg = segment_image(im, sigma, k, min_size);

    // initialize the output image with the segmented results
    Image < PixRGB<byte> > output = input;
    for (int x = 0; x < input.getWidth(); x++)
        for (int y = 0; y < input.getHeight(); y++) {
            rgb val = imRef(seg, x, y);
            PixRGB<byte> val2((byte) val.r, (byte) val.g, (byte) val.b);
            output.setVal(x, y, val2);
        }

    delete im;
    delete seg; 
    return output;
}
// ######################################################################
// ###### Private Functions related to the Adaptive algorithms #####
// ######################################################################

 /**
 *AdapThresh is an algorithm to apply adaptive thresholding to an image.
 *@author Timothy Sharman
 */

  /**
   *Applies the adaptive thresholding operator to the specified image array
   *using the mean function to find the threshold value
   *
   *@param src pixel array representing image to be thresholded
   *@param size the size of the neigbourhood used in finding the threshold
   *@param con the constant value subtracted from the mean
   *@return a thresholded pixel array of the input image array
   */

  Image<byte> Segmentation::mean_thresh(const Image<byte>& src,  const int size, const int con){
    Image<byte> resultfinal(src.getDims(), ZEROS);
    const int i_w = src.getWidth(), i_h = src.getHeight();
    int mean = 0;
    int count = 0;
    int a, b;

    //Now find the sum of values in the size X size neigbourhood
    for(int j = 0; j < i_h; j++){
      for(int i = 0; i < i_w; i++){
	mean = 0;
	count = 0;
        
	//Check the local neighbourhood
	for(int k = 0; k < size; k++){
	  for(int l = 0; l < size; l++){
	      a = i - ((int)(size/2)+k);
	      b = j - ((int)(size/2)+l);
	      if (a >=0 && b >=0) {
	         mean = mean + src.getVal(a,b);
	         count++;
		}
	  }
	}
	//Find the mean value
	if (count > 0) {
	  mean = (int)(mean /count) - con;
 	}
 
	//Threshold below the mean
	if(src.getVal(i,j) >= mean){
	  resultfinal.setVal(i,j,0); 
	}
	else {
	  resultfinal.setVal(i,j,255);
	}
      }
    }
    return resultfinal;
  }

  /**
   *Applies the adaptive thresholding operator to the specified image array
   *using the median function to find the threshold value
   *
   *@param src pixel array representing image to be thresholded
   *@param width width of the image in pixels
   *@param height height of the image in pixels
   *@param size the size of the neigbourhood used in finding the threshold
   *@param con the constant value subtracted from the median
   *@return a thresholded pixel array of the input image array
   */ 
  Image<byte> Segmentation::median_thresh(const Image<byte>& src, const int size, const int con){
    Image<byte> resultfinal(src.getDims(), ZEROS);
    const int i_w = src.getWidth(), i_h = src.getHeight();

    int median = 0;  
    vector<int> values(size*size);
    int count = 0;
    int a,b;

    //Now find the values in the size X size neigbourhood
    for(int j = 0; j < i_h; j++){
      for(int i = 0; i < i_w; i++){
	median = 0;
	count = 0;
        
	//Check the local neighbourhood
	for(int k = 0; k < size; k++){
	  for(int l = 0; l < size; l++){
	      a = i - ((int)(size/2)+k);
	      b = j - ((int)(size/2)+l);
	      if (a >=0 && b >=0) {
	        values[count] = src.getVal(a,b);
	        count++;
	      }
	  }
	}
	//Find the median value

	//First Sort the array
    sort(values.begin(), values.end());

	//Then select the median
	count = count / 2;
	median = values[count] - con;
   
	//Threshold below the mean
	if(src.getVal(i,j) >= median){
	  resultfinal.setVal(i,j,0);
	}
	else {
	  resultfinal.setVal(i,j,255); 
	}
      }
    }
    return resultfinal;
  }

  /**
   *Applies the adaptive thresholding operator to the specified image array
   *using the mean of max & min function to find the threshold value
   *
   *@param src pixel array representing image to be thresholded
   *@param size the size of the neigbourhood used in finding the threshold
   *@param con the constant value subtracted from the mean
   *@return a thresholded pixel array of the input image array
   */

  Image<byte> Segmentation::meanMaxMin_thresh(const Image<byte>& src, const int size, const int con){
    Image<byte> resultfinal(src.getDims(), ZEROS);
    const int i_w = src.getWidth(), i_h = src.getHeight();

    int mean = 0;
    int max = 0, min = 0;
    
    int a,b;
    int tmp;

    //Now find the max and min of values in the size X size neigbourhood
    for(int j = 0; j < i_h; j++){
      for(int i = 0; i < i_w; i++){
	mean = 0;
	max = src.getVal(i,j);
	min = src.getVal(i,j);
	//Check the local neighbourhood
	for(int k = 0; k < size; k++){
	  for(int l = 0; l < size; l++){
	      a = i - ((int)(size/2)+k);
	      b = j - ((int)(size/2)+l);
	      if (a >=0 && b >=0) {
	          tmp = src.getVal(a,b);
	          if(tmp > max){
		    max = tmp;
	          }
	         if(tmp < min){
		    min = tmp;
	          }
		}
	  }
	}
        
	//Find the mean value
	tmp = max + min;
	tmp = tmp / 2;
	mean = tmp - con;

	//Threshold below the mean
	if(src.getVal(i,j) >= mean){
	  resultfinal.setVal(i,j,0);
	}
	else {
	  resultfinal.setVal(i,j,255);
	}
      }
    }
    return resultfinal;
  }

  // ######################################################################
  Image< PixRGB<byte> > Segmentation::runGraph(Image< PixRGB<byte> > image, Rectangle region, float scale)
{
    DetectionParameters dp = DetectionParametersSingleton::instance()->itsParameters;
    vector<float> p = getFloatParameters(dp.itsSegmentGraphParameters);
    const float sigma = getSigma(p);
    const int k = (float)getK(p)*scale;
    const int min_size = (float)getMinSize(p);

    // run graph based segment algorithm on region of interest
    Image< PixRGB<byte> > segmentIn = crop(image, region);
    Image< PixRGB<byte> > graphImgRoi = runGraph(sigma, k, min_size, segmentIn);
    Image< PixRGB<byte> > graphImg(image.getDims(), ZEROS);

    // paste graphImgRoi into graphImg at given position
    inplacePaste(graphImg, graphImgRoi, Point2D<int>(region.left(), region.top()));
    return graphImg;
  }

  // ######################################################################
  void Segmentation::run(uint frameNum, Image<byte> &segmentIn, Image< PixRGB<byte> >&graphSegmentOut, Image<byte>& binSegmentOut)
  {
    DetectionParameters dp = DetectionParametersSingleton::instance()->itsParameters;
    Image<byte> se = twofiftyfives(dp.itsCleanupStructureElementSize);
    vector<float> p = getFloatParameters(dp.itsSegmentAdaptiveParameters);
    const int size = getNeighborhoodSize(p);
    const int conn = getOffset(p);

    LINFO("Running segmentation for frame %d", frameNum);

    if (dp.itsSegmentAlgorithmType == SAGraphCut || dp.itsSegmentAlgorithmType == SABest) {
        vector<float> p = getFloatParameters(dp.itsSegmentGraphParameters);
        const float sigma = getSigma(p);
        const int k = getK(p);
        const int min_size = getMinSize(p);

        graphSegmentOut = runGraph(sigma, k, min_size, segmentIn);

        const byte threshold = mean(segmentIn);
        binSegmentOut = makeBinary(segmentIn, threshold);
    }
    if (dp.itsSegmentAlgorithmType == SAMeanAdaptiveThreshold) {
        binSegmentOut = mean_thresh(segmentIn, size, conn);
        binSegmentOut = erodeImg(dilateImg(binSegmentOut, se), se);
    }
    else if (dp.itsSegmentAlgorithmType == SAMedianAdaptiveThreshold || dp.itsSegmentAlgorithmType == SABest) {
        binSegmentOut = median_thresh(segmentIn, size, conn);
        binSegmentOut = erodeImg(dilateImg(binSegmentOut, se), se);
    }
    else if (dp.itsSegmentAlgorithmType == SAMeanMinMaxAdaptiveThreshold) {
        binSegmentOut = meanMaxMin_thresh(segmentIn, size, conn);
        binSegmentOut = erodeImg(dilateImg(binSegmentOut, se), se);
    }
  }
