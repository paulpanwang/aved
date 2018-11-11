/*
 * Copyright 2018 MBARI
 *
 * Licensed under the GNU LESSER GENERAL PUBLIC LICENSE, Version 3.0
 * (the "License"); you may not use this file except in compliance
 * with the License. You may obtain a copy of the License at
 *
 * http://www.gnu.org/copyleitsFeatureCollection/lesser.html
 *
 * Unless required by applicable law or agreed to in writing, so its FeatureCollection are
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

/*!@file MotionOps.C a class for motion related functions used in classification */


#ifndef MOTIONOPS_C_DEFINED
#define MOTIONOPS_C_DEFINED

#define MAX_NUM_FEATURES   5000

// OpenCV must be first to avoid conflicting defs of int64, uint64
#include "Image/OpenCVUtil.H"
#include <cstdio>

#include "Image/CutPaste.H"
#include "Image/ColorOps.H"
#include "Image/MathOps.H"
#include "Image/DrawOps.H"
#include "Motion/MotionOps.H"

#ifdef HAVE_OPENCV

inline static void allocateOnDemand
( IplImage **img, CvSize size, int depth, int channels) 
{
  if ( *img != NULL ) return;
  *img = cvCreateImage( size, depth, channels );
}

#endif // HAVE_OPENCV

// ######################################################################
rutz::shared_ptr<MbariOpticalFlow> getOpticFlow
(Image<byte> image1, Image<byte> image2)
{
 rutz::shared_ptr<MbariOpticalFlow> oflow;

#ifndef HAVE_OPENCV
  LFATAL("OpenCV must be installed in order to use this function");
#else

  IplImage* frame1_1C = img2ipl(image1); 
  IplImage* frame2_1C = img2ipl(image2); 
  
  CvSize frame_size;
  frame_size.width  = image1.getWidth();  
  frame_size.height = image1.getHeight();
  
  static IplImage 
    *eig_image = NULL, *temp_image = NULL, 
    *pyramid1 = NULL, *pyramid2 = NULL;

  // Shi and Tomasi Feature Tracking!

  // Preparation: Allocate the necessary storage.
  allocateOnDemand( &eig_image,  frame_size, IPL_DEPTH_32F, 1 ); 
  allocateOnDemand( &temp_image, frame_size, IPL_DEPTH_32F, 1 );
  
  // Preparation: This array will contain the features found in frame 1.
  CvPoint2D32f frame1_features[MAX_NUM_FEATURES];
  int number_of_features = MAX_NUM_FEATURES;
  
  // Actually run the Shi and Tomasi algorithm!!:
  //   "frame1_1C" is the input image. 
  //    "eig_image" and "temp_image" are just workspace for the algorithm.
  //    The first ".01" specifies the minimum quality of the features 
  //      (based on the eigenvalues). 
  //    The second ".01" specifies the minimum Euclidean distance between features
  //    "NULL" means use the entire input image. can be just part of the image
  //    WHEN THE ALGORITHM RETURNS: * "frame1_features" 
  //      will contain the feature points. 
  //    "number_of_features" will be set to a value <= MAX_NUM_FEATURES 
  //        indicating the number of feature points found.
  cvGoodFeaturesToTrack(frame1_1C, eig_image, temp_image, frame1_features,
 			& number_of_features, .01, .01, NULL);

  // Pyramidal Lucas Kanade Optical Flow!
  
  CvPoint2D32f frame2_features[MAX_NUM_FEATURES];
  char optical_flow_found_feature[MAX_NUM_FEATURES];
  float optical_flow_feature_error[MAX_NUM_FEATURES];

  // // Window size used to avoid the aperture problem 
  CvSize optical_flow_window = cvSize(3,3);

  // termination criteria: 
  // stop after 20 iterations or epsilon is better than .3. 
  // play with these parameters for speed vs. accuracy 
  CvTermCriteria optical_flow_termination_criteria = 
    cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 100, .3 );

  // This is some workspace for the algorithm. * (The algorithm
  // actually carves the image into pyramids of different resolutions
  allocateOnDemand( &pyramid1, frame_size, IPL_DEPTH_8U, 1 ); 
  allocateOnDemand( &pyramid2, frame_size, IPL_DEPTH_8U, 1 );

  LINFO("number of features: %d", number_of_features);

   // Actually run Pyramidal Lucas Kanade Optical Flow!! 
   // "5" is the maximum number of pyramids to use. 0: just one level
   // level.
   // last "0" means disable enhancements. 
   // (For example, the second array isn't preinitialized with guesses.)
   cvCalcOpticalFlowPyrLK
     (frame1_1C, frame2_1C, pyramid1, pyramid2, 
      frame1_features, frame2_features, number_of_features, 
      optical_flow_window, 5, optical_flow_found_feature,
      optical_flow_feature_error, optical_flow_termination_criteria, 0 );

   std::vector<rutz::shared_ptr<MbariFlowVector> > fv; fv.clear();
   for(int i = 0; i < number_of_features; i++)
     {
       // If Pyramidal Lucas Kanade didn't really find the feature, skip it.
       if ( optical_flow_found_feature[i] == 0 ) continue;
       
       rutz::shared_ptr<MbariFlowVector> tflow
         (new MbariFlowVector
          (Point2D<float>((float)frame1_features[i].x, 
                          (float)frame1_features[i].y),
           Point2D<float>((float)frame2_features[i].x, 
                          (float)frame2_features[i].y),
           optical_flow_found_feature[i]));

       fv.push_back(tflow);

       LDEBUG("[%3d] flow: 1[%13.4f %13.4f] 2[%13.4f %13.4f] "
	      "ang: %13.4f mag: %13.4f val: %13.4f",
	      i, tflow->p1.i, tflow->p1.j, tflow->p2.i, tflow->p2.j, 
	      tflow->angle, tflow->mag, tflow->val);
     }

   // create the optical flow
   oflow.reset(new MbariOpticalFlow(fv, image1.getDims()));

  cvReleaseImage(&eig_image);
  cvReleaseImage(&temp_image);
  cvReleaseImage(&pyramid1);
  cvReleaseImage(&pyramid2);
#endif // HAVE_OPENCV

   return oflow;
}

// ######################################################################
Image<PixRGB<byte> > drawOpticFlow(Image<PixRGB<byte> > img, rutz::shared_ptr<MbariOpticalFlow> oflow)
{
  std::vector<rutz::shared_ptr<MbariFlowVector> > flow =
    oflow->getFlowVectors();

  // draw the flow field
  for(uint i = 0; i < flow.size(); i++)
    {
      Point2D<int> pt1((int)flow[i]->p1.i, (int)flow[i]->p1.j);
      Point2D<int> pt2((int)flow[i]->p2.i, (int)flow[i]->p2.j);
      drawLine(img, pt1, pt2, PixRGB<byte>(255,0,0), 1);
    }

    return img;
}



// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !MOTIONOPS_C_DEFINED
