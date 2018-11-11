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

/*!@file FOEestimator.H a class for estimating the focus of expansion
  in video
 */

#ifndef FOEESTIMATOR_H_DEFINED
#define FOEESTIMATOR_H_DEFINED

#include "Image/ImageCache.H"
#include "Util/Types.H"
#include "Image/Geometry2D.H"

// ######################################################################
//! compute the focus of expansion (FOE) from the pixel-based optical flow
class FOEestimator
{
public:

  //! constructor
  /*!@param numAvg the number of frames used for averaging to get a smooth estimate
    @param pyramidLevel the subsampling level at which to compute the optical flow*/
  FOEestimator(int numAvg = 20, int pyramidLevel = 0);

  // default =, copy and destr. okay

  //! update with a new frame of the movie sequence
  /*!@return if the cache contains less than three frames, an invalid Vector2D
    is returned, otherwise the estimated location of the FOE is returned*/
  Vector2D updateFOE(const Image<byte>& img, const Rectangle region);

  //! update with a new frame of the movie sequence
  /*!@return if the cache contains less than three frames, an invalid Vector2D
    is returned, otherwise the estimated location of the FOE is returned*/
  Vector2D updateFOE(const Image<byte>& img);

  //! returns the last estimate of the FOA
  Vector2D getFOE();

private:
  float getZeroCrossing(const Image<float>& vec);

  const int itsPyrLevel;
  ImageCache<byte> itsFrames;
  ImageCacheAvg<float> itsXvectors, itsYvectors;
  Vector2D itsFOE;
};


#endif
