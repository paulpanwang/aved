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

#ifndef MBARI_OPTICALFLOW_DEFINED
#define MBARI_OPTICALFLOW_DEFINED

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/Raster.H"

#include "Utils/triple.hh"
#include "rutz/shared_ptr.h"

// ######################################################################
//! a flow vector struct to describe 1 flow vector
//! it also provides motion/flow strength value
struct MbariFlowVector
{
  MbariFlowVector() { };

  MbariFlowVector(const Point2D<float> inP1,
             const Point2D<float> inP2,
             const float inVal) :
    p1(inP1),
    p2(inP2),
    val(inVal),
    mag(0)

  {
    double angleRad = atan2(double(p2.j - p1.j), double(p2.i - p1.i));
    angle = angleRad/M_PI * 180.F;
    angle = fmod(angle +360.0, 360.0);
    mag   = sqrt( pow(p2.j - p1.j, 2.0) + pow(p2.i - p1.i, 2.0) );
    xmag  = mag * cos(angleRad);
    ymag  = mag * sin(angleRad);
  }

  MbariFlowVector(const Point2D<float> inP1,
             const float inAngle,
             const float inMag,
             const float inVal) :
    p1(inP1),
    angle(inAngle),
    mag(inMag),
    val(inVal)
  {
    float ang = angle/180.0F * M_PI;

    float i = p1.i + (mag * cos(ang));
    float j = p1.j + (mag * sin(ang));
    p2 = Point2D<float>(i,j);
  }

  Point2D<float> p1;
  Point2D<float> p2;

  float angle; // should be in degrees
  float mag;
  float val;
  float xmag;
  float ymag;
};

//! an Optical flow class that can return both:
//!   a vector of MbariFlowVector: for sparse flow 
//!     (note: also feed in image size) 
//!   or a triple of images for direction, length, and strength
class MbariOpticalFlow
{
public:
  // ######################################################################
  //! @name Constructor, assignment and destructor
  //@{

  //! Constructor taking in sparse flow vector
  /*! @param flow the vector of flow vectors in the image
      @param dims size of the image
  */
  MbariOpticalFlow(std::vector<rutz::shared_ptr<MbariFlowVector> > MbariFlowVectors, 
	      Dims dims);

  //! Constructor taking in dense flow field
  /*! @param flow field of the image
  */
  MbariOpticalFlow
  (lobot::triple<Image<float>, Image<float>, Image<float> > flowField);

  //! Destructor
  ~MbariOpticalFlow();

  //@}

  // ######################################################################
  //! @name Access functions
  //@{

  //! get the flow vectors
  std::vector<rutz::shared_ptr<MbariFlowVector> > getFlowVectors();

  //! get the flow vector locations
  std::vector<Point2D<float> > getFlowLocations();
  
  //! get the image dimensions
  Dims getImageDims();

  //! get the flow field
  lobot::triple<Image<float>,Image<float>,Image<float> >  getFlowField();

  //! get the direction field
  Image<float> getDirectionField();

  //! get the vector length field
  Image<float> getVectorLengthField();

  //! get the strength field
  Image<float> getFlowStrengthField();

  //@}

 private:

  // ######################################################################
  //! @name Compute functions (all the inner-working functions)
  //@{

  //! compute functions to compute (sparse) flow field
  //! given flow vectors
  void computeFlowField();

  //! compute functions to compute flow vectors
  //! given a (what should be a sparse) flow field 
  void computeFlowVectors();

  //! put the flow locations in one vector
  void computeFlowLocations();

  //@}  

  //! flag to check if the flow field or flow vector is computed
  bool itsFlowFieldComputed;
  bool itsFlowVectorsComputed;
  bool itsFlowLocationsComputed;

  std::vector<rutz::shared_ptr<MbariFlowVector> > itsFlowVectors;

  //! dimension of the image of the flow
  Dims itsImageDims;

  //! its flow locations
  std::vector<Point2D<float> > itsFlowLocations;

  //! Flow field represented by a triple of images:
  //! first:  direction
  //! second: length
  //! third:  flow strength
  //! usually for dense maps
  lobot::triple<Image<float>,Image<float>,Image<float> > itsFlowField;
};


#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
