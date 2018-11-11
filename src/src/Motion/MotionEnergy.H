/*!@file RCBot/Motion/MotionEnergy.H detect motion in an image stream */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
// //////////////////////////////////////////////////////////////////// //
// This file is part of the iLab Neuromorphic Vision C++ Toolkit.       //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is free software; you can   //
// redistribute it and/or modify it under the terms of the GNU General  //
// Public License as published by the Free Software Foundation; either  //
// version 2 of the License, or (at your option) any later version.     //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  //
// that it will be useful, but WITHOUT ANY WARRANTY; without even the   //
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU General Public License for more details.       //
//                                                                      //
// You should have received a copy of the GNU General Public License    //
// along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   //
// to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   //
// Boston, MA 02111-1307 USA.                                           //
// //////////////////////////////////////////////////////////////////// //
//
// Primary maintainer for this file: Lior Elazary <lelazary@yahoo.com>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/RCBot/Motion/MotionEnergy.H $
// $Id: MotionEnergy.H 8429 2007-05-25 18:29:40Z rjpeters $
//

#ifndef MotionEnergy_H
#define MotionEnergy_H

#include <math.h>

#include "Image/PyramidTypes.H"
#include "Image/PyrBuilder.H"

// ######################################################################
//! This class implements Motion Energy detections using the Reichardt detectors

class MotionEnergyPyrBuilder
{
public:
  // ######################################################################
  // #### Constructors
  // ######################################################################
  MotionEnergyPyrBuilder(const PyramidType type, const float timeDecay);

  Image<byte> updateMotion(const Image<byte>& lum, const Image<byte>& itsClipMask);

  ~MotionEnergyPyrBuilder();

private:
  // Compute a motion conspicuity map from a motion pyramid
  Image<float> computeMMAP(const uint index, ImageSet<float>& clipMask, const char *label);

  Image<float> normalizMAP(const Image<float>& ima, const Dims& dims, const char *label);

  // Compute center surround
  Image<float> centerSurround(const uint cntrlev, const uint surrlev, const uint index, ImageSet<float>& clipMask);

 // Image<float> centerSurroundDiff(ImageSet<float>& pyr1, ImageSet<float>& pyr2,
 //                                 ImageSet<float>& clipMask, const char *label);

  PyramidType itsPyramidType;
  float itsTimeDecay;
  ReichardtPyrBuilder<float> *itsDir[4]; // four directions only
  std::deque<ImageSet<float> > itsPq[4]; // temporal queue of pyramids; one for each direction
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
