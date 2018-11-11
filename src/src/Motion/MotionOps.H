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

/*!@file MotionOps.H a class for motion related functions used in classification */
// ######################################################################

#ifndef MOTIONOPS_H_DEFINED
#define MOTIONOPS_H_DEFINED

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Motion/OpticalFlow.H"
#include "Raster/Raster.H"

// ######################################################################
//! get the Lucas Kanade optic flow for motion
//! from image1 to image2
rutz::shared_ptr<MbariOpticalFlow> getOpticFlow(Image<byte> image1, Image<byte> image2);

//! draw the optic flow given a set of correspondences
Image<PixRGB<byte> > drawOpticFlow(Image<PixRGB<byte> > img, rutz::shared_ptr<MbariOpticalFlow> oflow);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !MOTIONOPS_H_DEFINED
