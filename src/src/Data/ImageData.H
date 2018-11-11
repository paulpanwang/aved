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

#ifndef IMAGEDATA_H
#define IMAGEDATA_H

#include "Image/Geometry2D.H"
#include "Image/Image.H"
#include "Data/MbariMetaData.H"

template <class T> class Image;
template <class T> class PixRGB;

//! Collection of image related data needed when running algorithms
/*@param frameNum current frame number operating on
@param foe current focus of expansion point
@param metadata metadata about this frame, e.g. timecode
@param mask mask for masking equipment
@param img image to update events from
@param prevImg previous frame image used in Hough tracker initialization
@param segmentImg  image used to run segmentation to extract BitObjects from*/
typedef struct ImageData {
    uint frameNum;
    Vector2D foe;
    MbariMetaData metadata;
    Image<PixRGB<byte> > clampedImg;
    Image<PixRGB<byte> > img;
    Image<PixRGB<byte> > prevImg;
    Image<PixRGB<byte> > segmentImg;
    Image<byte> mask;
} ImageData;

#endif
