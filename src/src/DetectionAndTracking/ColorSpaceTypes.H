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
#ifndef COLORSPACETYPES_H_DEFINED
#define	COLORSPACETYPES_H_DEFINED

#include <string>

  // ! Color spaces types used for determining whether color is used in saliency computation
enum ColorSpaceType {
  SAColorRGB = 0,
  SAColorYCBCR = 1,
  SAColorGray = 2
  // if you add a new type here, also update the names in the function below!
};
//! number of color types
#define NCOLOR_TYPES 3

//! Returns name of color space
inline const char* colorSpaceType(const ColorSpaceType p)
{
  static const char n[NCOLOR_TYPES][25] = {
    "RGB", "YCBCR", "Gray"};
  return n[int(p)];
}

//! SegmentAlgorithmType overload */
void convertToString(const ColorSpaceType val,
                     std::string& str);

//! segmentAlgorithmInputImageType overload */
void convertFromString(const std::string& str, ColorSpaceType& val);

#endif	/* COLORSPACETYPES_H_DEFINED */
