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

#ifndef BITOBJECTDRAWMODES_H_DEFINED
#define BITOBJECTDRAWMODES_H_DEFINED

#include <string>

//! Possible BitObject draw modes
enum BitObjectDrawMode {
  BODMnone    = 0,   //! don't draw the BitObject
  BODMshape   = 1,   //! draw the shape of the BitObject
  BODMoutline = 2,   //! draw the outline of the BitObject
  BODMbbox    = 3    //! draw the bounding box of the BitObject
  // if you add a new mode here, also update the names in the function below!
};

//! number of known BitObject draw modes:
#define NBBITOBJECTDRAWMODES 4

//! Returns name of given BitObject draw mode
inline const char* bitObjectDrawModeName(const BitObjectDrawMode p)
{
  static const char n[NBBITOBJECTDRAWMODES][15] = {
    "None", "Shape", "Outline", "BoundingBox" };
  return n[int(p)];
}


//! BitObjectDrawMode overload
/*! Format is "name" as defined in BitObjectDrawModes.H */
void convertToString(const BitObjectDrawMode val,
                     std::string& str);
//! BitObjectDrawMode overload
/*! Format is "name" as defined in BitObjectDrawModes.H */
void convertFromString(const std::string& str, BitObjectDrawMode& val);

#endif
