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

/*!@file TrackingModes.H used for event tracking */

#ifndef TRACKINGMODES_H_DEFINED
#define TRACKINGMODES_H_DEFINED

#include <string>

// ! Tracking mode used for visual event tracking 
enum TrackingMode {
  TMKalmanFilter = 0, //! Kalman filter - assumes constant motion
  TMNearestNeighbor = 1,  //! Tracking nearest neighbor - good for non-motion
  TMHough = 2,  //! Hough-based tracking  - good for deformable bodies in slow video
  TMNearestNeighborHough = 3,  //! Combine nearest neighbor and Hough good for start/stop motion and/or poor video quality
  TMKalmanHough = 4,  //! Combine Kalman and Hough tracker
  TMNone = 5, //! No tracking - use for still frame images
  // if you add a new mode here, also update the names in the function below!
};
//! number of event tracking modes:
#define NTRACKINGMODES 6

//! Returns name of tracking mode
inline const char* trackingModeName(const TrackingMode p)
{
  static const char n[NTRACKINGMODES][25] = {
    "KalmanFilter", "NearestNeighbor", "Hough", "NearestNeighborHough", "KalmanFilterHough", "None"};
  return n[int(p)];
}

//! TrackingMode overload
/*! Format is "name" as defined in TrackingModes.H */
void convertToString(const TrackingMode val,
                     std::string& str);

//! TrackingMode overload
/*! Format is "name" as defined in TrackingModes.H */
void convertFromString(const std::string& str, TrackingMode& val);


#endif
