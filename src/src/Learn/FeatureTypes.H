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

#ifndef FEATURETYPES_H_DEFINED
#define	FEATURETYPES_H_DEFINED

#include <string>

// ! Feature type used in Bayes network for classifying objects
enum FeatureType {
  FT_HOG3 = 0,
  FT_HOG8 = 1,
  FT_MBH3 = 2,
  FT_MBH8 = 3,
  FT_JET = 4
  // if you add a new type here, also update the names in the function below!
};
//! number of feature types
#define NFEATURE_TYPES 5

//! Returns name of feature type
inline const char* featureType(const FeatureType p)
{
  static const char n[NFEATURE_TYPES][6] = {
    "HOG3", "HOG8", "MBH3", "MBH8", "JET"};
  return n[int(p)];
};

//! featureType overload */
void convertToString(const FeatureType val, std::string& str);

//! featureType overload */
void convertFromString(const std::string& str, FeatureType& val);

#endif	/* _FEATURETYPES_H */
