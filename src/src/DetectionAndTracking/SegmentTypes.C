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

#include "DetectionAndTracking/SegmentTypes.H"
#include "Util/StringConversions.H"
#include "Util/log.H"

void convertFromString(const std::string& str, SegmentAlgorithmType& val)
{
  // CAUTION: assumes types are numbered and ordered!
  for (int i = 0; i < NSEGMENT_ALGORITHMS; i ++)
    if (str.compare(segmentAlgorithmType(SegmentAlgorithmType(i))) == 0)
      { val = SegmentAlgorithmType(i); return; }

  conversion_error::raise<SegmentAlgorithmType>(str);
}

std::string convertToString(const SegmentAlgorithmType val)
{ return segmentAlgorithmType(val); }

void convertFromString(const std::string& str, SegmentAlgorithmInputImageType& val) {
  // CAUTION: assumes types are numbered and ordered!
  for (int i = 0; i < NSEGMENT_ALGORITHM_INPUT_IMAGE_TYPES; i ++)
    if (str.compare(segmentAlgorithmInputImageType(SegmentAlgorithmInputImageType(i))) == 0)
      { val = SegmentAlgorithmInputImageType(i); return; }

  conversion_error::raise<SegmentAlgorithmInputImageType>(str);
}

std::string convertToString(const SegmentAlgorithmInputImageType val)
{ return segmentAlgorithmInputImageType(val); }

