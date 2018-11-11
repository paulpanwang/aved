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

/*!@file PropertyVectorSet.C  - TODO: This can probably be removed */

#include "Image/OpenCVUtil.H"
#include "DetectionAndTracking/PropertyVectorSet.H"
#include "Image/ColorOps.H"
#include "Image/DrawOps.H"
#include "Image/Image.H"
#include "Image/Rectangle.H"
#include "Image/ShapeOps.H"
#include "Image/Transforms.H"
#include "Image/colorDefs.H"
#include "Util/Assert.H"
#include "Util/StringConversions.H"
#include "DetectionAndTracking/MbariFunctions.H"
#include "Image/Geometry2D.H"
#include <algorithm>
#include <istream>
#include <ostream>

using namespace std;

// ######################################################################
// ####### PropertyVectorSet
// ######################################################################
PropertyVectorSet::PropertyVectorSet()
{}

// ######################################################################
PropertyVectorSet::~PropertyVectorSet()
{
  vector< vector<float> >::iterator i;
  for (i= itsVectors.begin(); i != itsVectors.end(); ++i)
    i->clear();

  itsVectors.clear();
}
// ######################################################################
PropertyVectorSet::PropertyVectorSet(istream& is)
{
  readFromStream(is);
}
// ######################################################################
void PropertyVectorSet::writeHeaderToStream(ostream& os)
{
  uint s2;
  if (itsVectors.empty()) s2 = 0;
  else s2 = itsVectors.front().size();

  os << itsVectors.size() << " " << s2 << "\n";
}
// ######################################################################
void PropertyVectorSet::writeToStream(ostream& os)
{
  uint s2;
  if (itsVectors.empty()) s2 = 0;
  else s2 = itsVectors.front().size();

  for (uint i = 0; i < itsVectors.size(); ++i)
    {
      LDEBUG("Writing property vector set %d to stream", i);
      for (uint j = 0; j < s2; ++j)
        os << itsVectors[i][j] << " ";

      os << "\n";
    }
}

// ######################################################################
void PropertyVectorSet::readFromStream(istream& is)
{
  uint s1, s2;
  is >> s1; is >> s2;
  itsVectors = vector< vector<float> > (s1, vector<float>(s2));

  for (uint i = 0; i < s1; ++i)
    for (uint j = 0; j < s2; ++j)
      is >> itsVectors[i][j];
}

// ######################################################################
vector<float> PropertyVectorSet::getPropertyVectorForEvent(const int num)
{
  for (uint i = 0; i < itsVectors.size(); ++i)
    if ((int)(itsVectors[i][0]) == num) return itsVectors[i];

  LFATAL("property vector for event number %d not found!", num);
  return vector<float>();
}
