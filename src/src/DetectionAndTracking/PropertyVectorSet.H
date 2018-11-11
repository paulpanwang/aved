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

#ifndef PROPERTYVECTORSET_H_DEFINED
#define PROPERTYVECTORSET_H_DEFINED

#include <list>
#include <string>
#include <vector>

#include "Data/MbariMetaData.H"
#include "Image/BitObject.H"

template <class T> class Image;
template <class T> class PixRGB;

// ######################################################################
//! class that contains a set of property vectors for events
/*! each property vector contains the following fields:<br>
  <table><tr><td><b>index</b><td><b>content</b></th>
  <tr><td>0<td>event number
  <tr><td>1<td>interestingness index
  <tr><td>2<td>area
  <tr><td>3<td>Uxx
  <tr><td>4<td>Uyy
  <tr><td>5<td>Uxy
  <tr><td>6<td>major Axis
  <tr><td>7<td>minor Axis
  <tr><td>8<td>elongation
  <tr><td>9<td>orientation angle
  <tr><td>10<td>maximum intensity
  <tr><td>11<td>minimum intensity
  <tr><td>12<td>average intensity</table>*/
class PropertyVectorSet
{
public:

  //! default contructor
  PropertyVectorSet();

  //! default destructor
  ~PropertyVectorSet();

  //! read from stream constructor
  PropertyVectorSet(std::istream& is);

  //! write the entire PropertyVectorSet to the output stream os
  void writeToStream(std::ostream& os);

  //! write the PropertyVectorSet header to the output stream os
  void writeHeaderToStream(std::ostream& os);

  //! read the PropertyVectorSet from the input stream is
  void readFromStream(std::istream& is);

  //! return the PropertyVector for an event by its number
  std::vector<float> getPropertyVectorForEvent(const int num);

  //! the vector of property vectors
  std::vector< std::vector<float> > itsVectors;
};


#endif
