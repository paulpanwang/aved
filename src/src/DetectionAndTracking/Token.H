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
  
#ifndef TOKEN_H_DEFINED
#define TOKEN_H_DEFINED

#include <list>
#include <string>
#include <vector>

#include "Data/MbariMetaData.H"
#include "Image/BitObject.H"

template <class T> class Image;
template <class T> class PixRGB;

#define  DEFAULT_CLASS_NAME "Unknown"

// ######################################################################
//! public class that contains information for a visual token for tracking
class Token
{
public:

  //!default constructor
  Token();

  //!constructor with the location being the centroid of the BitObject
  Token (BitObject bo, uint frame);

  //!constructor with the location being the centroid of the BitObject
  Token (BitObject bo, uint frame, const MbariMetaData& m,
         std::vector<double> &featureJETred,
         std::vector<double> &featureJETgreen,
         std::vector<double> &featureJETblue,
         std::vector<double> &featureHOG3, std::vector<double> &featureHOG8);

  //!read the Token from the input stream is
  Token (std::istream& is);

  //!the BitObject that holds info about the shape of this Token
  BitObject bitObject;

  //!the x and y coordinates of the token in the frame
  Vector2D location;

  //!the expected location for this token
  Vector2D prediction;

  //! feature for this token to use with a classifier
  //TODO: refactor into class
  std::vector<double> featureHOG3;
  std::vector<double> featureHOG8;
  std::vector<double> featureJETred;
  std::vector<double> featureJETgreen;
  std::vector<double> featureJETblue;

  //!the straight line on which this token is moving
  StraightLine2D line;

  //!the angle between line and the connection to the FOE
  float angle;

  //!the focus of expansion (FOE)  for this token
  Vector2D foe;

  //!the frame number at which the token occurred
  uint frame_nr;

  //!the Mbari meta data for this frame token
  MbariMetaData mbarimetadata;

  //! write flag; defaults to false in constructor; set to false
  // in writeToStream
  bool written;

  //! write the entire Token to the output stream os
  void writeToStream(std::ostream& os);

  //! read the Token from the input stream is
  void readFromStream(std::istream& is);

  //! write the Token's position to the output stream os
  void writePosition(std::ostream& os) const;

   //! copy operator
  Token & operator=(const Token& tk);
};

#endif
