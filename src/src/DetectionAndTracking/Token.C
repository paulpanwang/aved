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

/*!@file Token.C classes useful for event tracking */

#include "Image/OpenCVUtil.H"
#include "DetectionAndTracking/Token.H"

#include <algorithm>
#include <istream>
#include <ostream>

using namespace std;

// ######################################################################
// ###### Token
// ######################################################################
Token::Token()
  : bitObject(),
    location(),
    prediction(),
    line(),
    angle(0.0F),
    foe(0.0F,0.0F),
    frame_nr(0),
    written(false)
{}
// ######################################################################
Token::Token (BitObject bo, uint frame)
  : bitObject(bo),
    location(bo.getCentroidXY()),
    prediction(),
    line(),
    angle(0.0F),
    foe(0.0F,0.0F),
    frame_nr(frame),
    written(false)
  {
  }
  // ######################################################################
  Token &Token::operator=(const Token& tk) {
      this->angle = tk.angle;
      this->foe = tk.foe;
      this->line = tk.line;
      this->bitObject = tk.bitObject;
      this->location = tk.location;
      this->prediction = tk.prediction;
      this->written = tk.written;
      this->featureJETred = tk.featureJETred;
      this->featureJETgreen = tk.featureJETgreen;
      this->featureJETblue = tk.featureJETblue;
      this->featureHOG8 = tk.featureHOG8;
      this->featureHOG3 = tk.featureHOG3;
  return *this;
  }
  // ######################################################################
  Token::Token(BitObject bo, uint frame, const MbariMetaData& m,
               vector<double> &featureJETred,
               vector<double> &featureJETgreen,
               vector<double> &featureJETblue,
               vector<double> &featureHOG3, vector<double> &featureHOG8)
  : bitObject(bo),
    location(bo.getCentroidXY()),
    prediction(),
    line(),
    angle(0.0F),
    foe(0.0F,0.0F),
    featureJETred(featureJETred),
    featureJETgreen(featureJETgreen),
    featureJETblue(featureJETblue),
    featureHOG3(featureHOG3),
    featureHOG8(featureHOG8),
    frame_nr(frame),
    mbarimetadata(m),
    written(false)
{ }

// ######################################################################
Token::Token (istream& is)
{
  readFromStream(is);
}

// ######################################################################
void Token::writeToStream(ostream& os)
{
  if(written == false) {
    os << frame_nr << ' ';
    mbarimetadata.writeToStream(os);
    location.writeToStream(os);
    prediction.writeToStream(os);
    line.writeToStream(os);
    os << angle << '\n';
    bitObject.writeToStream(os);
    os << "\n";
    foe.writeToStream(os);
    //TODO: add feature
  }
  written = true;
}

// ######################################################################
void Token::readFromStream(istream& is)
{
  is >> frame_nr;
  mbarimetadata.readFromStream(is);
  location.readFromStream(is);
  prediction.readFromStream(is);
  line.readFromStream(is);
  is >> angle;
  bitObject = BitObject(is);
  foe.readFromStream(is);
  //TODO: add feature
}

// ######################################################################
void Token::writePosition(ostream& os) const
{
  location.writeToStream(os);
}
