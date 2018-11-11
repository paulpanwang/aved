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

#ifndef MBARI_METADATA_H
#define MBARI_METADATA_H

#include <cstring>
#include <fstream>
#include <iostream>
#include <unistd.h>

// ######################################################################
//! Super class that manages MBARI specific metadata like timecode
class MbariMetaData
{
public:

  //! Constructor
  MbariMetaData() ;

  //! Constructor. Copies from existing MbariMetaData Class
  MbariMetaData( const MbariMetaData& m );

  //! Constructor. Create from string - used to parse string from
  // image file headers
  MbariMetaData( std::string s );

  // ! copy operator
  MbariMetaData & operator=(const MbariMetaData& m);

  //! read the MbariMetadata from the input stream is
  void readFromStream(std::istream& is);

  //! write the MbariMetadata to the output stream os
  void writeToStream(std::ostream& os);

  // ######################################################################
  inline std::string getTC() const { return tc; }

  // ######################################################################
  inline void setMetaData( std::string s ) { parseMetaData( s ); }
  
private:
  void parseMetaData( std::string s );  
  std::string tc;   // time code string
};
#endif
