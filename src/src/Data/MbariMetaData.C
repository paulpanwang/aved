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

#include "Data/MbariMetaData.H"
#include "Util/StringConversions.H"
#include "Util/log.H"
#
#include <cstring>
#include <fstream>
#include <iostream>
#include <unistd.h>

using namespace std;


// ######################################################################
MbariMetaData::MbariMetaData()
{}
// ###################################################################### 
MbariMetaData::MbariMetaData( const MbariMetaData& m ) :
  tc(m.tc)
{}
// ######################################################################  
MbariMetaData::MbariMetaData( string s )
{
  parseMetaData( s );
}
// ######################################################################
MbariMetaData & MbariMetaData::operator=(const MbariMetaData& m)
{
  tc = m.getTC();
  return *this;
}
// ######################################################################
void MbariMetaData::writeToStream(std::ostream& os)
{
  os << tc;
  os << '\n';
}

// ######################################################################
void MbariMetaData::readFromStream(std::istream& is)
{  
  is >> tc;
}

// ######################################################################  
void MbariMetaData::parseMetaData( string s ) {
  string temp = "";
  int space;    
  
  while((space = s.find(" ", 0)) && space != (int)(string::npos)) {            
    temp = s.substr( 0, space );
    s = s.substr( space, s.length() - space );
    if( temp.find("TIMECODE:", 0 ) != string::npos ){
      tc = s.substr(1, s.length()); 
    }      
  }
}
