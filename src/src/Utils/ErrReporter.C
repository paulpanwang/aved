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

#include <xercesc/sax/SAXParseException.hpp>
#include <iostream>
#include <string>

#include "Utils/ErrReporter.H"

using namespace std;
XERCES_CPP_NAMESPACE_USE;

void ErrReporter::warning(const SAXParseException&) {
}

void ErrReporter::error(const SAXParseException& mess) {
  fSawErrors = true;
  cerr << "Error in \"" << XMLString::transcode(mess.getSystemId())
       << "\", row " << mess.getLineNumber()
       << ", column " << mess.getColumnNumber()
       << "\nError message : " << XMLString::transcode(mess.getMessage()) << endl;
}

void ErrReporter::fatalError(const SAXParseException& mess) {
  fSawErrors = true;
  cerr << "Fatal Error in \"" << XMLString::transcode(mess.getSystemId())
       << "\", row " << mess.getLineNumber()
       << ", column " << mess.getColumnNumber()
       << "\nError message : " << XMLString::transcode(mess.getMessage()) << endl;
}

void ErrReporter::resetErrors() {
  fSawErrors = false;
}
