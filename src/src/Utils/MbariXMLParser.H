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

#ifndef MBARI_XML_PARSER_H_
#define MBARI_XML_PARSER_H_

#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/parsers/AbstractDOMParser.hpp>
#include <xercesc/dom/DOM.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>

#include <string>
#include <iostream>
#include <vector>
#include <list>

#include "Utils/ErrReporter.H"
#include "Util/Types.H"
#include "DetectionAndTracking/DetectionParameters.H"

class VisualEvent;

// ######################################################################
//! Includes all the functions to create and parse a AVED XML DOM document.
class MbariXMLParser {
public:
	//! default Constructor. Locates the AVED schemas and creates the DOM writer object
	MbariXMLParser(std::string binDir);

	//! default Destructor. Deletes all private objects and terminates XML writers.
	virtual ~MbariXMLParser();

	void display();

	int displayElements(xercesc::DOMNode *n, unsigned int nbr_child);

	//! Creates a DOM document
	/*! @param versionString the version string that represents the binary
	  @param fr frame range of this event sequence
	  @param timecodefirst starting timecode of this event sequence, e.g. in 00:01:12
	  @param timecodelast ending timecode of this event sequence, e.g. in 00:01:12
	  @param params detection parameters
	  * */
	void creatDOMDocument(std::string versionString,
						  int startFrame,
						  int endFrame,
						  std::string timecodefirst,
						  std::string timecodelast);

	//! Adds source meta data to the DOM output
	/*! @param metadataXMLFile the input xml file name that contains the source metadata
	**/
	void addSourceMetaData(std::string metadataXMLFile);

	void importSourceMetadata(std::string inputXML);

	void addDetectionParameters(DetectionParameters params);

	void add(bool saveNonInterestingEvents,
			 std::list<VisualEvent *> &eventList,
			 int eventframe,
			 std::string eventframetimecode,
			 float scaleW, float scaleH);

	void writeDocument(std::string path);

	bool isXMLValid(std::string inputXML);

private:

	xercesc::DOMWriter *itsDOMWriter;
	xercesc::DOMImplementation *impl;
	xercesc::DOMDocument *itsXMLdoc;
	xercesc::XMLFormatTarget *itsXMLFileFormatTarget;
	xercesc::XercesDOMParser *itsParser;
	ErrReporter *itsErrHandler;
	std::string itsEventDataSchemaLocation;
	std::string itsSourceMetadataSchema;

	//! Generic parser that parses and xml file based on give input schema. Used for testing.
	xercesc::DOMDocument *parseXMLFile(std::string inputXML, std::string inputSchema);

	std::string strcatX(std::string str, unsigned int x);
};
#endif /*MBARI_XML_PARSER_H_*/
