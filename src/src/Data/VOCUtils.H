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

/*!@file Logger.H methods to parse PASCAL VOC formatted .xml <object> values
 * rescales according to the --rescale-input=WxH argument */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#include <list>  // Change into a diff DS or make our own class
#include <string>
#include <fstream>
#include <sstream>
#include <boost/algorithm/string.hpp>

#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/parsers/AbstractDOMParser.hpp>
#include <xercesc/dom/DOM.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>

#include "Media/FrameState.H"
#include "Image/OpenCVUtil.H"
#include "Channels/ChannelOpts.H"
#include "Component/GlobalOpts.H"
#include "Component/ModelManager.H"
#include "Component/JobServerConfigurator.H"
#include "Features/HistogramOfGradients.H"
#include "Image/FilterOps.H"    // for lowPass3y()
#include "Image/Kernels.H"      // for twofiftyfives()
#include "Image/ColorOps.H"
#include "Image/fancynorm.H"
#include "Image/MorphOps.H"
#include "Image/ShapeOps.H"   // for rescale()
#include "Raster/GenericFrame.H"
#include "Raster/PngWriter.H"
#include "Media/FrameRange.H"
#include "Media/FrameSeries.H"
#include "Media/SimFrameSeries.H"
#include "Media/MediaOpts.H"
#include "Neuro/SpatialMetrics.H"
#include "Neuro/StdBrain.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/Retina.H"
#include "Neuro/VisualCortex.H"
#include "Neuro/SimulationViewerStd.H"
#include "SIFT/Histogram.H"
#include "Simulation/SimEventQueueConfigurator.H"
#include "Util/sformat.H"
#include "Util/StringConversions.H"
#include "Util/Pause.H"
#include "Data/Logger.H"
#include "Data/MbariMetaData.H"
#include "Data/MbariOpts.H"

#include "DetectionAndTracking/FOEestimator.H"
#include "DetectionAndTracking/VisualEventSet.H"
#include "DetectionAndTracking/DetectionParameters.H"
#include "DetectionAndTracking/MbariFunctions.H"
#include "DetectionAndTracking/Segmentation.H"
#include "DetectionAndTracking/ColorSpaceTypes.H"
#include "DetectionAndTracking/BoxObjectDetection.H"
#include "DetectionAndTracking/Preprocess.H"
#include "Image/MbariImage.H"
#include "Image/MbariImageCache.H"
#include "Image/BitObject.H"
#include "Image/DrawOps.H"
#include "Image/MathOps.H"
#include "Image/IO.H"
#include "Learn/BayesClassifier.H"
#include "Media/MbariResultViewer.H"
#include "Motion/MotionEnergy.H"
#include "Motion/MotionOps.H"
#include "Motion/OpticalFlow.H"
#include "Util/StringConversions.H"
#include "Utils/Version.H"
#include "Component/ModelManager.H"
#include "DetectionAndTracking/DetectionParameters.H"
#include "Simulation/SimEventQueueConfigurator.H"
#include "DetectionAndTracking/BoxObjectDetection.H"
#include "Data/VOC.H"

using namespace std;
using namespace xercesc;

struct Box {
	int xmin;
	int ymin;
	int xmax;
	int ymax;
};

// global variables
// Purpose: Store percent rescaled
double rescalePercentWidth;
double rescalePercentHeight;

// Main Functions
// 	- Stores value in temporary 'BoundingBox' VOCObject
void storeValue(Box& temp, const string& tagName, const string& tagValue) {    // Stores value in temporary 'VOCObject' VOCObject
	// Convert Bounding Box Value from 'string to int'
	istringstream is(tagValue);
	int dim = 0;
	is >> dim;

	if(tagName == "xmin") {
		temp.xmin = dim;
	} else if(tagName == "ymin") {
		temp.ymin = dim;
	} else if(tagName == "xmax") {
		temp.xmax = dim;
	} else if(tagName == "ymax") {
		temp.ymax = dim;
	}
}

int getRescaleValues(const string& value){
	istringstream is(value);
	int val = 0;
	is >> val;

	return val;
}

// 	- Get .xml in directory
string getmyXML(const string& descrip, const uint& fnum){
	vector<string>image_path;

	boost::split(image_path, descrip, boost::is_any_of("/"));

	vector<string>filename;
	boost::split(filename, image_path[image_path.size()-1], boost::is_any_of("#"));

	string fs = sformat("%s%06d.xml", filename[0].c_str() , fnum);

	string xml_path="";
	for(int i=1; i< image_path.size()-1; i++){
		xml_path.append(image_path[i]);
		xml_path.append("/");
	}
	xml_path.append(fs);
	return xml_path;
}
// 	- Reads .xml for values in <size> ... </size>
void getImageSize(XercesDOMParser *itsParser, const Dims& dims){
	DOMNodeList *list = NULL;
	DOMDocument *domDocParser = itsParser->getDocument();
	XMLCh *source = XMLString::transcode("size"); 			// Tag wanted
	list = domDocParser->getElementsByTagName(source);		// Returns list of '<tag>' found
	int originalWidth = 0, originalHeight = 0;
	for(int i = 0; i < list->getLength(); ++i){
		DOMNode *node = list->item(i); 						// Gets the ith <object> in the list
		DOMNodeList *length = node->getChildNodes(); 		// Lines counted, including: "<object> ... </object>" = 13 lines total
		for(int k = 0; k < length->getLength(); ++k) {
			DOMNode *childNode = length->item(k);

			if(childNode->getNodeType() == DOMNode::ELEMENT_NODE) {						// Ensures we found a <tag>
				string tagNameObj = XMLString::transcode(childNode->getNodeName());		// <Gets Tag Name>
				string tagValueObj = XMLString::transcode(childNode->getTextContent());	// <tag> Gets Value </tag>
				istringstream is(tagValueObj);
				if(tagNameObj == "width"){
					is >> originalWidth;
					LDEBUG("Original Width: %i", originalWidth);
				} else if(tagNameObj =="height"){
					is >> originalHeight;
					LDEBUG("Original Height: %i", originalHeight);
				}
			}
		}
	}

	// Ensures rescaled <bndbox> size, otherwise, no change to original size
	if(originalWidth != dims.w() && originalHeight != dims.h()){
		double result = (1.0 - (double(originalWidth - dims.w())/originalWidth));
		rescalePercentWidth = result;
		rescalePercentHeight = (1.0 - (double(originalHeight - dims.h())/originalHeight)); 
	} else{
		rescalePercentWidth = 1.0;
		rescalePercentHeight = 1.0;
	}
}

// Reads .xml for values in <object> ... </object>
void getObjectValues(XercesDOMParser *itsParser, list<VOCObject>& objectList, const Dims& dims) {
	DOMNodeList *list = NULL;
	DOMDocument *domDocParser = itsParser->getDocument();

	// How many instances of the '<tag>' found
	XMLCh *source = XMLString::transcode("object"); 		// Tag wanted
	list = domDocParser->getElementsByTagName(source);		// Returns list of '<tag>' found

	// Parse through each object to grab values
	for(int i = 0; i < list->getLength(); ++i) {

		DOMNode *node = list->item(i); 						// Gets the ith <object> in the list
		DOMNodeList *length = node->getChildNodes(); 		// Lines counted, including: "<object> ... </object>" = 13 lines total

		Box tempDim;
		VOCObject tempVOCObject;
		Rectangle invalidRect;
		tempVOCObject.setDimensions(invalidRect);

		// Iterate through each <tag> in <object> ... </object> to retrieve values
		for(int k = 0; k < length->getLength(); ++k) {
			DOMNode *childNode = length->item(k);

			if(childNode->getNodeType() == DOMNode::ELEMENT_NODE) {						// Ensures we found a <tag>
				string tagNameObj = XMLString::transcode(childNode->getNodeName());		// <Gets Tag Name>
				string tagValueObj = XMLString::transcode(childNode->getTextContent());	// <tag> Gets Value </tag>

				// Grab bounding box dimensions, otherwise, get the <name>
				if(tagNameObj == "bndbox") {
					DOMNodeList *dimensions = childNode->getChildNodes();						// Gets all the <tags> in <bndbox>

					for(int j = 0; j < dimensions->getLength(); ++j) {							// Iterate each for dim. value
						DOMNode *dim = dimensions->item(j);

						if(dim->getNodeType() == DOMNode::ELEMENT_NODE) {
							string tagNameBB = XMLString::transcode(dim->getNodeName());		// <Gets Tag Name>
							string tagValueBB = XMLString::transcode(dim->getTextContent());	// <tag> Gets Value </tag>
							storeValue(tempDim, tagNameBB, tagValueBB);							// Store the dim values 1 by 1
						}
					}
					Rectangle tmpBbox = Rectangle::tlbrO(
							int((float)tempDim.ymin * rescalePercentHeight),
							int((float)tempDim.xmin * rescalePercentWidth),
							int((float)tempDim.ymax * rescalePercentHeight),
							int((float)tempDim.xmax * rescalePercentWidth));
					Rectangle bbox = tmpBbox.getOverlap(Rectangle(Point2D<int>(0, 0), dims - 1));
					LINFO("Bounding box %s %d %d %d %d rescale width %f %% Height %f %%",
							toStr(bbox).data(), tempDim.xmin, tempDim.xmax, tempDim.ymin, tempDim.ymax, rescalePercentWidth, rescalePercentHeight);
					tempVOCObject.setDimensions(bbox);

					LDEBUG("Updated - W: %i H: %i ", int(tempDim.xmin * rescalePercentWidth), int(tempDim.ymin * rescalePercentHeight));
					break;
				} else if(tagNameObj =="name"){
					LDEBUG("Tag Value - Class Name: %s", tagValueObj.c_str());
					tempVOCObject.setName(tagValueObj);

				} else if(tagNameObj == "confidence") {

					// Convert Confidence value from 'string to int'
					istringstream is(tagValueObj);
					double con = 0;
					is >> con;

					tempVOCObject.setProbability(con);
				}

			}
		}

		if(tempVOCObject.getDimensions().isValid())
			objectList.push_back(tempVOCObject);
	}

	LDEBUG("#### VOC Object List Size: %lu #####", objectList.size());
}

