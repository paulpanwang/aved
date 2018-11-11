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
 * This code requires the The iLab Neuromorphic  C++ Toolkit developed
 * by the University of Southern California (USC) and the iLab at USC. 
 * See http://iLab.usc.edu for information about this project. 
 *  
 * This work would not be possible without the generous support of the 
 * David and Lucile Packard Foundation
 */

#include "Image/OpenCVUtil.H"
#include "Utils/MbariXMLParser.H"
#include "DetectionAndTracking/VisualEvent.H"
#include "DetectionAndTracking/Token.H"

#include <xercesc/util/OutOfMemoryException.hpp>
#include <xercesc/framework/LocalFileFormatTarget.hpp>
#include <xercesc/util/XMLDateTime.hpp>
#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;

MbariXMLParser::MbariXMLParser(string binDir) :
        itsXMLdoc(NULL),
        itsDOMWriter(NULL),
        itsErrHandler(NULL),
        itsParser(NULL) {
  ifstream file;

  if (getenv("AVED_BIN")) {
    itsEventDataSchemaLocation = string(string(getenv("AVED_BIN")) + string("/schema/EventDataSet.xsd"));
    itsSourceMetadataSchema = string(string(getenv("AVED_BIN")) + string("/schema/SourceMetadata.xsd"));
  }
  else {
    itsEventDataSchemaLocation = binDir + string("/schema/EventDataSet.xsd");
    itsSourceMetadataSchema = binDir + string("/schema/SourceMetadata.xsd");
  }

  try {
    xercesc::XMLPlatformUtils::Initialize();
  } catch (const xercesc::XMLException &toCatch) {
    char *pMsg = xercesc::XMLString::transcode(toCatch.getMessage());
    LERROR("Error during Xerces-c Initialization. Exception message: %s", pMsg);
    xercesc::XMLString::release(&pMsg);
  }
  // check for the existance of the schema file before starting
  file.open(itsEventDataSchemaLocation.c_str(), ifstream::in);
  if (file.is_open() == false)
    LFATAL("Error - cannot find the schema file %s. Check the environment variable AVED_BIN.",
           itsEventDataSchemaLocation.c_str());
  file.close();

  // check for the existance of the schema file before starting
  file.open(itsSourceMetadataSchema.c_str(), ifstream::in);
  if (file.is_open() == false)
    LFATAL("Error - cannot find the schema file %s. Check the environment variable AVED_BIN",
           itsSourceMetadataSchema.c_str());
  file.close();

  XMLCh *value = xercesc::XMLString::transcode("LS");
  impl = xercesc::DOMImplementationRegistry::getDOMImplementation(value);
  xercesc::XMLString::release(&value);

  if (impl != 0) {
    try {

      // Create the writer
      itsDOMWriter = ((xercesc::DOMImplementationLS *) impl)->createDOMWriter();

      if (itsDOMWriter->canSetFeature(xercesc::XMLUni::fgDOMWRTSplitCdataSections, true))
        itsDOMWriter->setFeature(xercesc::XMLUni::fgDOMWRTSplitCdataSections, true);

      if (itsDOMWriter->canSetFeature(xercesc::XMLUni::fgDOMWRTFormatPrettyPrint, true))
        itsDOMWriter->setFeature(xercesc::XMLUni::fgDOMWRTFormatPrettyPrint, true);

    } catch (const xercesc::OutOfMemoryException &) {
      LERROR("OutOfMemoryException");
    } catch (const xercesc::DOMException &e) {
      LERROR("DOMException code is:  %d ", e.code);
    }
  }
}

MbariXMLParser::~MbariXMLParser() {
  if (itsXMLdoc != NULL)
    delete itsXMLdoc;
  if (impl != NULL)
    delete impl;
  if (itsDOMWriter != NULL)
    delete itsDOMWriter;
  if (itsErrHandler != NULL)
    delete itsErrHandler;
  if (itsParser != NULL)
    delete itsParser;
  xercesc::XMLPlatformUtils::Terminate();
}

void MbariXMLParser::creatDOMDocument(string versionString,
                                      int startFrame,
                                      int endFrame,
                                      string startTimeCode,
                                      string endTimeCode) {
  try {

    // Create the document
    XMLCh *rootvalue = xercesc::XMLString::transcode("EventDataSet");
    XMLCh *rootnamespace = xercesc::XMLString::transcode("http://www.w3.org/2001/XMLSchema-instance");
    itsXMLdoc = impl->createDocument(NULL, rootvalue, NULL);
    XMLCh *xmlnsxsi = xercesc::XMLString::transcode("xmlns:xsi");
    itsXMLdoc->getDocumentElement()->setAttributeNS(xercesc::XMLUni::fgXMLNSURIName, xmlnsxsi, rootnamespace);
    xercesc::XMLString::release(&xmlnsxsi);
    xercesc::XMLString::release(&rootvalue);
    xercesc::XMLString::release(&rootnamespace);

    itsXMLdoc->setStandalone(false);

    xercesc::DOMElement *root = itsXMLdoc->getDocumentElement();

    // Add some comments
    string commentstring = "Created by: MBARI software " + versionString;
    XMLCh *comment = xercesc::XMLString::transcode(commentstring.c_str());
    xercesc::DOMComment *domcomment = itsXMLdoc->createComment(comment);
    root->appendChild(domcomment);
    xercesc::XMLString::release(&comment);

    char datestamp[24];
    time_t time_of_day;
    time_of_day = time(NULL);
    strftime(datestamp, 24, "%Y-%m-%d %X %Z", localtime(&time_of_day));

    XMLCh *creationdatestring = xercesc::XMLString::transcode("CreationDate");
    XMLCh *creationdate = xercesc::XMLString::transcode(datestamp);
    root->setAttribute(creationdatestring, creationdate);
    xercesc::XMLString::release(&creationdatestring);
    xercesc::XMLString::release(&creationdate);

    ostringstream smin, smax;
    // if conversion from frame to string worked,
    if (smin << startFrame) {
      XMLCh *startframe = xercesc::XMLString::transcode("StartFrame");
      XMLCh *startframevalue = xercesc::XMLString::transcode(smin.str().c_str());
      root->setAttribute(startframe, startframevalue);
      xercesc::XMLString::release(&startframe);
      xercesc::XMLString::release(&startframevalue);
    }
    if (smax << endFrame) {
      XMLCh *endframe = xercesc::XMLString::transcode("EndFrame");
      XMLCh *endframevalue = xercesc::XMLString::transcode(smax.str().c_str());
      root->setAttribute(endframe, endframevalue);
      xercesc::XMLString::release(&endframe);
      xercesc::XMLString::release(&endframevalue);
    }
    if (startTimeCode.length() > 0) {
      XMLCh *starttimecode = xercesc::XMLString::transcode("StartTimecode");
      XMLCh *starttimecodevalue = xercesc::XMLString::transcode(startTimeCode.c_str());
      root->setAttribute(starttimecode, starttimecodevalue);
      xercesc::XMLString::release(&starttimecode);
      xercesc::XMLString::release(&starttimecodevalue);
    }
    if (endTimeCode.length() > 0) {
      XMLCh *endtimecode = xercesc::XMLString::transcode("EndTimecode");
      XMLCh *endtimecodevalue = xercesc::XMLString::transcode(endTimeCode.c_str());
      root->setAttribute(endtimecode, endtimecodevalue);
      xercesc::XMLString::release(&endtimecode);
      xercesc::XMLString::release(&endtimecodevalue);
    }
  }
  catch (const xercesc::DOMException &err) {
    LFATAL("Error during the creation of the XML document ( DOM Exception ) :\n");
  }
}

void MbariXMLParser::addSourceMetaData(string inputXML) {

  xercesc::DOMDocument *sourcemetadata = NULL;
  sourcemetadata = parseXMLFile(inputXML, itsSourceMetadataSchema);

  if (sourcemetadata != NULL) {
    XMLCh *source = xercesc::XMLString::transcode("SourceMetadata");
    xercesc::DOMNodeList *list = sourcemetadata->getElementsByTagName(source);

    if (list != NULL) {
      xercesc::DOMNode *node = list->item(0);
      xercesc::DOMNode *newRoot = itsXMLdoc->importNode(node, false);
      itsXMLdoc->getDocumentElement()->appendChild(newRoot);
      display();
    }
    xercesc::XMLString::release(&source);
    delete sourcemetadata;
  } else {
    LINFO("Warning: the source metadata file given is not found or not valid, data will not be imported. \
AVED metadata should be formatted to the following schema: %s)", itsSourceMetadataSchema.c_str());
  }
}

bool MbariXMLParser::isXMLValid(string inputXML) {
  xercesc::DOMDocument *sourcemetadata = parseXMLFile(inputXML, itsEventDataSchemaLocation);
  bool ret = (sourcemetadata != NULL);
  delete sourcemetadata;
  return ret;
}

void MbariXMLParser::addDetectionParameters(DetectionParameters params) {

  // Add the DetectionParameters values
  XMLCh *detectionparameters = xercesc::XMLString::transcode("EventDetectionParameters");
  xercesc::DOMElement *eventsroot = itsXMLdoc->createElement(detectionparameters);
  ostringstream cachesizevalue, mineventareavalue, maxeventareavalue, segmentadaptiveparms, \
  maskxposvalue, maskyposvalue, maskwidthvalue, maskheightvalue, maxWTApointsvalue, \
  maxevolvetimevalue, maxframeseventvalue, minframeseventvalue, maxcostvalue, minstdevvalue;
  xercesc::XMLString::release(&detectionparameters);

  // This is actually better to put into DetectionParameters.C for maintenance purposes
  //...but we will leave them here for now. These should be all the model options
  // that one can set via command line.
  if (cachesizevalue << params.itsSizeAvgCache) {
    XMLCh *cachesize = xercesc::XMLString::transcode("CacheSize");
    XMLCh *cachesizexmlstring = xercesc::XMLString::transcode(cachesizevalue.str().c_str());
    eventsroot->setAttribute(cachesize, cachesizexmlstring);
    xercesc::XMLString::release(&cachesize);
    xercesc::XMLString::release(&cachesizexmlstring);
  }
  if (mineventareavalue << params.itsMinEventArea) {
    XMLCh *mineventarea = xercesc::XMLString::transcode("MinEventArea");
    XMLCh *mineventareaxmlstring = xercesc::XMLString::transcode(mineventareavalue.str().c_str());
    eventsroot->setAttribute(mineventarea, mineventareaxmlstring);
    xercesc::XMLString::release(&mineventarea);
    xercesc::XMLString::release(&mineventareaxmlstring);
  }

  if (minstdevvalue << params.itsMinStdDev) {
    XMLCh *minstdev = xercesc::XMLString::transcode("MinStdDev");
    XMLCh *minstdevxmlstring = xercesc::XMLString::transcode(minstdevvalue.str().c_str());
    eventsroot->setAttribute(minstdev, minstdevxmlstring);
    xercesc::XMLString::release(&minstdev);
    xercesc::XMLString::release(&minstdevxmlstring);
  }
  if (maxeventareavalue << params.itsMaxEventArea) {
    XMLCh *maxeventarea = xercesc::XMLString::transcode("MaxEventArea");
    XMLCh *maxeventareaxmlstring = xercesc::XMLString::transcode(maxeventareavalue.str().c_str());
    eventsroot->setAttribute(maxeventarea, maxeventareaxmlstring);
    xercesc::XMLString::release(&maxeventarea);
    xercesc::XMLString::release(&maxeventareaxmlstring);
  }
  XMLCh *trackingmode = xercesc::XMLString::transcode("TrackingMode");
  string tmstring = trackingModeName(params.itsTrackingMode);
  XMLCh *trackingmodexmlstring = xercesc::XMLString::transcode(tmstring.c_str());
  eventsroot->setAttribute(trackingmode, trackingmodexmlstring);
  xercesc::XMLString::release(&trackingmode);
  xercesc::XMLString::release(&trackingmodexmlstring);

  string sgpstring = params.itsSegmentGraphParameters;
  XMLCh *sgpttype = xercesc::XMLString::transcode("SegmentGraphParameters");
  XMLCh *sgpxmlstring = xercesc::XMLString::transcode(sgpstring.c_str());
  eventsroot->setAttribute(sgpttype, sgpxmlstring);
  xercesc::XMLString::release(&sgpttype);
  xercesc::XMLString::release(&sgpxmlstring);

  SegmentAlgorithmInputImageType saitype = params.itsSegmentAlgorithmInputType;
  string saistring = segmentAlgorithmInputImageType(saitype);
  XMLCh *inputtype = xercesc::XMLString::transcode("SegmentAlgorithmInputImageType");
  XMLCh *inputtypexmlstring = xercesc::XMLString::transcode(saistring.c_str());
  eventsroot->setAttribute(inputtype, inputtypexmlstring);
  xercesc::XMLString::release(&inputtype);
  xercesc::XMLString::release(&inputtypexmlstring);

  SegmentAlgorithmType satype = params.itsSegmentAlgorithmType;
  string sastring = segmentAlgorithmType(satype);
  XMLCh *algorithmtype = xercesc::XMLString::transcode("SegmentAlgorithmType");
  XMLCh *algorithmtypexmlstring = xercesc::XMLString::transcode(sastring.c_str());
  eventsroot->setAttribute(algorithmtype, algorithmtypexmlstring);
  xercesc::XMLString::release(&algorithmtype);
  xercesc::XMLString::release(&algorithmtypexmlstring);

  if (segmentadaptiveparms << params.itsSegmentAdaptiveParameters) {
    XMLCh *offset = xercesc::XMLString::transcode("SegmentAdaptiveParameters");
    XMLCh *offsetxmlstring = xercesc::XMLString::transcode(segmentadaptiveparms.str().c_str());
    eventsroot->setAttribute(offset, offsetxmlstring);
    xercesc::XMLString::release(&offset);
    xercesc::XMLString::release(&offsetxmlstring);
  }

  ColorSpaceType cstype = params.itsColorSpaceType;
  string cstring = colorSpaceType(cstype);
  XMLCh *colorspacetype = xercesc::XMLString::transcode("ColorSpaceType");
  XMLCh *colorspacetypexmlstring = xercesc::XMLString::transcode(cstring.c_str());
  eventsroot->setAttribute(colorspacetype, colorspacetypexmlstring);
  xercesc::XMLString::release(&colorspacetype);
  xercesc::XMLString::release(&colorspacetypexmlstring);

  SaliencyInputImageType sitype = params.itsSaliencyInputType;
  string sistring = saliencyInputImageType(sitype);
  XMLCh *saliencyinputtype = xercesc::XMLString::transcode("SaliencyInputImageType");
  XMLCh *saliencyinputtypexmlstring = xercesc::XMLString::transcode(sistring.c_str());
  eventsroot->setAttribute(saliencyinputtype, saliencyinputtypexmlstring);
  xercesc::XMLString::release(&saliencyinputtype);
  xercesc::XMLString::release(&saliencyinputtypexmlstring);

  XMLCh *savenoninteresting = xercesc::XMLString::transcode("SaveNonInteresting");
  XMLCh *savenoninterestingxmlstring = xercesc::XMLString::transcode(params.itsSaveNonInteresting ? "1" : "0");
  eventsroot->setAttribute(savenoninteresting, savenoninterestingxmlstring);
  xercesc::XMLString::release(&savenoninteresting);
  xercesc::XMLString::release(&savenoninterestingxmlstring);

  XMLCh *saveoriginalframespec = xercesc::XMLString::transcode("SaveOriginalFrameSpec");
  XMLCh *saveoriginalframespecxmlstring = xercesc::XMLString::transcode(params.itsSaveOriginalFrameSpec ? "1" : "0");
  eventsroot->setAttribute(saveoriginalframespec, saveoriginalframespecxmlstring);
  xercesc::XMLString::release(&saveoriginalframespec);
  xercesc::XMLString::release(&saveoriginalframespecxmlstring);

  XMLCh *keepWTAboring = xercesc::XMLString::transcode("KeepWTABoring");
  XMLCh *keepWTAboringxmlstring = xercesc::XMLString::transcode(params.itsKeepWTABoring ? "1" : "0");
  eventsroot->setAttribute(keepWTAboring, keepWTAboringxmlstring);
  xercesc::XMLString::release(&keepWTAboring);
  xercesc::XMLString::release(&keepWTAboringxmlstring);

  if (maxWTApointsvalue << params.itsMaxWTAPoints) {
    XMLCh *maxWTApoints = xercesc::XMLString::transcode("MaxWTAPoints");
    XMLCh *maxWTApointsxmlstring = xercesc::XMLString::transcode(maxWTApointsvalue.str().c_str());
    eventsroot->setAttribute(maxWTApoints, maxWTApointsxmlstring);
    xercesc::XMLString::release(&maxWTApoints);
    xercesc::XMLString::release(&maxWTApointsxmlstring);
  }
  if (maxevolvetimevalue << params.itsMaxEvolveTime) {
    XMLCh *maxevolvetime = xercesc::XMLString::transcode("MaxEvolveTime");
    XMLCh *maxevolvetimexmlstring = xercesc::XMLString::transcode(maxevolvetimevalue.str().c_str());
    eventsroot->setAttribute(maxevolvetime, maxevolvetimexmlstring);
    xercesc::XMLString::release(&maxevolvetime);
    xercesc::XMLString::release(&maxevolvetimexmlstring);
  }
  if (maxframeseventvalue << params.itsMaxEventFrames) {
    XMLCh *maxframesevent = xercesc::XMLString::transcode("MaxFramesEvent");
    XMLCh *maxframeseventxmlstring = xercesc::XMLString::transcode(maxframeseventvalue.str().c_str());
    eventsroot->setAttribute(maxframesevent, maxframeseventxmlstring);
    xercesc::XMLString::release(&maxframesevent);
    xercesc::XMLString::release(&maxframeseventxmlstring);
  }

  if (minframeseventvalue << params.itsMinEventFrames) {
    XMLCh *minframesevent = xercesc::XMLString::transcode("MinFramesEvent");
    XMLCh *minframeseventxmlstring = xercesc::XMLString::transcode(minframeseventvalue.str().c_str());
    eventsroot->setAttribute(minframesevent, minframeseventxmlstring);
    xercesc::XMLString::release(&minframesevent);
    xercesc::XMLString::release(&minframeseventxmlstring);
  }

  if (maxcostvalue << params.itsMaxCost) {
    XMLCh *maxcost = xercesc::XMLString::transcode("MaxCost");
    XMLCh *maxcostxmlstring = xercesc::XMLString::transcode(maxcostvalue.str().c_str());
    eventsroot->setAttribute(maxcost, maxcostxmlstring);
    xercesc::XMLString::release(&maxcost);
    xercesc::XMLString::release(&maxcostxmlstring);
  }

  // Only output the mask parameters if a valid path is set
  if (params.itsMaskPath.length() > 0) {
    XMLCh *maskpath = xercesc::XMLString::transcode("MaskPath");
    XMLCh *maskpathstring = xercesc::XMLString::transcode(params.itsMaskPath.c_str());
    eventsroot->setAttribute(maskpath, maskpathstring);
    xercesc::XMLString::release(&maskpath);
    xercesc::XMLString::release(&maskpathstring);

    if (maskxposvalue << params.itsMaskXPosition) {
      XMLCh *maskxpos = xercesc::XMLString::transcode("MaskXPosition");
      XMLCh *maskxposstring = xercesc::XMLString::transcode(maskxposvalue.str().c_str());
      eventsroot->setAttribute(maskxpos, maskxposstring);
      xercesc::XMLString::release(&maskxpos);
      xercesc::XMLString::release(&maskxposstring);
    }
    if (maskyposvalue << params.itsMaskYPosition) {
      XMLCh *maskypos = xercesc::XMLString::transcode("MaskYPosition");
      XMLCh *maskyposstring = xercesc::XMLString::transcode(maskyposvalue.str().c_str());
      eventsroot->setAttribute(maskypos, maskyposstring);
      xercesc::XMLString::release(&maskypos);
      xercesc::XMLString::release(&maskyposstring);
    }
    if (maskwidthvalue << params.itsMaskWidth) {
      XMLCh *maskwidth = xercesc::XMLString::transcode("MaskHeight");
      XMLCh *maskwidthstring = xercesc::XMLString::transcode(maskwidthvalue.str().c_str());
      eventsroot->setAttribute(maskwidth, maskwidthstring);
      xercesc::XMLString::release(&maskwidth);
      xercesc::XMLString::release(&maskwidthstring);
    }
    if (maskheightvalue << params.itsMaskYPosition) {
      XMLCh *maskheight = xercesc::XMLString::transcode("MaskWidth");
      XMLCh *maskheightstring = xercesc::XMLString::transcode(maskheightvalue.str().c_str());
      eventsroot->setAttribute(maskheight, maskheightstring);
      xercesc::XMLString::release(&maskheight);
      xercesc::XMLString::release(&maskheightstring);
    }
  }

  xercesc::DOMElement *root = itsXMLdoc->getDocumentElement();
  root->appendChild(eventsroot);
}

// Fonction permettant de concaténer X fois
// une chaine str
string MbariXMLParser::strcatX(string str, unsigned int x) {
  for (unsigned int y = 0; y <= x; y++)
    str += "  ";
  return str;
}


void MbariXMLParser::display() {
  displayElements((xercesc::DOMNode *) itsXMLdoc->getDocumentElement(), 0);
}

// Fonction displayElements :
// fonction récursive qui affiche tous les noeuds ayant pour valeur
// "directory" ou "file" en attribut "type"
int MbariXMLParser::displayElements(xercesc::DOMNode *n, unsigned int nbr_child) {
  xercesc::DOMNode *child;
  // Compte les éléments
  unsigned int count = 0;

  // Compte le nombre d'appels à la fonction (récursive)
  // afin de pouvoir établir une tabulation au texte de sortie
  // de l'exécutable. Un appel = un répertoire
  static unsigned count_call = 0;

  // Rajoute un nombre nbr_child de tabulations afin de faciliter
  // la lisibilité de l'arborescence
  string xTab = strcatX("  ", nbr_child);

  if (n) {
    if (n->getNodeType() == xercesc::DOMNode::ELEMENT_NODE) {
      char *nodename = xercesc::XMLString::transcode(n->getNodeName());
      LINFO("%s<%s>", xTab.c_str(), nodename);

      if (n->hasAttributes()) {
        ++count_call;

        for (child = n->getFirstChild(); child != 0; child = child->getNextSibling())
          count += displayElements(child, count_call);

        --count_call;
        ++count;
      } else {
        ++count_call;
        for (child = n->getFirstChild(); child != 0; child = child->getNextSibling())
          count += displayElements(child, count_call);
        --count_call;
      }
      LINFO("%s</%s>", xTab.c_str(), nodename);
      xercesc::XMLString::release(&nodename);

    }
  }
  return count;
}

xercesc::DOMDocument *MbariXMLParser::parseXMLFile(string inputXML, string inputSchema) {

  // DOM parser instance
  itsParser = new xercesc::XercesDOMParser();

  /*itsParser->setDoNamespaces(true);
  itsParser->setDoSchema(true);
  itsParser->setValidationScheme(XercesDOMParser::Val_Always);
  itsParser->setExternalNoNamespaceSchemaLocation(inputSchema.c_str());
  */

  // Error handler instance for the parser
  itsErrHandler = new ErrReporter();
  itsParser->setErrorHandler(itsErrHandler);

  // Parse le fichier XML et récupère le temps mis pour le parsing
  try {
    itsParser->resetDocumentPool();
    itsParser->parse(inputXML.c_str());

    if (itsParser->getErrorCount() == 0) {
      xercesc::DOMDocument *domDocParser = itsParser->getDocument();
      return domDocParser;
    }
    else {
      LINFO("Error when attempting to parse the XML file : %s ", inputXML.c_str());
      return NULL;
    }
  }
    // Exception XML
  catch (const xercesc::XMLException &err) {
    LINFO("Error during XML parsing of file : %s ", inputXML.c_str());
  }
    // Exception DOM
  catch (const xercesc::DOMException &err) {
    const unsigned int maxChars = 2047;
    XMLCh errText[maxChars + 1];
    char *message;

    LINFO("Error during XML parsing of file  : %s \n", inputXML.c_str());
    if (xercesc::DOMImplementation::loadDOMExceptionMsg(err.code, errText, maxChars))
      message = xercesc::XMLString::transcode(err.msg);
      LFATAL("Exception DOM : %s ", message);
      xercesc::XMLString::release(&message);

  } catch (...) {
    LINFO("Error during XML parsing of file  : %s", inputXML.c_str());
  }
  return NULL;
}

void MbariXMLParser::add(bool saveNonInterestingEvents,
                         list<VisualEvent *> &eventList,
                         int eventframe,
                         string eventframetimecode,
                         float scaleW, float scaleH) {
  // Create the body of the DOMDocument
  try {

    // add another leaf to root node for detection parameters
    XMLCh *frameeventsetstring = xercesc::XMLString::transcode("FrameEventSet");
    xercesc::DOMElement *frameeventset = itsXMLdoc->createElement(frameeventsetstring);
    xercesc::XMLString::release(&frameeventsetstring);

    ostringstream number;
    // if conversion from frame to string worked,
    if (number << eventframe) {
      XMLCh *framenumberstring = xercesc::XMLString::transcode("FrameNumber");
      XMLCh *framenumbervalue = xercesc::XMLString::transcode(number.str().c_str());
      frameeventset->setAttribute(framenumberstring, framenumbervalue);
      xercesc::XMLString::release(&framenumberstring);
      xercesc::XMLString::release(&framenumbervalue);
    }

    XMLCh *timecode = xercesc::XMLString::transcode("TimeCode");
    XMLCh *timecodevaluevalue = xercesc::XMLString::transcode(eventframetimecode.c_str());
    frameeventset->setAttribute(timecode, timecodevaluevalue);
    xercesc::XMLString::release(&timecode);
    xercesc::XMLString::release(&timecodevaluevalue);


    list<VisualEvent *>::iterator i;
    for (i = eventList.begin(); i != eventList.end(); ++i) {
      // if also saving non-interesting events and this is BORING event, be sure to save this
      // otherwise, save all INTERESTING events
      if ((saveNonInterestingEvents && (*i)->getCategory() == VisualEvent::BORING) ||
          (*i)->getCategory() == VisualEvent::INTERESTING) {
        ostringstream s1, s2, s3, s4, s5, s6, s7;
        uint eframe = (*i)->getEndFrame();
        Token tke = (*i)->getToken(eframe);

        //create event object element and add attributes
        XMLCh *eventobjectstring = xercesc::XMLString::transcode("EventObject");
        xercesc::DOMElement *eventObject = itsXMLdoc->createElement(eventobjectstring);
        xercesc::XMLString::release(&eventobjectstring);

        s1 << (*i)->getEventNum();
        XMLCh *objectidstring = xercesc::XMLString::transcode("ObjectID");
        XMLCh *objectidvalue = xercesc::XMLString::transcode(s1.str().c_str());
        eventObject->setAttribute(objectidstring, objectidvalue);
        xercesc::XMLString::release(&objectidstring);
        xercesc::XMLString::release(&objectidvalue);

        s2 << (*i)->getStartFrame();
        XMLCh *startframenumberstring = xercesc::XMLString::transcode("StartFrameNumber");
        XMLCh *startframenumbervalue = xercesc::XMLString::transcode(s2.str().c_str());
        eventObject->setAttribute(startframenumberstring, startframenumbervalue);
        xercesc::XMLString::release(&startframenumberstring);
        xercesc::XMLString::release(&startframenumbervalue);

        if ((*i)->getStartTimecode().length() > 0) {
          s3 << (*i)->getStartTimecode();
          XMLCh *starttimecodetring = xercesc::XMLString::transcode("StartTimecode");
          XMLCh *starttimecodevalue = xercesc::XMLString::transcode(s3.str().c_str());
          eventObject->setAttribute(starttimecodetring, starttimecodevalue);
          xercesc::XMLString::release(&starttimecodetring);
          xercesc::XMLString::release(&starttimecodevalue);
        }

        s4 << tke.bitObject.getSMV();
        XMLCh *saliencystring = xercesc::XMLString::transcode("Saliency");
        XMLCh *saliencyvalue = xercesc::XMLString::transcode(s4.str().c_str());
        eventObject->setAttribute(saliencystring, saliencyvalue);
        xercesc::XMLString::release(&saliencystring);
        xercesc::XMLString::release(&saliencyvalue);

        s5 << tke.bitObject.getArea();
        XMLCh *currsizestring = xercesc::XMLString::transcode("CurrSize");
        XMLCh *currsizevalue = xercesc::XMLString::transcode(s5.str().c_str());
        eventObject->setAttribute(currsizestring, currsizevalue);
        xercesc::XMLString::release(&currsizestring);
        xercesc::XMLString::release(&currsizevalue);

        Point2D<int> p = tke.bitObject.getCentroid();
        s6 << (int) ((float) p.i * scaleW);
        XMLCh *currxstring = xercesc::XMLString::transcode("CurrX");
        XMLCh *currxvalue = xercesc::XMLString::transcode(s6.str().c_str());
        eventObject->setAttribute(currxstring, currxvalue);
        xercesc::XMLString::release(&currxstring);
        xercesc::XMLString::release(&currxvalue);

        s7 << (int) ((float) p.j * scaleH);
        XMLCh *currystring = xercesc::XMLString::transcode("CurrY");
        XMLCh *curryvalue = xercesc::XMLString::transcode(s7.str().c_str());
        eventObject->setAttribute(currystring, curryvalue);
        xercesc::XMLString::release(&currystring);
        xercesc::XMLString::release(&curryvalue);

        Rectangle r = tke.bitObject.getBoundingBox();
        ostringstream llx, lly, urx, ury;
        llx << (int) ((float) r.left() * scaleW);
        lly << (int) ((float) r.bottomI() * scaleH);
        urx << (int) ((float) r.rightI() * scaleW);
        ury << (int) ((float) r.top() * scaleH);

        // create bounding box element and add attributes
        XMLCh *boundingboxstring = xercesc::XMLString::transcode("BoundingBox");
        xercesc::DOMElement *boundingBox = itsXMLdoc->createElement(boundingboxstring);
        xercesc::XMLString::release(&boundingboxstring);

        XMLCh *lowerleftxstring = xercesc::XMLString::transcode("LowerLeftX");
        XMLCh *lowerleftxvalue = xercesc::XMLString::transcode(llx.str().c_str());
        boundingBox->setAttribute(lowerleftxstring, lowerleftxvalue);
        xercesc::XMLString::release(&lowerleftxstring);
        xercesc::XMLString::release(&lowerleftxvalue);

        XMLCh *lowerleftystring = xercesc::XMLString::transcode("LowerLeftY");
        XMLCh *lowerleftyvalue = xercesc::XMLString::transcode(lly.str().c_str());
        boundingBox->setAttribute(lowerleftystring, lowerleftyvalue);
        xercesc::XMLString::release(&lowerleftystring);
        xercesc::XMLString::release(&lowerleftyvalue);

        XMLCh *upperrightxstring = xercesc::XMLString::transcode("UpperRightX");
        XMLCh *upperrightxvalue = xercesc::XMLString::transcode(urx.str().c_str());
        boundingBox->setAttribute(upperrightxstring, upperrightxvalue);
        xercesc::XMLString::release(&upperrightxstring);
        xercesc::XMLString::release(&upperrightxvalue);

        XMLCh *upperrightystring = xercesc::XMLString::transcode("UpperRightY");
        XMLCh *upperrightyvalue = xercesc::XMLString::transcode(ury.str().c_str());
        boundingBox->setAttribute(upperrightystring, upperrightyvalue);
        xercesc::XMLString::release(&upperrightystring);
        xercesc::XMLString::release(&upperrightyvalue);

        // append to appropriate elements
        eventObject->appendChild(boundingBox);
        frameeventset->appendChild(eventObject);
      }
    }

    xercesc::DOMElement *root = itsXMLdoc->getDocumentElement();
    root->appendChild(frameeventset);


  } catch (const xercesc::OutOfMemoryException &) {
    LERROR("OutOfMemoryException");
  } catch (const xercesc::DOMException &e) {
    LERROR("DOMException code is:  %d", e.code);
  } catch (...) {
    LERROR("An error occurred creating the document");
  }
}

void MbariXMLParser::writeDocument(string path) {

  XMLCh *out = xercesc::XMLString::transcode(path.c_str());
  itsXMLFileFormatTarget = new xercesc::LocalFileFormatTarget(out);
  xercesc::XMLString::release(&out);
  itsDOMWriter->writeNode(itsXMLFileFormatTarget, *itsXMLdoc);
  delete itsXMLFileFormatTarget;

}
