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

#include "Image/OpenCVUtil.H"
#include "Data/Logger.H"
#include "Utils/Version.H"

#include <xercesc/util/OutOfMemoryException.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/framework/LocalFileFormatTarget.hpp>
#include <xercesc/dom/DOM.hpp>
#include <sstream>
#include <cstdio>
#include <fstream>

#include "Image/CutPaste.H"
#include "Image/Pixels.H"
#include "Media/MediaOpts.H"
#include "Transport/FrameInfo.H"
#include "Data/MbariOpts.H"
#include "DetectionAndTracking/VisualEvent.H"
#include "DetectionAndTracking/VisualEventSet.H"
#include "Raster/GenericFrame.H"

using namespace std;

#define MAX_INT32 2147483647

// ######################################################################
// Logger member definitions:
// ######################################################################

// ######################################################################
Logger::Logger(OptionManager& mgr, nub::soft_ref<InputFrameSeries> ifs,
      nub::soft_ref<OutputFrameSeries> ofs,
      string logDir,
      const string& descrName,
      const string& tagName)
      : ModelComponent(mgr, descrName, tagName),
    itsInputFrameRange(&OPT_InputFrameRange, this),
    itsInputFrameSource(&OPT_InputFrameSource, this),
    itsOutputFrameSink(&OPT_OutputFrameSink, this),
    itsLoadEventsName(&OPT_LOGloadEvents, this),
    itsLoadPropertiesName(&OPT_LOGloadProperties, this),
    itsMetadataSource(&OPT_LOGmetadataSource, this),
    itsSaveBoringEvents(&OPT_MDPsaveBoringEvents, this),
    itsSaveEventsName(&OPT_LOGsaveEvents, this),
    itsPadEvents(&OPT_LOGpadEvents, this),
    itsSaveEventFeatures(&OPT_LOGsaveEventFeatures, this),
    itsSaveEventNumString(&OPT_LOGsaveEventNums, this),
    itsSaveOriginalFrameSpec(&OPT_MDPsaveOriginalFrameSpec, this),
    itsSaveOutput(&OPT_LOGsaveOutput, this),
    itsSavePositionsName(&OPT_LOGsavePositions, this),
    itsSavePropertiesName(&OPT_LOGsaveProperties, this),
    itsSaveSummaryEventsName(&OPT_LOGsaveSummaryEventsName, this),
    itsSaveXMLEventSetName(&OPT_LOGsaveXMLEventSet, this),
    itsIfs(ifs),
    itsOfs(ofs),
    itsXMLfileCreated(false),
    itsAppendEvt(false),
    itsAppendEvtSummary(false),
    itsAppendEvtXML(false),
    itsAppendProperties(false),
    itsSaveEventNumsAll(false),
    itsScaleW(1.0f),
    itsScaleH(1.0f),
    itsPad(0)
{
    itsXMLParser = new MbariXMLParser(logDir);
}

// ######################################################################
Logger::~Logger()
{
    freeMem();
}

// ######################################################################

void Logger::reset1() {
    // destroy our stuff
    freeMem();

    // propagate to our base class:
    ModelComponent::reset1();
}

// ######################################################################
void Logger::start1()
{
    DetectionParameters dp = DetectionParametersSingleton::instance()->itsParameters;

    // initialize the XML if requested to save event set to XML
    if (itsSaveXMLEventSetName.getVal().length() > 0 && itsFrameRange.getLast() > itsFrameRange.getFirst()) {

        Image< PixRGB<byte> > tmpimg;
        MbariImage< PixRGB<byte> > mstart(itsInputFrameSource.getVal());
        MbariImage< PixRGB<byte> > mend(itsInputFrameSource.getVal());

        // get the dimensions, starting and ending timecodes from the frames
        nub::ref<FrameIstream> rep = itsIfs->getFrameSource();

        rep->setFrameNumber(itsFrameRange.getFirst());
        tmpimg = rep->readRGB();
        mstart.updateData(tmpimg, itsFrameRange.getFirst());

        rep->setFrameNumber(itsFrameRange.getLast());
        tmpimg = rep->readRGB();
        mend.updateData(tmpimg, itsFrameRange.getLast());

        // reset the frame number back since we will be streaming this soon
        rep->setFrameNumber((itsFrameRange.getFirst()));

        // create the XML document with header information
        createXMLDocument(versionString(),
                          itsFrameRange,
                          mstart.getMetaData().getTC(),
                          mend.getMetaData().getTC(),
                          dp);
    }

    //MbariVisualEvent::VisualEventSet eventSet;

    // are we loading the event structure from a file?
    if (itsLoadEventsName.getVal().length() > 0) {
        //loadVisualEventSet(eventSet);
        // TODO: test preloading of events; hasn't been used in a while but
        // potentially useful for future so will leave it here
    }
}

// ######################################################################
void Logger::paramChanged(ModelParamBase* const param,
                                 const bool valueChanged,
                                 ParamClient::ChangeStatus* status)
{
    // if the param is itsSaveEventNum, parse the string and fill the vector
    if (param == &itsSaveEventNumString)
        parseSaveEventNums(itsSaveEventNumString.getVal());

    if (param == &itsInputFrameRange) {
        itsFrameRange = itsInputFrameRange.getVal();
        if (itsFrameRange.getLast() == MAX_INT32) {
            FrameRange range(itsFrameRange.getFirst(), itsFrameRange.getStep(), itsFrameRange.getFirst());
            itsFrameRange = range;
        }
    }

}

// ######################################################################
void Logger::run(nub::soft_ref<MbariResultViewer> rv, MbariImage<PixRGB <byte> >& img,
                 VisualEventSet& eventSet, const Dims scaledDims)
{
    // adjust scaling if needed
    Dims d = img.getDims();
    itsScaleW = (float)d.w()/(float)scaledDims.w();
    itsScaleH = (float)d.h()/(float)scaledDims.h();

    // optional padding for events
    if (itsPadEvents.getVal() > 0 && itsPadEvents.getVal() <= 200 ) itsPad = itsPadEvents.getVal();

    // initialize property vector and FOE estimator
    PropertyVectorSet pvs;

    // this is a list of all the events that have a token in this frame
    list<VisualEvent *> eventFrameList;

    // this is a complete list of all those events that are ready to be written
    list<VisualEvent *> eventListToSave;

    // get event frame list for this frame and those events that are ready to be saved
    // this is a list of all the events that have a token in this frame
    eventFrameList = eventSet.getEventsForFrame(img.getFrameNum());

    // this is a complete list of all those events that are ready to be written
    eventListToSave = eventSet.getEventsReadyToSave(img.getFrameNum());

    // write out eventSet?
    if (itsSaveEventsName.getVal().length() > 0 ) saveVisualEvent(eventSet, eventFrameList);

    // write out summary ?
    if (itsSaveSummaryEventsName.getVal().length() > 0) saveVisualEventSummary(versionString(), eventFrameList);

    // flag events that have been saved
    list<VisualEvent *>::iterator i;
    for (i = eventListToSave.begin(); i != eventListToSave.end(); ++i)
        (*i)->flagWriteComplete();

    // write out positions?
    if (itsSavePositionsName.getVal().length() > 0) savePositions(eventFrameList);

    PropertyVectorSet pvsToSave = eventSet.getPropertyVectorSetToSave();

    // write out property vector set?
    if (itsSavePropertiesName.getVal().length() > 0) saveProperties(pvsToSave);

    // TODO: this is currently not used...look back in history to where this got cut-out
    // need to obtain the property vector set?
    if (itsLoadPropertiesName.getVal().length() > 0) pvs = eventSet.getPropertyVectorSet();

    // write out eventSet to XML?
    if (itsSaveXMLEventSetName.getVal().length() > 0) {
        saveVisualEventSetToXML(eventFrameList,
                                img.getFrameNum(),
                                img.getMetaData().getTC(),
                                itsFrameRange);
    }

    const int circleRadiusRatio = 40;
    const int circleRadius = img.getDims().w() / circleRadiusRatio;

    Image< PixRGB<byte> > output = rv->createOutput(img,
                                                    eventSet,
                                                    circleRadius,
                                                    itsScaleW, itsScaleH);

    // write  ?
    if (itsSaveOutput.getVal())
        itsOfs->writeFrame(GenericFrame(output), "results", FrameInfo("results", SRC_POS));

    // display output ?
    rv->display(output, img.getFrameNum(), "Results");

    // need to save any event clips?
    if (itsSaveEventNumsAll) {
        //save all events
        list<VisualEvent *>::iterator i;
        for (i = eventFrameList.begin(); i != eventFrameList.end(); ++i)
            saveSingleEventFrame(img, img.getFrameNum(), *i);
    } else {
        // need to save any particular event clips?
        uint csavenum = numSaveEventClips();
        for (uint idx = 0; idx < csavenum; ++idx) {
            uint evnum = getSaveEventClipNum(idx);
            if (!eventSet.doesEventExist(evnum)) continue;

            VisualEvent *event = eventSet.getEventByNumber(evnum);
            if (event->frameInRange(img.getFrameNum()))
                saveSingleEventFrame(img, img.getFrameNum(), event);
        }
    }

    //flag events that have been saved for delete otherwise takes too much memory
    for (i = eventListToSave.begin(); i != eventListToSave.end(); ++i)
        (*i)->flagForDelete();
    while (!eventFrameList.empty()) eventFrameList.pop_front();
    while (!eventListToSave.empty()) eventListToSave.pop_front();

}

void Logger::saveVisualEventSetToXML(list<VisualEvent *> &eventList,
        int eventframe,
        string eventframetimecode,
        FrameRange fr) {
    if (!itsXMLfileCreated)
        LFATAL("Error: Create an XML document first with createXMLDocument()");
    else {
		itsXMLParser->add(itsSaveBoringEvents.getVal(), eventList, eventframe, eventframetimecode, itsScaleW, itsScaleH);
	}

    if (fr.getLast() == eventframe) {
        itsXMLParser->writeDocument(itsSaveXMLEventSetName.getVal().c_str());
        LINFO("The XML output is valid");
    }
}

// #############################################################################

void Logger::createXMLDocument(string versionString,
        FrameRange fr,
        string timecodefirst,
        string timecodelast,
        DetectionParameters params) {
    if (!itsXMLfileCreated) {
        itsXMLParser->creatDOMDocument(versionString,
                fr.getFirst(), fr.getLast(),
                timecodefirst, timecodelast);

        // add in source metadata if specified
        if (itsMetadataSource.getVal().length() > 0) {
            itsXMLParser->addSourceMetaData(itsMetadataSource.getVal());
        }
        // add in detection parameters
        itsXMLParser->addDetectionParameters(params);
        itsXMLParser->writeDocument(itsSaveXMLEventSetName.getVal().c_str());
        itsXMLfileCreated = true;
    }
}

// #############################################################################

void Logger::savePositions(const list<VisualEvent *> &eventList) const {

    ofstream ofs(itsSavePositionsName.getVal().data());

    list<VisualEvent *>::const_iterator i;
    for (i = eventList.begin(); i != eventList.end(); ++i)
        (*i)->writePositions(ofs);

    ofs.close();
}


// #############################################################################
// # Redifine the Logger::parseSaveEventNums

void Logger::parseSaveEventNums(const string& value) {
    itsSaveEventNums.clear();

    if (value.compare(string("all")) == 0) {
        itsSaveEventNumsAll = true;
    } else {
        // format here is "c,...,c"
        int curpos = 0, len = value.length();
        while (curpos < len) {
            // get end of next number
            int nextpos = value.find_first_not_of("-.0123456789eE", curpos);
            if (nextpos == -1) nextpos = len;

            // no number characters found -> bummer
            if (nextpos == curpos)
                LFATAL("Error parsing the SaveEventNum string '%s' - found '%c' "
                    "instead of a number.", value.data(), value[curpos]);

            // now let's see - can we get a number here?
            uint evNum;
            int rep = sscanf(value.substr(curpos, nextpos - curpos).data(), "%i", &evNum);

            // couldn't read a number -> bummer
            if (rep != 1)
                LFATAL("Error parsing SaveEventNum string '%s' - found '%s' instead of "
                    "a number.", value.data(),
                    value.substr(curpos, nextpos - curpos).data());

            // yeah! found a number -> store it
            itsSaveEventNums.push_back(evNum);

            LDEBUG("evNum = %i; value[nextpos] = '%c'", evNum, value[nextpos]);

            // not a comma -> bummer
            if ((nextpos < len) && (value[nextpos] != ','))
                LFATAL("Error parsing the SaveEventNum string '%s' - found '%c' "
                    "instead of ','.", value.data(), value[nextpos]);

            // the character right after the comma should be a number again
            curpos = nextpos + 1;
        }

        // end of string, done
    }
    return;
}

// #############################################################################

void Logger::saveProperties(PropertyVectorSet& pvs) {
    ofstream ofs;

    if (!itsAppendProperties) { // if file hasn't been opened for appending, open to rewrite file
        ofs.open(itsSavePropertiesName.getVal().data());
        pvs.writeHeaderToStream(ofs);
        itsAppendProperties = true;
    } else //otherwise for appending events
        ofs.open(itsSavePropertiesName.getVal().data(), ofstream::out | ofstream::app);

    pvs.writeToStream(ofs);//TODO: test if need scaling factor here
    ofs.close();
}

// #############################################################################

void Logger::loadVisualEventSet(VisualEventSet& ves) const {
    ifstream ifs(itsLoadEventsName.getVal().c_str());
    ves.readFromStream(ifs); //TODO: test if need scaling factor here
    ifs.close();
}

// ######################################################################

void Logger::freeMem() {

    itsSaveEventsName.setVal("");
    itsLoadEventsName.setVal("");
    itsSavePropertiesName.setVal("");
    itsLoadPropertiesName.setVal("");

    itsSaveSummaryEventsName.setVal("");
    itsLoadEventsName.setVal("");
    itsSaveXMLEventSetName.setVal("");

}

// #############################################################################

void Logger::saveFeatures(int frameNum, VisualEventSet &eventSet) {

    if (itsSaveEventFeatures.getVal()) {
        list <VisualEvent *> eventFrameList = eventSet.getEventsForFrame(frameNum);
        const string::size_type hashpos = itsOutputFrameSink.getVal().find_first_of(':');
        const string outputDir = itsOutputFrameSink.getVal().substr(hashpos + 1);

        // for each bit object, extract features from latest token and save the output
        list<VisualEvent *>::iterator event;
        for (event = eventFrameList.begin(); event != eventFrameList.end(); ++event) {
            Token token = (*event)->getToken(frameNum);

            if (token.bitObject.isValid()) {

                LINFO("Saving features for event %d frame %d to %s", (*event)->getEventNum(), frameNum,
                      outputDir.c_str());

                // create the file stem and write out the features
                string evnumPVS(
                        sformat("%s_evt%04d_%06d_PVS.dat", outputDir.c_str(), (*event)->getEventNum(), frameNum));
                string evnumHOG3(
                        sformat("%s_evt%04d_%06d_HOG_3.dat", outputDir.c_str(), (*event)->getEventNum(), frameNum));
                //string evnumMBH3(sformat("%s-evt%04d_%06d_MBH_3.dat", outputDir.c_str(), (*event)->getEventNum(), frameNum ));
                string evnumHOG8(
                        sformat("%s_evt%04d_%06d_HOG_8.dat", outputDir.c_str(), (*event)->getEventNum(), frameNum));
                //string evnumMBH8(sformat("%s-evt%04d_%06d_MBH_8.dat", outputDir.c_str(), (*event)->getEventNum(), frameNum));
                string evnumJETred(
                        sformat("%s_evt%04d_%06d_JET_red.dat", outputDir.c_str(), (*event)->getEventNum(), frameNum));
                string evnumJETgreen(
                        sformat("%s_evt%04d_%06d_JET_green.dat", outputDir.c_str(), (*event)->getEventNum(), frameNum));
                string evnumJETblue(
                        sformat("%s_evt%04d_%06d_JET_blue.dat", outputDir.c_str(), (*event)->getEventNum(), frameNum));

                ofstream eofsPVS(evnumPVS.c_str());
                ofstream eofsHOG3(evnumHOG3.c_str());
                //ofstream eofsMBH3(evnumMBH3.c_str());
                ofstream eofsHOG8(evnumHOG8.c_str());
                //ofstream eofsMBH8(evnumMBH8.c_str());
                ofstream eofsJETred(evnumJETred.c_str());
                ofstream eofsJETgreen(evnumJETgreen.c_str());
                ofstream eofsJETblue(evnumJETblue.c_str());

                eofsPVS.precision(12);
                eofsHOG3.precision(12);
                ///eofsMBH3.precision(12);
                eofsHOG8.precision(12);
                //eofsMBH8.precision(12);
                eofsJETred.precision(12);
                eofsJETgreen.precision(12);
                eofsJETblue.precision(12);

                vector<float> featurePVS =  (*event)->getPropertyVector();
                vector<float>::iterator eitrPVS = featurePVS.begin(), stopPVS = featurePVS.end();
                vector<double>::iterator eitrHOG3 = token.featureHOG3.begin(), stopHOG3 = token.featureHOG3.end();
                //vector<double>::iterator eitrMBH3 = token.featureMBH3.begin(), stopMBH3 = token.featureMBH3.end();
                vector<double>::iterator eitrHOG8 = token.featureHOG8.begin(), stopHOG8 = token.featureHOG8.end();
                //vector<double>::iterator eitrMBH8 = token.featureMBH8.begin(), stopMBH8 = token.featureMBH8.end();
                vector<double>::iterator eitrJETred = token.featureJETred.begin(), stopJETred = token.featureJETred.end();
                vector<double>::iterator eitrJETgreen = token.featureJETgreen.begin(), stopJETgreen = token.featureJETgreen.end();
                vector<double>::iterator eitrJETblue = token.featureJETblue.begin(), stopJETblue = token.featureJETblue.end();

                while (eitrPVS != stopPVS) eofsPVS << *eitrPVS++ << " ";
                eofsPVS.close();
                while (eitrHOG3 != stopHOG3) eofsHOG3 << *eitrHOG3++ << " ";
                eofsHOG3.close();
                while (eitrHOG3 != stopHOG3) eofsHOG3 << *eitrHOG3++ << " ";
                eofsHOG3.close();
                //while(eitrMBH3 != stopMBH3)   eofsMBH3 << *eitrMBH3++ << " ";
                //eofsMBH3.close();
                while (eitrHOG8 != stopHOG8) eofsHOG8 << *eitrHOG8++ << " ";
                //eofsHOG8.close();
                //while(eitrMBH8 != stopMBH8)   eofsMBH8 << *eitrMBH8++ << " ";
                //eofsMBH8.close();
                while (eitrJETred != stopJETred) eofsJETred << *eitrJETred++ << " ";
                eofsJETred.close();
                while (eitrJETgreen != stopJETgreen) eofsJETgreen << *eitrJETgreen++ << " ";
                eofsJETgreen.close();
                while (eitrJETblue != stopJETblue) eofsJETblue << *eitrJETblue++ << " ";
                eofsJETblue.close();
            }
        }
    }
}

// #############################################################################
void Logger::saveSingleEventFrame(MbariImage<PixRGB <byte> > &img,
                                  int frameNum,
                                  VisualEvent *event) {
    ASSERT(event->frameInRange(frameNum));

    // create the file stem
    string evnum = sformat("evt%04d_", event->getEventNum());

    Dims maxDims = event->getMaxObjectDims();
    Dims d((float) (maxDims.w()*itsScaleW) + itsPad, (float) (maxDims.h()*itsScaleH) + itsPad);

    // compute the correct bounding box and cut it out
    Rectangle bbox1 = event->getToken(frameNum).bitObject.getBoundingBox();
    Rectangle bbox = Rectangle::tlbrI(bbox1.top() * itsScaleH, bbox1.left() * itsScaleW,
                                      bbox1.bottomI() * itsScaleH, bbox1.rightI() * itsScaleW);
    //Point2D cen = event.getToken(frameNum).bitObject.getCentroid();

    // first the horizontal direction
    int wpad = (d.w() - bbox.width()) / 2;
    int ll = bbox.left() - wpad;
    //int ll = cen.i - d.w() / 2;
    int rr = ll + d.w();
    if (ll < 0) {
        rr -= ll;
        ll = 0;
    }
    if (rr >= img.getWidth()) {
        rr = img.getWidth() - 1;
        ll = rr - d.w();
    }

    // now the same thing with the vertical direction
    int hpad = (d.h() - bbox.height()) / 2;
    int tt = bbox.top() - hpad;
    //int tt = cen.j - d.h() / 2;
    int bb = tt + d.h();
    if (tt < 0) {
        bb -= tt;
        tt = 0;
    }
    if (bb >= img.getHeight()) {
        bb = img.getHeight() - 1;
        tt = bb - d.h();
    }

    Rectangle bboxFinal = Rectangle::tlbrI(tt, ll, bb, rr);
    bboxFinal = bboxFinal.getOverlap(Rectangle(Point2D<int>(0, 0), img.getDims() - 1));

    // scale if needed and cut out the rectangle and save it
    Image <PixRGB <byte> > cut = crop(img, bboxFinal);
    itsOfs->writeFrame(GenericFrame(cut), evnum, FrameInfo(evnum, SRC_POS));
}


// #############################################################################

void Logger::saveVisualEvent(VisualEventSet &ves,
                             list<VisualEvent *> &eventList) {
    ofstream ofs;

    if (!itsAppendEvt) { // if file hasn't been opened for appending, open and write header
        ofs.open(itsSaveEventsName.getVal().data());
        ves.writeHeaderToStream(ofs);
        itsAppendEvt = true;
    } else //otherwise write to append events and skip header
        ofs.open(itsSaveEventsName.getVal().data(), ofstream::out | ofstream::app);

    list<VisualEvent *>::iterator i;
    for (i = eventList.begin(); i != eventList.end(); ++i)
        (*i)->writeToStream(ofs);

    ofs.close();
}

// #############################################################################

void Logger::saveVisualEventSummary(string versionString,
                                    list<VisualEvent *> &eventList) {
    ofstream ofs;

    if (!itsAppendEvtSummary) { // if file hasn't been opened for appending, open and write header
        ofs.open(itsSaveSummaryEventsName.getVal().data());
        ofs << versionString;
        ofs << "filename:" << itsSaveSummaryEventsName.getVal();

        char datestamp[24];
        time_t time_of_day;
        time_of_day = time(NULL);
        strftime(datestamp, 24, "%Y-%m-%d %X %Z", localtime(&time_of_day));
        ofs << "\tcreated: ";
        ofs << datestamp << "\n";

        DetectionParameters p = DetectionParametersSingleton::instance()->itsParameters;
        p.writeToStream(ofs);

        ofs << "eventID" << "\t";
        ofs << "startTimecode" << "\t";
        ofs << "endTimecode" << "\t";
        ofs << "startFrame" << "\t";
        ofs << "endFrame" << "\t";
        ofs << "startXY" << "\t";
        ofs << "endXY" << "\t";
        ofs << "maxArea" << "\t";
        ofs << "isInteresting" << "\n";
        itsAppendEvtSummary = true;
    } else //otherwise write to append events and skip header
        ofs.open(itsSaveSummaryEventsName.getVal().data(), ofstream::out | ofstream::app);

    Token tks, tke;
    uint sframe, eframe;
    Point2D<int> p;
    string tc;
    list<VisualEvent *>::iterator i;

    for (i = eventList.begin(); i != eventList.end(); ++i) {

        // if this is an interesting event, or
        // if this is a non-interesting(boring) event and we are saving boring events
        if ((*i)->getCategory() == VisualEvent::INTERESTING ||
            ((*i)->getCategory() == VisualEvent::BORING &&
             itsSaveBoringEvents.getVal())) {
            ofs << (*i)->getEventNum() << "\t";

            if ((*i)->getStartTimecode().length() > 0)
                ofs << (*i)->getStartTimecode();
            else
                ofs << '-';
            ofs << "\t";

            if ((*i)->getEndTimecode().length() > 0)
                ofs << (*i)->getEndTimecode();
            else
                ofs << '-';
            ofs << "\t";

            sframe = (*i)->getStartFrame();
            eframe = (*i)->getEndFrame();
            tks = (*i)->getToken(sframe);
            tke = (*i)->getToken(eframe);

            ofs << sframe << "\t";
            ofs << eframe << "\t";
            p = tks.bitObject.getCentroid();
            ofs << p.i << "," << p.j;
            ofs << "\t";
            p = tke.bitObject.getCentroid();
            ofs << p.i << "," << p.j;
            ofs << "\t";
            ofs << (*i)->getMaxSize() << "\t";
            ofs << (*i)->getCategory() << "\t";
            ofs << "\n";
        }
    }

    ofs.close();
}

// #############################################################################

void Logger::save(const Image <PixRGB <byte> > &img,
                  const uint frameNum,
                  const string &resultName,
                  const int resNum) {
    itsOfs->writeFrame(GenericFrame(img), getFileStem(resultName, resNum), FrameInfo(resultName, SRC_POS));
}

// #############################################################################

void Logger::save(const Image <byte> &img,
                  const uint frameNum,
                  const string &resultName,
                  const int resNum) {
    itsOfs->writeFrame(GenericFrame(img), getFileStem(resultName, resNum), FrameInfo(resultName, SRC_POS));
}

// #############################################################################

void Logger::save(const Image<float> &img,
                  const uint frameNum,
                  const string &resultName,
                  const int resNum) {
    itsOfs->writeFrame(GenericFrame(img, FLOAT_NORM_0_255), getFileStem(resultName, resNum),
                       FrameInfo(resultName, SRC_POS));
}

// #############################################################################

void Logger::loadProperties(PropertyVectorSet &pvs) const {
    ifstream ifs(itsLoadPropertiesName.getVal().c_str());
    pvs.readFromStream(ifs);
    ifs.close();
}

// #############################################################################

void Logger::saveVisualEventSet(VisualEventSet &ves) const {
    ofstream ofs(itsSaveEventsName.getVal().c_str());
    ves.writeToStream(ofs);
    ofs.close();
}

/// #############################################################################

uint Logger::numSaveEventClips() const {
    return itsSaveEventNums.size();
}
// #############################################################################

void Logger::savePositions(const VisualEventSet &ves) const {
    ofstream ofs(itsSavePositionsName.getVal().c_str());
    ves.writePositions(ofs);
    ofs.close();
}

// #############################################################################

uint Logger::getSaveEventClipNum(uint idx) const {
    ASSERT(idx < itsSaveEventNums.size());
    return itsSaveEventNums[idx];
}

// #############################################################################

string Logger::getFileStem(const string &resultName,
                           const int resNum) {
    return sformat("%s%02d_", resultName.c_str(), resNum);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
