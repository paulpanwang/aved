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

/*!@file Logger.H used for event tracking */

#ifndef LOGGER_C_DEFINED
#define LOGGER_C_DEFINED

#include "Component/ModelManager.H"
#include "Component/ModelComponent.H"
#include "Component/ModelOptionDef.H"
#include "Media/FrameRange.H"
#include "Media/FrameSeries.H"
#include "Image/BitObject.H"
#include "Learn/Features.H"
#include "DetectionAndTracking/PropertyVectorSet.H"
#include "Utils/MbariXMLParser.H"

#include <string>
#include <vector>

template <class T> class MbariImage;

class VisualEvent;
class VisualEventSet;
class MbariResultViewer;
namespace nub { template <class T> class soft_ref; }

// ######################################################################
//! Data logger class
class Logger : public ModelComponent
{

public:
    //! Construct
    Logger(OptionManager& mgr, nub::soft_ref<InputFrameSeries> ifs, nub::soft_ref<OutputFrameSeries> ofs,
        std::string logDir,
        const std::string& descrName = "logger",
        const std::string& tagName = "logger");

    //! destructor
    virtual ~Logger();

    //! Overload so that we can reconfigure when our params get changed
    virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

    //! Logger
    void run(nub::soft_ref<MbariResultViewer> rv, MbariImage< PixRGB<byte> >& img,
             VisualEventSet& eventSet, const Dims scaledDims);

    //! save features from event clips
    void saveFeatures(int frameNum, VisualEventSet& eventSet);

    //! Creates AVED XML document with header information:
    //! free memory
    virtual void reset1();

protected:

    //! overload start1()
    virtual void start1();

private:

    //! destroy internal variables
    void freeMem();


    //! save intermediate result
    /*!@param img the image containing the intermediate result
    @param frameNum the frame number of the image
    @param resultName a string that defines the kind of intermediate
    result - this is appended to the filestem for the file name
    and/or window title
    @param resNum if there are several results of this type for each frame,
    the number of the result*/
    void save(const Image< PixRGB<byte> >& img, const uint frameNum,
        const std::string& resultName, const int resNum = -1);

    //! save intermediate result
    /*!@param img the image containing the intermediate result
    @param frameNum the frame number of the image
    @param resultName a string that defines the kind of intermediate
    result - this is appended to the filestem for the file name
    and/or window title
    @param resNum if there are several results of this type for each frame,
    the number of the resul*/
    void save(const Image<byte>& img, const uint frameNum,
        const std::string& resultName, const int resNum = -1);

    //! save intermediate result
    /*!@param img the image containing the intermediate result
    @param frameNum the frame number of the image
    @param resultName a string that defines the kind of intermediate
    result - this is appended to the filestem for the file name
    and/or window title
    @param resNum if there are several results of this type for each frame,
    the number of the resul*/
    void save(const Image<float>& img, const uint frameNum,
        const std::string& resultName, const int resNum = -1);

    //! save a cropped portion of the frame containing a single event
    void saveSingleEventFrame(const MbariImage< PixRGB<byte> >& img,
			    int frameNum, VisualEvent* event);

    //! save the PropertyVector to the file SavePropertiesName
    void saveProperties(PropertyVectorSet& pvs);

    //! load the VisualEventSet from the file LoadEventsName
    void loadVisualEventSet(VisualEventSet& ves) const;

    //! load the PropertyVector from the file LoadPropertiesName
    void loadProperties(PropertyVectorSet& pvs) const;

    //! save the VisualEventSet to the file SaveEventsName
    void saveVisualEventSet(VisualEventSet& ves) const;

    //! return the number of event clips we have to save
    uint numSaveEventClips() const;

    //! returns the event number with idx to be saved as a clip
    uint getSaveEventClipNum(uint idx) const;

    //! save a cut-out piece of frame containing a single event
    void saveSingleEventFrame(MbariImage< PixRGB<byte> >& img,
                int frameNum, VisualEvent *event);

    //! save the VisualEventList to the file SaveEventsName
    void saveVisualEvent(VisualEventSet& ves,
               std::list<VisualEvent *> &lves);

    //! save the positions to the file SavePositionsName
    void savePositions(const VisualEventSet& ves) const;

    /*! @param versionString the version string that represents this binary
    @param fr frame range of this event sequence
    @param timecodefirst starting timecode of this event sequence, e.g. in 00:01:12
    @param timecodelast ending timecode of this event sequence, e.g. in 00:01:12
    @param params detection parameters
    * */
    void createXMLDocument(std::string versionString,
             FrameRange fr,
             std::string timecodefirst,
             std::string timecodelast,
             DetectionParameters params);

    //! save the VisualEventSet to the file saveXMLEventSetName
    /*! @param ves Visual Event Set
    @param currframe current frame we are writing
    @param timecode current frame timecode
    @param fr frame range for this event set
    */
    void saveVisualEventSetToXML(std::list<VisualEvent *> &ves,
                   int currframe,
                   std::string timecode,
                   FrameRange fr);

    //! save the VisualEventList to the file SaveEventsSummaryName
    void saveVisualEventSummary(std::string versionString,
                  std::list<VisualEvent *> &ves);

    //! save the positions to the file SavePositionsName
    void savePositions(const std::list<VisualEvent *> &ves) const;

    //! parse the SaveEventNumString and store the numbers in itsSaveEventNums
    void parseSaveEventNums(const std::string& value);

    //! make the file stem for saving files
    std::string getFileStem(const std::string& resultName, const int resNum = -1);

    //! check if save all event clips is set
    bool isSaveAllEventClips() const;

    OModelParam<FrameRange> itsInputFrameRange;
    OModelParam<std::string> itsInputFrameSource;
    OModelParam<std::string> itsOutputFrameSink;
    OModelParam<std::string> itsLoadEventsName;
    OModelParam<std::string> itsLoadPropertiesName;
    OModelParam<std::string> itsMetadataSource;
    OModelParam<bool> itsSaveBoringEvents; //! whether to save non-interesting/boring events
    OModelParam<std::string> itsSaveEventsName;
    OModelParam<bool> itsSaveEventFeatures;
    OModelParam<std::string> itsSaveEventNumString;
    OModelParam<bool> itsSaveOriginalFrameSpec; //! True if saving output in the original (raw) frame specification
    OModelParam<bool> itsSaveOutput;      //! whether the output frames are saved
    OModelParam<std::string> itsSavePositionsName;
    OModelParam<std::string> itsSavePropertiesName;
    OModelParam<std::string> itsSaveSummaryEventsName;
    OModelParam<std::string> itsSaveXMLEventSetName;
    OModelParam<int> itsPadEvents;
    nub::soft_ref<InputFrameSeries> itsIfs;
    nub::soft_ref<OutputFrameSeries> itsOfs;

    MbariXMLParser* itsXMLParser;
    std::vector<uint> itsSaveEventNums;
    FrameRange itsFrameRange;
    bool itsXMLfileCreated;
    bool itsAppendEvt, itsAppendEvtSummary, itsAppendEvtXML, itsAppendProperties, itsSaveEventNumsAll;
    float itsScaleW, itsScaleH;
    int itsPad;
    Dims itsDims;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // LOGGER_C_DEFINED
