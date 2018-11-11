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

#ifndef MBARI_RESULTVIEWER_H_
#define MBARI_RESULTVIEWER_H_

#include "Component/ModelManager.H"
#include "Component/ModelComponent.H"
#include "Component/ModelOptionDef.H"
#include "Image/MbariImage.H"
#include "Image/BitObjectDrawModes.H"
#include "Media/FrameSeries.H"
#include "Image/PixelsTypes.H"

#include <string>
#include <vector>

class XWinManaged;
class VisualEventSet;

template <class T> class PixRGB;
template <class T> class Image;

// ######################################################################
//! Handles everything regarding displaying and saving results
class MbariResultViewer : public ModelComponent {
public:

    //! Constructor - give a ModelManager and an OutputFrameSeries
    /* for both the events and and full-frame output */
    MbariResultViewer(ModelManager& mgr);

    // ! Destructor
    virtual ~MbariResultViewer();

    //! implements ModelComponent::paramChanged
    /*! intercepts setting of MRVmarkXXX variables and sets drawing
      colors accordingly */
    virtual void paramChanged(ModelParamBase *const param,
                              const bool valueChanged,
                              ParamClient::ChangeStatus *status);

    //! destroy all the windows
    virtual void reset1();

    //! save and/or display image
    /*! whether the image is saved and/or displayed is
      determined by the ModelParams
      @param ofs the output frame series to use to save to
      @param img the image containing the intermediate result
      @param frameNum the frame number of the image
      @param resultName a string that defines the kind of intermediate
      result - this is appended to the filestem for the file name
      and/or window title
      @param resNum if there are several results of this type for each frame,
      the number of the result*/
    template<class T>
    void output(nub::soft_ref <OutputFrameSeries> ofs, const Image <T> &img,
                const uint frameNum, const std::string &resultName, const int resNum = -1);

    //! save intermediate result
    /*!@param img the image containing the intermediate result
      @param frameNum the frame number of the image
      @param resultName a string that defines the kind of interemediate
      result - this is appended to the filestem for the file name
      and/or window title
      @param resNum if there are several results of this type for each frame,
      the number of the resul*/
    void save(nub::soft_ref <OutputFrameSeries> ofs, const Image <PixRGB <byte> > &img, const uint frameNum,
              const std::string &resultName, const int resNum = -1);

    //! save intermediate result
    /*!@param img the image containing the intermediate result
      @param frameNum the frame number of the image
      @param resultName a string that defines the kind of interemediate
      result - this is appended to the filestem for the file name
      and/or window title
      @param resNum if there are several results of this type for each frame,
      the number of the resul*/
    void save(nub::soft_ref <OutputFrameSeries> ofs, const Image <byte> &img, const uint frameNum,
              const std::string &resultName, const int resNum = -1);

    //! save intermediate result
    /*!@param img the image containing the intermediate result
      @param frameNum the frame number of the image
      @param resultName a string that defines the kind of interemediate
      result - this is appended to the filestem for the file name
      and/or window title
      @param resNum if there are several results of this type for each frame,
      the number of the resul*/
    void save(nub::soft_ref <OutputFrameSeries> ofs, const Image<float> &img, const uint frameNum,
              const std::string &resultName, const int resNum = -1);

    //! Create image output
    /*!Depending on the ModelParams, interesting events, candidates
      for interesting events and skipped event locations are marked
      in the frame. Returns the generated image
      @param resultImg the image to overlay the output on
      @param evts the event set to be used for drawing the events
      @param circleRadius the radius of the circles used for marking*/
    Image <PixRGB <byte> > createOutput(MbariImage<PixRGB <byte> >& resultImg,
                                        VisualEventSet &evts,
                                        const int circleRadius,
                                        float scaleW = 1.0F,
                                        float scaleH = 1.0F);


    //! display image
    /*!@param img the image containing the image
      @param frameNum the frame number of the image
      @param resultName a string that defines what the image represents to put in the window title
      @param resNum if there are several results of this type for each frame,
      the number of the result*/
    template<class T>
    void display(const Image <T> &img, const uint frameNum,
                 const std::string &resultName, const int resNum = -1);

    //! true if results should be contrast enhanced
    bool contrastEnhance();

protected:

    //! destroy windows and other internal variables
    void freeMem();

    //! get an internal reference number from a resultName
    /*! If this resultName is new, a new reference is created*/
    uint getNumFromString(const std::string &resultName);

    //! make a label text from the reference number and the frame number
    std::string getLabel(const uint num, const uint frameNum, const int resNum = -1);

    //! draw an image to a window, deal with creating and rescaling the window
    template<class T>
    XWinManaged *displayImage(const Image <T> &img,
                              XWinManaged *win,
                              const char *label);

private:

    OModelParam<bool> itsDisplayResults;  //!<whether windows displayed are stored. useful for debugging.
    OModelParam<bool> itsMarkCandidate;   //!<whether candidates for int. events are marked
    OModelParam<bool> itsMarkFOE;         //!<whether the focus of expansion is marked
    OModelParam <BitObjectDrawMode> itsMarkInteresting; //!<the way interesting events are marked
    OModelParam<bool> itsMarkPrediction;  //!<whether predictions of event locations are marked
    OModelParam<float> itsOpacity;        //!<opacity of shape or outline markings of events
    OModelParam <Dims> itsRescaleDisplay;  //!<rescale intermediate results to this scale for display
    OModelParam<bool> itsSaveBoringEvents; //! whether to save non-interesting/boring events
    OModelParam<bool> itsSaveResults;     //!<whether intermediate results are saved
    OModelParam<bool> itsShowEventLabels; //!<whether the event labels are written into output frames

    PixRGB<byte> colInteresting, colCandidate, colPrediction, colFOE;
    std::vector <std::string> itsResultNames;
    std::vector<XWinManaged *> itsResultWindows;
    XWinManaged *resFrameWindow;
    bool displayResults;
};
#endif /*MBARI_RESULTVIEWER_H_*/
