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
#include "Data/MbariOpts.H"
#include "Media/MbariResultViewer.H"
#include "Image/colorDefs.H"
#include "Image/DrawOps.H"
#include "Image/ShapeOps.H"
#include "Image/SimpleFont.H"
#include "Image/PixelsTypes.H"
#include "Image/Transforms.H"
#include "Raster/GenericFrame.H"
#include "Transport/FrameInfo.H"
#include "DetectionAndTracking/VisualEventSet.H"
#include "GUI/XWinManaged.H"

using namespace std;

// #############################################################################
MbariResultViewer::MbariResultViewer(ModelManager &mgr)
        : ModelComponent(mgr, string("MbariResultViewer"),
                         string("MbariResultViewer")),
          itsDisplayResults(&OPT_MRVdisplayResults, this),
          itsMarkCandidate(&OPT_MRVmarkCandidate, this),
          itsMarkFOE(&OPT_MRVmarkFOE, this),
          itsMarkInteresting(&OPT_MRVmarkInteresting, this),
          itsMarkPrediction(&OPT_MRVmarkPrediction, this),
          itsOpacity(&OPT_MRVopacity, this),
          itsRescaleDisplay(&OPT_MRVrescaleDisplay, this),
          itsSaveBoringEvents(&OPT_MDPsaveBoringEvents, this),
          itsSaveResults(&OPT_MRVsaveResults, this),
          itsShowEventLabels(&OPT_MRVshowEventLabels, this),
          colInteresting(COL_INTERESTING),
          colCandidate(COL_CANDIDATE),
          colPrediction(COL_PREDICTION){
    displayResults = false;
}


MbariResultViewer::~MbariResultViewer() {
    // destroy everything
    freeMem();
}

// ######################################################################
void MbariResultViewer::paramChanged(ModelParamBase *const param,
                                     const bool valueChanged,
                                     ParamClient::ChangeStatus *status) {


    // if the param is out itsMarkCandidate set the color accordingly
    // if the param is set to Save all non-interesting events, mark candidates
    // the same as interesting to be less confusing
    if (param == &itsMarkCandidate) {
        if (itsMarkCandidate.getVal() && itsSaveBoringEvents.getVal())
            colCandidate = COL_INTERESTING;
        else if (itsMarkCandidate.getVal() && !itsSaveBoringEvents.getVal())
            colCandidate = COL_CANDIDATE;
        else colCandidate = COL_TRANSPARENT;
    }
        // if the param is out itsMarkSkip set the color accordingly
    else if (param == &itsMarkPrediction) {
        if (itsMarkPrediction.getVal()) colPrediction = COL_PREDICTION;
        else colPrediction = COL_TRANSPARENT;
    }
        // if the param is out itsMarkSkip set the color accordingly
    else if (param == &itsMarkFOE) {
        if (itsMarkFOE.getVal()) colFOE = COL_FOE;
        else colFOE = COL_TRANSPARENT;
    }
}

// ######################################################################
void MbariResultViewer::reset1() {
    // destroy our stuff
    freeMem();

    // propagate to our base class:
    ModelComponent::reset1();
}

// ######################################################################
void MbariResultViewer::freeMem() {
    for (uint i = 0; i < itsResultWindows.size(); ++i)
        if (itsResultWindows[i] != NULL) delete itsResultWindows[i];

    itsResultNames.clear();
    itsResultWindows.clear();
}

// #############################################################################
template<class T>
void MbariResultViewer::output(nub::soft_ref <OutputFrameSeries> ofs,
                               const Image<T> &img, const uint frameNum,
                               const string &resultName, const int resNum) {
    if (itsDisplayResults.getVal()) display(img, frameNum, resultName, resNum);
    if (itsSaveResults.getVal()) save(ofs, img, frameNum, resultName, resNum);
}

// #############################################################################
void MbariResultViewer::save(nub::soft_ref <OutputFrameSeries> ofs,
                             const Image< PixRGB<byte> > &img,
                             const uint frameNum,
                             const string &resultName,
                             const int resNum) {
    string fs = sformat("%s%02d_", resultName.c_str(), resNum);
    ofs->writeFrame(GenericFrame(img), fs, FrameInfo(resultName, SRC_POS));
}

// #############################################################################
void MbariResultViewer::save(nub::soft_ref <OutputFrameSeries> ofs,
                             const Image<byte> &img,
                             const uint frameNum,
                             const string &resultName,
                             const int resNum) {
    string fs = sformat("%s%02d_", resultName.c_str(), resNum);
    ofs->writeFrame(GenericFrame(img), fs, FrameInfo(resultName, SRC_POS));
}

// #############################################################################
void MbariResultViewer::save(nub::soft_ref <OutputFrameSeries> ofs,
                             const Image<float> &img,
                             const uint frameNum,
                             const string &resultName,
                             const int resNum) {
    string fs = sformat("%s%02d_", resultName.c_str(), resNum);
    ofs->writeFrame(GenericFrame(img, FLOAT_NORM_0_255), fs, FrameInfo(resultName, SRC_POS));
}

// #############################################################################
Image<PixRGB <byte> > MbariResultViewer::createOutput(MbariImage< PixRGB<byte> >& resultImg,
                                          VisualEventSet& evts,
                                          const int circleRadius, float scaleW, float scaleH) {
    MbariMetaData m = resultImg.getMetaData();
    Image< PixRGB<byte> > final_image = resultImg;
    evts.drawTokens(final_image, resultImg.getFrameNum(), circleRadius,
                    itsMarkInteresting.getVal(), itsOpacity.getVal(),
                    colInteresting, colCandidate, colPrediction,
                    colFOE,
                    itsShowEventLabels.getVal(),
                    itsMarkCandidate.getVal(),
                    itsSaveBoringEvents.getVal(),
                    scaleW, scaleH);

    // get timecode string
    string tc = m.getTC();
    string textboxstring;

    // if there is a timecode, create the text box and overlay on the image
    ostringstream os;
    if (tc.length() > 0) {
        os << tc << "/" << resultImg.getFrameNum();
    } else
        os << resultImg.getFrameNum();

    textboxstring = os.str();

    // create a text box scaled from 720x480
    Image< PixRGB<byte> > textImg;
    const Dims d = resultImg.getDims();
    const int numW = (10 * d.w()) / 720;
    const int numH = (25 * d.h()) / 480;
    const int fntH = (20 * d.h()) / 480;

    // create the timecode text adding padding extra 10 pixels to ensure fits
    textImg.resize(numW * textboxstring.length() + 10, numH, NO_INIT);
    textImg.clear(COL_WHITE);

    // set the maximum font height. This may not necessarily
    // be the maximum height, but will match the largest
    // that is closest to fntH
    const SimpleFont f = SimpleFont::fixedMaxHeight(fntH);

    writeText(textImg, Point2D<int>(0, 0), textboxstring.c_str(), COL_BLACK,
              COL_WHITE, f, true);

    // overlay the timecodeImg onto the final image
    Point2D<int> low_right(final_image.getWidth() - textImg.getWidth(),
                           final_image.getHeight() - textImg.getHeight());

    pasteImage(final_image, textImg, COL_TRANSPARENT, low_right, itsOpacity.getVal());

    return final_image;
}

// #############################################################################
template<class T>
void MbariResultViewer::display(const Image <T> &img, const uint frameNum,
                                const string &resultName, const int resNum) {
    if (itsDisplayResults.getVal()) {
        uint num = getNumFromString(resultName);
        itsResultWindows[num] = displayImage(img, itsResultWindows[num],
                                             getLabel(num, frameNum, resNum).c_str());
    }
}

// #############################################################################
template<class T>
XWinManaged *MbariResultViewer::displayImage(const Image <T> &img,
                                             XWinManaged *win,
                                             const char *label) {
    // need to rescale?
    Dims dims = itsRescaleDisplay.getVal();
    if (dims.isEmpty()) dims = img.getDims();
    bool doRescale = (dims != img.getDims());

    // does the window have to be re-constructed?
    if (win != NULL) {
        if (win->getDims() != dims) delete win;
    }

    if (win == NULL) {
        if (doRescale)
            win = new XWinManaged(rescale(img, dims), label);
        else
            win = new XWinManaged(img, label);
    } else {
        if (doRescale)
            win->drawImage(rescale(img, dims));
        else
            win->drawImage(img);

        win->setTitle(label);
    }

    return win;
}

// #############################################################################

    uint MbariResultViewer::getNumFromString(const string &resultName) {
        // see if we can find this guy in our list
        for (uint i = 0; i < itsResultNames.size(); ++i)
            if (itsResultNames[i].compare(resultName) == 0)
                return i;

        // didn't find it -> make a new entry and return index of this new entry
        itsResultNames.push_back(resultName);
        itsResultWindows.push_back(NULL);

        return (itsResultNames.size() - 1);
    }

// #############################################################################

string MbariResultViewer::getLabel(const uint num, const uint frameNum,
                                   const int resNum) {
    ASSERT(num < itsResultNames.size());
    char fnum[7];
    sprintf(fnum, "%06d", frameNum);
    return (sformat("%s%02d_", itsResultNames[num].c_str(), resNum) + string(fnum));
}

// #############################################################################
// Instantiations
#define INSTANTIATE(T) \
template void MbariResultViewer::output(nub::soft_ref<OutputFrameSeries> rv, \
                                        const Image< T >& img, \
                                        const uint frameNum, \
                                        const string& resultName, \
                                        const int resNum); \
template void MbariResultViewer::display(const Image< T >& img, \
                                         const uint frameNum, \
                                         const string& resultName, \
                                         const int resNum); \
template XWinManaged* MbariResultViewer::displayImage(const Image< T >& img, \
                                                      XWinManaged* win, \
                                                      const char* label);

INSTANTIATE(PixRGB<byte>);
INSTANTIATE(byte);
INSTANTIATE(float);
