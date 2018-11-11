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

#ifndef PREPROCESS_C_DEFINED
#define PREPROCESS_C_DEFINED

#include <map>
#include <vector>
#include <list>

#include "Component/ModelManager.H"
#include "Component/ModelParam.H"
#include "Component/OptionManager.H"
#include "Data/Winner.H"
#include "Image/CutPaste.H"
#include "Image/FilterOps.H"
#include "Image/Kernels.H"
#include "Image/ImageCache.H"
#include "Image/MbariImage.H"
#include "Image/Pixels.H"
#include "Image/PyramidOps.H"
#include "Media/FrameSeries.H"

// ######################################################################
//! Preprocessing class that create cache of incoming frames and contrast enhances
class Preprocess : public ModelComponent
{

public:
  //! Construct
  Preprocess(OptionManager& mgr, const std::string& descrName = "image preprocess", const std::string& tagName = "preprocessor");

  //! destructor
  virtual ~Preprocess();

  //! initialize cache using the @param ifs Input frame series
  void init(nub::soft_ref<InputFrameSeries> ifs, const Dims rescaledDims);

  //! Overload so that we can reconfigure when our params get changed
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);


  //! Update the cache using previous bit objects found then
  // return the latest image encapsulated in the MbariImage which may metadata like timecode
  MbariImage< PixRGB<byte> > update(const Image< PixRGB<byte> >& img, const Image< PixRGB<byte> >& prevImg,
                                    const uint frameNum, const std::list<BitObject> bitObjectFrameList);

  //! Return background image
  Image< PixRGB<byte> > background(const Image< PixRGB<byte> >& img, const Image< PixRGB<byte> >& prevImg,
                                            const uint frameNum, const std::list<BitObject> bitObjectFrameList);

  //! Returns the absolute difference between the image and the cache mean
  Image< PixRGB<byte> > absDiffMean(Image< PixRGB<byte> >& image);

  //! Returns the absolute difference between the image and the cache mean
  Image< PixRGB<byte> > clampedDiffMean(Image< PixRGB<byte> >& image);

  //! Returns the cache mean
  Image< PixRGB<byte> > mean();

  //! Contrast enhance using adaptive gamma
  Image< PixRGB<byte> > contrastEnhance(const Image< PixRGB<byte> >& img);

protected:

  //! overload start1()
  virtual void start1();

private:

  //! Update the cache and the model
  void update(const Image< PixRGB<byte> >& img, const uint framenum, bool updateModel=false);

  //! Checks the entropy of the image to flag when gamma needs adjusting
  void checkEntropy(Image< PixRGB<byte> >& img);

  // ! Adjust gamma  in place in @hsvRes image
  void adjustGamma(Image<byte>&lum, std::map<int, double> &cdfw, Image<PixHSV<float> >&hsvRes);

  // ! Contrast enhance image
  Image<PixRGB<byte> > enhanceImage(const Image<PixRGB<byte> >& img, std::map<int, double> &cdfw);

  // ! Update the mapping curve for contrast enhancement; returns the cumulative distribution function
  std::map<int, double> updateGammaCurve(const Image<PixRGB<byte> >& img, std::map<int, double> &pdf,  bool init = true);

  // ! Update the entropy model for contrast enhancement; returns the entropy approximation
  float updateEntropyModel(const Image<PixRGB<byte> >& img, std::map<int, double> &pdf);

  //! Input frame source
  OModelParam<std::string> itsFrameSource;
  OModelParam<int> itsSizeAvgCache;
  OModelParam<float> itsMinStdDev; //! minimum std dev for image to be included in averaging cache

  ImageCacheAvg< PixRGB<byte> > itsAvgCache;
  std::map<int, double> itspdf;
  std::map<int, double> itscdfw;
  float itsPrevEntropy;
  uint itsMinFrame;

};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // PREPROCESS_C_DEFINED
