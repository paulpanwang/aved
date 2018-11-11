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

/*!@file ObjectDetection.H used for object detection*/

#ifndef OBJECTDET_C_DEFINED
#define OBJECTDET_C_DEFINED

#include "Component/ModelManager.H"
#include "Component/ModelParam.H"
#include "Component/OptionManager.H"
#include "Data/Winner.H"
#include "Image/CutPaste.H"
#include "Image/FilterOps.H"
#include "Image/Kernels.H"
#include "Image/ImageCache.H"
#include "Image/Pixels.H"
#include "Image/PyramidOps.H"
#include "Media/FrameSeries.H"
#include "Media/MbariResultViewer.H"

#include <map>
#include <vector>
#include <list>

// ######################################################################
//! An object detection class
class ObjectDetection : public ModelComponent
{

public:
  //! Construct
  ObjectDetection(OptionManager& mgr, const std::string& descrName = "salientvision object detector",
                const std::string& tagName = "object detector");

  //! destructor
  virtual ~ObjectDetection();

  //! init
  void init(nub::soft_ref<InputFrameSeries> ifs);

  //! Overload so that we can reconfigure when our params get changed
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

  //! ObjectDetection
  std::list<BitObject> run(nub::soft_ref<MbariResultViewer> rv,
                const uint frameNum,
                const std::list<Winner> &winlist,
                const Image< PixRGB<byte> >& graphBitImg);


protected:

  //! overload start1()
  virtual void start1();

  //! find best candidate object for a given region
  BitObject findBestCandidate(const Rectangle &region,  const std::list<BitObject> &bos);

private:

};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // OBJECTDET_C_DEFINED
