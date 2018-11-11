//============================================================================
// Name        : BoxObjectDetection.H
// Author      : Mayra Ochoa & Raymond Esteybar
// Version     :
// Copyright   :
// Description : Alternate version of ObjectDetection.H but BoxObjectDetection.H
//				 incorporates the Rectangles instead of Winner objects
//				 from .xml.
//============================================================================

/*!@file BOXOBJECTDETECTION.H used for object detection*/
#ifndef SRC_BOXOBJECTDETECTION_H_
#define SRC_BOXOBJECTDETECTION_H_

#include <map>
#include <vector>
#include <list>
#include <string>

#include "Component/ModelManager.H"
#include "Component/ModelParam.H"
#include "Component/OptionManager.H"
#include "Data/Winner.H"
#include "Data/VOC.H"
#include "DetectionAndTracking/DetectionParameters.H"
#include "Image/CutPaste.H"
#include "Image/FilterOps.H"
#include "Image/Kernels.H"
#include "Image/ImageCache.H"
#include "Image/Pixels.H"
#include "Image/PyramidOps.H"
#include "Media/FrameSeries.H"
#include "Media/MbariResultViewer.H"
#include "Image/Rectangle.H" //New rectangle


class BoxObjectDetection : public ModelComponent //Ask if we need to inheritance ModelComponent
{
public:
	BoxObjectDetection(OptionManager& mgr, const std::string& descrName = "pascalvocvision box object detector",
            const std::string& tagName = "box object detector");
	//! destructor
	virtual ~BoxObjectDetection();

	 //! init
	void init(nub::soft_ref<InputFrameSeries> ifs);

	  //! Overload so that we can reconfigure when our params get changed
	virtual void paramChanged(ModelParamBase* const param,
	                            const bool valueChanged,
	                            ParamClient::ChangeStatus* status);

	  //! BoxObjectDetection
	std::list<BitObject> run(nub::soft_ref<MbariResultViewer> rv,
					const std::list<VOCObject> &objectList,
					const Image< PixRGB<byte> > &segmentInImg);

	protected:

	 //! overload start1()
	 virtual void start1();

	private:
};

#endif /* SRC_BOXOBJECTDETECTION_H_ */

