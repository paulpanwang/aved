//============================================================================
// Name        : BoxObjectDetection.C
// Author      : Mayra Ochoa & Raymond Esteybar
// Version     :
// Copyright   :
// Description : Alternate version of ObjectDetection.H but BoxObjectDetection.H
//				 incorporates the Rectangles instead of Winner objects
//				 from .xml.
//============================================================================

#include "Component/OptionManager.H"
#include "Component/ParamClient.H"
#include "DetectionAndTracking/MbariFunctions.H"
#include "DetectionAndTracking/BoxObjectDetection.H"
#include "Image/ColorOps.H"
#include "Image/Image.H"
#include "Image/FilterOps.H"
#include "Image/PixelsTypes.H"
#include "Util/StringConversions.H"

class ModelParamBase;
class DetectionParameters;

using namespace std;

BoxObjectDetection::BoxObjectDetection(OptionManager& mgr,
      const std::string& descrName,
      const std::string& tagName)
      : ModelComponent(mgr, descrName, tagName)
{

}
void BoxObjectDetection::start1(){

}
BoxObjectDetection::~BoxObjectDetection()
{}

void BoxObjectDetection::paramChanged(ModelParamBase* const param,
                                 const bool valueChanged,
                                 ParamClient::ChangeStatus* status){

}

std::list<BitObject> BoxObjectDetection::run(
    nub::soft_ref<MbariResultViewer> rv,
    const std::list<VOCObject> &objectList,
    const Image< PixRGB<byte> > &segmentInImg)
{
	DetectionParameters p = DetectionParametersSingleton::instance()->itsParameters;
    float maxCost = p.itsMaxCost;
    std::list<BitObject> bos;
    std::list<VOCObject>::const_iterator iter = objectList.begin();

    LINFO("%ld objects", objectList.size());

    //go through each object and find largest object per each
    while (iter != objectList.end()) {

        Rectangle region = (*iter).getDimensions();
        std::string name = (*iter).getName();
        float probability = (*iter).getProbability();

        Dims segmentDims = Dims((float)region.width()*DEFAULT_SEGMENT_FACTOR,(float)region.height()*DEFAULT_SEGMENT_FACTOR);
        Dims searchDims = Dims((float)region.width()*DEFAULT_SEARCH_FACTOR,(float)region.height()*DEFAULT_SEARCH_FACTOR);
        Rectangle segmentRegion = Rectangle::centerDims(region.center(), segmentDims);
        Rectangle searchRegion = Rectangle::centerDims(region.center(), searchDims);
        segmentRegion = segmentRegion.getOverlap(Rectangle(Point2D<int>(0, 0), segmentInImg.getDims() - 1));
        searchRegion = searchRegion.getOverlap(Rectangle(Point2D<int>(0, 0), segmentInImg.getDims() - 1));
        int boxArea = region.width()*region.height();
        int minSegmentArea = max(p.itsMinEventArea, boxArea/8);
        int maxSegmentArea = min(p.itsMaxEventArea, boxArea);
        Point2D<int> unusedSeed;
        std::list<BitObject> sobjs = extractBitObjects(segmentInImg, unusedSeed, searchRegion, segmentRegion, minSegmentArea, maxSegmentArea);

        // find the object with the lowest cost
        LINFO("Finding best candidate bit object");
        if (sobjs.size() > 0) {
            std::list<BitObject>::iterator best = sobjs.begin();
            float lCost = -1.0F;
            int maxArea = 0;

            // find the largest bit object close to the bounding box dimensions
            for(std::list<BitObject>::iterator it = sobjs.begin(); it != sobjs.end(); ++it) {
                // calculate cost function as distance between bounding box corners
                int area = it->getArea();
                Rectangle r1 = it->getBoundingBox();
                Rectangle r2 = region;
                float cost1 = sqrt(pow((double)(r1.top() - r2.top()),2.0) +  pow((double)(r1.left() - r2.left()),2.0));
                float cost2 = sqrt(pow((double)(r1.bottomI() - r2.bottomI()),2.0) + pow((double)(r1.rightI() - r2.rightI()),2.0));
                float cost = cost1 + cost2;
                if (it->isValid() && ((lCost == -1.0F) || (cost < maxCost && cost < lCost && area > maxArea)) ) {
                    best = it;
                    maxArea = area;
                    lCost = cost;
                    LDEBUG("Found lower cost %f max cost: %f area: %d class: %s prob: %f", lCost, maxCost, area, name.c_str(), probability);
                }
            }
            if (best->isValid()) {
                LINFO("Found best cost %f max cost: %f area: %d class: %s prob: %f", lCost, maxCost, best->getArea(), name.c_str(), probability);
                best->setClassProbability(name, probability);
                bos.push_back(*best);
            }
        }
        /*else { //otherwise set the entire box as the object
             Image<byte> bo(segmentInImg.getDims(), ZEROS);
             const byte value = byte(1);
             for (int ry = region.top(); ry <= region.bottomI(); ++ry)
                 for (int rx = region.left(); rx <= region.rightI(); ++rx)
                     bo.setVal(Point2D<int>(rx,ry), value);
             BitObject obj(bo);
             float maxI, minI, avgI;
             if (obj.isValid()) {
                 obj.setMaxMinAvgIntensity(luminance(segmentInImg));
                 obj.getMaxMinAvgIntensity(maxI, minI, avgI);
                 obj.setClassProbability(name, probability);
                 bos.push_back(obj);
             }
        }*/
        ++iter;
    }

    LINFO("Found total %lu objects", bos.size());
    for(std::list<BitObject>::iterator it = bos.begin(); it != bos.end(); ++it) {
        LINFO("area: %d bbox: %s class: %s prob: %f", it->getArea(), toStr(it->getBoundingBox()).data(),
                it->getClassName().c_str(), it->getClassProbability());
    }
    return bos;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */