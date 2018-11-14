#include "DetectionAndTracking/ObjectDetection.H"

#include "Component/OptionManager.H"
#include "Component/ParamClient.H"
#include "DetectionAndTracking/MbariFunctions.H"
#include "Image/ColorOps.H"
#include "Image/Image.H"
#include "Image/FilterOps.H"
#include "Image/PixelsTypes.H"
#include "Util/StringConversions.H"

class ModelParamBase;
class DetectionParameters;

// ######################################################################
// ObjectDetection member definitions:
// ######################################################################

// ######################################################################
ObjectDetection::ObjectDetection(OptionManager& mgr,
      const std::string& descrName,
      const std::string& tagName)
      : ModelComponent(mgr, descrName, tagName)
{

}

// ######################################################################
ObjectDetection::~ObjectDetection()
{  }

// ######################################################################
void ObjectDetection::start1()
{

}

// ######################################################################
void ObjectDetection::paramChanged(ModelParamBase* const param,
                                 const bool valueChanged,
                                 ParamClient::ChangeStatus* status)
{

}

// ######################################################################
BitObject ObjectDetection::findBestCandidate(const Rectangle &region,  const std::list<BitObject> &bos)
{
    // find the object with the lowest cost
    LINFO("Finding best candidate bit object");
    std::list<BitObject>::const_iterator best = bos.begin();
    float lCost = -1.0F;
    int maxArea = 0;
	DetectionParameters p = DetectionParametersSingleton::instance()->itsParameters;
    float maxCost = p.itsMaxCost;
    BitObject b = *best;

    // find the largest bit object close to the bounding box dimensions
    for(std::list<BitObject>::const_iterator it = bos.begin(); it != bos.end(); ++it) {
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
            LDEBUG("Found lower cost %f max cost: %f area: %d ", lCost, maxCost, area);
        }
    }
    if (best->isValid()) {
        LINFO("Found best cost %f max cost: %f area: %d ", lCost, maxCost, best->getArea());
        b = *best;
    }
    return b;
}

// ######################################################################
std::list<BitObject> ObjectDetection::run( nub::soft_ref<MbariResultViewer> rv,
    const uint frameNum,
    const std::list<Winner> &winlist,
    const Image< PixRGB<byte> > &segmentInImg)
{
    DetectionParameters p = DetectionParametersSingleton::instance()->itsParameters;
    std::list<BitObject> bosFiltered;
    std::list<BitObject> bosUnfiltered;
    std::list<Winner>::const_iterator winIter = winlist.begin();

    //go through each winner and extract salient objects
    while (winIter != winlist.end()) {

        // get the foa mask
        BitObject boFOA = (*winIter).getBitObject();
        WTAwinner winner = (*winIter).getWTAwinner();

        // if the foa mask area is too small, we aren't going to find any large enough objects so bail out
        if (boFOA.getArea() <  p.itsMinEventArea) {
            winIter++;
            continue;
        }

        Rectangle foaregion = boFOA.getBoundingBox();
        Point2D<int> center = boFOA.getCentroid();
        Dims d = segmentInImg.getDims();
        Dims segmentDims = Dims((float)foaregion.width()*4.,(float)foaregion.height()*4.);
        Dims searchDims = Dims((float)foaregion.width()*2.,(float)foaregion.height()*2.);
        Rectangle searchRegion = Rectangle::centerDims(center, searchDims);
        searchRegion = searchRegion.getOverlap(Rectangle(Point2D<int>(0, 0), segmentInImg.getDims() - 1));
        Rectangle segmentRegion = Rectangle::centerDims(center, segmentDims);
        segmentRegion = segmentRegion.getOverlap(Rectangle(Point2D<int>(0, 0), segmentInImg.getDims() - 1));

        LINFO("Extracting bit objects from frame %d winning point [%d;%d]/region %s minSize %d maxSize %d segment dims %dx%d", \
               (*winIter).getFrameNum(), winner.p.i, winner.p.j, convertToString(searchRegion).c_str(),
               p.itsMinEventArea, p.itsMaxEventArea, d.w(), d.h());

        std::list<BitObject> sobjs = extractBitObjectsDisplay(rv, frameNum, segmentInImg, center, searchRegion,
                                                                segmentRegion, p.itsMinEventArea, p.itsMaxEventArea);
        LINFO("Found %lu bitobject(s) in FOA region", sobjs.size());

        if (sobjs.size() > 0) {
            BitObject obj = findBestCandidate(foaregion, sobjs);
            // set the winning voltage
            obj.setSMV(winner.sv);
            bosUnfiltered.push_back(obj);
        }

        winIter++;
    }// end while winIter != winners.end()

    LINFO("Found %lu bitobject(s)", bosUnfiltered.size());

    bool found = false;
    int minSize = p.itsMinEventArea;
    if (p.itsRemoveOverlappingDetections) {
        LINFO("Removing overlapping detections");
        // loop until we find all non-overlapping objects starting with the smallest
        while (!bosUnfiltered.empty()) {

            std::list<BitObject>::iterator biter, siter, smallest;
            // find the smallest object
            smallest = bosUnfiltered.begin();
            for (siter = bosUnfiltered.begin(); siter != bosUnfiltered.end(); ++siter)
                if (siter->getArea() < minSize) {
                    minSize = siter->getArea();
                    smallest = siter;
                }

            // does the smallest object intersect with any of the already stored ones
            found = true;
            for (biter = bosFiltered.begin(); biter != bosFiltered.end(); ++biter) {
                if (smallest->isValid() && biter->isValid() && biter->doesIntersect(*smallest)) {
                    // no need to store intersecting objects -> get rid of smallest
                    // and look for the next smallest
                    bosUnfiltered.erase(smallest);
                    found = false;
                    break;
                }
            }

            if (found && smallest->isValid())
                bosFiltered.push_back(*smallest);
        }
    }
    else {
        LINFO("Keeping all valid %lu bitobject(s)", bosUnfiltered.size());
        std::list<BitObject>::iterator biter;
        for (biter = bosUnfiltered.begin(); biter != bosUnfiltered.end(); ++biter) {
            if (biter->isValid())
                bosFiltered.push_back(*biter);
        }
    }

    LINFO("Found total %lu objects", bosFiltered.size());
    return bosFiltered;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
