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
#include "Image/ColorOps.H"
#include "Image/DrawOps.H"
#include "Image/MorphOps.H"
#include "Image/Image.H"
#include "Image/Kernels.H"      // for twofiftyfives()
#include "Image/Rectangle.H"
#include "Image/ShapeOps.H"
#include "Image/Transforms.H"
#include "Image/colorDefs.H"
#include "Image/Geometry2D.H"
#include "Util/Assert.H"
#include "Util/StringConversions.H"
#include "DetectionAndTracking/VisualEventSet.H"
#include "DetectionAndTracking/MbariFunctions.H"

#include <algorithm>
#include <istream>
#include <ostream>

using namespace std;

// ######################################################################
// ###### VisualEventSet
// ######################################################################
VisualEventSet::VisualEventSet(const DetectionParameters &parameters,
                               const string& fileName,
							   const bool &saveEventFeatures)
  : startframe(-1),
    endframe(-1),
    itsFileName(fileName),
    itsDetectionParms(parameters),
    itsSaveEventFeatures(saveEventFeatures)
{
}

// ######################################################################
VisualEventSet::~VisualEventSet()
{
  reset();
}

// ######################################################################
VisualEventSet::VisualEventSet(istream& is)
{
  readFromStream(is);
}

void VisualEventSet::readHeaderFromStream(istream& is)
{
  is >> itsFileName;
  is >> itsDetectionParms.itsMaxDist;
  is >> itsDetectionParms.itsMaxCost;
  is >> itsDetectionParms.itsMinEventFrames;
  is >> itsDetectionParms.itsMinEventArea;
  is >> startframe;
  is >> endframe;
}

void VisualEventSet::writeHeaderToStream(ostream& os)
{
  os << itsFileName << "\n";
  os << itsDetectionParms.itsMaxDist << " "
     << itsDetectionParms.itsMaxCost << " "
     << itsDetectionParms.itsMinEventFrames << " "
     << itsDetectionParms.itsMinEventArea << "\n";
  os << startframe << ' ' << endframe << '\n';

  os << "\n";
}

// ######################################################################
void VisualEventSet::writeToStream(ostream& os)
{
  list<VisualEvent *>::iterator currEvent;

  os << itsFileName << "\n";
  os << itsDetectionParms.itsMaxDist << " "
     << itsDetectionParms.itsMaxCost << " "
     << itsDetectionParms.itsMinEventFrames << " "
     << itsDetectionParms.itsMinEventArea << "\n";
  os << startframe << ' ' << endframe << '\n';

  for (currEvent = itsEvents.begin(); currEvent != itsEvents.end(); ++currEvent)
    (*currEvent)->writeToStream(os);

  os << "\n";
}


// ######################################################################
void VisualEventSet::readFromStream(istream& is)
{
  is >> itsFileName; LINFO("filename: %s",itsFileName.data());
  is >> itsDetectionParms.itsMaxDist;
  is >> itsDetectionParms.itsMaxCost;
  is >> itsDetectionParms.itsMinEventFrames;
  is >> itsDetectionParms.itsMinEventArea;
  is >> startframe;
  is >> endframe;

  itsEvents.clear();

  while (is.eof() != false)
    itsEvents.push_back(new VisualEvent(is));
}

// ######################################################################
void VisualEventSet::writePositions(ostream& os) const
{
  list<VisualEvent *>::const_iterator currEvent;
  for (currEvent = itsEvents.begin(); currEvent != itsEvents.end(); ++currEvent)
    (*currEvent)->writePositions(os);
}
// ######################################################################
void VisualEventSet::insert(VisualEvent *event)
{
  itsEvents.push_back(event);
}
// ######################################################################
void VisualEventSet::runKalmanHoughTracker(nub::soft_ref<MbariResultViewer>&rv, VisualEvent *currEvent,
                                           const BayesClassifier &bayesClassifier,
                                           FeatureCollection& features,
                                           ImageData& imgData)
{
 bool found = false;
 Token evtToken;
 Image<byte> se = twofiftyfives(itsDetectionParms.itsCleanupStructureElementSize);

  // prefer the Kalman tracker, and fall back to the Hough tracker
  currEvent->setTrackerType(VisualEvent::KALMAN);
  if (runBaseTracker(rv, currEvent, bayesClassifier, features, imgData, true)) {
	  found = true;
  }
  else {

    // only use the Hough tracker if object found to be interesting or has high enough voltage
    // skip the first frame as this may be a FOA mask
    if (!currEvent->isClosed() && (evtToken.bitObject.getSMV() > .002F ||
    							currEvent->getCategory() == VisualEvent::INTERESTING)){
      currEvent->setTrackerType(VisualEvent::HOUGH);

      // reset Hough tracker if only now switching to this tracker to save computation
      if (currEvent->trackerChanged()) {
        evtToken = currEvent->getToken(currEvent->getEndFrame());
        LINFO("Resetting Hough Tracker frame: %d event: %d with bounding box %s",
               imgData.frameNum,currEvent->getEventNum(),toStr(evtToken.bitObject.getBoundingBox()).data());
         Image<byte> mask = evtToken.bitObject.getObjectMask(byte(1));
         BitObject obj(rescale(mask, Dims(DEFAULT_HOUGH_WIDTH, DEFAULT_HOUGH_HEIGHT)));
         obj.setSMV(evtToken.bitObject.getSMV());
         Image< PixRGB<byte> > prevImgRescaled = rescale(imgData.prevImg, Dims(DEFAULT_HOUGH_WIDTH, DEFAULT_HOUGH_HEIGHT));
         if (obj.isValid()) {
        	 LINFO("Object valid. Resetting tracker");
             currEvent->resetHoughTracker(prevImgRescaled, obj);
         }
      }

      // try to run the Hough tracker
      found = runHoughTracker(rv, currEvent, bayesClassifier, features, imgData, true);
    }
  }

  if (!found) {
      LINFO("Event %i - Hough Tracker failed, closing event",currEvent->getEventNum());
      currEvent->close();
  }
}

// ######################################################################
void VisualEventSet::checkFailureConditions(VisualEvent *currEvent, Dims d)
{
  Token evtToken = currEvent->getToken(currEvent->getEndFrame());
  float c = currEvent->getForgetConstant();

  // if growing small quickly, turn down forget constant to avoid drift or
  // if too small hough cannot effectively find the edges
  if (currEvent->getNumberOfFrames() > 1) {
    Token evtTokenLast = currEvent->getToken(currEvent->getEndFrame()-1);
    uint maxArea = evtTokenLast.bitObject.getArea();

    if (evtToken.bitObject.getArea() < (int)((float)maxArea*.25F)) {
	  c *= 0.5F;
	  LINFO("Event %i growing small in Hough tracking mode. Changing forget ratio from %3.2f to %3.2f to avoid drift", \
	  currEvent->getEventNum(), currEvent->getForgetConstant(), c);
	  currEvent->setForgetConstant(c);
    }
    if (maxArea < 100) {//TODO put in DEFINE
    	LINFO("Event %i too small %d < %d in Hough tracking mode. Closing", currEvent->getEventNum(), maxArea,
    			itsDetectionParms.itsMinEventArea);
  	  currEvent->close();
    }
  }

  Rectangle r1 = evtToken.bitObject.getBoundingBox();
  //check distance to edge and size; if close to edge, turn down forget ratio
  if ( (r1.bottomI() >= d.h()-20 || r1.rightI() >= d.w()-20 || r1.top() <=20 || r1.left() <=20)) {
    c *= 0.8F;
    LINFO("Event %i near edge. Changing forget ratio from %3.2f to %3.2f  to avoid drift", \
    currEvent->getEventNum(), currEvent->getForgetConstant(), c);
    currEvent->setForgetConstant(c);
  }

  if (c < 0.10F) {
	  LINFO("Forget ratio too small. Closing event %3.2f", c);
	  currEvent->close();
  }
}

// ######################################################################
void VisualEventSet::runNearestNeighborHoughTracker(nub::soft_ref<MbariResultViewer>&rv,
                                                    VisualEvent *currEvent,
                                                    const BayesClassifier &bayesClassifier,
                                                    FeatureCollection& features,
                                                    ImageData& imgData)
{

  bool found = false;
  Token evtToken;

  // prefer the NN tracker, and fall back to the Hough tracker
  currEvent->setTrackerType(VisualEvent::NN);
  if (runBaseTracker(rv, currEvent, bayesClassifier, features, imgData, true)){
	  found = true;
  }
  else {
    evtToken = currEvent->getToken(currEvent->getEndFrame());

    // only use the Hough tracker if object found to be interesting or has high enough voltage
    if (!currEvent->isClosed() && (evtToken.bitObject.getSMV() > .002F ||
    							currEvent->getCategory() == VisualEvent::INTERESTING)){

      currEvent->setTrackerType(VisualEvent::HOUGH);

      // reset Hough tracker if only now switching to this tracker to save computation
      if (currEvent->trackerChanged()) {
        evtToken = currEvent->getToken(currEvent->getEndFrame());
        LINFO("Resetting Hough Tracker frame: %d event: %d with bounding box %s",
              imgData.frameNum,currEvent->getEventNum(),toStr(evtToken.bitObject.getBoundingBox()).data());
        Image<byte> mask = evtToken.bitObject.getObjectMask(byte(1));
        BitObject obj(rescale(mask, Dims(DEFAULT_HOUGH_WIDTH, DEFAULT_HOUGH_HEIGHT)));
        obj.setSMV(evtToken.bitObject.getSMV());
        Image< PixRGB<byte> > prevImgRescaled = rescale(imgData.prevImg, Dims(DEFAULT_HOUGH_WIDTH, DEFAULT_HOUGH_HEIGHT));
        if (obj.isValid())
          currEvent->resetHoughTracker(prevImgRescaled, obj);
      }

      // try to run the Hough tracker; if fails, close event
      found = runHoughTracker(rv, currEvent, bayesClassifier, features, imgData, true);
    }
  }

  if (!found) {
      LINFO("Event %i - Hough Tracker failed, closing event",currEvent->getEventNum());
      currEvent->close();
  }

}
// ######################################################################
bool VisualEventSet::runHoughTracker(nub::soft_ref<MbariResultViewer>&rv,
                                     VisualEvent *currEvent,
                                     const BayesClassifier &bayesClassifier,
                                     FeatureCollection& features,
                                     ImageData& imgData,
                                     bool skip)
{
  Dims houghDims(DEFAULT_HOUGH_WIDTH, DEFAULT_HOUGH_HEIGHT);
  DetectionParameters dp = DetectionParametersSingleton::instance()->itsParameters;
  Rectangle region;
  bool found = true;
  bool search = true;
  Token evtToken = currEvent->getToken(currEvent->getEndFrame());

  // if already have a token for this frame, run prediction on last bitobject
  if (currEvent->frameInRange(imgData.frameNum))
  	  search = false;

  // get the predicted location
  const Point2D<int> pred = predictionCheck(currEvent, imgData);

  if (currEvent->isClosed())
	  return false;

  bool occluded = false;
  LINFO("Frame %d class %s", currEvent->getEndFrame(), evtToken.bitObject.getClassName().c_str());
  // if an object intersects, create a mask for it
  float opacity = 1.0F;
  Image< byte > occlusionImg(imgData.img.getDims(), ZEROS);
  const byte black = byte(0);
  occlusionImg = highThresh(occlusionImg, byte(0), byte(255)); //invert image
  Image< byte > binaryImg(houghDims, ZEROS);
  uint intersectEventNum;
  BitObject obj;

  if (doesIntersect(evtToken.bitObject, &intersectEventNum, imgData.frameNum) &&
		  intersectEventNum != currEvent->getEventNum()) {
	LINFO("Event %i - Hough Tracker intersection with event %i",currEvent->getEventNum(),\
																intersectEventNum);
	VisualEvent* vevt = getEventByNumber(intersectEventNum);
	BitObject intersectObj = vevt->getToken(imgData.frameNum).bitObject;
	intersectObj.drawShape(occlusionImg, black, opacity);
	occluded = true;
  }

  // then apply the mask
  occlusionImg = maskArea(occlusionImg, imgData.mask);

  // if need to find bit objects
  if (search) {

	  // calculate the scaling factors for adjusting input to the Hough tracker
	  Dims imgDims = imgData.img.getDims();
	  float scaleW = (float) houghDims.w() / (float) imgDims.w();
	  float scaleH = (float) houghDims.h() / (float) imgDims.h();

	  // get the region used for searching for a match based on the dimension of the last token
	  // centered on the predicted location
	  Rectangle rect = evtToken.bitObject.getBoundingBox();
	  Point2D<int> predHough((float)pred.i * scaleW, (float)pred.j * scaleH);
	  Dims searchDimsHough = Dims((float)rect.width() * scaleW,(float)rect.height() * scaleH);
	  Rectangle searchRegion = Rectangle::centerDims(predHough, searchDimsHough);
	  searchRegion = searchRegion.getOverlap(Rectangle(Point2D<int>(0, 0), houghDims - 1));
	  LINFO("Search region %i %s ", currEvent->getEventNum(),toStr(searchRegion).data());

	  if (!searchRegion.isValid()) {
		LINFO("Event %i - Hough Tracker invalid search region ",currEvent->getEventNum());
		if (!skip)
		  currEvent->close();

		return false;
	  }

	  Image< PixRGB<byte> > imgRescaled = rescale(imgData.img, houghDims);
	  Image< byte > occlusionImgRescaled = rescale(occlusionImg, houghDims);

	  LINFO("Running Hough Tracker for event %d", currEvent->getEventNum());
	  if (!currEvent->updateHoughTracker(rv, imgData.frameNum, imgRescaled,
			  occlusionImgRescaled, binaryImg, searchRegion)) {
		  if (!skip) {
			LINFO("Event %i - Hough Tracker failed, closing event",currEvent->getEventNum());
			currEvent->close();
		  }
		  return false;
	  }
	  // rescale back to original image dimensions
	  obj.reset(rescale(binaryImg, imgDims));
  }
  else {
	  obj = evtToken.bitObject;
  }

  if (!obj.isValid()) {
	  LINFO("Invalid object in Hough tracker");
	  currEvent->close();
	  return false;
  }

  float cost = -1.f;// if the first frame in the event, cost should be zero
  if (!search && currEvent->getNumberOfFrames() == 1)
	cost = 0.0F;
  else {
	  switch (dp.itsTrackingMode) {
		case (TMNearestNeighborHough):
			if (!search && currEvent->getNumberOfFrames() > 1)
				cost = currEvent->getCostNN(Token(obj, imgData.frameNum - 1));
			else
				cost = currEvent->getCostNN(Token(obj, imgData.frameNum));
		break;
		case(TMKalmanHough):
		case(TMHough):
			if (!search && currEvent->getNumberOfFrames() > 1)
				cost = currEvent->getCostKalman(Token(obj, imgData.frameNum - 1));
			else
				cost = currEvent->getCostKalman(Token(obj, imgData.frameNum));
		break;
		case(TMKalmanFilter):
		case(TMNearestNeighbor):
			LFATAL("Invalid mode, must be KalmanHough or NearestNeighborHough or Hough");
		break;
	  }
	}

  // cost too high but skip if occluded since this shifts the centroid
  if (cost > itsDetectionParms.itsMaxCost && !occluded ) {
	LINFO("Event %i - no token found, event cost: %g maxCost: %g ",
			  currEvent->getEventNum(), cost, itsDetectionParms.itsMaxCost);
	found = false;
  }

  // check for failure conditions unique to Hough-based algorithm
  checkFailureConditions(currEvent, imgData.img.getDims());

  if (currEvent->isClosed())
	  found = false;

  LINFO("%d %d", imgData.img.getDims().w(), obj.getImageDims().w());
  obj.setMaxMinAvgIntensity(luminance(imgData.img));
  obj.setClassProbability(evtToken.bitObject.getClassName(), evtToken.bitObject.getClassProbability());

  // skip over this when running multiple trackers and let the multiple tracker algorithm decide
  if (!skip && !found) {
	  LINFO("Switching trackers");
	  return false;
  }

  if (found) {
      // associate the best fitting one; compute features if requested
	  FeatureCollection::Data feature;
	  if (itsSaveEventFeatures)
		  feature = features.extract(obj.getBoundingBox(), imgData);
      Token tk(obj, imgData.frameNum, imgData.metadata, feature.featureJETred,
               feature.featureJETgreen, feature.featureJETblue,
               feature.featureHOG3,  feature.featureHOG8);
      LINFO("Event %i - token found at %g, %g area: %d",currEvent->getEventNum(),
            tk.location.x(),
            tk.location.y(),
            tk.bitObject.getArea());
      if (search) {
      	LINFO("Assigning %s to frame %d", tk.bitObject.getClassName().c_str(), tk.frame_nr);
      	tk.bitObject.computeSecondMoments();
      	currEvent->assign(tk, imgData.foe);
      }
      else {
      	LINFO("Updating prediction %s", tk.bitObject.getClassName().c_str());
      	currEvent->updatePrediction(tk, imgData.foe);
      }

    }
    else
  	  currEvent->close();

 return found;
}

BitObject VisualEventSet::getBitObjectPrediction(const Token &evtToken, const Point2D<int> &pred)
{
	Point2D<int> center = evtToken.bitObject.getCentroid();
	int shift_i = center.i - pred.i;
	int shift_j = center.j - pred.j;
	BitObject objPred; Image<byte> mask;
	LINFO("Shifting object from [%d;%d] to [%d;%d]", center.i,center.j, pred.i, pred.j);
	// if no shift, just return last bitobject
	if (center == pred) {
	    LINFO("No shift, returning last bitobject");
	    return evtToken.bitObject;
	}

	// shift the last bitobject to the predicted location
	mask = evtToken.bitObject.getObjectMask(byte(1));
	Dims d = mask.getDims();
	int w = d.w();
	int h = d.h();
	Image<byte> maskShifted(d,ZEROS);
	for(int i = 0;i< w;i++)
		for(int j = 0;j<h;j++){
			int ix = shift_i + i;
			int iy = shift_j + j;
			if(mask.coordsOk(ix,iy))
				maskShifted.setVal(i,j,mask.getVal(ix,iy));
		}

	objPred.reset(maskShifted);
	return objPred;
}

// ######################################################################
BitObject VisualEventSet::findBestCandidate(const Rectangle &region,  const std::list<BitObject> &bos)
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

list<BitObject> VisualEventSet::getBitObjects(nub::soft_ref<MbariResultViewer>&rv, VisualEvent *currEvent,
                                                const Point2D<int>& center,  const Image< PixRGB<byte> >& segmentImg) {
	list<BitObject> objs;
	Token evtToken = currEvent->getToken(currEvent->getEndFrame());
	LINFO("Searching for event %i objects", currEvent->getEventNum());

	// get the region used for searching for a match based on the dimension of the last token
	Rectangle r1 = evtToken.bitObject.getBoundingBox();
	Dims segmentDims = Dims((float)r1.width()*DEFAULT_SEGMENT_FACTOR,(float)r1.height()*DEFAULT_SEGMENT_FACTOR);
	Dims searchDims = Dims((float)r1.width()*DEFAULT_SEGMENT_FACTOR,(float)r1.height()*DEFAULT_SEGMENT_FACTOR);
	Rectangle segmentRegion = Rectangle::centerDims(center, segmentDims);
	Rectangle searchRegion = Rectangle::centerDims(center, searchDims);
	segmentRegion = segmentRegion.getOverlap(Rectangle(Point2D<int>(0, 0), segmentImg.getDims() - 1));
	searchRegion = searchRegion.getOverlap(Rectangle(Point2D<int>(0, 0), segmentImg.getDims() - 1));

	if (!searchRegion.isValid() || !segmentRegion.isValid() ) {
		LINFO("Invalid region. Closing event %i", currEvent->getEventNum());
		currEvent->close();
		return objs;
	}

	float minArea, maxArea, minIntersect;

	// constrain bit object size and intersection
    minArea =  int((float) evtToken.bitObject.getArea()*.5f);
    maxArea =  int((float) evtToken.bitObject.getArea()*2.0f);

	// extract bit objects removing those that fall outside area set by previous bitobject
	objs = extractBitObjectsDisplay(rv, currEvent->getEndFrame(), segmentImg, center, searchRegion, segmentRegion,
	                                minArea, maxArea);
	return objs;
}

// ######################################################################
const Point2D<int> VisualEventSet::predictionCheck(VisualEvent* currEvent,
		ImageData& imgData) {
	// get the predicted location
	const Point2D<int> pred = currEvent->predictedLocation();
	LINFO("Event %i prediction: %d,%d", currEvent->getEventNum(), pred.i, pred.j);
	// is the prediction too far outside the image?
	int gone = itsDetectionParms.itsMaxDist;
	if ((pred.i < -gone) || (pred.i >= (imgData.segmentImg.getWidth() + gone))
			|| (pred.j < -gone)
			|| (pred.j >= (imgData.segmentImg.getHeight() + gone))) {
		currEvent->close();
	LINFO("Event %i out of bounds - closed", currEvent->getEventNum());
	}
	return pred;
}

// ######################################################################
Image<PixRGB<byte> > VisualEventSet::occlusionCheck(ImageData& imgData,
		Token& evtToken, VisualEvent* currEvent, bool& occluded) {
	const byte black(0);
	float opacity = 1.0F;
	uint intersectEventNum;
	Image<byte> occlusionImg(imgData.segmentImg.getDims(), ZEROS);
	occlusionImg = highThresh(occlusionImg, byte(0), byte(255)); //invert image
	// if an object intersects, create a mask for it
	if (doesIntersect(evtToken.bitObject, &intersectEventNum,
			imgData.frameNum) && intersectEventNum != currEvent->getEventNum()) {
		LINFO("Event %i - Tracker intersection with event %i",
				currEvent->getEventNum(), intersectEventNum);
		VisualEvent* vevt = getEventByNumber(intersectEventNum);
		BitObject intersectObj = vevt->getToken(imgData.frameNum).bitObject;
		intersectObj.drawShape(occlusionImg, black, opacity);
		occluded = true;
		LINFO("Object occluded area %d %d", intersectObj.getArea(), evtToken.bitObject.getArea());
	}
	Image<PixRGB<byte> > segmentImg = maskArea(imgData.segmentImg, occlusionImg);
	return segmentImg;
}

// ######################################################################
bool VisualEventSet::runBaseTracker(nub::soft_ref<MbariResultViewer> rv,
                                      VisualEvent *currEvent,
                                      const BayesClassifier &bayesClassifier,
                                      FeatureCollection& features,
                                      ImageData& imgData,
                                      bool skip)
{
  bool search = true;

  // if already have a token for this frame, run prediction on last bitobject
  if (currEvent->frameInRange(imgData.frameNum))
	  search = false;

  // get a copy of the last token in this event for prediction
  Token evtToken = currEvent->getToken(currEvent->getEndFrame());

  // get the predicted location
  const Point2D<int> pred = predictionCheck(currEvent, imgData);

  if (currEvent->isClosed())
	  return false;

  bool occluded = false;
  occlusionCheck(imgData, evtToken, currEvent, occluded);

  // adjust prediction if negative
  const Point2D<int> center =  Point2D<int>(max(pred.i,0), max(pred.j,0));

  // now find which one fits best
  list<BitObject> objs;

  // if need to find bit objects
  if (search) {
	  objs = getBitObjects(rv, currEvent, center, imgData.segmentImg);
	  LINFO("SEARCHING");
	  }
  else
	  objs.push_back(evtToken.bitObject);

  LINFO("pred. location: [%s]; Number of extracted objects: %ld",  toStr(pred).data(),objs.size());

  float maxCost = itsDetectionParms.itsMaxCost;
  bool found = false;
  float lCost = -1.0F;
  int bestArea = 0;
  int area = 0;
  float areaIntersect = 0.;
  float cost = -1.f;
  BitObject predObj = getBitObjectPrediction(evtToken, pred);
  int predArea = 0;
  list<BitObject>::iterator cObj, lObj = objs.begin();
  for (cObj = objs.begin(); cObj != objs.end(); ++cObj) {
    list<BitObject>::iterator next = cObj;
    ++next;
    area = (*cObj).getArea();

	switch (currEvent->getTrackerType()) {
	case (VisualEvent::NN):
		if (!search && currEvent->getNumberOfFrames() > 0)
			cost = currEvent->getCostNN(Token(*cObj, imgData.frameNum - 1));
		else
			cost = currEvent->getCostNN(Token(*cObj, imgData.frameNum));
	break;
	case(VisualEvent::KALMAN):
		if (!search && currEvent->getNumberOfFrames() > 0)
			cost = currEvent->getCostKalman(Token(*cObj, imgData.frameNum - 1));
		else
			cost = currEvent->getCostKalman(Token(*cObj, imgData.frameNum));
	break;
	case(VisualEvent::HOUGH):
		LFATAL("Cannot run base tracker for Hough tracker. Run Hough directly instead.");
	break;
	}

	// if the first frame in the event, cost should be zero and no predicted area
    if (currEvent->getNumberOfFrames() == 1) {
    	cost = 0.0F;
    	predArea = 0;
    }
    else
        predArea = predObj.getArea(); 

    if (cost < 0.0F) {
      LINFO("Erasing object, cost too low");
      objs.erase(cObj);
      cObj = next;
      continue;
    }

    // if cost valid and a larger object or one with lower cost
    if ((lCost == -1.0F) || (cost < maxCost && cost <= lCost)) {
      lCost = cost;
      lObj = cObj;
      bestArea = area;
      found = true;
      if (predArea > 0)
        areaIntersect = (float) predObj.intersect(*cObj) / (float) predArea;
    }
    
	LINFO("lCost %g cost %g max cost %g intersection %g size %d pred size %d", lCost, cost, maxCost, 
	                                                                            areaIntersect, bestArea, predArea);
  }

  // cost too high but skip if occluded since this shifts the centroid
  if ( (lCost > maxCost || lCost == -1.0)  && !occluded ) {
    LINFO("Event %i - no token found, event cost: %g maxCost: %g ",
              currEvent->getEventNum(), lCost, maxCost);
    found = false;
  }

  // skip over this when running multiple trackers and let the multiple tracker algorithm decide
  if (skip && !found) {
	  LINFO("Switching trackers");
	  objs.clear();
	  return false;
  }

  if (found) {
    // associate the best fitting one; compute features if requested
    FeatureCollection::Data feature;
    if (itsSaveEventFeatures)
		  features.extract((*lObj).getBoundingBox(), imgData);
    Token tk(*lObj, imgData.frameNum, imgData.metadata, feature.featureJETred,
             feature.featureJETgreen, feature.featureJETblue,
             feature.featureHOG3,  feature.featureHOG8);
    LINFO("Event %i - token found at [%g;%g] area: %d",currEvent->getEventNum(),
          tk.location.x(),
          tk.location.y(),
          tk.bitObject.getArea());
    if (search) {
    	LINFO("Assigning %s", tk.bitObject.getClassName().c_str());
    	tk.bitObject.computeSecondMoments();
    	currEvent->assign(tk, imgData.foe);
    }
    else {
      	LINFO("Updating prediction %s intersection %f", tk.bitObject.getClassName().c_str(), areaIntersect);
    	currEvent->updatePrediction(tk, imgData.foe);
    }
  }
  else
	  currEvent->close();

  objs.clear();
  return found;

}

// ######################################################################
void VisualEventSet::getAreaRange(int &minArea, int &maxArea)
{
  list<VisualEvent *>::iterator currEvent;
  int currMinArea = itsDetectionParms.itsMinEventArea;
  int currMaxArea = itsDetectionParms.itsMaxEventArea;

  for (currEvent = itsEvents.begin(); currEvent != itsEvents.end(); ++currEvent) {
    if((*currEvent)->getMaxSize() > currMaxArea)
     currMaxArea = (*currEvent)->getMaxSize();

    if((*currEvent)->getMinSize() < currMinArea)
     currMinArea = (*currEvent)->getMinSize();
  }
}

// ######################################################################
float VisualEventSet::getAcceleration(uint skipEventNum)
{
  list<VisualEvent *>::iterator currEvent;
  float sumAccel = 0.F;
  int i = 0;

  for (currEvent = itsEvents.begin(); currEvent != itsEvents.end(); ++currEvent) {
    if((*currEvent)->getEventNum() != skipEventNum) {
     sumAccel += (*currEvent)->getAcceleration();
     i++;
     }
  }
  if (i > 0)
    return sumAccel/(float)i;
  return 0.F;
}


// ######################################################################
void VisualEventSet::updateEvents(nub::soft_ref<MbariResultViewer>&rv,
                                  const BayesClassifier &bayesClassifier,
                                  FeatureCollection& features,
                                  ImageData& imgData)
{
  if (startframe == -1) {startframe = (int) imgData.frameNum; endframe = (int) imgData.frameNum;}
  if ((int) imgData.frameNum > endframe) endframe = (int) imgData.frameNum;

  list<VisualEvent *>::iterator currEvent;

  for (currEvent = itsEvents.begin(); currEvent != itsEvents.end(); ++currEvent)
    if ((*currEvent)->isOpen()) {
      switch(itsDetectionParms.itsTrackingMode) {
      case(TMKalmanFilter):
      case(TMNearestNeighbor):
		runBaseTracker(rv, *currEvent, bayesClassifier, features, imgData);
        break;
      case(TMHough):
        runHoughTracker(rv, *currEvent, bayesClassifier, features, imgData);
        break;
      case(TMNearestNeighborHough):
        runNearestNeighborHoughTracker(rv, *currEvent, bayesClassifier, features, imgData);
        break;
      case(TMKalmanHough):
        runKalmanHoughTracker(rv, *currEvent, bayesClassifier, features, imgData);
        break;
      case(TMNone):
        break;
      default:
        LFATAL("Invalid tracking mode");
        break;
      }
    }
}

// ######################################################################
void VisualEventSet::initiateEvents(list<BitObject>& bos,
                                    FeatureCollection& features,
                                    ImageData &imgData)
{
  DetectionParameters p = DetectionParametersSingleton::instance()->itsParameters;

  if (startframe == -1) {startframe = imgData.frameNum; endframe = imgData.frameNum;}
  if (imgData.frameNum > endframe) endframe = imgData.frameNum;
  list<BitObject> sobjs;
  // reset object if is there an intersection with an event
  list<BitObject>::iterator biter;
  for (biter = bos.begin(); biter != bos.end(); ++biter) {
	  if (!resetIntersect(imgData.img, *biter, imgData.foe, imgData.frameNum)) { 
		 sobjs.push_back(*biter);
     }
  }
 
  // now go through all the remaining BitObjects and create new events for them
  list<BitObject>::iterator currObj;
  Point2D<int> center;
  for (currObj = sobjs.begin(); currObj != sobjs.end(); ++currObj)
    {
	  FeatureCollection::Data feature;
	  if (itsSaveEventFeatures)
		  feature = features.extract(currObj->getBoundingBox(), imgData);
      Token token = Token(*currObj, imgData.frameNum, imgData.metadata, feature.featureJETred,
                          feature.featureJETgreen, feature.featureJETblue,
                          feature.featureHOG3, feature.featureHOG8);
      itsEvents.push_back(new VisualEvent(token, itsDetectionParms, imgData.img));
      center = currObj->getCentroid();
      LINFO("assigning object found at [%d;%d] area: %i to new event %i frame %d class %s prob %f",
            center.i, center.j, currObj->getArea(), itsEvents.back()->getEventNum(), imgData.frameNum, 
            currObj->getClassName().c_str(), currObj->getClassProbability());
    }
}

// ######################################################################
bool VisualEventSet::resetIntersect(Image< PixRGB<byte> >& img, BitObject& obj, const Vector2D& curFOE, int frameNum)
{
  // ######## Initialization of variables, reading of parameters etc.
  DetectionParameters dp = DetectionParametersSingleton::instance()->itsParameters;
  list<VisualEvent *>::iterator cEv;
  Token evtToken;
  BitObject predObj;
  Image< PixRGB<byte> > imgRescaled = rescale(img, Dims(DEFAULT_HOUGH_WIDTH, DEFAULT_HOUGH_HEIGHT));
  Image<byte> se = twofiftyfives(dp.itsCleanupStructureElementSize);
  std::string className = obj.getClassName();
  float classProb = obj.getClassProbability();
  Point2D<int> pred;

  for (cEv = itsEvents.begin(); cEv != itsEvents.end(); ++cEv) {
	evtToken = (*cEv)->getToken((*cEv)->getEndFrame());
	pred = (*cEv)->predictedLocation();
	predObj = getBitObjectPrediction(evtToken, pred);

	if (predObj.doesIntersect(obj)) {
		evtToken = (*cEv)->getToken((*cEv)->getEndFrame());
		std::string evtClassName = evtToken.bitObject.getClassName();
		float area = (float) (predObj.getArea());
		float areaIntersect = (float) predObj.intersect(obj) / area;

		LINFO("Reset frame: %d class: %s probability: %f intersect %f",
				frameNum, className.c_str(), classProb,
				areaIntersect);

		LINFO("Event %i in frame %d intersects %f with event class %s "
				"as class %s probability %f at predicted location %d,%d",
				(*cEv)->getEventNum(), frameNum, areaIntersect, evtClassName.c_str(),
				className.c_str(), classProb, pred.i, pred.j);

		if (areaIntersect > MIN_INTERSECTION) {
				if ((*cEv)->getEndFrame() == frameNum) {
                    Token prevEvtToken = (*cEv)->getToken(frameNum - 1);
                    float prevAreaIntersect = (float) prevEvtToken.bitObject.intersect(obj) /
                                             (float) (prevEvtToken.bitObject.getArea());

                    // not a better match if intersecting area is less than this intersection
                    // and not strong probability
                    LINFO("Found previous valid intersecting event intersect %f with probability %f",
                            prevAreaIntersect, classProb);
                    if (prevAreaIntersect > areaIntersect && classProb < 0.9f)
                        return true;
                    LINFO("Found better matching object with intersection %f > %f. Remove last intersecting event.",
                            areaIntersect,prevAreaIntersect);
                    (*cEv)->removeLast();
				}
				if ((*cEv)->getTrackerType() != VisualEvent::HOUGH) {
					LINFO("Assigning frame %d event %i with area %f class %s probability %f",
									frameNum, (*cEv)->getEventNum(), area,
									className.c_str(), classProb);
					Token newToken = evtToken;
					newToken.frame_nr = frameNum;
					Image<byte> mask = obj.getObjectMask(byte(1)) + predObj.getObjectMask(byte(1));
					obj.reset(mask);
                    obj.setClassProbability(className, classProb);
					newToken.bitObject = obj;
					(*cEv)->assignNoPrediction(newToken, curFOE);
				}
				else {
					// reset Hough tracker and assign bitobject
					predObj.reset(rescale(obj.getObjectMask(byte(1)),imgRescaled.getDims()));

					if (predObj.isValid()){
						LINFO("Resetting Hough Tracker frame: %d event: %d with bit object in bounding box %s"
								" with class %s probability %f ",
								frameNum,(*cEv)->getEventNum(),toStr(obj.getBoundingBox()).data(),
								className.c_str(), classProb);
						(*cEv)->resetHoughTracker(imgRescaled, predObj);
						Token newToken = evtToken;
						newToken.frame_nr = frameNum;
						obj.setClassProbability(className, classProb);
						newToken.bitObject = obj;
						(*cEv)->assignNoPrediction(newToken, curFOE);
					}
					else
						LFATAL("Invalid Hough mask");

				}// end else
		}//end intersect conditions
		return true;
    }//end does intersect
  }//end of events
  return false;
}

// ######################################################################
bool VisualEventSet::doesIntersect(BitObject& obj, int frameNum)
{
  list<VisualEvent *>::iterator cEv;
  for (cEv = itsEvents.begin(); cEv != itsEvents.end(); ++cEv)
      if ((*cEv)->doesIntersect(obj,frameNum)) {
      // reset the SMV for this bitObject
      Token  evtToken = (*cEv)->getToken(frameNum);
      evtToken.bitObject.setSMV(obj.getSMV());
      return true;
    }
  return false;
}

// ######################################################################
bool VisualEventSet::doesIntersect(BitObject& obj, uint* eventNum, int frameNum)
{
  list<VisualEvent *>::iterator cEv;
  for (cEv = itsEvents.begin(); cEv != itsEvents.end(); ++cEv)
    // return the first object that intersects
    if ((*cEv)->doesIntersect(obj,frameNum)) {
      *eventNum = (*cEv)->getEventNum();
      return true;
    }
  return false;
}

// ######################################################################
uint VisualEventSet::numEvents() const
{
  return itsEvents.size();
}

// ######################################################################
void VisualEventSet::reset()
{
  itsEvents.clear();
}

// ######################################################################
void VisualEventSet::replaceEvent(uint eventnum, VisualEvent *event)
{
  list<VisualEvent *>::iterator currEvent = itsEvents.begin();
  while (currEvent != itsEvents.end())  {
    if((*currEvent)->getEventNum() == eventnum) {
      itsEvents.insert(currEvent, event);
      delete *currEvent;
      itsEvents.erase(currEvent);
      return;
    }
    ++currEvent;
  }
  LFATAL("Event %d does not exist in event list cannot replace", eventnum);
}
// ######################################################################
void VisualEventSet::cleanUp(uint currFrame, uint lastFrame)
{
  list<VisualEvent *>::iterator currEvent = itsEvents.begin();

  while(currEvent != itsEvents.end()) {
    list<VisualEvent *>::iterator next = currEvent;
    ++next;

    switch((*currEvent)->getState())
      {
      case(VisualEvent::DELETE):
        LINFO("Erasing event %i", (*currEvent)->getEventNum());
        delete *currEvent;
        itsEvents.erase(currEvent);
        break;
      case(VisualEvent::WRITE_FINI):
        LINFO("Event %i flagged as written", (*currEvent)->getEventNum());
        break;
      case(VisualEvent::CLOSED):
        LINFO("Event %i flagged as closed", (*currEvent)->getEventNum());
        break;
      case(VisualEvent::OPEN):
        if (itsDetectionParms.itsMaxEventFrames > 0 && currFrame > ((*currEvent)->getStartFrame() + itsDetectionParms.itsMaxEventFrames)){
          //limit event to itsMaxFrames
          LINFO("Event %i reached max frame count:%d - flagging as closed", (*currEvent)->getEventNum(),\
                itsDetectionParms.itsMaxEventFrames);
          (*currEvent)->close();
        }
        break;
      default:
        //this event is still within the window of itsMaxFrames
        LDEBUG("Event %d still open", (*currEvent)->getEventNum());
        break;
      }

    currEvent = next;
  } // end for loop over events
}

// ######################################################################
void VisualEventSet::closeAll()
{
  list<VisualEvent *>::iterator cEvent;
  for (cEvent = itsEvents.begin(); cEvent != itsEvents.end(); ++cEvent)
    (*cEvent)->close();
}
// ######################################################################
void VisualEventSet::printAll()
{
  list<VisualEvent *>::iterator cEvent;
  for (cEvent = itsEvents.begin(); cEvent != itsEvents.end(); ++cEvent)
    {
      LINFO("EVENT %d sframe %d eframe %d numtokens %d",
            (*cEvent)->getEventNum(),
            (*cEvent)->getStartFrame(),
            (*cEvent)->getEndFrame(),
            (*cEvent)->getNumberOfTokens());
    }
}
// ######################################################################
vector<Token> VisualEventSet::getTokens(uint frameNum)
{
  vector<Token> tokens;
  list<VisualEvent *>::iterator currEvent;
  for (currEvent = itsEvents.begin(); currEvent != itsEvents.end(); ++currEvent)
    {
      // does this guy participate in frameNum?
      if (!(*currEvent)->frameInRange(frameNum)) continue;

      tokens.push_back((*currEvent)->getToken(frameNum));
    } // end loop over events

  return tokens;
}

// ######################################################################
void VisualEventSet::drawTokens(Image< PixRGB<byte> >& img,
                                uint frameNum,
                                int circleRadius,
                                BitObjectDrawMode mode,
                                float opacity,
                                PixRGB<byte> colorInteresting,
                                PixRGB<byte> colorCandidate,
                                PixRGB<byte> colorPred,
                                PixRGB<byte> colorFOE,
                                bool showEventLabels,
                                bool showCandidate,
                                bool saveNonInterestingEvents,
                                float scaleW,
                                float scaleH)
{
  // dimensions of the number text and location to put it at
  const int numW = 10;
  const int numH = 21;
  Token tk;

  list<VisualEvent *>::iterator currEvent;
  for (currEvent = itsEvents.begin(); currEvent != itsEvents.end(); ++currEvent)
  {
      // does this guy  participate in frameNum ? and
      // if also saving non-interesting events and this is BORING event, be sure to save this
      // otherwise, save all INTERESTING events
      if( (*currEvent)->frameInRange(frameNum) &&
          ( (saveNonInterestingEvents && (*currEvent)->getCategory() == VisualEvent::BORING ) ||
          (*currEvent)->getCategory() == VisualEvent::INTERESTING  ||
          showCandidate ) )
        {
          PixRGB<byte> circleColor;
          tk = (*currEvent)->getToken(frameNum);

          if(!tk.location.isValid())
            continue;

          Point2D<int> center = tk.location.getPoint2D();
          center.i *= scaleW;
          center.j *= scaleH;

          if ((*currEvent)->getCategory() == VisualEvent::INTERESTING)
            circleColor = colorInteresting;
          else
            circleColor = colorCandidate;

          // if requested, prepare the event labels
          Image< PixRGB<byte> > textImg;
          if (showEventLabels)
            {
              // write the text and create the overlay
              string numText = toStr((*currEvent)->getEventNum());
              ostringstream ss;
              ss.precision(2);
              ss << numText;
              if (tk.bitObject.getClassProbability() >= 0.F && tk.bitObject.getClassProbability() <= 1.0F) {
                ss << "," << tk.bitObject.getClassName();
                ss << "," << tk.bitObject.getClassProbability();
              }
              //ss << numText << "," << (*currEvent)->getForgetConstant();

              textImg.resize(numW * ss.str().length(), numH, NO_INIT);
              //textImg.resize(numW * numText.length(), numH, NO_INIT);
              textImg.clear(COL_WHITE);
              writeText(textImg, Point2D<int>(0,0), ss.str().data());
              //writeText(textImg, Point2D<int>(0,0), numText.data());
            }

          // draw the event object itself if requested
          if (circleColor != COL_TRANSPARENT)
            {
              // the box so that the text knows where to go
              Rectangle bbox;

              // draw rectangle or circle and determine the pos of the number label
              if (tk.bitObject.isValid())
                {
                  tk.bitObject.draw(mode, img, circleColor, opacity);
                  bbox = tk.bitObject.getBoundingBox(BitObject::IMAGE);
                  Point2D<int> topleft((float)(bbox.left())*scaleW, (float)(bbox.top())*scaleH);
                  Dims dims((float)(bbox.width())*scaleW, (float)(bbox.height())*scaleH);
                  bbox = Rectangle(topleft, dims);
                  bbox = bbox.getOverlap(img.getBounds());
                }
              else
                {
                  LINFO("BitObject has invalid: area: %i;",tk.bitObject.getArea());
                  LFATAL("bounding box: %s",toStr(tk.bitObject.getBoundingBox()).data());
                  drawCircle(img, center, circleRadius, circleColor);
                  bbox = Rectangle::tlbrI(center.j - circleRadius, center.i - circleRadius,
                                          center.j + circleRadius, center.i + circleRadius);
                  bbox = bbox.getOverlap(img.getBounds());
                }

              // if requested, write the event labels into the image
              if (showEventLabels && bbox.isValid())
                {
                  Point2D<int> numLoc = getLabelPosition(img.getDims(),bbox,textImg.getDims());
                  Image<PixRGB <byte> > textImg2 = replaceVals(textImg,COL_BLACK,circleColor);
                  textImg2 = replaceVals(textImg2,COL_WHITE,COL_TRANSPARENT);
                  pasteImage(img,textImg2,COL_TRANSPARENT, numLoc, opacity);
                } // end if (showEventLabels)

            } // end if we're not transparent

          // now do the same for the predicted value
          if ((colorPred != COL_TRANSPARENT) && tk.prediction.isValid())
            {
              Point2D<int> ctr = tk.prediction.getPoint2D();
              ctr.i *= scaleW;
              ctr.j *= scaleH;
              Rectangle ebox =
                Rectangle::tlbrI(ctr.j - circleRadius, ctr.i - circleRadius, ctr.j + circleRadius, ctr.i + circleRadius);
                  ebox = ebox.getOverlap(img.getBounds());

                  // round down the radius in case near the edges
                  if (ebox.isValid() && ebox.width() > 0 && ebox.height() > 0) {
                      int radius = (int) sqrt(pow(ebox.width(), 2.0) + pow(ebox.height(), 2.0))/2;
                      drawCircle(img, ctr, radius, colorPred);
                      if (showEventLabels) {
                          Point2D<int> numLoc = getLabelPosition(img.getDims(), ebox, textImg.getDims());
                          Image< PixRGB<byte> > textImg2 = replaceVals(textImg, COL_BLACK, colorPred);
                          textImg2 = replaceVals(textImg2, COL_WHITE, COL_TRANSPARENT);
                          pasteImage(img, textImg2, COL_TRANSPARENT, numLoc, opacity);
                      }
                  }
            }

        }
    } // end loop over events

  if ((colorFOE != COL_TRANSPARENT) && tk.foe.isValid())
    {
      Point2D<int> ctr = tk.foe.getPoint2D();
      ctr.i *= scaleW;
      ctr.j *= scaleH;
      drawDisk(img, ctr,2,colorFOE);
    }
}


// ######################################################################
Point2D<int> VisualEventSet::getLabelPosition(Dims imgDims,
                                         Rectangle bbox,
                                         Dims textDims) const
{
  // distance of the text label from the bbox
  const int dist = 2;

  Point2D<int> loc(bbox.left(),(bbox.top() - dist - textDims.h()));

  // not enough space to the right? -> shift as appropriate
  if ((loc.i + textDims.w()) > imgDims.w())
    loc.i = imgDims.w() - textDims.w() - 1;

  // not enough space on the top? -> move to the bottom
  if (loc.j < 0)
    loc.j = bbox.bottomI() + dist;

  return loc;
}

// ######################################################################
PropertyVectorSet VisualEventSet::getPropertyVectorSet()
{
  PropertyVectorSet pvs;

  list<VisualEvent *>::iterator currEvent;
  for (currEvent = itsEvents.begin(); currEvent != itsEvents.end();
       ++currEvent)
    pvs.itsVectors.push_back((*currEvent)->getPropertyVector());

  return pvs;
}

// ######################################################################
PropertyVectorSet VisualEventSet::getPropertyVectorSetToSave()
{
  PropertyVectorSet pvs;
  list<VisualEvent *>::iterator currEvent;
  for (currEvent = itsEvents.begin(); currEvent != itsEvents.end();
       ++currEvent) {
    if((*currEvent)->isClosed()) {
      pvs.itsVectors.push_back((*currEvent)->getPropertyVector());
    }
  }

  return pvs;
}

// ######################################################################
int VisualEventSet::getAllClosedFrameNum(uint currFrame)
{
  list<VisualEvent *>::iterator currEvent;
  for (int frame = (int)currFrame; frame >= -1; --frame)
    {
      bool done = true;

      for (currEvent = itsEvents.begin(); currEvent != itsEvents.end();
           ++currEvent)
        {
          done &= ((frame < (int)(*currEvent)->getStartFrame())
                   || (*currEvent)->isClosed());
          if (!done) break;
        }

      if (done) return frame;
    }
  return -1;
}

// ######################################################################
bool VisualEventSet::doesEventExist(uint eventNum) const
{
  list<VisualEvent *>::const_iterator evt;
  for (evt = itsEvents.begin(); evt != itsEvents.end(); ++evt)
    if ((*evt)->getEventNum() == eventNum) return true;

  return false;
}
// ######################################################################
VisualEvent *VisualEventSet::getEventByNumber(uint eventNum) const
{
  list<VisualEvent *>::const_iterator evt;
  for (evt = itsEvents.begin(); evt != itsEvents.end(); ++evt)
    if ((*evt)->getEventNum() == eventNum) return *evt;

  LFATAL("Event with number %i does not exist.",eventNum);

  return *evt;
}
// ######################################################################
list<VisualEvent *>
VisualEventSet::getEventsReadyToSave(uint framenum)
{
  list<VisualEvent *> result;
  list<VisualEvent *>::iterator evt;
  for (evt = itsEvents.begin(); evt != itsEvents.end(); ++evt)
    if ((*evt)->isClosed()) result.push_back(*evt);

  return result;
}

// ######################################################################
list<VisualEvent *>
VisualEventSet::getEventsForFrame(uint framenum)
{
  list<VisualEvent *> result;
  list<VisualEvent *>::iterator evt;
  for (evt = itsEvents.begin(); evt != itsEvents.end(); ++evt)
    if ((*evt)->frameInRange(framenum)) result.push_back(*evt);

  return result;
}


// ######################################################################
list<BitObject>
VisualEventSet::getBitObjectsForFrame(uint framenum)
{
  list<BitObject> result;
  list<VisualEvent *>::iterator evt;

  for (evt = itsEvents.begin(); evt != itsEvents.end(); ++evt)
    if ((*evt)->frameInRange(framenum))
      if((*evt)->getToken(framenum).bitObject.isValid())
        result.push_back((*evt)->getToken(framenum).bitObject);

  return result;
}

// ######################################################################
const int VisualEventSet::minSize()
{
  return itsDetectionParms.itsMinEventArea;
}
