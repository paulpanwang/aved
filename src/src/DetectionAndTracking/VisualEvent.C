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
#include "Image/Image.H"
#include "Image/Rectangle.H"
#include "Image/ShapeOps.H"
#include "Image/Transforms.H"
#include "Image/colorDefs.H"
#include "Util/Assert.H"
#include "Util/StringConversions.H"
#include "DetectionAndTracking/VisualEvent.H"
#include "DetectionAndTracking/Token.H"
#include "DetectionAndTracking/PropertyVectorSet.H"
#include "DetectionAndTracking/MbariFunctions.H"
#include "Media/MbariResultViewer.H"
#include "Image/Geometry2D.H"
#include <algorithm>
#include <istream>
#include <ostream>

using namespace std;

// ######################################################################
// ####### VisualEvent
// ######################################################################
VisualEvent::VisualEvent(Token tk, const DetectionParameters &parms, Image< PixRGB<byte> >& img)
  : startframe(tk.frame_nr),
    endframe(tk.frame_nr),
    max_size(tk.bitObject.getArea()),
    min_size(tk.bitObject.getArea()),
    maxsize_framenr(tk.frame_nr),
    itsState(VisualEvent::OPEN),
    itsTrackerChanged(true),
    itsHoughReset(false),
    houghConstant(DEFAULT_FORGET_CONSTANT),
    itsDetectionParms(parms)
{
  LDEBUG("tk.location = (%g, %g); area: %i",tk.location.x(),tk.location.y(), tk.bitObject.getArea());
  tokens.push_back(tk);
  ++counter;
  myNum = counter;
  vector< float > p;
  float mnoise = 0.1F;
  float pnoise = 0.0F;
   //TODO: need check on values
  p = getFloatParameters(parms.itsXKalmanFilterParameters);
  pnoise = p.at(0); mnoise = p.at(1);
  xTracker.init(tk.location.x(),pnoise,mnoise);
  LINFO("Kalman X tracker parameters process noise: %g measurement noise: %g", pnoise, mnoise);

  p = getFloatParameters(parms.itsYKalmanFilterParameters);
  pnoise = p.at(0); mnoise = p.at(1);
  LINFO("Kalman Y tracker parameters process noise: %g measurement noise: %g", pnoise, mnoise);
  yTracker.init(tk.location.y(),pnoise,mnoise);

  Image<byte> mask;
  BitObject o;
  Image< PixRGB<byte> > imgRescaled = rescale(img, Dims(960, 540));

  switch (parms.itsTrackingMode) {
    case(TMKalmanFilter):
    case(TMKalmanHough):
      itsTrackerType = KALMAN;
    break;
    case(TMNearestNeighbor):
    case(TMNearestNeighborHough):
      itsTrackerType = NN;
    break;
    case(TMHough):
      mask = tk.bitObject.getObjectMask(byte(1));
      mask = rescale(mask, Dims(960,540));
      o.reset(mask);
      o.setSMV(tk.bitObject.getSMV());
      if (o.isValid()) {
        resetHoughTracker(imgRescaled, o);
        itsTrackerType = HOUGH;
      }
    break;
    case(TMNone):
      // just placeholder
      itsTrackerType = NN;
    break;
  }
}
// initialize static variables
uint VisualEvent::counter = 0;
const string VisualEvent::trackerName[3] = {"NearestNeighbor", "Kalman", "Hough"};

// ######################################################################
VisualEvent::~VisualEvent()
{
  tokens.clear();
  hTracker.free();
}
// ######################################################################
VisualEvent::VisualEvent(istream& is)
{
  readFromStream(is);
}

// ######################################################################
void VisualEvent::writeToStream(ostream& os)
{

  os << myNum << " " << (int) itsState << " " << startframe << " " << endframe << "\n";
  os << max_size << " " << maxsize_framenr << "\n";

  xTracker.writeToStream(os);
  yTracker.writeToStream(os);

  int ntokens = 0;
  for (uint i = 0; i < tokens.size(); ++i)
    if(tokens[i].written == false) ntokens++;

  os << ntokens << "\n";

  for (uint i = 0; i < tokens.size(); ++i)
    if(tokens[i].written == false) {
      LDEBUG("Writing VisualEvent  %d Token %d", myNum, i);
      tokens[i].writeToStream(os);
    }

  os << "\n";
}

// ######################################################################
void VisualEvent::readFromStream(istream& is)
{
  int state;
  is >> myNum;
  is >> state;
  is >> startframe;
  is >> endframe;
  is >> max_size;
  is >> maxsize_framenr;

  itsState = (VisualEvent::State)state;

  xTracker.readFromStream(is);
  yTracker.readFromStream(is);

  int t = 0;
  is >> t;

  for (int i = 0; i < t; ++i) {
    tokens.push_back(Token(is));
    LINFO("Reading VisualEvent %d Token %ld", myNum, tokens.size());
  }
}
// ######################################################################
void VisualEvent::writePositions(ostream& os) const
{
  for (uint i = 0; i < tokens.size(); ++i)
    tokens[i].writePosition(os);

  os << "\n";
}

// ######################################################################
float VisualEvent::getAcceleration() const
{
  if (getNumberOfFrames() > 2) {
    //int numFrames = getNumberOfFrames();
    int numSamples = 3;//min(4,numFrames); if need to do larger average
    uint endFrame = getEndFrame();
    uint frameNum = endFrame - numSamples + 1;
    bool init = false;
    float lv = 0.F;
    float asum = 0.F;

    while(frameNum < endFrame) {
      Token t1 = getToken(frameNum);
      Token t2 = getToken(frameNum+1);
      if (t1.bitObject.isValid() && t2.bitObject.isValid()) {
      Point2D<int> p2 = t2.bitObject.getCentroid();
      Point2D<int> p1 = t1.bitObject.getCentroid();
      // distance between centroids
      float v = sqrt(pow((double)(p1.i - p2.i),2.0) + pow((double)(p1.j - p2.j),2.0));

      if (init)
        asum += (v - lv);
      else
       init = true;
      lv = v;
      }
      frameNum++;
    }
    return asum/numSamples;
  }
  return 0.F;
}

// ######################################################################
Point2D<int> VisualEvent::predictedLocation()
{
  int x = int(xTracker.getEstimate() + 0.5F);
  int y = int(yTracker.getEstimate() + 0.5F);
  return Point2D<int>(x,y);
}

// ######################################################################
bool VisualEvent::isTokenOk(const Token& tk) const
{
  LDEBUG("tk.frame_nr %d startframe %d endframe %d itsState: %i", \
          tk.frame_nr, startframe, endframe, (int) itsState);
  return ((tk.frame_nr - endframe) >= 1) && (itsState != CLOSED);
}

// ######################################################################
float VisualEvent::getCostNN(const Token& tk)
{
  if (!isTokenOk(tk)) return -1.0F;
  Point2D<int> p1 = tk.bitObject.getCentroid();
  Rectangle r1 = tk.bitObject.getBoundingBox();
  Point2D<int> p2 = tk.bitObject.getCentroid();
  Rectangle r2 = tokens.back().bitObject.getBoundingBox();

  // calculate cost function as distance between centroids
  float cost1 = sqrt(pow((double)(p1.i - p2.i),2.0) + pow((double)(p1.j - p2.j),2.0));
  // plus distance between bounding box corners
  float cost2 = sqrt(pow((double)(r1.top() - r2.top()),2.0) +  pow((double)(r1.left() - r2.left()),2.0));
  float cost3 = sqrt(pow((double)(r1.bottomI() - r2.bottomI()),2.0) +
					   pow((double)(r1.rightI() - r2.rightI()),2.0)); 
  float cost = cost1 + cost2 + cost3;
  LINFO("Event %i [%g;%g] nearest neighbor predicted location: [%g;%g] cost: %g maxCost: %g",
         myNum, tk.location.x(), tk.location.y(), xTracker.getEstimate(),
         yTracker.getEstimate(), cost, itsDetectionParms.itsMaxCost);
  return cost;
}

// ######################################################################
float VisualEvent::getCostKalman(const Token& tk)
{
  if (!isTokenOk(tk)) return -1.0F;

  float cost = (xTracker.getCost(tk.location.x()) +
                yTracker.getCost(tk.location.y()));

  LINFO("Event %i [%g;%g] Kalman predicted location: [%g;%g] cost: %g maxCost: %g",
         myNum, tk.location.x(), tk.location.y(), xTracker.getEstimate(),
         yTracker.getEstimate(), cost, itsDetectionParms.itsMaxCost);
  return cost;
}

 // ######################################################################
void VisualEvent::assignNoPrediction(const Token& tk, const Vector2D& foe)
{
  ASSERT(isTokenOk(tk));
  BitObject obj = tk.bitObject;
  double smv = obj.getSMV();
  float class_prob = obj.getClassConfidence();
  string class_name = obj.getClassName();

  tokens.push_back(tk);
  tokens.back().bitObject.setClassProbability(class_name, class_prob);
  tokens.back().location = Vector2D(tk.location.x(), tk.location.y()); 

  LINFO("Getting token for frame: %d actual location: %g %g", tk.frame_nr,
          tokens.back().prediction.x(), tokens.back().prediction.y());
          
  // initialize token SMV to last token SMV
  // this is sort of a strange way to propagate values
  // need a bitObject copy operator?
  tokens.back().bitObject.setSMV(smv);

  tokens.back().foe = foe;

  if (tk.bitObject.getArea() > (int) max_size)
    {
      max_size = tk.bitObject.getArea();
      maxsize_framenr = tk.frame_nr;
    }
  if (tk.bitObject.getArea() < (int) min_size)
    {
      min_size = tk.bitObject.getArea();
    }
  endframe = tk.frame_nr;
}

// ######################################################################
void VisualEvent::resetHoughTracker(Image< PixRGB<byte> >& img, BitObject &bo )
{
  itsHoughReset = true;
  houghConstant = DEFAULT_FORGET_CONSTANT;
  hTracker.reset(img, bo, houghConstant);
}

// ######################################################################
void VisualEvent::freeHoughTracker()
{
  hTracker.free();
}

// ######################################################################
bool VisualEvent::updateHoughTracker(nub::soft_ref<MbariResultViewer>&rv, uint frameNum,
                                      Image< PixRGB<byte> >& img,
                                      const Image<byte>& occlusionImg,
                                      Image<byte>& binaryImg,
                                      Rectangle &boundingBox)
{
  itsHoughReset = false;
  return hTracker.update(rv, frameNum, img, occlusionImg, boundingBox, binaryImg, myNum, houghConstant);
}

// ######################################################################
void VisualEvent::updatePrediction(const Token& tk, const Vector2D& foe)
{
	double smv = tokens.back().bitObject.getSMV();
	BitObject bo = tk.bitObject;
	float class_prob = bo.getClassConfidence();
	string class_name = bo.getClassName();
	LINFO("Updating prediction to %s %f at [%g;%g]", class_name.c_str(), class_prob, tk.location.x(), tk.location.y());
 
	// TODO: put voting class here
	tokens.back().bitObject.setClassProbability(class_name, class_prob);
	tokens.back().bitObject.setSMV(smv);

	tokens.back().prediction = Vector2D(xTracker.getEstimate(),
									  yTracker.getEstimate());
	tokens.back().location = Vector2D(xTracker.update(tk.location.x()),
									yTracker.update(tk.location.y()));
	tokens.back().foe = foe;

	// update the straight line
	//Vector2D dir(xTracker.getSpeed(), yTracker.getSpeed());
	Vector2D dir = tokens.front().location - tokens.back().location;
	tokens.back().line.reset(tokens.back().location, dir);

	if (foe.isValid())
		tokens.back().angle = dir.angle(tokens.back().location - foe);
	else
		tokens.back().angle = 0.0F;

	if (tk.bitObject.getArea() > (int) max_size)
	{
	  max_size = tk.bitObject.getArea();
	  maxsize_framenr = tk.frame_nr;
	}
	if (tk.bitObject.getArea() < (int) min_size)
	{
	  min_size = tk.bitObject.getArea();
	}
	endframe = tk.frame_nr;
}

// ######################################################################
void VisualEvent::assign(const Token& tk, const Vector2D& foe)
{
  ASSERT(isTokenOk(tk));
  tokens.push_back(tk);
  updatePrediction(tk, foe);
}
// ######################################################################
bool VisualEvent::doesIntersect(const BitObject& obj, int frameNum) const
{
  if (frameNum > 0 && !frameInRange(frameNum)) return false;
  else return getToken(frameNum).bitObject.doesIntersect(obj);
}
// ######################################################################
VisualEvent::Category VisualEvent::getCategory() const
{
  // If is at least itsDetectionParms.itsMinFrameNum is INTERESTING
  // otherwise BORING
  return ((int)  getNumberOfFrames() >=  itsDetectionParms.itsMinEventFrames ? \
          (VisualEvent::Category)INTERESTING:(VisualEvent::Category)BORING);
}
// ######################################################################
vector<float>  VisualEvent::getPropertyVector()
{
  vector<float> vec;
  Token tk = getMaxSizeToken();
  BitObject bo = tk.bitObject;

  // 0 - event number
  vec.push_back(getEventNum());

  // 1 - interesting value
  vec.push_back(getCategory());

  // not valid?
  if (!bo.isValid())
    {
      // 2  - set area to -1
      vec.push_back(-1);

      // 3-12 set everyone to 0
      for (uint i = 3; i <= 12; ++i)
        vec.push_back(0);

      // done
      return vec;
    }

  // we're valid

  // 2 - area
  vec.push_back(bo.getArea());

  // 3, 4, 5 - uxx, uyy, uxy
  float uxx, uyy, uxy;
  bo.getSecondMoments(uxx, uyy, uxy);
  vec.push_back(uxx);
  vec.push_back(uyy);
  vec.push_back(uxy);

  // 6 - major axis
  vec.push_back(bo.getMajorAxis());

  // 7 - minor axis
  vec.push_back(bo.getMinorAxis());

  // 8 - elongation
  vec.push_back(bo.getElongation());

  // 9 - orientation angle
  vec.push_back(bo.getOriAngle());

  // 10, 11, 12 - max, min, avg intensity
  float maxIntens,minIntens,avgIntens;
  bo.getMaxMinAvgIntensity(maxIntens, minIntens, avgIntens);
  vec.push_back(maxIntens);
  vec.push_back(minIntens);
  vec.push_back(avgIntens);

  // 13 - angle with respect to expansion
  vec.push_back(tk.angle);

  // done -> return the vector
  return vec;
}

// ######################################################################
Dims VisualEvent::getMaxObjectDims() const
{
  int w = -1, h = -1;
  vector<Token>::const_iterator t;
  for (t = tokens.begin(); t != tokens.end(); ++t)
    {
      Dims d = t->bitObject.getObjectDims();
      w = max(w, d.w());
      h = max(h, d.h());
    }
  return Dims(w,h);
}
