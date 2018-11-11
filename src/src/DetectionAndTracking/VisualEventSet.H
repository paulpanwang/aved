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

/*!@file VisualEventSet.H collection of VisualEvents and tracking on those events */
  
#ifndef VISUALEVENTSET_H_DEFINED
#define VISUALEVENTSET_H_DEFINED

#include "DetectionAndTracking/DetectionParameters.H"
#include "DetectionAndTracking/VisualEvent.H"
#include "DetectionAndTracking/PropertyVectorSet.H"
#include "Data/MbariMetaData.H"
#include "Data/ImageData.H"
#include "Image/BitObject.H"
#include "Learn/Features.H"
#include "Learn/BayesClassifier.H"

#include <list>
#include <string>
#include <vector>

class BayesClassifier;
class MbariResultViewer;
namespace nub { template <class T> class soft_ref; }

// ######################################################################
// ######## VisualEventSet
// ######################################################################
//! contains a set of visual events to which a set of tokens can be matched
class VisualEventSet
{
public:
  //! constructor
  /*!@param maxDist the maximum distance that a token can have from an
    event and still be considered for becoming part of the event*/
  VisualEventSet(const DetectionParameters &parameters, const std::string& filename, const bool &saveEventFeatures);

  //! desctructor
  ~VisualEventSet();

  //! read the VisualEventSet from the input stream is
  VisualEventSet(std::istream& is);

  //! write the VisualEventSet header to the output stream os
  void writeHeaderToStream(std::ostream& os);

  //! read the VisualEventSet header from the input stream is
  void readHeaderFromStream(std::istream& is);

  //! write the entire VisualEventSet to the output stream os
  void writeToStream(std::ostream& os);

  //! read the VisualEventSet from the input stream is
  void readFromStream(std::istream& is);

  //! write the positions of all events to the output stream os
  void writePositions(std::ostream& os) const;

    //!update events with new binary map
  /*!param rv ResultsViewer for displaying the output
    @frameNum frame number
     @param curFOE the current focus of expansion for detecting unusual motion
    @param metadata associated with current frame number*/
  void updateEvents(nub::soft_ref<MbariResultViewer>&rv,
                    const BayesClassifier &bayesClassifier,
                    FeatureCollection& features,
                    ImageData& imgData);

  //! return the average speed events are moving
  float getAverageSpeed();

  //! get the min and max areas across all events
  void getAreaRange(int &minArea, int &maxArea);

  //! get average acceleration for all events except the skipEventNum
  float getAcceleration(uint skipEventNum);

  //! insert event into current list of event
  void insert(VisualEvent *event);

  //! initiate new events for all BitObjects in sobjs if they aren't tracked yet
  void initiateEvents(std::list<BitObject>& sobjs, FeatureCollection& features, ImageData& imgData);

  //! if obj intersects with any of the event at frameNum, reset SMV and Hough bounds
  bool resetIntersect(Image< PixRGB<byte> >& img, BitObject& obj, const Vector2D& curFOE, int frameNum);

  //! if obj intersects with any of the event at frameNum, reset SMV
  bool doesIntersect(BitObject& obj, int frameNum);

  //! if obj intersects with any of the events in frameNum, return true and first found intersecting eventNum
  bool doesIntersect(BitObject& obj, uint *eventNum, int frameNum);

  //! return the number of stored events
  uint numEvents() const;

  //! delete all stored events
  void reset();

  // ! returns minimum size for an "interesting event"
  const int minSize();

  //! clean up the event list - erase all unsuccessful candidates
  /*!@param currFrame - the current frame number in processing
    @param lastframe in this sequence*/
  void cleanUp(uint currFrame, uint lastframe=1);

  //! close all events (for clean-up at the end)
  void closeAll();

  //! print out debugging info on all events
  void printAll();

  //! returns a set of all tokens stored at a particular frame number
  std::vector<Token> getTokens(uint frameNum);

  void drawTokens(Image< PixRGB<byte> >& img,
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
                  float scaleW = 1.0F, float scaleH = 1.0F);

  //! returns a PropertyVectorSet for this VisualEventSet
  PropertyVectorSet getPropertyVectorSet();

  //! returns a PropertyVectorSet that can be saved for this VisualEventSet
  PropertyVectorSet getPropertyVectorSetToSave();

  //! return the latest frame number before currFrame for which events have been closed already
  int getAllClosedFrameNum(uint currFrame);

  //! test whether the event with eventNum exists in this set
  bool doesEventExist(uint eventNum) const;

  //! Returns the event with number eventNum
  VisualEvent* getEventByNumber(uint eventNum) const;

  //! Replace the event for eventnum
  void replaceEvent(uint eventum, VisualEvent *event);

  //! Returns iterators pointing to all events at framenum
  std::list<VisualEvent *> getEventsForFrame(uint framenum);

  //! Returns a list of all BitObject at framnum
  std::list<BitObject> getBitObjectsForFrame(uint framenum);

  //! Returns an iterator pointing to all (interesting or boring)
  // ready to be written for given framenum
  std::list<VisualEvent *> getEventsReadyToSave(uint framenum);

private:

  //! find best candidate object for a given region
  BitObject findBestCandidate(const Rectangle &region,  const std::list<BitObject> &bos);

  // get bit objects from a given token
  std::list<BitObject> getBitObjects(nub::soft_ref<MbariResultViewer>&rv, VisualEvent *currEvent,
                                    const Point2D<int>& center, const Image< PixRGB<byte> >& segmentImg);

  // get bit object shifted to next prediction
  BitObject getBitObjectPrediction(const Token &evtToken, const Point2D<int> &pred);

  // compute the right position for a text label
  Point2D<int> getLabelPosition(Dims imgDims,Rectangle bbox,
                           Dims textDims) const;

  // runs the Kalman or Nearest Neighbor filter tracking on @param event
  // returns true if able to track
  bool runBaseTracker(nub::soft_ref<MbariResultViewer> rv, VisualEvent *event,
                        const BayesClassifier &bayesClassifier,
                        FeatureCollection& features,
                        ImageData& imgData,
                        bool skip = false);

  // run the Hough-based tracker on @param event
  // returns true if able to track
  bool runHoughTracker(nub::soft_ref<MbariResultViewer>&rv, VisualEvent *event,
                       const BayesClassifier &bayesClassifier,
                       FeatureCollection& features,
                       ImageData& imgData,
                       bool skip = false);

  // run the combination Hough and NearestNeighbor tracker on @param event
  void runNearestNeighborHoughTracker(nub::soft_ref<MbariResultViewer>&rv,
                                      VisualEvent *event,
                                      const BayesClassifier &bayesClassifier,
                                      FeatureCollection& features,
                                      ImageData& imgData);

  // run the combination Hough and Kalman tracker on @param event
  void runKalmanHoughTracker(nub::soft_ref<MbariResultViewer>&rv, VisualEvent *event,
                             const BayesClassifier &bayesClassifier,
                             FeatureCollection& features,
                             ImageData& imgData );

  // run the check for failure conditions on the @param event
  void checkFailureConditions(VisualEvent *currEvent, Dims d);
	const Point2D<int> predictionCheck(VisualEvent* currEvent,
			ImageData& imgData);


  // run check for occlusion with another event
  Image<PixRGB<byte> > occlusionCheck(ImageData& imgData, Token& evtToken,
			VisualEvent* currEvent, bool& occlusion);

  std::list<VisualEvent *> itsEvents;
  int startframe;
  int endframe;
  std::string itsFileName;
  DetectionParameters itsDetectionParms;
  bool itsSaveEventFeatures;
};
#endif
