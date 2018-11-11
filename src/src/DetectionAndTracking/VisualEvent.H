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

/*!@file VisualEvent.H contained Tokens of events and used in event tracking */
  
#ifndef VISUALEVENT_H_DEFINED
#define VISUALEVENT_H_DEFINED

#include <list>
#include <string>
#include <vector>

#include "DetectionAndTracking/DetectionParameters.H"
#include "DetectionAndTracking/Token.H"
#include "DetectionAndTracking/HoughTracker.H"
#include "Image/BitObject.H"
#include "Image/KalmanFilter.H"
#include "Image/Geometry2D.H"
#include "Image/ArrayData.H" // for class Dims

template <class T> class Image;
template <class T> class PixRGB;

#define  DEFAULT_CLASS_NAME "Unknown"

class DetectionParameters;
class MbariResultViewer;
namespace nub { template <class T> class soft_ref; }

// ######################################################################
//!stores a visual event as a sequence of tokens with additional information
class VisualEvent
{
public:
  //! constructor
  /*!@param tk the first token for this event
  @param parms the detection parameters
  @param img the image the token was extracted from*/
  VisualEvent(Token tk, const DetectionParameters &parms, Image< PixRGB<byte> >& img);

  //! destructor
  ~VisualEvent();

  //! read the VisualEvent from the input stream is
  VisualEvent(std::istream& is);

  //! write the entire VisualEvent to the output stream os
  void writeToStream(std::ostream& os);

  //! read the VisualEvent from the input stream is
  void readFromStream(std::istream& is);

  //! write all the positions for this event to the output stream os
  void writePositions(std::ostream& os) const;

  //! get the prediction for the location of the next token
  Point2D<int> predictedLocation() ;

  //! get the average acceleration speed the token is moving
  float getAcceleration() const;

  //! is this token allowed as the next one?
  bool isTokenOk(const Token& tk) const;

  //! returns the Kalman filter cost of associating tk with this event
  /*!@return returns -1.0F if the token is not valid for this event*/
  float getCostKalman(const Token& tk);

  //! returns the Nearest Neighbor filter cost of associating tk with this event
  /*!@return returns -1.0F if the token is not valid for this event*/
  float getCostNN(const Token& tk);

  //! assign tk to this event, use foe as the focus of expansion
  void assign(const Token& tk, const Vector2D& foe,  uint validendframe);

  //! updates prediction for the latest assigned token
  void updatePrediction(const Token& tk, const Vector2D& foe, uint validendframe);

  //! assign tk to this event, use foe as the focus of expansion, don't update the prediction
  void assignNoPrediction(const Token& tk, const Vector2D& foe);

  //! if the BitObject intersects with the one for this event at frameNum
  bool doesIntersect(const BitObject& obj, int frameNum) const;

  //! updates the Hough-based tracker
  // !@returns false if tracker fails
  bool updateHoughTracker(nub::soft_ref<MbariResultViewer>&rv,  uint frameNum, Image< PixRGB<byte> >& img,
                          const Image<byte>& occlusionImg, Image<byte>& binaryImg, Rectangle &boundingBox);

  //! reset the Hough-based tracker
  void resetHoughTracker(Image< PixRGB<byte> >& img, BitObject &bo);

  //! free up memory associated with the Hough-based tracker
  void freeHoughTracker();

  // ! VisualEvent states
  enum State {
    OPEN,
    CLOSED,
    WRITE_FINI,
    DELETE
  };

  //! set the state of the VisualEvent
  inline void setState(VisualEvent::State state);

  //! get the state of the VisualEvent
  inline VisualEvent::State getState();

  //! mark this event as "closed"
  inline void close();

  //! whether this event is closed
  inline bool isClosed();

  //! flag this event as written
  inline void flagWriteComplete ();

  //! flag this event for deletion
  inline void flagForDelete();

  //! whether event is ready to be deleted
  inline bool isReadyToDelete();

  //! whether this event is still open
  inline bool isOpen();

  //! return the event identification number of this event
  inline uint getEventNum() const;

  //! return the frame number of the first token
  inline uint getStartFrame() const;

  //! return the frame number of the last token
  inline uint getEndFrame() const;

  //! return the frame number of the last valid( non duplicated token)
  inline uint getValidEndFrame() const;

  //! return starting timecode string
  inline std::string getStartTimecode() const;

  //! return starting timecode string
  inline std::string getEndTimecode() const;

  //! return the number of frames that this event spans
  inline uint getNumberOfFrames() const;

  //! return the number of tokens stored for this event
  // note that this may not be the same as getNumberOfFrames()
  // as frames may be deleted during runtime to save space
  inline uint getNumberOfTokens() const;

  //! return the maximum object size of any of the tokens
  inline int getMaxSize() const;

  //! return the minimum object size of any of the tokens
  inline int getMinSize() const;

  //! return the token that has the maximum object size
  inline Token getMaxSizeToken() const;

  //!return a token based on a frame number
  inline Token getToken(const uint frame_num) const;

  //!return reset bit object contained in frame number
  inline void resetBitObject(const uint frame_num, BitObject &obj);

  //!return whether frame_num is between start frame and end frame
  inline bool frameInRange(const uint frame_num) const;

  //! return true if need to initialize the Hough tracker
  inline bool getHoughReset() const;

  //! return the forget constant that is used with the Hough tracker
  inline void setForgetConstant(float constant);

  //! return the forget constant that is used with the Hough tracker
  inline float getForgetConstant();

  //!remove the last element
  inline void removeLast();

  //! return a vector that contains a bunch of numbers that describe the event
  /*! see source code for details */
  std::vector<float>  getPropertyVector();

  //! returns the maximum dimensions of the tracked object in any of the frames
  Dims getMaxObjectDims() const;

  enum Category {
    BORING,
    INTERESTING
  };

  //! get category for this event returns INTERESTING OR BORING
  VisualEvent::Category getCategory() const;

  enum TrackerType {
    NN,
    KALMAN,
    HOUGH
  };

  //! set the type of tracker for the visualEvent
  inline void setTrackerType(VisualEvent::TrackerType type);

  //! returns TrackerType
  inline TrackerType getTrackerType();

  //! return true if the tracker changed
  inline bool trackerChanged();

private:
  static uint counter;
  uint myNum;
  std::vector<Token> tokens;
  uint startframe;
  uint endframe;
  uint validendframe;
  int max_size,min_size;
  uint maxsize_framenr;
  // ! VisualEvent state
  VisualEvent::State itsState;
  KalmanFilter xTracker, yTracker;
  HoughTracker hTracker;
  TrackerType itsTrackerType;
  bool itsTrackerChanged;
  bool itsHoughReset;
  float houghConstant;
  static const std::string trackerName[];
  //! True if interesting, otherwise false (for boring) defaults to boring
  Category itsCategory;
  DetectionParameters itsDetectionParms;

};

// ######################################################################
// ########### INLINED METHODS
// ######################################################################
inline VisualEvent::TrackerType VisualEvent::getTrackerType()
{ return itsTrackerType; }

// ######################################################################
inline bool VisualEvent::trackerChanged()
{ return itsTrackerChanged; }

// ######################################################################
inline void VisualEvent::setState(VisualEvent::State state)
{ itsState = state; }

// ######################################################################
inline VisualEvent::State VisualEvent::getState()
{ return itsState; }

// ######################################################################
inline void VisualEvent::close()
{ itsState = CLOSED; }

// ######################################################################
inline bool VisualEvent::isClosed()
{ return (itsState == CLOSED ? true: false); }

// ######################################################################
inline bool VisualEvent::isOpen()
{ return (itsState == OPEN ? true: false); }

// ######################################################################
inline void VisualEvent::flagForDelete()
{ itsState = DELETE; }

// ######################################################################
inline void VisualEvent::flagWriteComplete()
{ itsState = WRITE_FINI; }

// ######################################################################
inline bool VisualEvent::isReadyToDelete()
{ return (itsState == DELETE ? true: false); }

// ######################################################################
inline uint VisualEvent::getEventNum() const
{ return myNum; }

// ######################################################################
inline uint VisualEvent::getStartFrame() const
{ return startframe; }

// ######################################################################
inline uint VisualEvent::getValidEndFrame() const
{ return validendframe; }

  // ######################################################################
inline uint VisualEvent::getEndFrame() const
{ return endframe; }
// ######################################################################
inline std::string VisualEvent::getStartTimecode() const
{
  Token tk  = getToken(startframe);
  return tk.mbarimetadata.getTC();
}
// ######################################################################
std::string VisualEvent::getEndTimecode() const
{
  Token tk  = getToken(endframe);
  return tk.mbarimetadata.getTC();
}
// ######################################################################
inline uint VisualEvent::getNumberOfFrames() const
{ return (endframe - startframe + 1); }

// ######################################################################
inline uint VisualEvent::getNumberOfTokens() const
{ return tokens.size(); }

// ######################################################################
inline int VisualEvent::getMaxSize() const
{ return max_size; }

// ######################################################################
inline int VisualEvent::getMinSize() const
{ return min_size; }

// ######################################################################
inline Token VisualEvent::getMaxSizeToken() const
{ return getToken(maxsize_framenr); }

// ######################################################################
inline Token VisualEvent::getToken(uint frame_num) const
{
  Token empty;
  ASSERT (frameInRange(frame_num));
  for(uint i=0; i<tokens.size(); i++) {
    Token tk = tokens[i];
    if(tk.frame_nr == frame_num)
      return tk;
  }
  return empty;
}

// ######################################################################
inline void VisualEvent::resetBitObject(const uint frame_num,
                                        BitObject &obj)
{
  ASSERT (frameInRange(frame_num));
  tokens[frame_num - startframe].bitObject = obj;
}

// ######################################################################
inline bool VisualEvent::frameInRange(const uint frame_num) const
{
  return ((frame_num >= startframe) && (frame_num <= endframe));
}

// ######################################################################
inline void VisualEvent::removeLast()
{
  tokens.pop_back();
  endframe -= 1;
}

// ######################################################################
inline bool VisualEvent::getHoughReset() const
{
  return itsHoughReset;
}

// ######################################################################
inline float VisualEvent::getForgetConstant()
{
  return houghConstant;
}

// ######################################################################
inline void VisualEvent::setForgetConstant(float constant)
{
  houghConstant = constant;
}

// ######################################################################
inline void VisualEvent::setTrackerType(VisualEvent::TrackerType type)
{
  if (itsTrackerType != type) {
    LINFO("Event %i switching to %s ", myNum, trackerName[type].c_str());
    itsTrackerChanged = true;
  }
  else {
    itsTrackerChanged = false;
  }
  itsTrackerType = type;
}
#endif
