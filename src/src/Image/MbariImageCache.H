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

#ifndef MBARI_IMAGECACHE_H_DEFINED
#define MBARI_IMAGECACHE_H_DEFINED

#include <deque>

#include "Util/Promotions.H"
#include "Image/MbariImage.H"

// ######################################################################
//! base class for image caches that do computations on the fly
/*! This base class has no op doWhenAdd and doWhenRemove functions
  that should be overridden in classes derived from this one.*/
template <class T>
class MbariImageCache
{
public:
  //! Constructor
  /*! @param maxSize the maximum size of the cache. If this size is exceeded,
    images are popped off the front of the cache and disregarded for the
    computation of the mean. If maxSize = 0, the cache is not limited.*/
  MbariImageCache(uint maxSize = 0);

  //! Destructor
  virtual ~MbariImageCache();

  //! Set maximum number of images in the cache
  /*! If the cache currently has more images than the new specified
    maximum, it will be truncated to that maximum. */
  inline void setMaxSize(const uint maxSize);

  //! Get maximum number of images in the cache
  inline uint getMaxSize() const;

  //! add image to cache - if the cache gets to big, old images are popped off
  inline void push_back(const MbariImage<T>& img);

  //! pop the front Image (oldest) off the cache and return it
  inline MbariImage<T> pop_front();

  //! access the last Image (newest) in the queue
  inline MbariImage<T>& back() const;

  //! access the first Image (oldest) in the queue
  inline MbariImage<T>& front();

  //! Get image from a given level.
  inline const MbariImage<T> getImage(const uint lev) const;

  //! Get image from a given level (shorthand for getImage()).
  inline const MbariImage<T> operator[](const uint lev) const;

  //! return the current size of the cache
  /*! This may be smaller than the maximum size specified at
    construction or with setMaxSize(), if the cache is not full yet. */
  inline uint size() const;

  //! true if the cache contains no elements
  inline bool empty() const;

  //! clear the cache, preserving its maxSize
  inline void clear();

protected:
  //! checks whether the cache has become too big and removes old entries
  inline void popOffOld();

  //! called when an image is added - override in your derived classes!
  /*! in ImageCache, this function is no op*/
  inline void doWhenAdd(const MbariImage<T>& img);

  //! called when an image is removed - override in your derived classes
  /*! in ImageCache, this function is no op*/
  inline void doWhenRemove(const MbariImage<T>& img);

  //! the maximum size of images to be stored
  uint itsMaxSize;

  //! the cache of the images
  std::deque< MbariImage<T> > itsCache;
};

// ######################################################################
// ##### Implementation of ImageCache<T>
// ######################################################################
template <class T> inline
MbariImageCache<T>::MbariImageCache(uint maxSize)
  : itsMaxSize(maxSize)
{}

// ######################################################################
template <class T> inline
MbariImageCache<T>::~MbariImageCache()
{
  popOffOld();
}

// ######################################################################
template <class T> inline
void MbariImageCache<T>::setMaxSize(const uint maxSize)
{
  itsMaxSize = maxSize;
  // truncate if necessary:
  popOffOld();
}

// ######################################################################
template <class T> inline
uint MbariImageCache<T>::getMaxSize() const
{ return itsMaxSize; }

// ######################################################################
template <class T> inline
void MbariImageCache<T>::push_back(const MbariImage<T>& img)
{
  doWhenAdd(img);
  itsCache.push_back(img);
  popOffOld();
  return;
}

// ######################################################################
template <class T> inline
void MbariImageCache<T>::popOffOld()
{
  // now pop off old images
  if (itsMaxSize == 0) return;

  while (itsCache.size() > itsMaxSize)
    pop_front();

  return;
}

// ######################################################################
template <class T> inline
MbariImage<T> MbariImageCache<T>::pop_front()
{
  ASSERT(!itsCache.empty());
  MbariImage<T> ret = itsCache.front();
  doWhenRemove(ret);
  itsCache.pop_front();
  return ret;
}

// ######################################################################
template <class T> inline
MbariImage<T>& MbariImageCache<T>::back() const
{
  ASSERT(!itsCache.empty());
  return itsCache.back();
}

// ######################################################################
template <class T> inline
MbariImage<T>& MbariImageCache<T>::front()
{
  ASSERT(!itsCache.empty());
  return itsCache.front();
}

// ######################################################################
template <class T> inline
const MbariImage<T> MbariImageCache<T>::getImage(const uint lev) const
{
  ASSERT(lev < itsCache.size());
  return itsCache[lev];
}

// ######################################################################
template <class T> inline
const MbariImage<T> MbariImageCache<T>::operator[](const uint lev) const
{ return getImage(lev); }

// ######################################################################
template <class T> inline
uint MbariImageCache<T>::size() const
{ return itsCache.size(); }

// ######################################################################
template <class T> inline
bool MbariImageCache<T>::empty() const
{ return itsCache.empty(); }

// ######################################################################
template <class T> inline
void MbariImageCache<T>::clear()
{
  // hopefully, overloads of pop_front() will take care of updating any
  // additional data members (like the sliding average image in
  // ImageCache):
  while(size()) this->pop_front();
}

// ######################################################################
template <class T> inline
void MbariImageCache<T>::doWhenAdd(const MbariImage<T>& img)
{ }

// ######################################################################
template <class T> inline
void MbariImageCache<T>::doWhenRemove(const MbariImage<T>& img)
{ }

#endif
