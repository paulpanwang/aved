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

#ifndef MBARIIMAGE_H_
#define MBARIIMAGE_H_

#include "Image/Image.H"
#include "Raster/Raster.H"
#include "Util/StringUtil.H" // for split()
#include "Data/MbariMetaData.H"

template <class T> class Image;
template <class T> class PixRGB;

namespace {
// ######################################################################
//! compute input filename for current frame
    std::string computeInputFileName(const std::string& stem,
                                     const int framenumber)
    {
      ASSERT(framenumber >= 0);

      // if there is a '#' in the stem, replace it with the frame number
      std::string::size_type hashpos = stem.find_first_of('#');

      if (hashpos != stem.npos)
      {
        std::string fname = stem;
        fname.replace(hashpos, 1, sformat("%06d", framenumber));

        std::vector<std::string> tokens;
        split(fname, ":", std::back_inserter(tokens));

        return tokens[1].c_str();
      }

      // else... no '#', so just return the filename as-is
      return stem;
    }
}

// ######################################################################
//! Subclasses saliency toolkit Image<T> class for managing MBARI specific
// operations and metadata class for image caches that do computations on the fly
template <class T>
class MbariImage : public Image<T>
{
public:
  inline MbariImage(std::string ifms="");
  inline MbariImage(Image<T> img, std::string ifms);
  inline MbariImage(Image<T> img, MbariMetaData md, std::string ifms);
  inline void updateData(Image<T> img, int nf);
  inline void updateData(Image<T> img, MbariMetaData md, int nf);
  inline MbariMetaData getMetaData();
  inline std::string getStem();
  inline int getFrameNum();
  inline ~MbariImage();

private:
  MbariMetaData metaData;
  std::string ifmsStem;
  int framenum;
};

// ######################################################################
template <class T> inline
MbariImage<T>::MbariImage(std::string ifms)
  : Image<T>(), metaData(""), ifmsStem(ifms)
{}

// ######################################################################
template <class T> inline
MbariImage<T>::MbariImage(Image<T> img, std::string ifms)
  : Image<T>(img), metaData(""), ifmsStem(ifms)
{}

// ######################################################################
template <class T> inline
MbariImage<T>::MbariImage(Image<T> img, MbariMetaData md, std::string ifms)
  : Image<T>(img), metaData(md), ifmsStem(ifms)
{}

// ######################################################################
template <class T> inline
void MbariImage<T>::updateData(Image<T> img, int nf)
{
  Image<T> A_copy( img );
  this->swap(A_copy);
  framenum = nf;
	
  std::string fname = computeInputFileName(ifmsStem, framenum);

  std::string comments(Raster::getImageComments(fname));
  metaData.setMetaData(comments);
}
// ######################################################################
template <class T> inline
void MbariImage<T>::updateData(Image<T> img, MbariMetaData md, int nf)
{
  Image<T> A_copy( img );
  this->swap(A_copy);
  framenum = nf;
  metaData = md;
}

// ######################################################################
template <class T> inline
MbariMetaData MbariImage<T>::getMetaData()
{ return metaData; }

// ######################################################################
template <class T> inline
std::string MbariImage<T>::getStem()
{ return ifmsStem; }

// ######################################################################
template <class T> inline
int MbariImage<T>::getFrameNum()
{ return framenum; }

// ######################################################################
template <class T> inline
MbariImage<T>::~MbariImage()
{ /* memory deallocation is handled by ArrayData's destructor */ }


#endif /*MBARIIMAGE_H_*/
