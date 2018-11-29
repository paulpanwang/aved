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

#ifndef BITOBJECT_H_DEFINED
#define BITOBJECT_H_DEFINED

#include <string>

#include "Image/Image.H"
#include "Image/Rectangle.H"
#include "Util/Types.H"

#include "Image/BitObjectDrawModes.H"
#include "Image/Geometry2D.H"


#define DEFAULT_CLASS_NAME "Unknown"
#define DEFAULT_CLASS_CONFIDENCE 1.0f


//! Object defined by a connected binary pixel region
/*! This class extracts a connected binary pixel region from a
  grayscale image and analyzes a few of its properties*/

class BitObject
{
public:

  //! default Constructor
  BitObject();

  //! Constructor from a byte Image and a location
  /*!@param img Image from which the object is extracted
    @param location a point that is inside the object - the object
    is flooded starting from this seed
    @param threshold for flooding*/
  BitObject(const Image<byte>& img, const Point2D<int> location, 
            const byte threshold = 1);

  //! Constructor from a byte Image, location and rectangle
  /*!@param img Image from which the object is extracted
    @param boundingBox the bounding box of the object
    @param center a point that represents the center of the object
    @param threshold for flooding*/
  BitObject(const Image<byte>& img, const Point2D<int> center, const Rectangle boundingBox, const byte threshold = 1);

  //! custom operator used when combining std::lists of
  /*BitObjects   */
  bool operator < ( const BitObject object ) {
      return true;
  };

  //! set class name and probability
  void inline setClassProbability(const std::string className, const float classProbability) {
	  itsClassName = className;
	  itsClassConfidence = classProbability;
  };

  //! Construct from a byte Image
  /*! @param img image containing only the object 
    the object pixels are 1, all other pixels are 0*/
  BitObject(const Image<byte>& img);
  
  //! Construct from the data contained in the input stream is
  BitObject(std::istream& is);

  // default copy constructor, operator= and destructor okay

  //! Reset to a new object
  /*!@param img Image from which the object is extracted
    @param location a point that is inside the object - the object
    is flooded starting from this seed
    @param threshold for flooding
    @return a mask of the extracted object (in IMAGE coordinates) */
  Image<byte> reset(const Image<byte>& img, const Point2D<int> location,
                    const byte threshold = 1);
  
  //! Reset to a new object
  /*! @param img image containing only the object 
    the object pixels are 1, all other pixels are 0;
    @return the area of the extracted object; -1 if no object could
    be extracted - in this case the BitObject is invalid */
  int reset(const Image<byte>& img);

  //! Reset to a new object not accounting for flooding the area
  // useful for unconnected objects
  /*!@param img Image from which the object is extracted
    @param boundingBox the bounding box of the object
    @param center a point that represents the center of the object
    @return the area of the extracted object; -1 if no object could
    be extracted - in this case the BitObject is invalid */
  int reset(const Image<byte>& img, const Point2D<int> center, const Rectangle boundingBox, const byte threshold = 1);

  //! delete all stored data, makes the object invalid
  void freeMem();

  //! write the entire BitObject to the output stream os
  void writeToStream(std::ostream& os) const;

  //! read the BitObject from the input stream is
  void readFromStream(std::istream& is);

  //! Coordinate system for return values
  /*! These values are used to specify whether return values should be 
    given in coordinates of the extracted object or in coordinates
    of the image from which the object was extracted.*/
  enum Coords { OBJECT, IMAGE };

  //! Set the winning saliency map voltage for this BitObject  
  void setSMV(double smv);

  //! Extract the maximum, minimum and average intensity over the object shape in img
  /*! The results are stored in BitObject and can be accessed 
    using getMaxMinAvgIntensity*/
  template <class T>
  void setMaxMinAvgIntensity(const Image<T>& img);

  //! Return the maximum, minimum and average intensity
  /*! See setMinMaxAvgIntensity for details*/
  void getMaxMinAvgIntensity(float& maxIntensity, float& minIntensity, 
                             float& avgIntensity);

  //! Returns the bounding box of the object
  Rectangle getBoundingBox(const Coords coords = IMAGE) const;

  //! Returns the object as a binary mask
  /*!@param value the value that is used for the object*/
  Image<byte> getObjectMask(const byte value = byte(1),
                            const Coords coords = IMAGE) const;

  //! Returns the object and masks out background in @param img
  /*!@param backgroundcolor is the value used as background color*/  
  template <class T_or_RGB>
  void drawMaskedObject(Image<T_or_RGB>& img, 
                        const T_or_RGB backgroundcolor);
  
  //! The dimensions of the bounding box of the object
  Dims getObjectDims() const;

   // ! The dimensions of the image the BitObject was extracted from
  Dims getImageDims() const;

  //! The location of the upper left corner of the boundary box in the original image
  Point2D<int> getObjectOrigin() const;

  //! The centroid of the object rounded to integer coordinates
  Point2D<int> getCentroid(const Coords coords = IMAGE) const;

  //! The centroid, not rounded
  Vector2D getCentroidXY(const Coords coords = IMAGE) const;

  //! The area (number of pixels) of the object
  int getArea() const;

  //! The area (number of pixels) of the object
  //float getStdDev() const;

  //! Returns the second moments of the object (normalized by the area)
  void getSecondMoments(float& uxx, float& uyy, float& uxy);

  //! Returns the length of the major axis of the object
  float getMajorAxis();

  //! Returns the length of the minor axis of the object
  float getMinorAxis();

  //! Returns the elongation of the object (major axis / minor axis)
  float getElongation();

  //! Returns the angle between the major axis and the x axis
  /* @return "--" is 0; "\" is 45; "|" is 90; "/" is 135 */
  float getOriAngle(); 
  
   // ! Returns the winning Saliency Map Voltage for this BitMap
  double getSMV();

  // ! Returns the Potential Object's Name
  std::string getClassName();

  // ! Returns the Potential Object's Confidence
  float getClassConfidence();

  //! whether the object is valid
  /*! This is going to be false if no object could be extracted
    from the image that was given at the constructor or at reset */
  bool isValid() const;
 
  //! returns the intersection of this BitObject with other
  /*! if the intersection is empty, an invalid BitObject is returned*/
  //BitObject intersection(const BitObject& other);
  bool doesIntersect(const BitObject& other) const;

  //! returns the amount of pixel intersection of this BitObject with other
  double intersect(const BitObject& other) const;
 
  //! draw the shape of this BitObject into img with color
  template <class T_or_RGB>
  void drawShape(Image<T_or_RGB>&, const T_or_RGB& color,
                 float opacity = 1.0F);
 
  //! draw the outline of this BitObject into img with color
  template <class T_or_RGB>
  void drawOutline(Image<T_or_RGB>&, const T_or_RGB& color,
                   float opacity = 1.0F);
 
  //! draw the bounding box of this BitObject into img with color
  template <class T_or_RGB>
  void drawBoundingBox(Image<T_or_RGB>&, 
                       const T_or_RGB& color, float opacity = 1.0F);
 
  //! draw this BitObject according to mode
  template <class T_or_RGB>
  void draw(BitObjectDrawMode mode, 
            Image<T_or_RGB>&, 
            const T_or_RGB& color,
            float opacity = 1.0F);
 
  // compute the second moments and values derived from them
  void computeSecondMoments();

private:

  Image<byte> itsObjectMask;
  Rectangle itsBoundingBox; // in image coordinates
  Vector2D itsCentroidXY; // in image coordinates
  int itsArea;
  float itsUxx, itsUyy, itsUxy;
  float itsMajorAxis, itsMinorAxis, itsElongation, itsOriAngle;
  Dims itsImageDims;
  float itsMaxIntensity, itsMinIntensity, itsAvgIntensity;
  bool haveSecondMoments;
  double itsSMV;
  std::string itsClassName;
  float itsClassConfidence;
};

#endif
