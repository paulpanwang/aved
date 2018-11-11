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

#include "Image/BitObject.H"
#include "Image/CutPaste.H"    // for crop()
#include "Image/DrawOps.H"     // for drawDisk
#include "Image/IO.H"
#include "Image/Kernels.H"     // for twofiftyfives()
#include "Image/MorphOps.H"    // for erode/dilate
#include "Image/Image.H"
#include "Image/MathOps.H"
#include "Image/ShapeOps.H"   // for rescale()
#include "Image/Transforms.H"
#include "Raster/GenericFrame.H"
#include "Raster/PnmParser.H"
#include "Raster/PnmWriter.H"
#include "Util/Assert.H"
#include "Util/MathFunctions.H"
#include "Util/StringConversions.H"

#include <cmath>
#include <istream>
#include <ostream>

using namespace std;

// ######################################################################
BitObject::BitObject()
{
  freeMem();
}

// ######################################################################
BitObject::BitObject(const Image<byte>& img, const Point2D<int> location,
                     const byte threshold)
{
  reset(img, location, threshold);
}

// ######################################################################
BitObject::BitObject(const Image<byte>& img)
{
  reset(img);
}

// ######################################################################
BitObject::BitObject(istream& is)
{
  readFromStream(is);
}
// ######################################################################
BitObject::BitObject(const Image<byte>& img, const Point2D<int> location, const Rectangle boundingBox, const byte threshold)
{
  reset(img, location, boundingBox, threshold);
}

// ######################################################################
int BitObject::reset(const Image<byte>& img, const Point2D<int> location, const Rectangle boundingBox, const byte threshold)
{

  ASSERT(img.initialized());

  // first, reset everything to defaults
  freeMem();

  itsBoundingBox = boundingBox;
  Image<byte> dest;

  // threshold to get binary object
  dest = highThresh(img, threshold, byte(1));

  // count all foreground pixels as area
  int area = countParticles(dest, byte(1));

  LINFO("area %d", area);

  // no object found? return -1
  if (area == 0)
    {
      itsCentroidXY.reset(location);
      return area;
    }

  // set the dimensions of the original image
  itsImageDims = img.getDims();

  // crop the object mask from the flooding destination
  itsObjectMask = crop(dest, itsBoundingBox);

  // get the area, the centroid, and the bounding box
  vector<float> sumx, sumy;
  itsArea = (int)sumXY(itsObjectMask, sumx, sumy);
  //itsStdDev = stdev(luminance(img));

  int firstX, lastX, firstY, lastY;
  float cX, cY;
  bool success = (getCentroidFirstLast(sumx, cX, firstX, lastX) |
                  getCentroidFirstLast(sumy, cY, firstY, lastY));
  itsCentroidXY.reset(cX,cY);

  if (!success) LFATAL("determining the centroid failed");

  if ((firstX != 0) || (lastX != itsObjectMask.getWidth()-1) ||
      (firstY != 0) || (lastY != itsObjectMask.getHeight()-1))
    LFATAL("boundary box doesn't match the one from flooding");

  itsCentroidXY += Vector2D(itsBoundingBox.left(),itsBoundingBox.top());


  return itsArea;
}

// ######################################################################
Image<byte> BitObject::reset(const Image<byte>& img, const Point2D<int> location,
                             const byte threshold)
{
  ASSERT(img.initialized());

  // first, reset everything to defaults
  freeMem();

  // now, flood img to get the object
  Image<byte> dest;
 
  int area = floodCleanBB(img, dest, location, threshold, 
                          byte(1), itsBoundingBox);

  // no object found? return -1
  if (area == -1)
    {
      LDEBUG("no object found");
      itsCentroidXY.reset(location);
      return Image<byte>();
    }

  // set the dimensions of the original image
  itsImageDims = img.getDims();

  // crop the object mask from the flooding destination 
  itsObjectMask = crop(dest, itsBoundingBox);

  // get the area, the centroid, and the bounding box
  vector<float> sumx, sumy;
  itsArea = (int)sumXY(itsObjectMask, sumx, sumy);
  //itsStdDev = stdev(luminance(itsObjectMask));

  if (area != itsArea)
    LFATAL("area %i doesn't match the one from flooding %i", itsArea, area);

  int firstX, lastX, firstY, lastY;
  float cX, cY;
  bool success = (getCentroidFirstLast(sumx, cX, firstX, lastX) |
                  getCentroidFirstLast(sumy, cY, firstY, lastY));
  itsCentroidXY.reset(cX,cY);

  if (!success) LFATAL("determining the centroid failed");

  if ((firstX != 0) || (lastX != itsObjectMask.getWidth()-1) ||
      (firstY != 0) || (lastY != itsObjectMask.getHeight()-1))
    LFATAL("boundary box doesn't match the one from flooding");

  itsCentroidXY += Vector2D(itsBoundingBox.left(),itsBoundingBox.top());

  return dest;
}

// ######################################################################
int BitObject::reset(const Image<byte>& img)
{
  ASSERT(img.initialized());

  // first, reset everything to defaults
  freeMem();

  // set the dimensions of the original image
  itsImageDims = img.getDims();

  // get the area, stddev, centroid, and the bounding box
  vector<float> sumx, sumy;
  itsArea = (int)sumXY(img, sumx, sumy);

  if (itsArea == 0) 
    {
      freeMem();
      return -1;
    }

  int firstX, lastX, firstY, lastY;
  float cX, cY;
  bool success = (getCentroidFirstLast(sumx, cX, firstX, lastX) |
                  getCentroidFirstLast(sumy, cY, firstY, lastY));
  itsCentroidXY.reset(cX,cY);

  if (!success)
    {
      freeMem();
      return -1;
    }

  itsBoundingBox = Rectangle::tlbrI(firstY, firstX, lastY, lastX);

  // cut out the object mask
  itsObjectMask = crop(img, itsBoundingBox);

  LDEBUG("BB: size: %i; %s; dims: %s",itsBoundingBox.width()*itsBoundingBox.height(),
      toStr(itsBoundingBox).data(),toStr(itsObjectMask.getDims()).data());

  return itsArea;
}

// ######################################################################
void BitObject::computeSecondMoments()
{
  ASSERT(isValid());

  int w = itsObjectMask.getWidth();
  int h = itsObjectMask.getHeight();

  // The bounding box is stored in image coordinates, and so is the centroid. For
  // computing the second moments, however we need the centroid in object coords.
  float cenX = itsCentroidXY.x() - itsBoundingBox.left();
  float cenY = itsCentroidXY.y() - itsBoundingBox.top();

  // compute the second moments
  vector<float> diffX(w), diffX2(w), diffY(h), diffY2(h);
  for (int y = 0; y < h; ++y) 
    {
      diffY[y] = y - cenY;
      diffY2[y] = diffY[y] * diffY[y];
    }
  for (int x = 0; x < w; ++x) 
    {
      diffX[x] = x - cenX;
      diffX2[x] = diffX[x] * diffX[x];
    }

  Image<byte>::const_iterator optr = itsObjectMask.begin();
  for (int y = 0; y < h; ++y)
    for(int x = 0; x < w; ++x)
      {
        if (*optr != 0)
          {
            itsUxx += diffX2[x];
            itsUyy += diffY2[y];
            itsUxy += (diffX[x] * diffY[y]);
          }
        ++optr;
      }
  itsUxx /= itsArea; 
  itsUyy /= itsArea;
  itsUxy /= itsArea;

  // compute the parameters d, e and f for the ellipse:
  // d*x^2 + 2*e*x*y + f*y^2 <= 1
  float coeff = 0.F;
  if((itsUxx * itsUyy - itsUxy * itsUxy) > 0)
    coeff = 1 / (4 * (itsUxx * itsUyy - itsUxy * itsUxy));
  float d =  coeff * itsUyy;
  float e = -coeff * itsUxy;
  float f =  coeff * itsUxx;

  // from these guys, compute the parameters d and f for the
  // ellipse when it is rotated so that x is the main axis
  // and figure out the angle of rotation for this.
  float expr = sqrt(4*e*e + squareOf(d - f));
  float d2 = 0.5 * (d + f + expr);
  float f2 = 0.5 * (d + f - expr);

  // the angle is defined in clockwise (image) coordinates:
  //  --  is 0
  //  \   is 45
  //  |   is 90
  //  /   is 135
  if( d != f)  itsOriAngle = 90 * atan(2 * e / (d - f)) / M_PI;
  else	itsOriAngle = 0.F;
  if (itsUyy > itsUxx) itsOriAngle += 90.0F;
  if (itsOriAngle < 0.0F) itsOriAngle += 180.0F;

  // this checks if itsOriAngle is nan
  if (itsOriAngle != itsOriAngle) itsOriAngle = 0.0F;

  // now get the length of the major and the minor axes:
  if(f2) itsMajorAxis = 2 / sqrt(f2);
  if(d2) itsMinorAxis = 2 / sqrt(d2);
  if(itsMinorAxis) itsElongation = itsMajorAxis / itsMinorAxis;
  else itsElongation = 0.F;
  // We're done
  haveSecondMoments = true;
  return;
}
  
// ######################################################################
void BitObject::freeMem()
{
  itsObjectMask.freeMem();
  itsBoundingBox = Rectangle();
  itsCentroidXY = Vector2D();
  itsArea = 0;
  itsSMV = 0.;
  itsUxx = 0.0F;
  itsUyy = 0.0F;
  itsUxy = 0.0F;
  itsMajorAxis = 0.0F;
  itsMinorAxis = 0.0F;
  itsElongation = 0.0F;
  itsOriAngle = 0.0F;
  itsImageDims = Dims(0,0);
  itsMaxIntensity = -1.0F;
  itsMinIntensity = -1.0F;
  itsAvgIntensity = -1.0F;
  haveSecondMoments = false;
  itsClassName = DEFAULT_CLASS_NAME;
  itsClassProbability = DEFAULT_CLASS_PROBABILITY;
}
// ######################################################################
void BitObject::writeToStream(ostream& os) const
{
  // class name and probability
  os << itsClassName;
  os << itsClassProbability;

  // bounding box
  if (itsBoundingBox.isValid())
    {
      os << itsBoundingBox.top() << " "
         << itsBoundingBox.left() << " "
         << itsBoundingBox.bottomI() << " "
         << itsBoundingBox.rightI() << "\n";
    }
  else
    {
      os << "-1 -1 -1 -1\n";
    }

  // image dimensions
  os << itsImageDims.w() << " " << itsImageDims.h() << "\n";

  // centroid
  itsCentroidXY.writeToStream(os);

  // area
  os << itsArea << "\n";

  // have second moments?
  if (haveSecondMoments) os << "0\n";
  else os <<"1\n";

  // second moments
  os << itsUxx << " " << itsUyy << " " << itsUxy << "\n";

  // axes, elongation, angle
  os << itsMajorAxis << " "
     << itsMinorAxis << " "
     << itsElongation << " "
     << itsOriAngle << "\n";

  // max, min and avg intensity
  os << itsMaxIntensity << " "
     << itsMinIntensity << " "
     << itsAvgIntensity << "\n";

  // the object mask
  PnmWriter::writeAsciiBW(itsObjectMask, 1, os);

  os << "\n";

  // done
  return;
}

// ######################################################################
void BitObject::readFromStream(istream& is)
{
  // class name and probability
  is >> itsClassName;
  is >> itsClassProbability;

  // bounding box
  int t, l, b, r;
  is >> t; is >> l; is >> b; is >> r;
  if (t >= 0)
    itsBoundingBox = Rectangle::tlbrI(t, l, b, r);
  else
    itsBoundingBox = Rectangle();

  // image dims
  int w, h;
  is >> w; is >> h;
  itsImageDims = Dims(w,h);

  // centroid
  itsCentroidXY = Vector2D(is);

  // area
  is >> itsArea;

  // have second moments?
  int hs; is >> hs;
  haveSecondMoments = (hs == 1);

  // second moments
  is >> itsUxx; is >> itsUyy; is >> itsUxy;

  // axes, elongation, angle
  is >> itsMajorAxis; is >> itsMinorAxis;
  is >> itsElongation; is >> itsOriAngle;

  // max, min, avg intensity
  is >> itsMaxIntensity;
  is >> itsMinIntensity;
  is >> itsAvgIntensity;

  // object mask
  PnmParser pp(is);
  itsObjectMask = pp.getFrame().asGray();
  
}
// ######################################################################
void BitObject::setSMV(double smv)
{
  itsSMV = smv;
}
// ######################################################################
template <class T>
void BitObject::setMaxMinAvgIntensity(const Image<T>& img)
{
  ASSERT(img.getDims() == itsImageDims);
  if (!isValid()) return;

  float sum = 0.0F;
  int num = 0;

  // loop over bounding box
  const int iw = img.getWidth();
  typename Image<byte>::const_iterator bptr = itsObjectMask.begin();
  typename Image<T>::const_iterator iptr2, iptr = img.begin();
  iptr += (iw * itsBoundingBox.top() + itsBoundingBox.left());

  for (int y = itsBoundingBox.top(); y <= itsBoundingBox.bottomI(); ++y)
    {
      iptr2 = iptr;
      for (int x = itsBoundingBox.left(); x <= itsBoundingBox.rightI(); ++x)
        {
          // check if we're indeed inside the object
          if (*bptr > byte(0))
            {
              sum += (float)(*iptr2);
              ++num;
              if ((itsMaxIntensity == -1.0F) || (*iptr2 > itsMaxIntensity))
                itsMaxIntensity = *iptr2;
              if ((itsMinIntensity == -1.0F) || (*iptr2 < itsMinIntensity))
                itsMinIntensity = *iptr2;
            }
          ++bptr;
          ++iptr2;
        }
      iptr += iw;
    }

  if (sum == 0) itsAvgIntensity = 0.0F;
  else itsAvgIntensity = sum / (float)num;

}

// ######################################################################
void BitObject::getMaxMinAvgIntensity(float& maxIntensity,
                                      float& minIntensity, 
                                      float& avgIntensity)
{
  maxIntensity = itsMaxIntensity;
  minIntensity = itsMinIntensity;
  avgIntensity = itsAvgIntensity;
}

// ######################################################################
Rectangle BitObject::getBoundingBox(const BitObject::Coords coords) const
{
  switch(coords)
    {
    case OBJECT: return Rectangle::tlbrI(0, 0, itsBoundingBox.height() - 1,
                                  itsBoundingBox.width() - 1);
    case IMAGE: return itsBoundingBox;
    default: LFATAL("Unknown Coords type - don't know what to do.");
    }
  //this is never reached but we have to make the compiler happy
  return Rectangle();
}

// ######################################################################
template <class T_or_RGB>
    void BitObject::drawMaskedObject(Image<T_or_RGB>& img, const T_or_RGB backgroundcolor)
{
  ASSERT(isValid());
  ASSERT(img.initialized());
  const byte maskcolor = byte(1);
  Image<byte> mask = getObjectMask(maskcolor,IMAGE);

  // rescale if needed
  if (img.getDims() != itsImageDims)
    mask = rescale(mask, img.getDims());

  typename Image<T_or_RGB>::iterator itr = img.beginw();
  typename Image<byte>::iterator mitr = mask.beginw();

  while(itr != img.end()) {
    if((*mitr) != maskcolor) 
      *itr = backgroundcolor;
    ++itr; 
    ++mitr;
  }  
}
// ######################################################################
Image<byte> BitObject::getObjectMask(const byte value,
                                     const BitObject::Coords coords) const
{ 
  ASSERT(isValid());
  Image<byte> objectCopy = replaceVals(itsObjectMask,byte(1),value);

  switch (coords)
    {
    case OBJECT: return objectCopy;

    case IMAGE: 
      {
        Image<byte> result(itsImageDims, ZEROS);
        pasteImage(result, objectCopy, byte(0), getObjectOrigin());
        return result;
      }
   
    default: LFATAL("Unknown Coords type - don't know what to do.");
    }

  //this is never reached but we have to make the compiler happy
  return Image<byte>();
}

// ######################################################################
Dims BitObject::getObjectDims() const
{ return itsObjectMask.getDims(); }

// ######################################################################
Point2D<int> BitObject::getObjectOrigin() const
{
  return Point2D<int>(itsBoundingBox.left(),itsBoundingBox.top());
}

// ######################################################################
Point2D<int> BitObject::getCentroid(const BitObject::Coords coords) const
{
  return getCentroidXY().getPoint2D();
}

// ######################################################################
Vector2D BitObject::getCentroidXY(const BitObject::Coords coords) const
{ 
  switch (coords)
    {
    case OBJECT: return itsCentroidXY - Vector2D(itsBoundingBox.left(),itsBoundingBox.top());
    case IMAGE: return itsCentroidXY;
    default: LFATAL("Unknown Coords type - don't know what to do.");
    }
  //this is never reached but we have to make the compiler happy
  return Vector2D();
}

// ######################################################################
int BitObject::getArea() const
{ return itsArea; }

// ######################################################################
/*float BitObject::getStdDev() const
{ return itsStdDev; }*/

// ######################################################################
void BitObject::getSecondMoments(float& uxx, float& uyy, float& uxy)
{ 
  if (!haveSecondMoments) computeSecondMoments();
  uxx = itsUxx; uyy = itsUyy; uxy = itsUxy; 
}

// ######################################################################
float BitObject::getMajorAxis()
{ 
  if (!haveSecondMoments) computeSecondMoments();
  return itsMajorAxis; 
}

// ######################################################################
float BitObject::getMinorAxis()
{ 
  if (!haveSecondMoments) computeSecondMoments();
  return itsMinorAxis; 
}

// ######################################################################
float BitObject::getElongation()
{ 
  if (!haveSecondMoments) computeSecondMoments();
  return itsElongation; 
}

// ######################################################################
float BitObject::getOriAngle()
{ 
  return itsOriAngle; 
}
// ######################################################################
double BitObject::getSMV()
{
  return itsSMV;
}
// ######################################################################
std::string BitObject::getClassName()
{
  return itsClassName;
}
// ######################################################################
float BitObject::getClassProbability()
{
  return itsClassProbability;
}
// ######################################################################
Dims BitObject::getImageDims() const
{
  return itsImageDims;
}
// ######################################################################
bool BitObject::isValid() const
{ return ((itsArea > 0) && itsBoundingBox.isValid()); }

// ######################################################################
bool BitObject::doesIntersect(const BitObject& other) const
{
  double s = intersect(other);
  return (s > 0.0);
}

// ######################################################################
double BitObject::intersect(const BitObject& other) const
{
  // are this and other actually valid? no -> return false
  if (!(isValid() && other.isValid()))
    {
      LINFO("no intersect, because one of the objects is invalid.");
      return 0;
    }

  Rectangle tBB = getBoundingBox(IMAGE);
  Rectangle oBB = other.getBoundingBox(IMAGE);

  // compute the intersecting bounding box params
  int ll = max(tBB.left(),oBB.left());
  int rr = min(tBB.rightI(),oBB.rightI());
  int tt = max(tBB.top(),oBB.top());
  int bb = min(tBB.bottomI(),oBB.bottomI()); ;

  // is this a valid rectangle?
  if ((ll > rr)||(tt > bb))
    {
      LDEBUG("No intersect because the bounding boxes don't overlap: %s and %s",
         toStr(tBB).data(),toStr(oBB).data());
      return 0;
    }

  // get the rectangles in order to crop the object masks
  Rectangle tCM = Rectangle::tlbrI(tt - tBB.top(), ll - tBB.left(),
                                  bb - tBB.top(), rr - tBB.left());
  Rectangle oCM = Rectangle::tlbrI(tt - oBB.top(), ll - oBB.left(),
                                  bb - oBB.top(), rr - oBB.left());

  // crop the object masks and get the intersecting image
  Image<byte> cor = takeMin(crop(getObjectMask(byte(1),OBJECT),tCM),
                            crop(other.getObjectMask(byte(1),OBJECT),oCM));
  double s = sum(cor);

  LDEBUG("tCM = %s; oCM = %s; this.ObjMask.dims = %s; other.ObjMask.dims = %s; sum = %g",
      toStr(tCM).data(),toStr(oCM).data(),
      toStr(getObjectMask(byte(1),OBJECT).getDims()).data(),
      toStr(other.getObjectMask(byte(1),OBJECT).getDims()).data(),s);

  return s;
}

// ######################################################################
template <class T_or_RGB>
void BitObject::drawShape(Image<T_or_RGB>& img, 
                          const T_or_RGB& color,
                          float opacity)
{
  ASSERT(isValid());
  ASSERT(img.initialized());
  Dims d = img.getDims();
  Image<byte> mask = itsObjectMask;
  Rectangle bbox = itsBoundingBox;

  // rescale if needed
  if (d != itsImageDims) {
    float scaleW = (float) d.w() / (float) itsImageDims.w();
    float scaleH = (float) d.h() / (float) itsImageDims.h();
    int i = (int) ((float) bbox.left() * scaleW);
    int j = (int) ((float) bbox.top() * scaleH);
    int w = (int) ((float) bbox.width() * scaleW);
    int h = (int) ((float) bbox.height() *scaleH);
    const Point2D<int> topleft(i,j);
    bbox = Rectangle(topleft, Dims(w,h));
    mask = rescaleNI(mask, d.w(), d.h());
  }

  int w = img.getWidth();
  float op2 = 1.0F - opacity;

  typename Image<T_or_RGB>::iterator iptr, iptr2;
  Image<byte>::const_iterator mptr = mask.begin();
  iptr2 = img.beginw() + bbox.top() * w + bbox.left();
  for (int y = bbox.top(); y <= bbox.bottomI(); ++y)
    {
      iptr = iptr2;
      for (int x = bbox.left(); x <= bbox.rightI(); ++x)
        {
          if (*mptr > 0) *iptr = T_or_RGB(*iptr * op2 + color * opacity);
          ++iptr; ++mptr;
        }
      iptr2 += w;
    }
}

// ######################################################################
template <class T_or_RGB>
void BitObject::drawOutline(Image<T_or_RGB>& img, 
                            const T_or_RGB& color,
                            float opacity)
{
  ASSERT(isValid());
  ASSERT(img.initialized());
  float op2 = 1.0F - opacity;

  Dims d = img.getDims();
  Image<byte> mask = getObjectMask();

  // rescale if needed
  if (d != itsImageDims)
    mask = rescaleNI(mask, d.w(), d.h());

  // object-shaped drawing
  int thick = 1;
  Image<byte> om(mask);
  om = contour2D(om);       // compute binary contour image
  const int w = img.getWidth();
  const int h = img.getHeight();
  Point2D<int> ppp;
  for (ppp.j = 0; ppp.j < h; ppp.j ++)
    for (ppp.i = 0; ppp.i < w; ppp.i ++)
      if (om.getVal(ppp.i, ppp.j))  // got a contour point -> draw here
        drawDisk(img, ppp, thick, T_or_RGB(img.getVal(ppp) * op2 + color * opacity));  // small disk for each point

} // end drawOutline
  

// ######################################################################
template <class T_or_RGB>
void BitObject::drawBoundingBox(Image<T_or_RGB>& img, 
                                const T_or_RGB& color,
                                float opacity)
{
  ASSERT(isValid());
  ASSERT(img.initialized());
  Rectangle bbox = itsBoundingBox;

  // rescale if needed
  if (img.getDims() != itsImageDims) {
    Dims d = img.getDims();
    float scaleW = (float) d.w() / (float) itsImageDims.w();
    float scaleH = (float) d.h() / (float) itsImageDims.h();
    int i = (int) ((float) bbox.left() * scaleW);
    int j = (int) ((float) bbox.top() * scaleH);
    int w = (int) ((float) bbox.width() * scaleW);
    int h = (int) ((float) bbox.height() *scaleH);
    const Point2D<int> topleft(i,j);
    bbox = Rectangle(topleft, Dims(w,h));
  }

  float op2 = 1.0F - opacity;
  int t = bbox.top();
  int b = bbox.bottomI();
  int l = bbox.left();
  int r = bbox.rightI();
  
  for (int x = l; x <= r; ++x)
    {
      Point2D<int> p1(x,t), p2(x,b);
      img.setVal(p1,img.getVal(p1) * op2 + color * opacity);
      img.setVal(p2,img.getVal(p2) * op2 + color * opacity);
    }
  for (int y = t+1; y < b; ++y)
    {
      Point2D<int> p1(l,y), p2(r,y);
      img.setVal(p1,img.getVal(p1) * op2 + color * opacity);
      img.setVal(p2,img.getVal(p2) * op2 + color * opacity);
    }
}

// ######################################################################
template <class T_or_RGB>
void BitObject::draw(BitObjectDrawMode mode, Image<T_or_RGB>& img, 
                     const T_or_RGB& color, float opacity)
{
  switch(mode)
    {
    case BODMnone:    break;
    case BODMshape:   drawShape(img, color, opacity); break;
    case BODMoutline: drawOutline(img, color, opacity); break;
    case BODMbbox:    drawBoundingBox(img, color, opacity); break;
    default: LERROR("Unknown BitObjectDrawMode: %i - ignoring!",mode);
    }
}

// ######################################################################
// Instantiation of template functions
// ######################################################################
template void BitObject::setMaxMinAvgIntensity(const Image<byte>& img);
template void BitObject::setMaxMinAvgIntensity(const Image<float>& img);

#define INSTANTIATE(T_or_RGB) \
template void BitObject::drawShape(Image< T_or_RGB >& img, \
                                   const T_or_RGB& color, \
                                   float opacity); \
template void BitObject::drawOutline(Image< T_or_RGB >& img, \
                                     const T_or_RGB& color, \
                                     float opacity); \
template void BitObject::drawBoundingBox(Image< T_or_RGB >& img, \
                                         const T_or_RGB& color, \
                                         float opacity); \
template void BitObject::draw(BitObjectDrawMode mode, \
                              Image< T_or_RGB >& img, \
                              const T_or_RGB& color, \
                              float opacity); \
template void BitObject::drawMaskedObject(Image< T_or_RGB >& img, \
                                          const T_or_RGB backgroundcolor); 

INSTANTIATE(PixRGB<float>);
INSTANTIATE(PixRGB<byte>);
INSTANTIATE(byte);
INSTANTIATE(float);
