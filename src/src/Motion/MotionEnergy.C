/*!@file Motion/MotionEnergy.C detect motion in an image stream   */
// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
// //////////////////////////////////////////////////////////////////// //
// This file is part of the iLab Neuromorphic Vision C++ Toolkit.       //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is free software; you can   //
// redistribute it and/or modify it under the terms of the GNU General  //
// Public License as published by the Free Software Foundation; either  //
// version 2 of the License, or (at your option) any later version.     //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  //
// that it will be useful, but WITHOUT ANY WARRANTY; without even the   //
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU General Public License for more details.       //
//                                                                      //
// You should have received a copy of the GNU General Public License    //
// along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   //
// to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   //
// Boston, MA 02111-1307 USA.                                           //
// //////////////////////////////////////////////////////////////////// //
//
// Primary maintainer for this file: Lior Elazary <lelazary@yahoo.com>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/RCBot/Motion/MotionEnergy.C $
// $Id: MotionEnergy.C 13756 2016-08-04 21:57:32Z siagian $
//

#include "MotionEnergy.H"

#include "Image/ColorOps.H"
#include "Image/DrawOps.H"
#include "Image/PyramidOps.H"
#include "Image/fancynorm.H"
#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "Image/ImageSetOps.H"
#include "Image/MathOps.H"
#include "Image/ShapeOps.H"
#include "Util/sformat.H"
#include "rutz/compat_cmath.h" // for M_PI
#include "rutz/trace.h"

#define max_queue    2      /* maximum queue length for motion pyramid queue */
#define sml          0//4   /* pyramid level of the saliency map */
#define level_min    0//2   /* min center level */
#define level_max    2//4   /* max center level */
#define delta_min    1//3   /* min center-surround delta */
#define delta_max    2//4   /* max center-surround delta */
#define maxdepth       (level_max + delta_max + 1)
#define num_feat_maps  ((level_max - level_min + 1) * (delta_max - delta_min + 1)) /* num of feature maps per pyramid */

using namespace std;

// ######################################################################
// ##### MotionEnergyPyrBuilder Functions:
// ######################################################################

MotionEnergyPyrBuilder::MotionEnergyPyrBuilder(const PyramidType type, const float timeDecay):
itsPyramidType(type),
itsTimeDecay(timeDecay)
{

    float direction = 0.;
    itsDir[0] =  new ReichardtPyrBuilder<float>
                           (cos(direction * M_PI / 180.0),
                            -sin(direction * M_PI / 180.0),
                            itsPyramidType, direction + 90.0);
    direction = 90.;
    itsDir[1] =  new ReichardtPyrBuilder<float>
                           (cos(direction * M_PI / 180.0),
                            -sin(direction * M_PI / 180.0),
                            itsPyramidType, direction + 90.0);
    direction = 180.;
    itsDir[2] =  new ReichardtPyrBuilder<float>
                           (cos(direction * M_PI / 180.0),
                            -sin(direction * M_PI / 180.0),
                            itsPyramidType, direction + 90.0);

    direction = 270.;
    itsDir[3] =  new ReichardtPyrBuilder<float>
                           (cos(direction * M_PI / 180.0),
                            -sin(direction * M_PI / 180.0),
                            itsPyramidType, direction + 90.0);
}

MotionEnergyPyrBuilder::~MotionEnergyPyrBuilder()
{
    delete[] itsDir;
}

// ##############################################################################################################
Image<byte> MotionEnergyPyrBuilder::updateMotion(const Image<byte>& lum, const Image<byte>& clipMask) {

    for (int i = 0; i < 4; i++)
        if (itsPq[i].size() > max_queue)
            itsPq[i].pop_back();

    ImageSet<float> clipPyr = buildPyrGaussian(Image<float>(clipMask)/255.0f, 0, maxdepth, 9);
    doLowThresh(clipPyr, 1.0f, 0.0f);

    Image<float> smap;

    ImageSet<float> d = itsDir[0]->build(lum, level_min, maxdepth);
    itsPq[0].push_front(d);
    Image<float> chanm = normalizMAP(computeMMAP(0, clipPyr, "dir0"), Dims(0,0), "dir0");

    d = itsDir[1]->build(lum, level_min, maxdepth);
    itsPq[1].push_front(d);
    chanm += normalizMAP(computeMMAP(1, clipPyr, "dir90"), Dims(0,0), "dir0");

    d = itsDir[2]->build(lum, level_min, maxdepth);
    itsPq[2].push_front(d);
    chanm += normalizMAP(computeMMAP(2, clipPyr, "dir180"), Dims(0,0), "dir180");

    d = itsDir[3]->build(lum, level_min, maxdepth);
    itsPq[3].push_front(d);
    chanm += normalizMAP(computeMMAP(3, clipPyr, "dir270"), Dims(0,0), "dir270");

    smap = normalizMAP(chanm, Dims(0,0), "Motion")/ 4.0F;

    // *** do one final round of spatial competition for salience
    // (one-iteration of fancy maxnorm) on the combined saliency map:
    smap = maxNormalize(smap, 0.0F, 2.0F, VCXNORM_FANCYONE);

    // create a composite image with input + saliency map
    Image<float> smapn = smap; inplaceNormalize(smapn, 0.0F, 255.0F);

    // enhance gamma with map
    smapn = rescale(smapn, lum.getDims());
    return static_cast< Image<byte> > (smapn);
 }

// ######################################################################
// Compute a motion conspicuity map from a motion pyramid
Image<float> MotionEnergyPyrBuilder::computeMMAP(const uint index, ImageSet<float>& clipMask, const char *label)
{
  LDEBUG("Building %s channel:", label);

  // alloc conspicuity map and clear it:
  Image<float> cmap(clipMask[sml].getDims(), ZEROS);

  // get all the center-surround maps and combine them:
  for (int delta = delta_min; delta <= delta_max; ++delta)
    for (int lev = level_min; lev <= level_max; ++lev)
      {
        Image<float> tmp = centerSurround(lev, lev + delta, index, clipMask);
        string lbl = sformat("  %s(%d,%d)", label, lev, lev + delta);
        tmp = normalizMAP(tmp, cmap.getDims(), lbl.c_str());
        cmap += tmp;
      }

  float mi, ma;
  getMinMax(cmap, mi, ma); LDEBUG("%s: final cmap range [%f .. %f]", label, mi, ma);

  return cmap;
}

// ######################################################################
Image<float> MotionEnergyPyrBuilder::centerSurround(const uint cntrlev,
                                                    const uint surrlev,
                                                    const uint index,
                                                    ImageSet<float>& clipMask)
{
  // do basic center-surround for the front (latest) pyramid in queue:
  const ImageSet<float> pyr = itsPq[index].front();

  // compute center-surround:
  Image<float> cs = ::centerSurround(pyr, cntrlev, surrlev, true, &clipMask);

  // do additional processing with other pyramids in queue:
  for (uint i = 1; i < itsPq[index].size(); ++i)
    {
      const ImageSet<float> pyr2 = itsPq[index][i];
      float fac = exp(itsTimeDecay);
      cs += ::centerSurroundDiff(pyr, pyr2, cntrlev, surrlev, true, &clipMask) * fac;
    }

  return cs;
}

// ######################################################################
// Compute center surround difference
/*Image<float> MotionEnergyPyrBuilder::centerSurroundDiff(ImageSet<float>& pyr1, ImageSet<float>& pyr2, ImageSet<float>& clipMask, const char *label)
{
  // alloc conspicuity map and clear it:
  Image<float> cmap(pyr1[sml].getDims(), ZEROS);

  // get all the center-surround maps and combine them:
  for (int delta = delta_min; delta <= delta_max; ++delta)
    for (int lev = level_min; lev <= level_max; ++lev)
      {
        Image<float> tmp = ::centerSurroundDiff(pyr1, pyr2, lev, lev + delta, true, &clipMask);
        string lbl = sformat("  %s(%d,%d)", label, lev, lev + delta);
        tmp = normalizMAP(tmp, cmap.getDims(), lbl.c_str());

        cmap += tmp;
      }

  float mi, ma;
  getMinMax(cmap, mi, ma); LINFO("%s: final cmap range [%f .. %f]", label, mi, ma);

  return cmap;
}*/

// ######################################################################
//! Max-normalize a feature map or conspicuity map
Image<float> MotionEnergyPyrBuilder::normalizMAP(const Image<float>& ima, const Dims& dims, const char *label)
{
  // do we want to downsize the map?
  Image<float> dsima;
  if (dims.isNonEmpty()) dsima = rescale(ima, dims); else dsima = ima;

  // apply spatial competition for salience (mx-normalization):
  Image<float> result = maxNormalize(dsima, MAXNORMMIN, MAXNORMMAX, VCXNORM_MAXNORM);

  // show some info about this map:
  float mi, ma, nmi, nma; getMinMax(ima, mi, ma); getMinMax(result, nmi, nma);
  LDEBUG("%s: raw range [%f .. %f] max-normalized to [%f .. %f]", label, mi, ma, nmi, nma);

  return result;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
