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

/*!@file FOEestimator.C a class for estimating the focus of expansion in a video */ 

#include "DetectionAndTracking/FOEestimator.H"

#include "Image/CutPaste.H"  // for crop()
#include "Image/ImageSet.H"
#include "Image/PyramidOps.H"

#include <vector>

// ######################################################################
FOEestimator::FOEestimator(int numAvg, int pyramidLevel)
  : itsPyrLevel(pyramidLevel),
    itsFrames(3),
    itsXvectors(numAvg),
    itsYvectors(numAvg)
{ }

// ######################################################################
Vector2D FOEestimator::updateFOE(const Image<byte>& img,
                                 const Rectangle region)
{
  return (updateFOE(crop(img,region)) +
          Vector2D(region.left(),region.top()));
}

// ######################################################################
Vector2D FOEestimator::updateFOE(const Image<byte>& img)
{
  // subsample the image to the desired level
  ImageSet<byte> iset = buildPyrGaussian(img,0,itsPyrLevel+1,3);
  itsFrames.push_back(iset[itsPyrLevel]);

  // need at least three frames to compute optical flow
  if (itsFrames.size() < 3) return Vector2D();

  int w = itsFrames.back().getWidth();
  int h = itsFrames.back().getHeight();

  // get the iterators ready to step through the three images
  // at times (t-1), t, and (t+1)
  Image<byte>::const_iterator it,tm,tp,xm,xp,ym,yp;
  it = itsFrames[1].begin();
  xm = it + w;
  xp = it + w + 2;
  ym = it + 1;
  yp = it + w + w + 1;
  tm = itsFrames[0].begin() + w + 1;
  tp = itsFrames[2].begin() + w + 1;

  // prepare the vectors for the results
  Image<float> vx(w-2,1,ZEROS);
  Image<float> vy(h-2,1,ZEROS);
  std::vector<int> cx(w-2,0), cy(h-2,0);

  // compute optical flow and sum up the x components over y, and the
  // y components over x
  for (int y = 0; y < h-2; ++y)
    {
      for (int x = 0; x < w-2; ++x)
        {
          // x component of optical flow
          if (*xp != *xm)
            {
              vx[x] += (float(*tm)-float(*tp)) / (float(*xp) - float(*xm));
              cx[x]++;
            }

          // y component of optical flow
          if (*yp != *ym)
            {
              vy[y] += (float(*tm)-float(*tp)) / (float(*yp) - float(*ym));
              cy[y]++;
            }
          ++tm; ++tp; ++xm; ++xp; ++ym; ++yp;
        }

      // divide the y guys by the number of contributions
      if (cy[y] > 0) vy[y] /= cy[y];

      // offset to jump to the next line
      tm += 2; tp += 2;
      xm += 2; xp += 2;
      ym += 2; yp += 2;
    }

  // divide the x guys by the number of contributions
  for (int x = 0; x < w-2; ++x)
      if (cx[x] > 0) vx[x] /= cx[x];

  // store the vectors in the cache to generate averages
  itsXvectors.push_back(vx);
  itsYvectors.push_back(vy);

  // now do the linear regression for the x and y directions over the mean
  float x0 = getZeroCrossing(itsXvectors.mean());
  float y0 = getZeroCrossing(itsYvectors.mean());

  // need to scale the coordinates according to the subsampling done
  float pw2 = pow(2,itsPyrLevel);
  itsFOE.reset(pw2*(x0+1),pw2*(y0+1));

  return itsFOE;
}



// ######################################################################
float FOEestimator::getZeroCrossing(const Image<float>& vec)
{
  float N = vec.getWidth();
  float sx = N * (N - 1) / 2;
  float sxx = N * (N - 1) * (2*N - 1) / 6;
  float sy = 0.0F, sxy = 0.0F;
  for (int i = 0; i < N; ++i)
    {
      sy += vec[i];
      sxy += i*vec[i];
    }
  return (sx * sxy - sxx * sy) / (N * sxy - sx * sy);
}


// ######################################################################
Vector2D FOEestimator::getFOE()
{
  return itsFOE;
}

