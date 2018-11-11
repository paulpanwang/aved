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

/*!@file FeatureCollection.H a class for calculating features in video.
 */

#ifndef FEATURECOLLECTION_H_DEFINED_
#define FEATURECOLLECTION_H_DEFINED_

#include "Image/BitObject.H"
#include "Image/Dims.H"
#include "Image/ImageSet.H"
#include "nub/ref.h"
#include "Media/MbariResultViewer.H"
#include "Data/ImageData.H"
#include "Features/HistogramOfGradients.H"

#include <list>

class MbariResultViewer;
namespace nub { template <class T> class soft_ref; }

// ######################################################################
//! runs the FeatureCollection extraction algorithms
class FeatureCollection {
public:

    //! constructor
    FeatureCollection(Dims scaledDims);

    //! destructor
    ~FeatureCollection();

    //TODO: refactor info a class
    typedef struct Data {
        std::vector<double> featureHOG3;
        std::vector<double> featureMBH3;
        std::vector<double> featureHOG8;
        std::vector<double> featureMBH8;
        std::vector<double> featureJETred;
        std::vector<double> featureJETgreen;
        std::vector<double> featureJETblue;
    } Data;


    //! extract features from the bounding box
    /* @bbox bounding box that defines the object
    @imgData struct that contains images data used in computing features
    @return Data*/
    Data extract(Rectangle bbox, ImageData &imgData);

    // ! Return measure of the feature similarity
    double getFeatureSimilarity(std::vector<double> &feat1, std::vector<double> &feat2);

private:

    Dims itsScaledDims;
    bool fixedHistogram;
    bool normalizeHistogram;
    nub::soft_ref <MbariResultViewer> itsRv;

    // HOG feature
    //8x8 = 1296 features
    //3x3 =  36 features
    HistogramOfGradients itsHog3x3;
    HistogramOfGradients itsHog8x8;

    //! Compute HOG features on an RGB image at a location defined by the bounding box
    std::vector<double> getFeatureCollectionHOG(Image< PixRGB<byte> > &input, HistogramOfGradients &hog,
                                                 Rectangle bboxScaled);

    //! Compute Motion Boundary Histogram features on an RGB image at a location defined by the bounding box
    std::vector<double> getFeatureCollectionMBH(const Image< PixRGB<byte> > &input,
                                       const Image< PixRGB<byte> > &prevInput,
                                       HistogramOfGradients &hog, Rectangle bboxScaled);

    //! Compute the gradient on a color img by taking the max gradient
    //! If numOrientations > 0 then snap to the best orientation
    void getMaxGradient(const Image< PixRGB<byte> > &img,
                        Image<float> &mag, Image<float> &ori,
                        int numOrientations);

    //! Compute HOG features on an RGB image
    ImageSet<float> getFeatureCollection(const Image< PixRGB<byte> > &img, int numBins);

    Image<PixRGB<byte> > getHistogramImage(const ImageSet<float> &hist, const int lineSize);

    ImageSet<double> computeFeatureCollection(const ImageSet<float> &hist);

    ImageSet<float> getOriHistogram(const Image<float> &mag, const Image<float> &ori, int numOrientations,
                                    int numBins);

    Image<float> convolveFeatureCollection(const ImageSet<float> &imgFeatureCollection,
                                  const ImageSet<float> &filterFeatureCollection);

    Image<double> getHistogramEnergy(const ImageSet<float> &hist);

    std::vector<double> computeInvariants(const Image<float> &input, int scale);

    float absMean(const Image<float> &in);

    float negMean(const Image<float> &in);

    float posMean(const Image<float> &in);

    float computeBorderMean(const Image<float> &img);

    Image<float> getInner(const Image<float> &img, int border);

    void getComponents(const Image<PixRGB<byte> > &src,
                       Image<float> &red, Image<float> &green, Image<float> &blue);
};
#endif
