/*
 * Copyright 2018 MBARI
 *
 * Licensed under the GNU LESSER GENERAL PUBLIC LICENSE, Version 3.0
 * (the "License"); you may not use this file except in compliance
 * with the License. You may obtain a copy of the License at
 *
 * http://www.gnu.org/copyleitsFeatureCollection/lesser.html
 *
 * Unless required by applicable law or agreed to in writing, so its FeatureCollection are
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

/*!@file FeatureCollection.C a class for calculating features used in classification */

#include "Learn/Features.H"
#include "Image/ColorOps.H"
#include "Image/DrawOps.H"
#include "Image/ShapeOps.H"
#include "Image/CutPaste.H"
#include "Image/Convolutions.H"
#include "Image/FilterOps.H"
#include "Image/PixelsTypes.H"
#include "Image/MathOps.H"
#include "Motion/OpticalFlow.H"
#include "Motion/MotionOps.H"
#include "Util/StringConversions.H"

#include <csignal>
#include <vector>
#include <algorithm>

// unit vectors used to compute gradient orientation
const double itsUU[9] = {1.0000,
                         0.9397,
                         0.7660,
                         0.500,
                         0.1736,
                         -0.1736,
                         -0.5000,
                         -0.7660,
                         -0.9397};
const double itsVV[9] = {0.0000,
                         0.3420,
                         0.6428,
                         0.8660,
                         0.9848,
                         0.9848,
                         0.8660,
                         0.6428,
                         0.3420};

using namespace std;

// ######################################################################
FeatureCollection::FeatureCollection(Dims scaledDims)
        : itsScaledDims(scaledDims),
          fixedHistogram(true),
          normalizeHistogram(true),
          itsHog8x8(normalizeHistogram,Dims(8,8),fixedHistogram),
          itsHog3x3(normalizeHistogram,Dims(3,3),fixedHistogram)
{
}

// ######################################################################
FeatureCollection::~FeatureCollection() {
}

// ######################################################################
FeatureCollection::Data FeatureCollection::extract(Rectangle bbox, ImageData &imgData) {
    // compute the correct bounding box and cut it out
    Dims dims = imgData.img.getDims();
    float scaleW = (float) dims.w() / (float) itsScaledDims.w();
    float scaleH = (float) dims.h() / (float) itsScaledDims.h();
    Rectangle bboxScaled = Rectangle::tlbrI(bbox.top() * scaleH, bbox.left() * scaleW,
                                            bbox.bottomI() * scaleH,
                                            bbox.rightI() * scaleW);
    bboxScaled = bboxScaled.getOverlap(Rectangle(Point2D<int>(0, 0), dims - 1));

    Data data;
    data.featureHOG3 = getFeatureCollectionHOG(imgData.clampedImg, itsHog3x3, bboxScaled);
    data.featureHOG8 = getFeatureCollectionHOG(imgData.clampedImg, itsHog8x8, bboxScaled);
    data.featureMBH3 = getFeatureCollectionMBH(imgData.prevImg, imgData.img, itsHog3x3, bboxScaled);
    data.featureMBH8 = getFeatureCollectionMBH(imgData.prevImg, imgData.img, itsHog8x8, bboxScaled);

    // compute the correct bounding box and cut it out
    dims = imgData.img.getDims();
    scaleW = (float) dims.w() / (float) itsScaledDims.w();
    scaleH = (float) dims.h() / (float) itsScaledDims.h();
    bboxScaled = Rectangle::tlbrI(bbox.top() * scaleH, bbox.left() * scaleW,
                                  (bbox.top() + bbox.height()) * scaleH,
                                  (bbox.left() + bbox.width()) * scaleW);
    bboxScaled = bboxScaled.getOverlap(Rectangle(Point2D<int>(0, 0), dims - 1));
    Image< PixRGB<byte> > rawCroppedInput = crop(imgData.img, bboxScaled);
    Image< PixRGB<byte> > inputImg = rescale(rawCroppedInput, 100, 100);

    Image<float> inputRed;
    Image<float> inputBlue;
    Image<float> inputGreen;
    getComponents(inputImg, inputRed, inputGreen, inputBlue);

    data.featureJETred = computeInvariants(inputRed, 3);
    data.featureJETblue =  computeInvariants(inputGreen, 3);;
    data.featureJETgreen = computeInvariants(inputBlue, 3);

    return data;
}


// ######################################################################
double FeatureCollection::getFeatureSimilarity(vector<double> &feat1, vector<double> &feat2) {
    // feature similarity  [ 0.0 ... 1.0 ]:
    double cval = 0.0;
    for (uint i = 0; i < feat1.size(); i++) {
        double val = pow(feat1[i] - feat2[i], 2.0);
        cval += val;
    }
    cval = sqrt(cval / feat1.size());
    LDEBUG("feature similarity: %g", cval);
    return cval;
}

// ######################################################################
vector<double> FeatureCollection::getFeatureCollectionHOG(Image< PixRGB<byte> > &in,
                                         HistogramOfGradients &hog,
                                         Rectangle bboxScaled)
{
    Image< PixRGB<byte> > rawCroppedInput(Dims(bboxScaled.width(),bboxScaled.height()), ZEROS);

    // cut out the rectangle and save it
    if (in.initialized())
        rawCroppedInput = crop(in, bboxScaled);

    // get the HOG features used in training
    Image<float>  lum,rg,by;
    getLAB(rawCroppedInput, lum, rg, by);
    vector<float> hist = hog.createHistogram(lum,rg,by);
    vector<double> histDouble(hist.begin(), hist.end());

    return histDouble;
}


// ######################################################################
vector<double> FeatureCollection::getFeatureCollectionMBH(const Image< PixRGB<byte> > &input,
                                         const Image< PixRGB<byte> > &prevInput,
                                         HistogramOfGradients &hog,
                                         Rectangle bboxScaled)
{ 
// scale if needed and cut out the rectangle and save it
Image< PixRGB<byte> > evtImg(Dims(bboxScaled.width(),bboxScaled.height()), ZEROS);
Image< PixRGB<byte> > evtImgPrev(Dims(bboxScaled.width(),bboxScaled.height()), ZEROS);

if (prevInput.initialized()) {
    evtImg = crop(prevInput, bboxScaled);
}

if (input.initialized()) {
    evtImgPrev = crop(input, bboxScaled);
}

//rutz::shared_ptr<MbariOpticalFlow> flow2 =
//Image< PixRGB<byte> > in(itsScaledDims, ZEROS);
//Image< PixRGB<byte> > inPrev(itsScaledDims, ZEROS);

//    getOpticFlow
//            (Image<byte>(luminance(inPrev)),
//             Image<byte>(luminance(in)));
//Image<PixRGB<byte> > opticFlow2 = drawOpticFlow(in, flow2);
//itsRv->display(opticFlow2, -1, "opticflow");

    // get the features used in training
    Image<float> lum, rg, by;
    getLAB(evtImg, lum, rg, by);

    // compute the optic flow
    rutz::shared_ptr<MbariOpticalFlow> flow =
        getOpticFlow
                (Image<byte>(luminance(evtImg)),
                 Image<byte>(luminance(evtImgPrev)));

    Image<PixRGB<byte> > opticFlow = drawOpticFlow(evtImgPrev, flow);
    int frameNum = -1;
    //itsRv->display(opticFlow, frameNum, "opticflow");
    vector<rutz::shared_ptr<MbariFlowVector> > vectors = flow->getFlowVectors();
    Image< float > xflow(evtImg.getDims(), ZEROS);
    Image< float > yflow(evtImg.getDims(), ZEROS);

    // we are going to assume we have a sparse flow field
    // and random access on the field
    for(uint v = 0; v < vectors.size(); v++)
    {
        Point2D<float> pt = vectors[v]->p1;
        uint i = pt.i;
        uint j = pt.j;
        float xmag  = 100.0f * vectors[v]->xmag;
        float ymag  = 100.0f * vectors[v]->ymag;
        xflow.setVal(i,j, xmag );
        yflow.setVal(i,j, ymag );
        //if(xmag > 0.0) xflow.setVal(i,j, xmag );
        //if(ymag > 0.0) yflow.setVal(i,j, ymag );
    }

    Image<float> mag, ori;
    vector<double> hist;
    gradientSobel(xflow, mag, ori, 3);
    //itsRv->display(static_cast< Image<byte> >(xflow), frameNum, "XFlow");
    //itsRv->display(static_cast< Image<byte> >(mag), frameNum, "XGradmag");
    vector<float> histx = hog.createHistogram(static_cast< Image<byte> >(xflow),rg,by);
    hist.insert(hist.begin(), histx.begin(), histx.end());

    gradientSobel(yflow, mag, ori, 3);
    //itsRv->display(static_cast< Image<byte> >(yflow), frameNum, "YFlow");
    //itsRv->display(static_cast< Image<byte> >(mag), frameNum, "YGradmag");
    vector<float> histy = hog.createHistogram(static_cast< Image<byte> >(yflow),rg,by);
    hist.insert(hist.begin(), histy.begin(), histy.end());

    Image<float> yFilter(1, 3, ZEROS);
    Image<float> xFilter(3, 1, ZEROS);
    yFilter.setVal(0, 0, 1.F);
    yFilter.setVal(0, 2, 1.F);
    xFilter.setVal(0, 0, 1.F);
    xFilter.setVal(2, 0, 1.F);

    Image<float> yyflow = sepFilter(yflow, Image<float>(), yFilter, CONV_BOUNDARY_ZERO);
    histy = hog.createHistogram(static_cast< Image<byte> >(yyflow),rg,by);
    hist.insert(hist.begin(), histy.begin(), histy.end());

    Image<float> xxflow = sepFilter(xflow, xFilter, Image<float>(), CONV_BOUNDARY_ZERO);
    histx = hog.createHistogram(static_cast< Image<byte> >(xxflow),rg,by);
    hist.insert(hist.begin(), histx.begin(), histx.end());

    //itsRv->display(static_cast< Image<byte> >(yyflow), frameNum, "YYder");
    //itsRv->display(static_cast< Image<byte> >(xxflow), frameNum, "XXder");

    Image<float> yxflow = sepFilter(yflow, xFilter, Image<float>(), CONV_BOUNDARY_ZERO);
    histy = hog.createHistogram(static_cast< Image<byte> >(yxflow),rg,by);
    hist.insert(hist.begin(), histy.begin(), histy.end());

    Image<float> xyflow = sepFilter(xflow, Image<float>(), yFilter, CONV_BOUNDARY_ZERO);
    histx = hog.createHistogram(static_cast< Image<byte> >(xyflow),rg,by);
    hist.insert(hist.begin(), histx.begin(), histx.end());

    //itsRv->display(static_cast< Image<byte> >(yxflow), frameNum, "YXder");
    //itsRv->display(static_cast< Image<byte> >(xyflow), frameNum, "XYder");
    histy = hog.createHistogram(static_cast< Image<byte> >(yxflow),rg,by);
    hist.insert(hist.begin(), histy.begin(), histy.end());
    histx = hog.createHistogram(static_cast< Image<byte> >(xyflow),rg,by);
    hist.insert(hist.begin(), histx.begin(), histx.end());

    vector<double> histDouble(hist.begin(), hist.end());

    return histDouble;
}

// ######################################################################
Image<float> FeatureCollection::convolveFeatureCollection(const ImageSet<float> &imgFeatureCollection,
                                        const ImageSet<float> &filterFeatureCollection) {
    if (imgFeatureCollection.size() == 0)
        return Image<float>();

    ASSERT(imgFeatureCollection.size() == filterFeatureCollection.size());

    //Compute size of output
    int w = imgFeatureCollection[0].getWidth() - filterFeatureCollection[0].getWidth() + 1;
    int h = imgFeatureCollection[0].getHeight() - filterFeatureCollection[0].getHeight() + 1;

    int filtWidth = filterFeatureCollection[0].getWidth();
    int filtHeight = filterFeatureCollection[0].getHeight();
    int srcWidth = imgFeatureCollection[0].getWidth();

    Image<float> score(w, h, ZEROS);

    for (uint i = 0; i < imgFeatureCollection.size(); i++) {
        Image<float>::const_iterator srcPtr = imgFeatureCollection[i].begin();
        Image<float>::const_iterator filtPtr = filterFeatureCollection[i].begin();
        Image<float>::iterator dstPtr = score.beginw();

        for (int y = 0; y < h; y++)
            for (int x = 0; x < w; x++) {
                //Convolve the filter
                float val = 0;
                for (int yp = 0; yp < filtHeight; yp++)
                    for (int xp = 0; xp < filtWidth; xp++)
                        val += srcPtr[(y + yp) * srcWidth + (x + xp)] * filtPtr[yp * filtWidth + xp];

                *(dstPtr++) += val;
            }
    }

    return score;
}

// ######################################################################
ImageSet<float> FeatureCollection::getOriHistogram(const Image<float> &mag, const Image<float> &ori, int numOrientations,
                                          int numBins) {
    Dims blocksDims = Dims(
            (int) round((double) mag.getWidth() / double(numBins)),
            (int) round((double) mag.getHeight() / double(numBins)));

    ImageSet<float> hist(numOrientations, blocksDims, ZEROS);

    Image<float>::const_iterator magPtr = mag.begin(), oriPtr = ori.begin();
    //Set the with an height to a whole bin numbers.
    //If needed replicate the data when summing the bins
    int w = blocksDims.w() * numBins;
    int h = blocksDims.h() * numBins;
    int magW = mag.getWidth();
    int magH = mag.getHeight();
    int histWidth = blocksDims.w();
    int histHeight = blocksDims.h();

    for (int y = 1; y < h - 1; y++)
        for (int x = 1; x < w - 1; x++) {
            // add to 4 histograms around pixel using linear interpolation
            double xp = ((double) x + 0.5) / (double) numBins - 0.5;
            double yp = ((double) y + 0.5) / (double) numBins - 0.5;
            int ixp = (int) floor(xp);
            int iyp = (int) floor(yp);
            double vx0 = xp - ixp;
            double vy0 = yp - iyp;
            double vx1 = 1.0 - vx0;
            double vy1 = 1.0 - vy0;


            //If we are outside out mag/ori data, then use the last values in it
            int magX = min(x, magW - 2);
            int magY = min(y, magH - 2);
            double mag = magPtr[magY * magW + magX];
            int ori = int(oriPtr[magY * magW + magX]);

            Image<float>::iterator histPtr = hist[ori].beginw();

            if (ixp >= 0 && iyp >= 0)
                histPtr[iyp * histWidth + ixp] += vx1 * vy1 * mag;

            if (ixp + 1 < histWidth && iyp >= 0)
                histPtr[iyp * histWidth + ixp + 1] += vx0 * vy1 * mag;

            if (ixp >= 0 && iyp + 1 < histHeight)
                histPtr[(iyp + 1) * histWidth + ixp] += vx1 * vy0 * mag;

            if (ixp + 1 < histWidth && iyp + 1 < histHeight)
                histPtr[(iyp + 1) * histWidth + ixp + 1] += vx0 * vy0 * mag;
        }

    return hist;
}

// ######################################################################
ImageSet<double> FeatureCollection::computeFeatureCollection(const ImageSet<float> &hist) {

    // compute energy in each block by summing over orientations
    Image<double> norm = getHistogramEnergy(hist);

    const int w = norm.getWidth();
    const int h = norm.getHeight();

    const int numFeatureCollection = hist.size() +   //Contrast-sensitive features
                            hist.size() / 2 + //contrast-insensitive features
                            4 +             //texture features
                            1;              //trancation feature (this is zero map???)

    const int featuresW = max(w - 2, 0);
    const int featuresH = max(h - 2, 0);

    ImageSet<double> features(numFeatureCollection, Dims(featuresW, featuresH), ZEROS);

    Image<double>::const_iterator normPtr = norm.begin();
    Image<double>::const_iterator ptr;

    // small value, used to avoid division by zero
    const double eps = 0.0001;

    for (int y = 0; y < featuresH; y++)
        for (int x = 0; x < featuresW; x++) {

            //Combine the norm values of neighboring bins
            ptr = normPtr + (y + 1) * w + x + 1;
            const double n1 = 1.0 / sqrt(*ptr + *(ptr + 1) +
                                         *(ptr + w) + *(ptr + w + 1) +
                                         eps);
            ptr = normPtr + y * w + x + 1;
            const double n2 = 1.0 / sqrt(*ptr + *(ptr + 1) +
                                         *(ptr + w) + *(ptr + w + 1) +
                                         eps);
            ptr = normPtr + (y + 1) * w + x;
            const double n3 = 1.0 / sqrt(*ptr + *(ptr + 1) +
                                         *(ptr + w) + *(ptr + w + 1) +
                                         eps);
            ptr = normPtr + y * w + x;
            const double n4 = 1.0 / sqrt(*ptr + *(ptr + 1) +
                                         *(ptr + w) + *(ptr + w + 1) +
                                         eps);

            //For texture features
            double t1 = 0, t2 = 0, t3 = 0, t4 = 0;

            // contrast-sensitive features
            uint featureId = 0;
            for (uint ori = 0; ori < hist.size(); ori++) {
                Image<float>::const_iterator histPtr = hist[ori].begin();
                const float histVal = histPtr[(y + 1) * w + x + 1];
                double h1 = min(histVal * n1, 0.2);
                double h2 = min(histVal * n2, 0.2);
                double h3 = min(histVal * n3, 0.2);
                double h4 = min(histVal * n4, 0.2);

                t1 += h1;
                t2 += h2;
                t3 += h3;
                t4 += h4;

                Image<double>::iterator featuresPtr = features[featureId++].beginw();
                featuresPtr[y * featuresW + x] = 0.5 * (h1 + h2 + h3 + h4);
            }

            // contrast-insensitive features
            int halfOriSize = hist.size() / 2;
            for (int ori = 0; ori < halfOriSize; ori++) {
                Image<float>::const_iterator histPtr1 = hist[ori].begin();
                Image<float>::const_iterator histPtr2 = hist[ori + halfOriSize].begin();
                const double sum = histPtr1[(y + 1) * w + x + 1] + histPtr2[(y + 1) * w + x + 1];
                double h1 = min(sum * n1, 0.2);
                double h2 = min(sum * n2, 0.2);
                double h3 = min(sum * n3, 0.2);
                double h4 = min(sum * n4, 0.2);

                Image<double>::iterator featuresPtr = features[featureId++].beginw();
                featuresPtr[y * featuresW + x] = 0.5 * (h1 + h2 + h3 + h4);
            }

            // texture features
            Image<double>::iterator featuresPtr = features[featureId++].beginw();
            featuresPtr[y * featuresW + x] = 0.2357 * t1;

            featuresPtr = features[featureId++].beginw();
            featuresPtr[y * featuresW + x] = 0.2357 * t2;

            featuresPtr = features[featureId++].beginw();
            featuresPtr[y * featuresW + x] = 0.2357 * t3;

            featuresPtr = features[featureId++].beginw();
            featuresPtr[y * featuresW + x] = 0.2357 * t4;

            // truncation feature
            // This seems to be just 0, do we need it?
            featuresPtr = features[featureId++].beginw();
            featuresPtr[y * featuresW + x] = 0;

        }


    return features;

}

// ######################################################################
Image< PixRGB<byte> > FeatureCollection::getHistogramImage(const ImageSet<float> &hist, const int lineSize) {
    if (hist.size() == 0)
        return Image< PixRGB<byte> >();

    Image<float> img(hist[0].getDims() * lineSize, ZEROS);
    //Create a one histogram with the maximum features for the 9 orientations
    //TODO: features need to be separated
    for (uint feature = 0; feature < 9; feature++) {
        float ori = (float) feature / M_PI + (M_PI / 2);
        for (int y = 0; y < hist[feature].getHeight(); y++) {
            for (int x = 0; x < hist[feature].getWidth(); x++) {
                float histVal = hist[feature].getVal(x, y);

                //TODO: is this redundant since the first 9 features are
                //contained in the signed 18 features?
                if (hist[feature + 9].getVal(x, y) > histVal)
                    histVal = 100.f * hist[feature + 9].getVal(x, y);
                if (hist[feature + 18].getVal(x, y) > histVal)
                    histVal = 100.f * hist[feature + 18].getVal(x, y);
                if (histVal < 0) histVal = 0; //TODO: do we want this?

                drawLine(img, Point2D<int>((lineSize / 2) + x * lineSize,
                                           (lineSize / 2) + y * lineSize),
                         -ori, lineSize,
                         histVal);
            }
        }
    }

    inplaceNormalize(img, 0.0F, 255.0F);

    return toRGB(img);
}

// ######################################################################
ImageSet<float> FeatureCollection::getFeatureCollection(const Image< PixRGB<byte> > &img, int numBins) {
    int itsNumOrientations = 18;

    Image<float> mag, ori;
    getMaxGradient(img, mag, ori, itsNumOrientations);

    ImageSet<float> histogram = getOriHistogram(mag, ori, itsNumOrientations, numBins);

    ImageSet<double> features = computeFeatureCollection(histogram);

    return features;
}

// ######################################################################
void FeatureCollection::getMaxGradient(const Image< PixRGB<byte> > &img,
                              Image<float> &mag, Image<float> &ori,
                              int numOrientations) {
    if (numOrientations != 0 &&
        numOrientations > 18)
        LFATAL("Can only support up to 18 orientations for now.");

    mag.resize(img.getDims());
    ori.resize(img.getDims());

    Image< PixRGB <byte> > ::const_iterator
    src = img.begin();
    Image<float>::iterator mPtr = mag.beginw(), oPtr = ori.beginw();
    const int w = mag.getWidth(), h = mag.getHeight();

    float zero = 0;

    // first row is all zeros:
    for (int i = 0; i < w; i++) {
        *mPtr++ = zero;
        *oPtr++ = zero;
    }
    src += w;

    // loop over inner rows:
    for (int j = 1; j < h - 1; j++) {
        // leftmost pixel is zero:
        *mPtr++ = zero;
        *oPtr++ = zero;
        ++src;

        // loop over inner columns:
        for (int i = 1; i < w - 1; i++) {
            PixRGB<int> valx = src[1] - src[-1];
            PixRGB<int> valy = src[w] - src[-w];

            //Mag
            double mag1 = (valx.red() * valx.red()) + (valy.red() * valy.red());
            double mag2 = (valx.green() * valx.green()) + (valy.green() * valy.green());
            double mag3 = (valx.blue() * valx.blue()) + (valy.blue() * valy.blue());

            double mag = mag1;
            double dx = valx.red();
            double dy = valy.red();

            //Get the channel with the strongest gradient
            if (mag2 > mag) {
                dx = valx.green();
                dy = valy.green();
                mag = mag2;
            }
            if (mag3 > mag) {
                dx = valx.blue();
                dy = valy.blue();
                mag = mag3;
            }

            *mPtr++ = sqrt(mag);
            if (numOrientations > 0) {
                //Snap to num orientations
                double bestDot = 0;
                int bestOri = 0;
                for (int ori = 0; ori < numOrientations / 2; ori++) {
                    double dot = itsUU[ori] * dx + itsVV[ori] * dy;
                    if (dot > bestDot) {
                        bestDot = dot;
                        bestOri = ori;
                    } else if (-dot > bestDot) {
                        bestDot = -dot;
                        bestOri = ori + (numOrientations / 2);
                    }
                }
                *oPtr++ = bestOri;

            } else {
                *oPtr++ = atan2(dy, dx);
            }
            ++src;
        }

        // rightmost pixel is zero:
        *mPtr++ = zero;
        *oPtr++ = zero;
        ++src;
    }

    // last row is all zeros:
    for (int i = 0; i < w; i++) {
        *mPtr++ = zero;
        *oPtr++ = zero;
    }
}

// ######################################################################
Image<double> FeatureCollection::getHistogramEnergy(const ImageSet<float> &hist) {
    if (hist.size() == 0)
        return Image<double>();

    Image<double> norm(hist[0].getDims(), ZEROS);

    //TODO: check for overflow
    int halfOriSize = hist.size() / 2;
    // compute energy in each block by summing over orientations
    for (int ori = 0; ori < halfOriSize; ori++) {
        Image<float>::const_iterator src1Ptr = hist[ori].begin();
        Image<float>::const_iterator src2Ptr = hist[ori + halfOriSize].begin();

        Image<double>::iterator normPtr = norm.beginw();
        Image<double>::const_iterator normPtrEnd = norm.end();

        while (normPtr < normPtrEnd) {
            *(normPtr++) += (*src1Ptr + *src2Ptr) * (*src1Ptr + *src2Ptr);
            src1Ptr++;
            src2Ptr++;
        }
    }

    return norm;
}

// Computes Schmid's invariants at different scales.
// ######################################################################
vector<double>  FeatureCollection::computeInvariants(const Image<float> &input, int scale) {

    vector<double> features(9 * scale * 3, 0.0F);

    // kernel local jet
    Image<float> gaus(1, 3, ZEROS);
    gaus.setVal(0, 0, .25F);
    gaus.setVal(0, 1, .5F);
    gaus.setVal(0, 2, .25F);
    Image<float> gaus_p(3, 1, ZEROS);
    gaus_p.setVal(0, 0, .25F);
    gaus_p.setVal(1, 0, .5F);
    gaus_p.setVal(2, 0, .25F);
    Image<float> dd(1, 3, ZEROS);
    dd.setVal(0, 0, -1.F);
    dd.setVal(0, 2, 1.F);
    Image<float> dx = dd;
    Image<float> dy(3, 1, ZEROS);
    dy.setVal(0, 0, -1.F);
    dy.setVal(2, 0, 1.F);
    Image<float> ima_g = input;

    //CONV_BOUNDARY_ZERO,      ///< zero-pad, a.k.a. truncated
    //CONV_BOUNDARY_CLEAN,     ///< normalize by the sum of the used filter coefficients
    //CONV_BOUNDARY_REPLICATE  ///< replicate the nearest image pixel value

    int numFeatureCollection = 9 * scale;
    int k = 0;
    for (int s = 0; s < scale; s++) {
        int featureId = 0;
        ImageSet<double> data(9, Dims(100, 100), ZEROS);
        Image<float> ima_x = convolve(ima_g, dx, CONV_BOUNDARY_ZERO);
        Image<float> ima_y = convolve(ima_g, dy, CONV_BOUNDARY_ZERO);
        Image<float> ima_xy = convolve(ima_x, dy, CONV_BOUNDARY_ZERO);
        Image<float> ima_xx = convolve(ima_x, dx, CONV_BOUNDARY_ZERO);
        Image<float> ima_yy = convolve(ima_y, dy, CONV_BOUNDARY_ZERO);
        Image<float> ima_xyy = convolve(ima_yy, dx, CONV_BOUNDARY_ZERO);
        Image<float> ima_xxy = convolve(ima_xx, dy, CONV_BOUNDARY_ZERO);
        Image<float> ima_xxx = convolve(ima_xx, dx, CONV_BOUNDARY_ZERO);
        Image<float> ima_yyy = convolve(ima_yy, dy, CONV_BOUNDARY_ZERO);

        Image<float> L = ima_g;
        data[0] = getInner(L, 8);

        Image<float> LiLi = toPower(ima_x, 2) + toPower(ima_y, 2);
        data[1] = getInner(LiLi, 8);

        Image<float> a = ima_x;
        a *= ima_xx;
        a *= ima_x;
        Image<float> b = ima_x;
        b *= ima_xy;
        b *= ima_y;
        b *= 2;
        Image<float> c = ima_y;
        c *= ima_yy;
        c *= ima_y;
        Image<float> LiLijLj = a + b + c;
        data[2] = getInner(LiLijLj, 8);

        data[3] = getInner(ima_xx + ima_yy, 8);

        a = toPower(ima_xx, 2);
        b = toPower(ima_xy, 2);
        b *= 2;
        c = toPower(ima_yy, 2);
        data[4] = getInner(a + b + c, 8);

        a *= ima_xxx;
        a *= toPower(ima_y, 3);
        b = ima_yyy;
        b *= toPower(ima_x, 3);
        c = ima_xyy;
        c *= toPower(ima_x, 2);
        c *= ima_y;
        c *= 4;
        Image<float> d = ima_xxy;
        d *= ima_x;
        d *= toPower(ima_y, 2);
        d *= 4;
        data[5] = getInner(a - b + c - d, 8);

        a = ima_xxy;
        a *= toPower(ima_y, 3);
        b = ima_xxy;
        b *= ima_x;
        b *= 2;
        b *= ima_y;
        c = ima_xyy;
        c *= ima_x;
        c *= toPower(ima_y, 2);
        d = ima_xyy;
        d *= toPower(ima_x, 3);
        data[6] = getInner(a + b - c - d, 8);

        a = ima_xxy;
        a *= toPower(ima_x, 3);
        a *= -1;
        b = ima_xyy;
        b *= toPower(ima_x, 2);
        b *= ima_y;
        b *= 2;
        c = ima_yyy;
        c *= ima_x;
        c *= toPower(ima_y, 2);
        d = ima_xxx;
        d *= ima_y;
        d *= toPower(ima_x, 2);
        Image<float> e = ima_xxy;
        e *= toPower(ima_y, 2);
        e *= ima_x;
        e *= 2;
        Image<float> f = ima_xyy;
        f *= toPower(ima_y, 3);
        data[7] = getInner(a - b - c + d + e + f, 8);

        a = ima_xxx;
        a *= toPower(ima_x, 3);
        b = ima_xxy;
        b *= toPower(ima_x, 2);
        b *= ima_y;
        b *= 3;
        c = ima_xyy;
        c *= ima_x;
        c *= toPower(ima_y, 2);
        c *= 3;
        d = ima_yyy;
        d *= toPower(ima_y, 3);
        data[8] = getInner(a + b + c + d, 8);

        if (s < scale)
            ima_g = convolve(convolve(ima_g, gaus, CONV_BOUNDARY_ZERO), gaus_p, CONV_BOUNDARY_ZERO);

        // normalize
        Image<float> din(Dims(100, 100), ZEROS);
        for (int j = 0; j < 9; j++) {
            din = data[j];
            double mean = computeBorderMean(din);
            din -= mean;
            double sumin = 0;
            sumin = sum(din);
            if (sumin > 0.) {
                features[k++] = FeatureCollection::negMean(din);
                features[k++] = FeatureCollection::posMean(din);
                features[k++] = FeatureCollection::absMean(din) + mean;
            }
            else {
                features[k++] = 0.;
                features[k++] = 0.;
                features[k++] = 0.;
            }
        }
    }

    vector<double> featuresDouble(features.begin(), features.end());
    return featuresDouble;
}

// ######################################################################
float FeatureCollection::negMean(const Image<float> &in) {
    Image<float> result(in.getDims(), NO_INIT);

    typename Image<float>::const_iterator inptr = in.begin();
    typename Image<float>::iterator dptr = result.beginw();
    typename Image<float>::iterator stop = result.endw();

    while (dptr != stop) {
        float t1 = *inptr++;
        if (t1 < 0.0F)
            *dptr++ = t1;
        else
            *dptr++ = 0.0F;
    }

    return mean(result);
}

// ######################################################################
float FeatureCollection::posMean(const Image<float> &in) {
    Image<float> result(in.getDims(), NO_INIT);

    typename Image<float>::const_iterator inptr = in.begin();
    typename Image<float>::iterator dptr = result.beginw();
    typename Image<float>::iterator stop = result.endw();

    while (dptr != stop) {
        float t1 = *inptr++;
        if (t1 > 0)
            *dptr++ = t1;
        else
            *dptr++ = 0.0F;
    }

    return mean(result);
}

// ######################################################################
float FeatureCollection::absMean(const Image<float> &in) {
    Image<float> result(in.getDims(), NO_INIT);

    typename Image<float>::const_iterator inptr = in.begin();
    typename Image<float>::iterator dptr = result.beginw();
    typename Image<float>::iterator stop = result.endw();

    while (dptr != stop) {
        float t1 = *inptr++;
        *dptr++ = abs(t1);
    }

    return mean(mean(result));
}

// ######################################################################
float FeatureCollection::computeBorderMean(const Image<float> &img) {

    Dims dims = img.getDims();
    float size = min(10.F, (float)round(dims.w() / 2));
    Image<float> resultfinal(img.getDims(), ZEROS);
    resultfinal = img;

    // mask the inner values from the border to zero
    for (int i = size; i < (dims.w() - size); i++)
        for (int j = size; j < (dims.h() - size); j++)
            resultfinal.setVal(i, j, 0.0F);

    // get the mean of the remaining border
    float mean = 0;
    float sumfinal = 0;
    sumfinal = sum(resultfinal);
    if (sumfinal > 0.)
        mean = sum(resultfinal) / (size * size);

    return mean;
}

// ######################################################################
Image<float> FeatureCollection::getInner(const Image<float> &img, int border) {
    Dims dims = img.getDims();
    Image<float> resultfinal(img.getDims() - border, ZEROS);

    for (int i = border; i < (dims.w() - border); i++)
        for (int j = border; j < (dims.h() - border); j++) {
            float val = img.getVal(i, j);
            resultfinal.setVal(i - border, j - border, val);
        }

    return resultfinal;
}

// ######################################################################
void FeatureCollection::getComponents(const Image< PixRGB<byte> > &src,
                             Image<float> &red, Image<float> &green, Image<float> &blue) {
    ASSERT(src.initialized());

    red = Image<float>(src.getDims(), ZEROS);
    green = Image<float>(src.getDims(), ZEROS);
    blue = Image<float>(src.getDims(), ZEROS);

    typename Image<float>::iterator rptr = red.beginw();
    typename Image<float>::iterator gptr = green.beginw();
    typename Image<float>::iterator bptr = blue.beginw();

    typename Image< PixRGB<byte> > ::const_iterator
    aptr = src.begin();
    typename Image< PixRGB<byte> > ::const_iterator
    stop = src.end();

    while (aptr != stop) {
        *rptr++ = float(aptr->red());
        *gptr++ = float(aptr->green());
        *bptr++ = float(aptr->blue());
        ++aptr;
    }
}
