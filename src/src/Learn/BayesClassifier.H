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

/*!@file BayesClassifier.H a class for Bayes-based tracking of Non-Rigid Objects
  in video. Based on work done by
 */

#ifndef BAYESCLASSIFIER_H_DEFINED
#define BAYESCLASSIFIER_H_DEFINED

#include "Learn/Bayes.H"
#include "Learn/FeatureTypes.H"
#include "Learn/Features.H"
#include "Image/Dims.H"
#include "Image/Rectangle.H"
#include "Image/Geometry2D.H"
#include "Image/MbariImage.H"
#include "nub/ref.h"
#include "DetectionAndTracking/VisualEventSet.H"

template <class T> class Image;
template <class T> class PixRGB;
template <class T> class MbariImage;


// ######################################################################
//! runs the BayesClassifier algorithm
class BayesClassifier {
public:

    //! constructor
    BayesClassifier(std::string bayesPath, FeatureType featureType, Dims scaledDims);

    //! destructor
    ~BayesClassifier();

    //! free up memory associated with this Classifier
    void free();

    //! run classifier on all events and assign */
    void runEvents(int frameNum, VisualEventSet& eventSet, std::list<FeatureCollection::Data> &dataList);

    //! run classifier on event and assign */
    void run(int frameNum, VisualEvent *event, FeatureCollection::Data data);

    //! run classifier feature data and assign */
    void classify(int *cls, double *prob, struct FeatureCollection::Data &data);

    std::string getClassName(int index);

private:


    // bayesian network to be loaded on startup
    Bayes bn;

    // feature type, HOG, MMAP, etc.
    FeatureType itsFeatureType;

};
#endif
