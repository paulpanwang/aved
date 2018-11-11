/*
 * Copyright 2018 MBARI
 *
 * Licensed under the GNU LESSER GENERAL PUBLIC LICENSE, Version 3.0
 * (the "License"); you may not use this file except in compliance 
 * with the License. You may obtain a copy of the License at
 *
 * http://www.gnu.org/copyleitsFeatures/lesser.html
 *
 * Unless required by applicable law or agreed to in writing, so its Features are
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

/*!@file BayesClassifier.C a class for using Bayes transform tracking algorithm */

#include "Image/OpenCVUtil.H"
#include <csignal>
#include <vector>

#include "Learn/BayesClassifier.H"
#include "DetectionAndTracking/MbariFunctions.H"
#include "Image/BitObject.H"
#include "Image/ImageSet.H"
#include "Image/ColorOps.H"
#include "Learn/FeatureTypes.H"

template <class T> class MbariImage;

using namespace std;

// ######################################################################
BayesClassifier::BayesClassifier(string bayesPath, FeatureType featureType, Dims scaledDims)
	:bn(324,0),
	 itsFeatureType(featureType)
{
	if (bayesPath.length() > 0) {
		LINFO("Loading %s", bayesPath.c_str());
		bn.load(bayesPath.c_str());

		LINFO("Classifier num classes: %d", bn.getNumClasses());

		// dump information about the bayes classifier
		for(uint i = 0; i < bn.getNumFeatures(); i++)
			LINFO("Feature %i: mean %f, stddevSq %f", i, bn.getMean(0, i), bn.getStdevSq(0, i));

		for(uint i = 0; i < bn.getNumClasses(); i++)
			LINFO("Trained class %s", bn.getClassName(i));
	}
}

// ######################################################################
BayesClassifier::~BayesClassifier() {
}

// ######################################################################
string BayesClassifier::getClassName(int index) {
	if (index < bn.getNumClasses())
		return bn.getClassName(index);
	return "Unknown";
}

// ######################################################################
void BayesClassifier::runEvents(int frameNum, VisualEventSet& eventSet, list<FeatureCollection::Data> &dataList)
{
	// for each event, run classifier on precomputed features
	list<FeatureCollection::Data> featureData;
	list<VisualEvent *>::iterator event;
	list<VisualEvent *> eventFrameList;
	eventFrameList = eventSet.getEventsForFrame(frameNum);

	list<FeatureCollection::Data>::iterator data = dataList.begin();
	for (event = eventFrameList.begin(); event != eventFrameList.end(); ++event) {
		run(frameNum, *event, *data);
		data++;
	}
}

// ######################################################################
void BayesClassifier::classify(int *cls, double *prob, FeatureCollection::Data &data) {

	// classify
	switch(itsFeatureType) {
		case FT_HOG3:
			*cls = bn.classify(data.featureHOG3, prob);
			break;
		case FT_HOG8:
			*cls = bn.classify(data.featureHOG8, prob);
			break;
		/*case FT_MBH3:
			*cls = bn.classify(data.featureMBH3, &prob);
			break;
		case FT_MBH8:
			*cls = bn.classify(data.featureMBH8, &prob);
			break;*/
		case FT_JET:
			// TODO: combine red and green and vote?
			*cls = bn.classify(data.featureJETred, prob);
			break;
		default: LFATAL("Unknown feature type %d - don't know what to do.", itsFeatureType);
	}
}

// ######################################################################
void BayesClassifier::run(int frameNum, VisualEvent *event, FeatureCollection::Data data)
{
	// classify
	double prob = 0.0F;
	int cls = 0;
	classify(&cls, &prob, data);

	float prob_class = float(prob);
	if (cls < bn.getNumClasses() && prob_class >= 0.0F && prob <= 1.0) {
		string class_name = bn.getClassName(cls);
		LINFO("========>Classified event %d as class %s prob %.2f frame %d<========", event->getEventNum(), class_name.c_str(), prob_class, frameNum);
		Token tk = event->getToken(frameNum);
		tk.bitObject.setClassProbability(class_name, prob_class);
	}
}


/*	// calculate the bounding box for sliding window based on the prediction
	const Point2D<int> pred = (*event)->predictedLocation();

	Dims dims = input.getDims();

	// is the prediction too far outside the image?
	int gone = dp.itsMaxDist;
	if ((pred.i < -gone) || (pred.i >= (dims.w() + gone)) ||
		(pred.j < -gone) || (pred.j >= (dims.h() + gone)))
	{
		break;
	}

	// adjust prediction if negative
	const Point2D<int> center =  Point2D<int>(max(pred.i,0), max(pred.j,0));

	// get the region used for searching for a match based on the maximum dimension
	Dims maxDims = (*event)->getMaxObjectDims();
	Dims searchDims = Dims((float)maxDims.w()*4.0,(float)maxDims.h()*4.0);
	Rectangle region = Rectangle::centerDims(center, searchDims);
	region = region.getOverlap(Rectangle(Point2D<int>(0, 0), inputRaw.getDims() - 1));
	Dims windowDims = (*event)->getToken(frameNum - 1).bitObject.getObjectDims();

	// run overlapping sliding box over a region centered at the predicted location
	Dims stepDims(windowDims.w()/4.F, windowDims.h()/4.F);
 */
