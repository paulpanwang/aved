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
 * This is a program that combines top-down and bottom-up methods to
 * detect, track and classify events in underwater video. This is
 * based on modified version from Dirk Walther's work that originated
 * at the 2002 Workshop  Neuromorphic Engineering in Telluride, CO, USA.
 * and work done by CSUMB students Mayra Ochoa & Raymond Esteybar to
 * parse xml and lay the ground work for combining the
 *
 * It is designed to take PASCAL XML formatted output from a CNN object detector
 * and combine that with multiple tracking systems, e.g.
 * Kalman, Nearest Neighbor, and/or a Hough-based transform that tracks
 * deformable shapes to track a VisualEvent.
 *
 * This code requires:
*
*  The iLab Neuromorphic Vision C++ Toolkit developed
 * by the University of Southern California (USC) and the iLab at USC.
 * See http://iLab.usc.edu for information about this project.
 *
 * This work would not be possible without the generous support of the
 * David and Lucile Packard Foundation
 */
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#include <list>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <boost/algorithm/string.hpp>

#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/parsers/AbstractDOMParser.hpp>
#include <xercesc/dom/DOM.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>

#include "Media/FrameState.H"
#include "Image/OpenCVUtil.H"
#include "Channels/ChannelOpts.H"
#include "Component/GlobalOpts.H"
#include "Component/ModelManager.H"
#include "Component/JobServerConfigurator.H"
#include "Features/HistogramOfGradients.H"
#include "Image/FilterOps.H"    // for lowPass3y()
#include "Image/Kernels.H"      // for twofiftyfives()
#include "Image/ColorOps.H"
#include "Image/fancynorm.H"
#include "Image/MorphOps.H"
#include "Image/ShapeOps.H"   	// for rescale()
#include "Raster/GenericFrame.H"
#include "Raster/PngWriter.H"
#include "Media/FrameRange.H"
#include "Media/FrameSeries.H"
#include "Media/SimFrameSeries.H"
#include "Media/MediaOpts.H"
#include "Neuro/SpatialMetrics.H"
#include "Neuro/StdBrain.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/Retina.H"
#include "Neuro/VisualCortex.H"
#include "Neuro/SimulationViewerStd.H"
#include "SIFT/Histogram.H"
#include "Simulation/SimEventQueueConfigurator.H"
#include "Util/sformat.H"
#include "Util/StringConversions.H"
#include "Util/Pause.H"
#include "Data/Logger.H"
#include "Data/MbariMetaData.H"
#include "Data/MbariOpts.H"

#include "DetectionAndTracking/FOEestimator.H"
#include "DetectionAndTracking/VisualEventSet.H"
#include "DetectionAndTracking/DetectionParameters.H"
#include "DetectionAndTracking/MbariFunctions.H"
#include "DetectionAndTracking/Segmentation.H"
#include "DetectionAndTracking/ColorSpaceTypes.H"
#include "DetectionAndTracking/ObjectDetection.H"
#include "DetectionAndTracking/Preprocess.H"
#include "Image/MbariImage.H"
#include "Image/MbariImageCache.H"
#include "Image/BitObject.H"
#include "Image/DrawOps.H"
#include "Image/MathOps.H"
#include "Image/IO.H"
#include "Learn/BayesClassifier.H"
#include "Media/MbariResultViewer.H"
#include "Motion/MotionEnergy.H"
#include "Motion/MotionOps.H"
#include "Motion/OpticalFlow.H"
#include "Util/StringConversions.H"
#include "Utils/Version.H"
#include "Component/ModelManager.H"
#include "Data/VOCUtils.H"
#include "DetectionAndTracking/DetectionParameters.H"
#include "Simulation/SimEventQueueConfigurator.H"
#include "DetectionAndTracking/BoxObjectDetection.H"

using namespace std;
using namespace xercesc;

//#define DEBUG
#define MAX_INT32 2147483647

struct BoundingBox {
    int xmin;
    int ymin;
    int xmax;
    int ymax;
};

int main(const int argc, const char** argv) {

	DetectionParameters dp = DetectionParametersSingleton::instance()->itsParameters;
	Segmentation segmentation;

	#ifdef DEBUG
    PauseWaiter pause;
    setPause(true);
    #endif
    const int foaSizeRatio = 19;

    try {
    	XMLPlatformUtils::Initialize();
    } catch(...) {
    	LINFO("CAUGHT EXCEPTION");
    	return -1;
    }

	// Variables Utilized
	XercesDOMParser *itsParser = new XercesDOMParser;	// Ensures File is readable

	bool singleFrame = false;
	uint frameNum = 0;
	Image< PixRGB<byte> > inputRaw, inputScaled;

	ModelManager manager("pascalvocvision");

	nub::soft_ref<SimEventQueueConfigurator>seqc(new SimEventQueueConfigurator(manager));
    manager.addSubComponent(seqc);
	nub::soft_ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
	nub::soft_ref<InputFrameSeries> ifs(new InputFrameSeries(manager));

	nub::soft_ref<BoxObjectDetection> objdet(new BoxObjectDetection(manager));
    manager.addSubComponent(objdet);

    nub::soft_ref<Preprocess> preprocess(new Preprocess(manager));
    manager.addSubComponent(preprocess);

	manager.addSubComponent(ofs);
	manager.addSubComponent(ifs);

	// Get the directory of this executable
    std::string exe(argv[0]);
    size_t found = exe.find_last_of("/\\");
    nub::soft_ref<Logger> logger(new Logger(manager, ifs, ofs, exe.substr(0, found)));
    manager.addSubComponent(logger);

    nub::soft_ref<MbariResultViewer> rv(new MbariResultViewer(manager));
    manager.addSubComponent(rv);

    nub::ref<DetectionParametersModelComponent> parms(new DetectionParametersModelComponent(manager));
    manager.addSubComponent(parms);

    // Request MBARI specific option aliases
    REQUEST_OPTIONALIAS_MBARI(manager);

    if (manager.parseCommandLine(argc, argv, "", 0, -1) == false)
        LFATAL("Invalid command line argument. Aborting program now !");

    // fix empty frame range bug and set the range to be the same as the input frame range
    FrameRange fr = ifs->getModelParamVal< FrameRange > ("InputFrameRange");

    if (fr.getLast() == MAX_INT32) {
        FrameRange range(0, 0, 0);
        ifs->setModelParamVal(string("InputFrameRange"), range);
        ofs->setModelParamVal(string("OutputFrameRange"), range);
        singleFrame = true;
    }
    else
        ofs->setModelParamVal(string("OutputFrameRange"), fr);

    // get image dimensions and set a few parameters that depend on it
    parms->reset(&dp);

    // is this a a gray scale sequence ? if so disable computing the color channels
    // to save computation time. This assumes the color channel has no weight !
    if (dp.itsColorSpaceType == SAColorGray) {
    	std::string search = "C";
    	std::string source = manager.getOptionValString(&OPT_VisualCortexType);
        size_t pos = source.find(search);
        if (pos != string::npos) {
        	std::string replace = source.erase(pos, 1);
            manager.setOptionValString(&OPT_VisualCortexType, replace);
        }
    }

	Dims scaledDims = ifs->peekDims();

	// if the user has selected to retain the original dimensions in the events disable scaling in the frame series
    // and use the scaling factors directly
    if (dp.itsSaveOriginalFrameSpec) {
        ofs->setModelParamVal(string("OutputFrameDims"), Dims(0, 0));
        ifs->setModelParamVal(string("InputFrameDims"), Dims(0, 0));
    }

    int foaRadius;
    const std::string foar = manager.getOptionValString(&OPT_FOAradius);
    convertFromString(foar, foaRadius);

    // calculate the foa size based on the image size if set to defaults
    // A zero foa radius indicates to set defaults from input image dims
    if (foaRadius == 0) {
        foaRadius = scaledDims.w() / foaSizeRatio;
        char str[256];
        sprintf(str, "%d", foaRadius);
        manager.setOptionValString(&OPT_FOAradius, str);
    }

    // get reference to the SimEventQueue
    nub::soft_ref<SimEventQueue> seq = seqc->getQ();
	manager.start();
	// set defaults for detection model parameters
    DetectionParametersSingleton::initialize(dp, scaledDims, foaRadius);

    // initialize the visual event set
    bool saveFeatures = false;
    const string f = manager.getOptionValString(&OPT_LOGsaveEventFeatures);
    convertFromString(f, saveFeatures);
    VisualEventSet eventSet(dp, manager.getExtraArg(0), saveFeatures);

    // initialize masks
    Image<byte> mask(scaledDims, ZEROS);
    mask = highThresh(mask, byte(0), byte(255));

    Image<byte> staticClipMask(scaledDims, ZEROS);
    mask = highThresh(mask, byte(0), byte(255));
    staticClipMask = maskArea(mask, &dp);

    // initialize the preprocess
    preprocess->init(ifs, scaledDims);
    ifs->reset1(); //reset to state after construction since the preprocessing caches input frames

    // main loop:
    LINFO("MAIN_LOOP");

    MbariImage< PixRGB<byte> > input(manager.getOptionValString(&OPT_InputFrameSource).c_str());
    MbariImage< PixRGB<byte> > prevInput(manager.getOptionValString(&OPT_InputFrameSource).c_str());
    MbariImage< PixRGB <byte> > output(manager.getOptionValString(&OPT_InputFrameSource).c_str());
    Image<byte>  foaIn;
    ImageData imgData;
    Image< PixRGB<byte> > segmentIn(input.getDims(), ZEROS);
    Image< PixRGB<byte> > clampedInput(input.getDims(), ZEROS);

    // initialize property list and FOE estimator
    FOEestimator foeEst(20, 0);
    Vector2D curFOE;

    // Initialize bayesian network
    BayesClassifier bayesClassifier(dp.itsBayesPath, dp.itsFeatureType, scaledDims);
    FeatureCollection features(scaledDims);

	//Get rescale values.
	string rescaleValues(manager.getOptionValString(&OPT_InputFrameDims).c_str());
	int i = rescaleValues.find("x");
	int width = getRescaleValues(rescaleValues.substr(0,i));
	int height = getRescaleValues(rescaleValues.substr(i+1,rescaleValues.size()-1));
	LDEBUG("W: %i H: %i ", width, height);
	Dims dims(width, height);

	while(1) {

		// Read new image in
		FrameState is = FRAME_NEXT;

		if (!singleFrame)
			is = ifs->updateNext();
		else
			is = FRAME_FINAL;

		if (is == FRAME_COMPLETE )	break; // done
		if (is == FRAME_NEXT || is == FRAME_FINAL) { // new frame

			LINFO("Reading new frame");

			// initialize the default mask
			mask = staticClipMask;

			// Get Frame
			inputRaw = ifs->readRGB();
			inputScaled = rescale(inputRaw, scaledDims);

			frameNum = ifs->frame();

			// get updated input image erasing previous bit objects
			const list<BitObject> bitObjectFrameList = eventSet.getBitObjectsForFrame(frameNum - 1);

			// update the background cache
			input = preprocess->update(inputScaled, prevInput, frameNum, bitObjectFrameList);

			rv->display(input, frameNum, "Input");

			// choose segmentation method
			if (dp.itsSegmentAlgorithmInputType == SAILuminance) {
				segmentIn = input;
			} else if (dp.itsSegmentAlgorithmInputType == SAIDiffMean) {
				segmentIn = preprocess->clampedDiffMean(input);
			} else if (dp.itsSegmentAlgorithmInputType == SAIContrastEnhance) {
				segmentIn = preprocess->contrastEnhance(input);
			} else {
				segmentIn = preprocess->contrastEnhance(input);
			}

			Dims d = input.getDims();
			if (prevInput.initialized())
				 clampedInput = preprocess->clampedDiffMean(prevInput);

			imgData.foe = curFOE;
			imgData.frameNum = frameNum;
			imgData.clampedImg = clampedInput;
			imgData.img = input;
			imgData.metadata = input.getMetaData();
			imgData.prevImg = prevInput;
			imgData.segmentImg = segmentIn;
			imgData.mask = mask;

			std::list<VOCObject> vocs;
			std::list<BitObject> objs;

			// Read File - get list<Rectangle>
			string description(manager.getOptionValString(&OPT_InputFrameSource).c_str());
			description = "/" + getmyXML(description, frameNum);

			itsParser->resetDocumentPool();
			itsParser->parse(description.c_str()); // Ensures the file is readable

			// Extract Values
			if (itsParser->getErrorCount() == 0) {
				// Checks if it's a rescaled image, & if so, change each object <bndbox> dimension
				getImageSize(itsParser, dims);
				getObjectValues(itsParser, vocs, dims);
			} else
			  LFATAL("Error when attempting to parse the XML file : %s", description.c_str());

			LINFO("START > objdet->run() <");
			objs = objdet->run(rv, vocs, segmentIn);

			// create new events with this
			LINFO("Initiate events frame num: %i ", frameNum);
			eventSet.initiateEvents(objs, features, imgData);

			vocs.clear();
			objs.clear();
		}

		LINFO("Updating events frame num: %i ", frameNum);
		eventSet.updateEvents(rv, bayesClassifier, features, imgData);

		FrameState os = FRAME_NEXT;
		if (!singleFrame)
			os = ofs->updateNext();
		else
			os = FRAME_FINAL;

		if (os == FRAME_NEXT || os == FRAME_FINAL) {

			// save features for each event
			logger->saveFeatures(frameNum, eventSet);

			// classify
			/*bayesClassifier.runEvents(frameNum, eventSet, featureSet);
			float w = (float)d.w()/(float)scaledDims.w();
			float h = (float)d.h()/(float)scaledDims.h();
			Image< PixRGB<byte> > o = rv->createOutput(input,
													   eventSet,
													   40,
													   w, h);

			// display output ?
			rv->display(o, frameNum, "ResultsClassified");*/

			// create MBARI image with metadata from input and original input frame
			if (dp.itsContrastEnhance)
				output.updateData(preprocess->contrastEnhance(input), input.getMetaData(), ofs->frame());
			else
				output.updateData(input, input.getMetaData(), ofs->frame());

			// write out/display anything that's ready
			logger->run(rv, output, eventSet, scaledDims);

			// prune invalid events
			eventSet.cleanUp(ofs->frame());

			// save the input image
			prevInput = input;
		}
		#ifdef DEBUG
		if ( pause.checkPause()) Raster::waitForKey();// || ifs->shouldWait() || ofs->shouldWait()) Raster::waitForKey();
		#endif

		if (os == FRAME_FINAL) {
			 // last frame? -> close everyone
			eventSet.closeAll();
			eventSet.cleanUp(ofs->frame());
			break;
		}
	}
	manager.stop();

	LDEBUG("Reached END");

	delete itsParser;

	return EXIT_SUCCESS;
}
