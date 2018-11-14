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

#include <iostream>
#include <sstream>
#include <signal.h>
#include <fstream>

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
#include "Image/ShapeOps.H"   // for rescale()
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

#define DEBUG

#define MAX_INT32 2147483647

using namespace std;

int main(const int argc, const char** argv) {

    // ######## Initialization of variables, reading of parameters etc.
    DetectionParameters dp = DetectionParametersSingleton::instance()->itsParameters;
    Segmentation segmentation;
    #ifdef DEBUG
    PauseWaiter pause;
    setPause(true);
    #endif
    const int foaSizeRatio = 19;

    //initialize a few things
    ModelManager manager("salientvision");

    // turn down log messages until after initialization
    MYLOGVERB = LOG_INFO;

    nub::soft_ref<SimEventQueueConfigurator>
            seqc(new SimEventQueueConfigurator(manager));
    manager.addSubComponent(seqc);

    nub::soft_ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
    manager.addSubComponent(ofs);

    nub::soft_ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
    manager.addSubComponent(ifs);

    nub::soft_ref<ObjectDetection> objdet(new ObjectDetection(manager));
    manager.addSubComponent(objdet);

    nub::soft_ref<Preprocess> preprocess(new Preprocess(manager));
    manager.addSubComponent(preprocess);

    // Get the directory of this executable
    string exe(argv[0]);
    size_t found = exe.find_last_of("/\\");
    nub::soft_ref<Logger> logger(new Logger(manager, ifs, ofs, exe.substr(0, found)));
    manager.addSubComponent(logger);

    nub::soft_ref<MbariResultViewer> rv(new MbariResultViewer(manager));
    manager.addSubComponent(rv);

    nub::ref<DetectionParametersModelComponent> parms(new DetectionParametersModelComponent(manager));
    manager.addSubComponent(parms);

    nub::ref<StdBrain> brain(new StdBrain(manager));
    manager.addSubComponent(brain);

    // Request MBARI specific option aliases
    REQUEST_OPTIONALIAS_MBARI(manager);

    // Request a bunch of toolkit option aliases
    REQUEST_OPTIONALIAS_NEURO(manager);

    // Initialize with some reasonable defaults
    manager.setOptionValString(&OPT_UseRandom, "true");
    manager.setOptionValString(&OPT_SVdisplayBoring, "false");
    manager.setOptionValString(&OPT_ShapeEstimatorMode, "ConspicuityMap");
    manager.setOptionValString(&OPT_ShapeEstimatorSmoothMethod, "None");
    manager.setOptionValString(&OPT_IORtype, "ShapeEst");
    manager.setOptionValString(&OPT_UseOlderVersion, "false");
    manager.setOptionValString(&OPT_MaxNormType, "FancyOne");
    manager.setOptionValString(&OPT_BrainBoringSMmv, "2.0");
    manager.setOptionValString(&OPT_OriInteraction, "SubtractMean");
    manager.setOptionValString(&OPT_OrientComputeType, "Steerable");
    manager.setOptionValString(&OPT_IORtype, "ShapeEst");

    // parse the command line
    if (manager.parseCommandLine(argc, argv, "", 0, -1) == false)
        LFATAL("Invalid command line argument. Aborting program now !");

    // fix empty frame range bug and set the range to be the same as the input frame range
    FrameRange fr = ifs->getModelParamVal< FrameRange > ("InputFrameRange");
    bool singleFrame = false;
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
        string search = "C";
        string source = manager.getOptionValString(&OPT_VisualCortexType);
        size_t pos = source.find(search);
        if (pos != string::npos) {
            string replace = source.erase(pos, 1);
            manager.setOptionValString(&OPT_VisualCortexType, replace);
        }
    }

    // get the dimensions of the potentially scaled input frames
    Dims scaledDims = ifs->peekDims();

    // if the user has selected to retain the original dimensions in the events disable scaling in the frame series
    // and use the scaling factors directly
    if (dp.itsSaveOriginalFrameSpec) {
        ofs->setModelParamVal(string("OutputFrameDims"), Dims(0, 0));
        ifs->setModelParamVal(string("InputFrameDims"), Dims(0, 0));
    }

    int foaRadius;
    const string foar = manager.getOptionValString(&OPT_FOAradius);
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

    // start all the ModelComponents
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

    int numSpots = 0;
    uint frameNum = 0;
    MbariImage< PixRGB<byte> > input(manager.getOptionValString(&OPT_InputFrameSource).c_str());
    MbariImage< PixRGB<byte> > prevInput(manager.getOptionValString(&OPT_InputFrameSource).c_str());
    MbariImage< PixRGB <byte> > output(manager.getOptionValString(&OPT_InputFrameSource).c_str());
    Image<byte>  foaIn;
    ImageData imgData;
    Image< PixRGB<byte> > segmentIn(input.getDims(), ZEROS);
    Image< PixRGB<byte> > inputRaw, inputScaled;
    Image< PixRGB<byte> > clampedInput(input.getDims(), ZEROS);

    // count between frames to run saliency
    uint countFrameDist = 1;
    bool hasCovert; // flag to monitor whether visual cortex had any output

    // initialize property vector and FOE estimator
    PropertyVectorSet pvs;
    FOEestimator foeEst(20, 0);
    Vector2D curFOE;

    // Initialize bayesian network
    BayesClassifier bayesClassifier(dp.itsBayesPath, dp.itsFeatureType, scaledDims);
    FeatureCollection features(scaledDims);

    std::string featureFileName = "predictions.txt";
    std::ofstream featureFile;
    featureFile.open(featureFileName.c_str(),std::ios::out);

    while(1)
    {
     // read new image in?
     FrameState is = FRAME_NEXT;

     if (!singleFrame)
        is = ifs->updateNext();
     else
        is = FRAME_FINAL;

     if (is == FRAME_COMPLETE) break; // done
     if (is == FRAME_NEXT || is == FRAME_FINAL) // new frame
     {
        LINFO("Reading new frame");
        numSpots = 0;

        // initialize the default mask
        mask = staticClipMask;

        // cache image
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

        segmentIn = maskArea(segmentIn, mask);

        // update the focus of expansion - is this still needed ?
        foaIn = luminance(input);
        double threshold = mean(foaIn);
        curFOE = foeEst.updateFOE(makeBinary(foaIn, (const byte)threshold));

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

         // is counter within 1 of reset? queue two successive images in the brain for motion and flicker computation
        --countFrameDist;
        if (countFrameDist <= 1 ) {

            Dims dims = dp.itsRescaleSaliency;
            if (dp.itsRescaleSaliency.w() == 0 && dp.itsRescaleSaliency.h() ==0)
                dims = scaledDims;

            Image< PixRGB<byte> > brainInput;
            Image< PixRGB<byte> > processedInput = inputScaled;

            // if we have a cache which implies we are processing video, not still frames,
            // subtract out existing bit objects to focus attention on new ones only
            // TODO: put in as option - this works best on uniform background
            if (dp.itsSizeAvgCache > 1) {
                const list <BitObject> boList = eventSet.getBitObjectsForFrame(frameNum - 1);
                if (!boList.empty())
                    processedInput = preprocess->background(input, prevInput, frameNum, boList);
            }

            // Get image to input into the brain
            if (dp.itsSaliencyInputType == SIDiffMean) {
                if (dp.itsSizeAvgCache > 1)
                    brainInput = rescale(preprocess->clampedDiffMean(processedInput), dims);
                else
                    LFATAL("ERROR - must specify an imaging cache size "
                        "to use the DiffMean option. Try setting the"
                        "--mbari-cache-size option to something > 1");
            }
            else if (dp.itsSaliencyInputType == SIRaw) {
                if(dp.itsContrastEnhance)
                    brainInput = rescale(preprocess->contrastEnhance(processedInput), dims);
                else
                    brainInput = rescale(processedInput, dims);
            }
            else if (dp.itsSaliencyInputType == SIRG) {
                Image<float> limg;
                Image<float> aimg;
                Image<float> bimg;
                getLAB(preprocess->clampedDiffMean(processedInput),limg,aimg,bimg);
                rv->display(aimg, frameNum, "Aimg");
                brainInput = rescale(aimg, dims);
            }
            else if (dp.itsSaliencyInputType == SIMax) {
                brainInput = rescale(maxRGB(processedInput), dims);
            }
            else
                brainInput = rescale(processedInput, dims);

            rv->display(brainInput, frameNum, "BrainInput");

            // post new input frame for processing
            rutz::shared_ptr<SimEventInputFrame> e(new SimEventInputFrame(brain.get(), GenericFrame(brainInput), 0));
            seq->resetTime(seq->now());
            seq->post(e);
        }

    }

    // check for map output and mask if needed on frame before saliency run
    // the reason mask here and not in the pyramid is because the blur around the inside of the clip mask in the model
    // can mask out interesting objects, particularly for large masks around the edge
    SeC<SimEventVisualCortexOutput> s = seq->check<SimEventVisualCortexOutput>(brain.get());
    if ( s && (is == FRAME_NEXT || is == FRAME_FINAL) && countFrameDist == 0  ) {

        LINFO("Updating visual cortex output for frame %d", frameNum);

        // update the laser mask
        if (dp.itsMaskLasers) {
            LINFO("Masking lasers in L*a*b color space");
            Image< PixRGB<float> > in = input;
            Image<byte>::iterator mitr = mask.beginw();
            Image< PixRGB<float> >::const_iterator ritr = in.beginw(), stop = in.end();
            float thresholda = 50.F;
            // mask out any significant red in the L*a*b color space where strong red has positive a values
            while(ritr != stop) {
                const PixLab<float> pix = PixLab<float>(*ritr++);
                float a = pix.p[1]/3.0F; // 1/3 weight
                *mitr++  = (a > thresholda) ? 0 : *mitr;
            }
        }

        // mask is inverted so morphological operations are in reverse; here we are enlarging the mask to cover
        Image<byte> se = twofiftyfives(3*dp.itsCleanupStructureElementSize);
        mask = erodeImg(mask, se);
        rv->output(ofs, mask, frameNum, "Mask");

        // get saliency map and dimensions
        Image<float> sm = s->vco();
        Dims dimsm = sm.getDims();

        // rescale the mask if needed
        Image<byte> maskRescaled = rescale(mask, dimsm);

        // mask out equipment, etc. in saliency map
        Image<float>::iterator smitr = sm.beginw();
        Image<byte>::const_iterator mitr = maskRescaled.beginw(), stop = maskRescaled.end();
        // set voltage to 0 where mask is 0
        while(mitr != stop) {
           *smitr  = ( (*mitr) == 0 ) ? 0.F : *smitr;
           mitr++; smitr++;
        }

        rv->display(rescale(sm, scaledDims), frameNum, "SaliencyMap");
        // post revised saliency map as new output from the Visual Cortex so other simulation modules can iterate on this
        LINFO("Posting revised saliency map");
        rutz::shared_ptr<SimEventVisualCortexOutput> newsm(new SimEventVisualCortexOutput(brain.get(), sm));
        seq->post(newsm);
    }

    hasCovert = false;
    SimStatus status = SIM_CONTINUE;

    // reached distance between computing saliency in frames ?
    if (countFrameDist == 0) {
        countFrameDist = dp.itsSaliencyFrameDist;

        // initialize the max time to simulate
        const SimTime simMaxEvolveTime = SimTime::MSECS(seq->now().msecs()) + SimTime::MSECS(dp.itsMaxEvolveTime);

        std::list<Winner> winlist;
        std::list<BitObject> objs;
        float scaleH = 1.0f, scaleW = 1.0F;

        // search for new winners until reached max time, max spots or boring WTA point
        LINFO("Searching for new winners...");

        // Put Code Here
        while (status == SIM_CONTINUE) {

            // evolve the brain and other simulation modules
            status = seq->evolve();

            // found a new winner ?
            if (SeC<SimEventWTAwinner> e = seq->check<SimEventWTAwinner>(brain.get())) {
                LINFO("##### time now:%f msecs max evolve time:%f msecs frame: %d #####", \
                        seq->now().msecs(), simMaxEvolveTime.msecs(), frameNum);
                hasCovert = true;
                numSpots++;
                WTAwinner win = e->winner();
                LINFO("##### winner #%d found at [%d; %d] with %f voltage frame: %d#####",
                        numSpots, win.p.i, win.p.j, win.sv, frameNum);

                if (win.boring && !dp.itsKeepWTABoring) {
                    LINFO("##### boring event detected #####");
                    break;
                }
 
                // grab Focus Of Attention (FOA) mask shape to later guide object selection
                if (SeC<SimEventShapeEstimatorOutput> se = seq->check<SimEventShapeEstimatorOutput>(brain.get())) {
                    Image<byte> foamask = Image<byte>(se->smoothMask()*255); 

                    // rescale if needed back to the dimensions of the potentially rescaled input
                    if (scaledDims != foamask.getDims()) {
                        scaleW = (float) scaledDims.w()/(float) foamask.getDims().w();
                        scaleH = (float) scaledDims.h()/(float) foamask.getDims().h();
                        foamask = rescale(foamask, scaledDims);
                        win.p.i = (int) ( (float) win.p.i*scaleW );
                        win.p.j = (int) ( (float) win.p.j*scaleH );
                    }

                    // create bit object out of FOA mask
                    BitObject bo;
                    bo.reset(makeBinary(foamask,byte(0),byte(0),byte(1)));
                    bo.setSMV(win.sv);

                    // if have valid bit object out of the FOA mask, keep winner
                    if (bo.isValid()) {
                        Winner w(win, bo, frameNum);
                        winlist.push_back(w);
                    }
                }

                if (numSpots >= dp.itsMaxWTAPoints) {
                    LINFO("##### found maximum number of salient spots #####");
                    break;
                }

            } // check for winner

            if (seq->now().msecs() >= simMaxEvolveTime.msecs()) {
                LINFO("##### time limit reached time now:%f msecs max evolve time:%f msecs frame: %d #####", \
                            seq->now().msecs(), simMaxEvolveTime.msecs(), frameNum);
                break;
            }
        }// end brain while iteration loop

        objs = objdet->run(rv, frameNum, winlist, segmentIn);

        // create new events with this
        eventSet.initiateEvents(objs, features, imgData);

        rv->output(ofs, showAllWinners(winlist, input, dp.itsMaxDist), frameNum, "Winners");
        winlist.clear();
        objs.clear();
    }

    // update the open events
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

        // save anything requested from brain model
        if (hasCovert)
            brain->save(SimModuleSaveInfo(ofs, *seq));

        // save the input image
        prevInput = input;

        // reset the brain, but only when distance between running saliency is more than every frame
        if (countFrameDist == dp.itsSaliencyFrameDist && dp.itsSaliencyFrameDist > 1) {
            brain->reset(MC_RECURSE);
        }
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
    } // end while
    //######################################################
    LINFO("done!!!");
    manager.stop();
    return 0;
} // end main


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
