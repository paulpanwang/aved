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

#include <string>
#include <Component/ModelParam.H>

#include "DetectionAndTracking/DetectionParameters.H"
#include "Util/MathFunctions.H"
#include "Data/MbariOpts.H"

#include <algorithm>
#include <cmath>

/*!@file DetectionParameters.C classes useful for containing event detection
parameters*/


// ######################################################################
/// DetectionParameters class 
// ######################################################################

DetectionParameters::DetectionParameters()
: itsContrastEnhance(CONTRAST_ENHANCE),
itsMaxEvolveTime(DEFAULT_MAX_EVOLVE_TIME),
itsMaxWTAPoints(DEFAULT_MAX_WTA_POINTS),
itsSaveNonInteresting(DEFAULT_SAVE_NON_INTERESTING),
itsSaveOriginalFrameSpec(DEFAULT_SAVE_ORG_FRAME_SPEC),
itsTrackingMode(DEFAULT_TRACKING_MODE),
itsColorSpaceType(DEFAULT_COLOR_SPACE),
itsMinStdDev(DEFAULT_MIN_STD_DEV),
itsMaxDist(40),
itsMaxEventFrames(DEFAULT_MAX_EVENT_FRAMES),
itsMinEventFrames(DEFAULT_MIN_EVENT_FRAMES),
itsMaxEventArea(0),
itsMinEventArea(0),
itsSaliencyFrameDist(DEFAULT_SALIENCY_FRAME_DIST),
itsMaskPath(""),
itsBayesPath(""),
itsFeatureType(DEFAULT_FEATURE_TYPE),
itsSizeAvgCache(DEFAULT_SIZE_AVG_CACHE),
itsMaskXPosition(DEFAULT_MASK_X_POSITION),
itsMaskYPosition(DEFAULT_MASK_Y_POSITION),
itsMaskWidth(DEFAULT_MASK_HEIGHT),
itsMaskHeight(DEFAULT_MASK_WIDTH),
itsRescaleSaliency(Dims(0,0)),
itsRemoveOverlappingDetections(DEFAULT_REMOVE_OVERLAP_DETECTIONS),
itsSegmentAlgorithmType(DEFAULT_SEGMENT_ALGORITHM_TYPE),
itsSegmentAlgorithmInputType(DEFAULT_SEGMENT_ALGORITHM_INPUT_TYPE),
itsSegmentGraphParameters(DEFAULT_SEGMENT_GRAPH_PARAMETERS),
itsSegmentAdaptiveParameters(DEFAULT_SEGMENT_ADAPTIVE_PARAMETERS),
itsCleanupStructureElementSize(DEFAULT_SE_SIZE),
itsSaliencyInputType(DEFAULT_SALIENCY_INPUT_TYPE),
itsKeepWTABoring(DEFAULT_KEEP_WTA_BORING),
itsMaskDynamic(DEFAULT_DYNAMIC_MASK),
itsMaskLasers(DEFAULT_MASK_LASERS),
itsXKalmanFilterParameters(DEFAULT_KALMAN_PARAMETERS),
itsYKalmanFilterParameters(DEFAULT_KALMAN_PARAMETERS)
{
    //initialize with some defaults
    float maxDist = itsMaxDist;
    float maxAreaDiff = pow((double) maxDist, 2) / (double) 4.0;
    itsMaxCost = pow((double) maxDist, 2) + pow((double) maxAreaDiff, 2);
}

void DetectionParameters::writeToStream(std::ostream& os) {
    // Only write the parameters that are set by model options
    // these are the options that a user can set
    os << "\tcontrastenhance" << itsContrastEnhance;
    os << "\tcachesize:" << itsSizeAvgCache;
    os << "\tminarea:" << itsMinEventArea << "\tmaxarea:" << itsMaxEventArea;
    os << "\ttrackingmode:" << trackingModeName(itsTrackingMode); 
    os << "\tsegmentalgorithminputimagetype:" << segmentAlgorithmInputImageType(itsSegmentAlgorithmInputType);
    os << "\tsegmentalgorithmtype:" << segmentAlgorithmType(itsSegmentAlgorithmType);
    os << "\tsegmentadaptiveparameters:" << itsSegmentAdaptiveParameters;
    os << "\tsaliencyinputimagetype:" << saliencyInputImageType(itsSaliencyInputType);
    os << "\featuretype:" << featureType(itsFeatureType);
    os << "\tremoveoverlapdetections:" << itsRemoveOverlappingDetections;
    os << "\tsaliencyrescale:" << toStr(itsRescaleSaliency);
    os << "\tsegmentgraphparameters:" << itsSegmentGraphParameters;
    os << "\txkalmanfilterparameters:" << itsXKalmanFilterParameters;
    os << "\tykalmanfilterparameters:" << itsYKalmanFilterParameters;
    os << "\tcleanupelementsize:" << itsCleanupStructureElementSize;
    os << "\tminframes:" << itsMinEventFrames;
    os << "\tmaxframes:" << itsMaxEventFrames;
    os << "\tmaxdist:" << itsMaxDist;
    os << "\tsaliencyframedist:" << itsSaliencyFrameDist;
    os << "\tmaxcost:" << itsMaxCost;
    os << "\tmaxevolvetime(msecs):" << itsMaxEvolveTime;
    os << "\tmaxwtapoints:" << itsMaxWTAPoints;
    os << "\tsavenoninteresting:" << itsSaveNonInteresting;
    os << "\tsaveoriginalframespec:" << itsSaveOriginalFrameSpec;
    os << "\taddgraphpoints:" << itsMaskLasers;
    os << "\tcolorspace:" << colorSpaceType(itsColorSpaceType);
    os << "\tminstddev:" << itsMinStdDev;

    os << "\tdynamicmask:" << itsMaskDynamic;
    if (itsMaskPath.length() > 0) {
        os << "\tmaskpath:" << itsMaskPath;
        os << "\tmaskxposition:" << itsMaskXPosition;
        os << "\tmaskyposition:" << itsMaskYPosition;
        os << "\tmaskwidth:" << itsMaskWidth;
        os << "\tmaskheight:" << itsMaskHeight;
    }

    if (itsBayesPath.length() > 0)
        os << "\tbayesPath:" << itsBayesPath;
    os << "\n";
}
// ######################################################################
DetectionParameters &DetectionParameters::operator=(const DetectionParameters& p) {
    this->itsContrastEnhance = p.itsContrastEnhance;
    this->itsMaxEvolveTime = p.itsMaxEvolveTime;
    this->itsMaxWTAPoints = p.itsMaxWTAPoints;
    this->itsMaxDist = p.itsMaxDist;
    this->itsMaxCost = p.itsMaxCost;
    this->itsMaxEventFrames = p.itsMaxEventFrames;
    this->itsMinEventFrames = p.itsMinEventFrames;
    this->itsMinEventArea = p.itsMinEventArea;
    this->itsMaxEventArea = p.itsMaxEventArea;
    this->itsTrackingMode = p.itsTrackingMode;
    this->itsSegmentAdaptiveParameters = p.itsSegmentAdaptiveParameters;
    this->itsSegmentAlgorithmInputType = p.itsSegmentAlgorithmInputType;
    this->itsSegmentAlgorithmType = p.itsSegmentAlgorithmType;
    this->itsSegmentGraphParameters = p.itsSegmentGraphParameters;
    this->itsXKalmanFilterParameters = p.itsXKalmanFilterParameters;
    this->itsYKalmanFilterParameters = p.itsYKalmanFilterParameters;
    this->itsCleanupStructureElementSize = p.itsCleanupStructureElementSize;
    this->itsRescaleSaliency = p.itsRescaleSaliency;
    this->itsRemoveOverlappingDetections = p.itsRemoveOverlappingDetections;
    this->itsSaliencyInputType = p.itsSaliencyInputType;
    this->itsSaliencyFrameDist = p.itsSaliencyFrameDist;
    this->itsKeepWTABoring = p.itsKeepWTABoring;
    this->itsSaveNonInteresting = p.itsSaveNonInteresting;
    this->itsSaveOriginalFrameSpec = p.itsSaveOriginalFrameSpec;
    this->itsColorSpaceType = p.itsColorSpaceType;
    this->itsMinStdDev = p.itsMinStdDev;
    this->itsMaskPath = p.itsMaskPath;
    this->itsBayesPath = p.itsBayesPath;
    this->itsFeatureType = p.itsFeatureType;
    this->itsMaskWidth = p.itsMaskWidth;
    this->itsMaskXPosition = p.itsMaskXPosition;
    this->itsMaskYPosition = p.itsMaskYPosition;
    this->itsMaskDynamic = p.itsMaskDynamic;
    this->itsMaskLasers = p.itsMaskLasers;
    return *this;
}
// ######################################################################
// DetectionParameters Singleton class static member initialization
// ######################################################################
DetectionParametersSingleton* DetectionParametersSingleton::itsInstance = 0;

// ######################################################################
// ###### DetectionParametersSingleton class
// ######################################################################

DetectionParametersSingleton::~DetectionParametersSingleton() {
    if (itsInstance) {
        delete itsInstance;
    }
}
// ######################################################################

DetectionParametersSingleton::DetectionParametersSingleton(const DetectionParameters &d) {
    this->itsParameters = d;
}
// ######################################################################

void DetectionParametersSingleton::initialize(DetectionParameters &p, const Dims &dims, const int foaRadius) {
    DetectionParametersSingleton *dp = instance();

    // calculate cost parameter from other derived values
    // initialize parameters
    const int maxDist = dims.w() / MAX_DIST_RATIO;
    float maxAreaDiff = maxDist * maxDist / 4.0F;
    float maxDistFloat = (float) maxDist;

    p.itsMaxCost = pow(maxDistFloat,2.0F);
    p.itsMaxDist = maxDist;

    if (p.itsMinEventArea == 0) 
    	p.itsMinEventArea = foaRadius * MIN_SIZE_FACTOR;
    if (p.itsMaxEventArea == 0) 
    	p.itsMaxEventArea = foaRadius * MAX_SIZE_FACTOR;

    dp->itsParameters = p;
}
// ######################################################################

DetectionParametersSingleton* DetectionParametersSingleton::instance() {
    if (itsInstance == 0) {
	DetectionParameters d; 
    	itsInstance = new DetectionParametersSingleton(d);
    }
    return itsInstance;
}
// ######################################################################
/// DetectionParametersModelComponent class 
// ######################################################################

DetectionParametersModelComponent::DetectionParametersModelComponent(ModelManager &mgr)
: ModelComponent(mgr, std::string("DetectionParameters"), std::string("DetectionParameters")),
itsContrastEnhance(&OPT_MDPcontrastEnhance, this),
itsMaxEvolveTime(&OPT_MDPmaxEvolveTime, this),
itsMaxWTAPoints(&OPT_MDPmaxWTAPoints, this),
itsSaveNonInteresting(&OPT_MDPsaveBoringEvents, this),
itsSaveOriginalFrameSpec(&OPT_MDPsaveOriginalFrameSpec, this),
itsTrackingMode(&OPT_MDPtrackingMode, this),
itsColorSpaceType(&OPT_MDPcolorSpace, this),
itsMinStdDev(&OPT_MDPminStdDev, this),
itsMaxEventFrames(&OPT_MDPmaxEventFrames, this),
itsMinEventFrames(&OPT_MDPminEventFrames, this),
itsMaxEventArea(&OPT_MDPmaxEventArea, this),
itsMinEventArea(&OPT_MDPminEventArea, this),
itsSaliencyFrameDist(&OPT_MDPsaliencyFrameDist, this),
itsRescaleSaliency(&OPT_MDPrescaleSaliency, this),
itsRemoveOverlappingDetections(&OPT_MDPremoveOvelappingDetections, this),
itsMaskPath(&OPT_MDPmaskPath, this),
itsBayesPath(&OPT_MLbayesPath, this),
itsSizeAvgCache(&OPT_MDPsizeAvgCache, this),
itsMaskXPosition(&OPT_MDPmaskXPosition, this),
itsMaskYPosition(&OPT_MDPmaskYPosition, this),
itsMaskWidth(&OPT_MDPmaskWidth, this),
itsMaskHeight(&OPT_MDPmaskHeight, this),
itsSegmentAlgorithmType(&OPT_MDPsegmentAlgorithmType, this),
itsSegmentAlgorithmInputType(&OPT_MDPsegmentAlgorithmInputImage, this),
itsSegmentGraphParameters(&OPT_MDPsegmentGraphParameters, this),
itsSegmentAdaptiveParameters(&OPT_MDPsegmentAdaptiveParameters, this),
itsCleanupStructureElementSize(&OPT_MDPcleanupSESize, this),
itsSaliencyInputType(&OPT_MDPsaliencyInputImage, this),
itsFeatureType(&OPT_MLfeatureType, this),
itsKeepWTABoring(&OPT_MDPkeepBoringWTAPoints, this),
itsMaskLasers(&OPT_MDPmaskLasers, this),
itsMaskDynamic(&OPT_MDPmaskDynamic, this),
itsXKalmanFilterParameters(&OPT_MDPXKalmanFilterParameters, this),
itsYKalmanFilterParameters(&OPT_MDPYKalmanFilterParameters, this)
{
};
// ######################################################################

void DetectionParametersModelComponent::reset(DetectionParameters *p) {
    if (itsMaxEvolveTime.getVal() > 0)
        p->itsMaxEvolveTime = itsMaxEvolveTime.getVal();
    if (itsMaxWTAPoints.getVal() > 0)
        p->itsMaxWTAPoints = itsMaxWTAPoints.getVal();
    if (itsTrackingMode.getVal() >= TMKalmanFilter)
        p->itsTrackingMode = itsTrackingMode.getVal();
     if (itsSaliencyInputType.getVal() > 0)
        p->itsSaliencyInputType = itsSaliencyInputType.getVal();
    if (itsFeatureType.getVal() > 0)
        p->itsFeatureType = itsFeatureType.getVal();
    p->itsRescaleSaliency = itsRescaleSaliency.getVal();
    p->itsRemoveOverlappingDetections = itsRemoveOverlappingDetections.getVal();
    if (itsCleanupStructureElementSize.getVal() > 1 && itsCleanupStructureElementSize.getVal() <= MAX_SE_SIZE)
        p->itsCleanupStructureElementSize = itsCleanupStructureElementSize.getVal();
    if (itsBayesPath.getVal().length() > 0)
        p->itsBayesPath = itsBayesPath.getVal().data();
    if (itsMaskPath.getVal().length() > 0)
        p->itsMaskPath = itsMaskPath.getVal().data();
    if (itsMaskXPosition.getVal() >= 0)
        p->itsMaskXPosition = itsMaskXPosition.getVal();
    if (itsMaskYPosition.getVal() >= 0)
        p->itsMaskYPosition = itsMaskYPosition.getVal();
    if (itsMaskWidth.getVal() > 0)
        p->itsMaskWidth = itsMaskWidth.getVal();
    if (itsMaskHeight.getVal() > 0)
        p->itsMaskHeight = itsMaskHeight.getVal();
    if (itsSizeAvgCache.getVal() > 0)
        p->itsSizeAvgCache = itsSizeAvgCache.getVal();
    if (itsMinEventArea.getVal() > 0)
        p->itsMinEventArea = itsMinEventArea.getVal();
    if (itsMaxEventArea.getVal() > 0)
        p->itsMaxEventArea = itsMaxEventArea.getVal();

    if (p->itsMinEventArea >= p->itsMaxEventArea && p->itsMinEventArea > 0 && p->itsMaxEventArea > 0)
	p->itsMinEventArea = p->itsMaxEventArea - 1;
	
    if (itsMinEventFrames.getVal() >= 0)
        p->itsMinEventFrames = itsMinEventFrames.getVal();
    else
        p->itsMinEventFrames = DEFAULT_MIN_EVENT_FRAMES;
    if (itsMaxEventFrames.getVal() > 0)
        p->itsMaxEventFrames = itsMaxEventFrames.getVal();
    else
        p->itsMaxEventFrames = DEFAULT_MAX_EVENT_FRAMES;
    
    if (p->itsMinEventFrames >= p->itsMaxEventFrames && p->itsMinEventFrames > 0 && p->itsMaxEventFrames > 0 )
	p->itsMinEventFrames = p->itsMaxEventFrames - 1;

    if (itsMinStdDev.getVal() > 0.f)
        p->itsMinStdDev = itsMinStdDev.getVal();
    else
        p->itsMinStdDev = DEFAULT_MIN_STD_DEV;

    if (itsSaliencyFrameDist.getVal() > 0)
        p->itsSaliencyFrameDist = itsSaliencyFrameDist.getVal();

    p->itsKeepWTABoring = itsKeepWTABoring.getVal();
    p->itsSaveNonInteresting = itsSaveNonInteresting.getVal();
    p->itsSaveOriginalFrameSpec = itsSaveOriginalFrameSpec.getVal();
    p->itsColorSpaceType = itsColorSpaceType.getVal();
    p->itsSegmentAlgorithmInputType = itsSegmentAlgorithmInputType.getVal();
    p->itsSegmentAlgorithmType = itsSegmentAlgorithmType.getVal();
    p->itsSegmentAdaptiveParameters = itsSegmentAdaptiveParameters.getVal();
    p->itsSegmentGraphParameters = itsSegmentGraphParameters.getVal();
    p->itsMaskLasers = itsMaskLasers.getVal();
    p->itsMaskDynamic = itsMaskDynamic.getVal();
    p->itsXKalmanFilterParameters = itsXKalmanFilterParameters.getVal();
    p->itsYKalmanFilterParameters = itsYKalmanFilterParameters.getVal();
    p->itsContrastEnhance = itsContrastEnhance.getVal();
}
