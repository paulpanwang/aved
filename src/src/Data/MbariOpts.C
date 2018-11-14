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
 * This code requires the The iLab Neuromorphic Vision C++ Toolkit developedF
 * by the University of Southern California (USC) and the iLab at USC. 
 * See http://iLab.usc.edu for information about this project. 
 *  
 * This work would not be possible without the generous support of the 
 * David and Lucile Packard Foundation
 */ 
#include "Data/MbariOpts.H"

#include "Component/ModelOptionDef.H"
#include "Image/ArrayData.H"
#include "Component/OptionManager.H" 

#include "DetectionAndTracking/TrackingModes.H"
#include "DetectionAndTracking/SaliencyTypes.H"
#include "DetectionAndTracking/SegmentTypes.H"
#include "DetectionAndTracking/ColorSpaceTypes.H"
#include "Learn/FeatureTypes.H"
#include "Image/BitObjectDrawModes.H"

// Format here is:
//
// { MODOPT_TYPE, "name", &MOC_CATEG, OPTEXP_CORE,
//   "description of what option does",
//   "long option name", 'short option name', "valid values", "default value" }
//



// alternatively, for MODOPT_ALIAS option types, format is:
//
// { MODOPT_ALIAS, "", &MOC_ALIAS, OPTEXP_CORE,
//   "description of what alias does",
//   "long option name", 'short option name', "", "list of options" }
//

// NOTE: do not change the default value of any existing option unless
// you really know what you are doing!  Many components will determine
// their default behavior from that default value, so you may break
// lots of executables if you change it.

const ModelOptionCateg MOC_MBARI = {
  MOC_SORTPRI_4, "MBARI Related Options" };

const ModelOptionType MODOPT_ARG_INT = {
  MOK_ARG, &(typeid(int))
};
const ModelOptionType MODOPT_ARG_FLOAT = {
  MOK_ARG, &(typeid(float))
};
void REQUEST_OPTIONALIAS_MBARI(OptionManager& m)
{
  m.requestOptionAlias(&OPT_MBenthicVideo);
  m.requestOptionAlias(&OPT_MEyeInTheSeaVideo);
  m.requestOptionAlias(&OPT_MToolSledVideo);
  m.requestOptionAlias(&OPT_MMidwaterVideo);
  m.requestOptionAlias(&OPT_MMosaicStills);
  m.requestOptionAlias(&OPT_MTimeLapseStills);
}

// #################### Mbari Alias options:
const ModelOptionDef OPT_MToolSledVideo =
  { MODOPT_ALIAS, "ALIASToolSledVideo", &MOC_MBARI, OPTEXP_MRV,
    "Implements good choice of options to experiment with detecting  summary of results from a camera mounted on a "
    "benthic toolsled with shadowing and a moving frame",
    "mbari-toolsled-video", '\0',"",
    "--mbari-tracking-mode=KalmanFilterHough --mbari-cache-size=60 "
    "--mbari-save-original-frame-spec --test-mode=true "
    "--mbari-color-space=RGB --vc-chans=COKMF  --use-random=true --mbari-dynamic-mask=true "
    "--maxnorm-type=FancyOne --mbari-saliency-input-image=Raw --mbari-mask-lasers=true --boring-sm-mv=0.25 "
    "--levelspec=1-3,2-4,1 --mbari-se-size=10 --qtime-decay=1.0 --mbari-max-evolve-msec=1000 "
    "--mbari-segment-algorithm=GraphCut  --mbari-segment-algorithm-input-image=DiffMean "
    "--mbari-saliency-dist=2 --shape-estim-smoothmethod=None --boring-sm-mv=1.0 "
    "--mbari-x-kalman-parameters=0.1,20.0 --mbari-y-kalman-parameters=0.1,20.0 "};

const ModelOptionDef OPT_MEyeInTheSeaVideo =
  { MODOPT_ALIAS, "ALIASEyeInTheSeaVideo", &MOC_MBARI, OPTEXP_MRV,
    "Options used for processing Eye-in-the-Sea Video from the  "
    "Ocean Research and Conservation Association (ORCA)",
    "mbari-eits-video", '\0',"", 
    "--mbari-tracking-mode=KalmanFilter "
    "--mbari-keep-boring-WTA-points=yes "
    "--mbari-save-boring-events=yes --mbari-save-original-frame-spec "
    "--mbari-segment-algorithm-input-image=Luminance --mbari-color-space=RGB "
    "--mbari-saliency-input-image=Raw --mbari-cache-size=2 "
    "--mbari-max-WTA-points=15 --mbari-max-evolve-msec=1000 "
    "--vc-chans=OIC --use-random=true  --maxnorm-type=Maxnorm "
    "--oricomp-type=Steerable --levelspec=1-3,2-5,3 "
    "--mbari-cache-size=2 "
    "--mbari-max-event-area=30000 --mbari-min-std-dev=10.0 "
    "--mbari-segment-algorithm=GraphCut "
    "--mbari-event-expiration-frames=3 --rescale-input=320x240 "
    "--mbari-segment-graph-parameters=0.5,1500,500"  };

const ModelOptionDef OPT_MBenthicVideo =
  { MODOPT_ALIAS, "ALIASBenthicVideo", &MOC_MBARI, OPTEXP_MRV,
    "Implements good choice of options to experiment with "
    "processing video from a moving camera traversing the sea bottom",
    "mbari-benthic-video", '\0',"",
    "--levelspec=1-3,2-4,3 --num-orient=4 --mbari-save-original-frame-spec "
    "--mbari-tracking-mode=Hough --mbari-mask-lasers=true "
    "--mbari-saliency-input-image=Raw --mbari-min-event-frames=3 --sm-type=StdOptim "
    "--vc-chans=OIMF --use-random=true  --mbari-se-size=4 --rescale-input=960x540  --boring-sm-mv=0.25 "
    "--ori-interaction=SubtractMean --oricomp-type=Steerable --shape-estim-smoothmethod=None --mbari-saliency-dist=5 "
    "--mbari-cache-size=30 --mbari-max-evolve-msec=1000 --mbari-keep-boring-WTA-points=true "
    "--maxnorm-type=FancyOne --mbari-min-event-frames=1 "
    "--mbari-dynamic-mask=true "};

const ModelOptionDef OPT_MMidwaterVideo =
  { MODOPT_ALIAS, "ALIASMidwaterVideo", &MOC_MBARI, OPTEXP_MRV,
    "Implements good choice of options to experiment with "
    "processing video from a moving camera traversing the midwater sea column",
    "mbari-midwater-video", '\0',"",
    "--mbari-tracking-mode=KalmanFilterHough --mbari-cache-size=60 --mbari-segment-graph-parameters=0.90,50,30 "
    "--mbari-save-original-frame-spec --test-mode=true --mbari-dynamic-mask=true "
    "--mbari-color-space=RGB --vc-chans=O  --use-random=true --maxnorm-type=FancyOne "
    "--mbari-saliency-input-image=DiffMean --rescale-input=960x540 --contrast-enhance=true "
    "--levelspec=1-3,2-4,1 --mbari-se-size=10 --qtime-decay=1.0 --mbari-max-evolve-msec=500 "
    "--mbari-segment-algorithm=Best  --mbari-segment-algorithm-input-image=ContrastEnhance "
    "--mbari-saliency-dist=5 --shape-estim-smoothmethod=None --boring-sm-mv=1.0 "
    "--mbari-x-kalman-parameters=0.1,10.0 --mbari-y-kalman-parameters=0.1,10.0 "};

const ModelOptionDef OPT_MMosaicStills =
  { MODOPT_ALIAS, "ALIASMosaicStills", &MOC_MBARI, OPTEXP_MRV,
    "Implements good choice of options to experiment with "
    "still frames collected from a moving camera in mosaic form",
    "mbari-mosaic-stills", '\0',"",
    "--mbari-saliency-dist=1 --mbari-tracking-mode=None --mbari-max-event-frames=1 --mbari-min-event-frames=1 "
    "--mbari-keep-boring-WTA-points=yes --mbari-remove-overlapping-detections=False "
    "--boring-sm-mv=1.0 --levelspec=1-3,2-5,2 --shape-estim-smoothmethod=None "
    "--mbari-save-boring-events=yes --maxnorm-type=FancyOne --mbari-pad-events=75 "
    "--vc-chans=OIC --num-orient=8 --use-random=true --mbari-rescale-saliency=960x540 "
    "--mbari-saliency-input-image=Raw "
    "--mbari-max-WTA-points=50 --mbari-max-evolve-msec=5000" };

const ModelOptionDef OPT_MTimeLapseStills =
  { MODOPT_ALIAS, "ALIASTimeLapseStills", &MOC_MBARI, OPTEXP_MRV,
    "Implements good choice of options to experiment with "
    "still frames collected from a stationary time-lapse camera",
    "mbari-timelapse-stills", '\0',"",
    "--mbari-saliency-dist=1 --mbari-tracking-mode=NearestNeighbor "
    "--mbari-keep-boring-WTA-points=yes --mbari-rescale-saliency=960x540 "
    "--mbari-save-boring-events=yes  "
    "--mbari-segment-algorithm-input-image=DiffMean --mbari-color-space=RGB"
    "--mbari-saliency-input-image=Raw --mbari-cache-size=10 "
    "--qtime-decay=1.0 --boring-sm-mv=1.0 --levelspec=1-3,2-5,2 --shape-estim-smoothmethod=None "
    "--vc-chans=OIC  --use-random=true --maxnorm-type=FancyOne "
    "--mbari-max-WTA-points=30 --mbari-max-evolve-msec=15000 " };

// #################### Logger options:


const ModelOptionDef OPT_LOGsaveOutput =
  { MODOPT_FLAG, "LOGsaveOutput", &MOC_MBARI, OPTEXP_MRV,
    "Save output frames in MBARI programs",
    "mbari-save-output", '\0', "", "false" };

const ModelOptionDef OPT_LOGdisplayOutput =
  { MODOPT_FLAG, "LOGdisplayOutput", &MOC_MBARI, OPTEXP_MRV,
    "Display output frames in MBARI programs",
    "mbari-display-output", '\0', "", "false" };

// New option: --have_xml
// Remember we're going to pass a string
const ModelOptionDef OPT_LOGinputFrames =
  { MODOPT_FLAG, "LOGhavexml", &MOC_MBARI, OPTEXP_MRV,
    "Takes a xml(change this)",
    "--have_xml", '\0', "false", ""};

// Used by: Logger
const ModelOptionDef OPT_LOGsaveEvents =
  { MODOPT_ARG_STRING, "LOGsaveEvents", &MOC_MBARI, OPTEXP_MRV,
    "Save the event structure to a text file",
    "mbari-save-events", '\0', "fileName", "" };

const ModelOptionDef OPT_LOGsaveEventFeatures =
  { MODOPT_FLAG, "LOGsaveEventFeatures", &MOC_MBARI, OPTEXP_MRV,
    "Save the event features to .dat files. Used in classification.",
    "mbari-save-event-features", '\0', "", "false" };

// Used by: Logger
const ModelOptionDef OPT_LOGloadEvents =
  { MODOPT_ARG_STRING, "LOGloadEvents", &MOC_MBARI, OPTEXP_MRV,
    "Load the event structure from a text file "
    "instead of computing it from the frames",
    "mbari-load-events", '\0', "fileName", "" };

// Used by: Logger
const ModelOptionDef OPT_LOGsaveProperties =
  { MODOPT_ARG_STRING, "LOGsaveProperties", &MOC_MBARI, OPTEXP_MRV,
    "Save the event property vector to a text file",
    "mbari-save-properties", '\0', "fileName", "" };

// Used by: Logger
const ModelOptionDef OPT_LOGloadProperties =
  { MODOPT_ARG_STRING, "LOGloadProperties", &MOC_MBARI, OPTEXP_MRV,
    "Load the event property vector from a text file",
    "mbari-load-properties", '\0', "fileName", "" };

// Used by: Logger
const ModelOptionDef OPT_LOGsavePositions =
  { MODOPT_ARG_STRING, "LOGsavePositions", &MOC_MBARI, OPTEXP_MRV,
    "Save the positions of events to a text file",
    "mbari-save-positions", '\0', "fileName", "" };

// Used by: Logger
const ModelOptionDef OPT_LOGsaveEventNums =
  { MODOPT_ARG_STRING, "LOGsaveEventNums", &MOC_MBARI, OPTEXP_MRV,
    "Save cropped, event-centered images of specific events, or all events. Crops according to bounding box plus optional --mbari-event-pad.",
    "mbari-save-event-num", '\0', "ev1,ev1,...,evN; or: all", "" };

const ModelOptionDef OPT_LOGpadEvents =
  { MODOPT_ARG_INT, "OPT_LOGpadEvents", &MOC_MBARI, OPTEXP_MRV,
    "Size to pad event-centered image",
    "mbari-pad-events", '\0', "0-200",
    "0" };

const ModelOptionDef OPT_LOGsaveSummaryEventsName =
  { MODOPT_ARG_STRING, "LOGsummaryEvents", &MOC_MBARI, OPTEXP_MRV,
    "Save a human readable summary of all the events to a text file",
    "mbari-save-event-summary", '\0', "fileName", "" };

const ModelOptionDef OPT_LOGsaveXMLEventSet =
  { MODOPT_ARG_STRING, "LOGsaveXMLEventSet", &MOC_MBARI, OPTEXP_MRV,
    "Save a XML output per all events",
    "mbari-save-events-xml", '\0', "fileName", "" };

const ModelOptionDef OPT_LOGmetadataSource =
  { MODOPT_ARG_STRING, "LOGmetadataSource", &MOC_MBARI, OPTEXP_MRV,
    "Add video input source information to XML output",
    "mbari-source-metadata", '\0', "fileName", "" };


// #################### MbariResultViewer options:
// Used by: MbariResultViewer
const ModelOptionDef OPT_MRVmarkInteresting =
  { MODOPT_ARG(BitObjectDrawMode), "MRVmarkInteresting", &MOC_MBARI, OPTEXP_MRV,
    "Way to mark interesting events in output frames of MBARI programs",
    "mbari-mark-interesting", '\0', "<None|Shape|Outline|BoundingBox>",
    "BoundingBox" };

// Used by: MbariResultViewer
const ModelOptionDef OPT_MRVopacity =
  { MODOPT_ARG(float), "MRVopacity", &MOC_MBARI, OPTEXP_MRV,
    "Opacity of shape or outline markings of events",
    "mbari-opacity", '\0', "<0.0 ... 1.0>", "1.0" };

// Used by: MbariResultViewer
const ModelOptionDef OPT_MRVmarkCandidate =
  { MODOPT_FLAG, "MRVmarkCandidate", &MOC_MBARI, OPTEXP_MRV,
    "Mark candidates for interesting events in output frames of MBARI programs",
    "mbari-mark-candidate", '\0', "", "true" };

// Used by: MbariResultViewer
const ModelOptionDef OPT_MRVmarkPrediction =
  { MODOPT_FLAG, "MRVmarkPrediction", &MOC_MBARI, OPTEXP_MRV,
    "Mark the Kalman Filter's prediction for the location of an object "
    "in output frames of MBARI programs",
    "mbari-mark-prediction", '\0', "", "false" };

// Used by: MbariResultViewer
const ModelOptionDef OPT_MRVmarkFOE =
  { MODOPT_FLAG, "MRVmarkFOE", &MOC_MBARI, OPTEXP_MRV,
    "Mark the focus of expansion in the output frames of MBARI programs",
    "mbari-mark-foe", '\0', "", "false" };

// Used by: MbariResultViewer
const ModelOptionDef OPT_MRVsaveResults =
  { MODOPT_FLAG, "MRVsaveResults", &MOC_MBARI, OPTEXP_MRV,
    "Save algorithm results at various points in the MBARI programs to disk",
    "mbari-save-results", '\0', "", "false" };

// Used by: MbariResultViewer
const ModelOptionDef OPT_MRVdisplayResults =
  { MODOPT_FLAG, "MRVdisplayResults", &MOC_MBARI, OPTEXP_MRV,
    "Display algorithm output at various points in MBARI programs",
    "mbari-display-results", '\0', "", "false" };

// Used by: MbariResultViewer
const ModelOptionDef OPT_MRVshowEventLabels =
  { MODOPT_FLAG, "MRVshowEventLabels", &MOC_MBARI, OPTEXP_MRV,
    "Write event labels into the output frames in MBARI programs",
    "mbari-label-events", '\0', "", "true" };

// Used by: MbariResultViewer
const ModelOptionDef OPT_MRVrescaleDisplay =
  { MODOPT_ARG(Dims), "MRVrescaleDisplay", &MOC_MBARI, OPTEXP_MRV,
    "Rescale MBARI displays to <width>x<height>, or 0x0 for no rescaling",
    "mbari-rescale-display", '\0', "<width>x<height>", "0x0" };

// #################### DetectionParametersModelComponent options:
const ModelOptionDef OPT_MDPcontrastEnhance =
  { MODOPT_FLAG, "MRVcontrastEnhance", &MOC_MBARI, OPTEXP_MRV,
   "Automatically contrast enhance gamma in input frames in MBARI programs",
    "mbari-contrast-enhance", '\0', "", "true" };
const ModelOptionDef OPT_MDPrescaleSaliency =
  { MODOPT_ARG(Dims), "MDPrescaleSaliency", &MOC_MBARI, OPTEXP_MRV,
    "Rescale input to the saliency algorithm to <width>x<height>, or 0x0 for no rescaling",
    "mbari-rescale-saliency", '\0', "<width>x<height>", "0x0" };
const ModelOptionDef OPT_MDPremoveOvelappingDetections =
  { MODOPT_FLAG, "OPT_MDPremoveOvelappingDetections", &MOC_MBARI, OPTEXP_MRV,
    "Do not remove overlapping detections",
    "mbari-remove-overlapping-detections", '\0', "", "true" };
const ModelOptionDef OPT_MDPtrackingMode =
  { MODOPT_ARG(TrackingMode), "MDPtrackingMode", &MOC_MBARI, OPTEXP_MRV,
    "Way to mark interesting events in output of MBARI programs",
    "mbari-tracking-mode", '\0', "<KalmanFilter|NearestNeighbor|Hough|NearestNeighborHough|KalmanFilterHough|None>",
    "KalmanFilter" };
const ModelOptionDef OPT_MDPsegmentAlgorithmType =
  { MODOPT_ARG(SegmentAlgorithmType), "MDPsegmentAlgorithm", &MOC_MBARI, OPTEXP_MRV,
    "Segment algorithm to find foreground objects",
    "mbari-segment-algorithm", '\0', "<MeanAdaptive|MedianAdaptive|MeanMinMaxAdapative|GraphCut|Best>",
    "Best" };
const ModelOptionDef OPT_MDPsegmentAdaptiveParameters =
  { MODOPT_ARG_STRING, "MDPsegmentAdaptiveParameters", &MOC_MBARI, OPTEXP_MRV,
    "Neighborhood size and size of the offset to subtract from the mean or median in the segment algorithm",
    "mbari-segment-adaptive-parameters", '\0', "neighborhood, offset","20,7" };
const ModelOptionDef OPT_MDPsegmentGraphParameters =
  { MODOPT_ARG_STRING, "MDPsegmentGraphParameters", &MOC_MBARI, OPTEXP_MRV,
    "Graph segment parameters, in the order sigma, k, minsize. Generally,the defaults work.\
     Dont mess with this unless you need to.  See algorithm details in Segmentation.C.",
    "mbari-segment-graph-parameters", '\0', "sigma, k, minsize", "0.75,500,50" };
const ModelOptionDef OPT_MDPXKalmanFilterParameters =
  { MODOPT_ARG_STRING, "MDPXKalmanFilterParameters", &MOC_MBARI, OPTEXP_MRV,
    "X direction Kalman filter parameters, in the order process noise, measurement noise",
    "mbari-x-kalman-parameters", '\0', "process noise, measurement noise", "0.1,10.0" };
const ModelOptionDef OPT_MDPYKalmanFilterParameters =
  { MODOPT_ARG_STRING, "MDPYKalmanFilterParameters", &MOC_MBARI, OPTEXP_MRV,
    "Y direction Kalman filter parameters, in the order process noise, measurement noise",
    "mbari-y-kalman-parameters", '\0', "process noise, measurement noise", "0.1,10.0" };
const ModelOptionDef OPT_MDPcolorSpace = {
   MODOPT_ARG(ColorSpaceType), "MDPcolorSpace", &MOC_MBARI, OPTEXP_MRV,
   "Input image color space. Used to determine whether to compute saliency on color channels or not",
    "mbari-color-space", '\0', "<RGB|YCBCR|Gray>",
    "RGB" };
const ModelOptionDef OPT_MDPsegmentAlgorithmInputImage = {
   MODOPT_ARG(SegmentAlgorithmInputImageType), "MDPsegmentInputImage", &MOC_MBARI, OPTEXP_MRV,
   "Segment algorithm input images type",
    "mbari-segment-algorithm-input-image", '\0', "<DiffMean|Luminance|ContrastEnhance>",
    "ContrastEnhance" };
const ModelOptionDef OPT_MDPsaliencyInputImage = {
      MODOPT_ARG(SaliencyInputImageType), "MDPsaliencyInputImage", &MOC_MBARI, OPTEXP_MRV,
    "Saliency input image type",
    "mbari-saliency-input-image", '\0', "<Raw|DiffMean|Max|RedGreenOpponent>",
    "Raw" };
const ModelOptionDef OPT_MDPcleanupSESize =
  { MODOPT_ARG_INT, "MDPcleanupSESize", &MOC_MBARI, OPTEXP_MRV,
    "Size of structure element to do morhphological erode/dilate to clean-up segmented image",
    "mbari-se-size", '\0', "1-20",
    "5" };
const ModelOptionDef OPT_MDPmaskPath =
  { MODOPT_ARG_STRING, "MDPmaskPath", &MOC_MBARI, OPTEXP_MRV,
    "MaskPath: path to the mask image",
    "mbari-mask-path", '\0', "<file>",
    "" };
const ModelOptionDef OPT_MDPmaskXPosition =
  { MODOPT_ARG_INT, "MDPmaskXPosition", &MOC_MBARI, OPTEXP_MRV,
    "MaskXPosition: x position of the mask point of reference; ",
    "mbari-mask-xposition", '\0', "<int>", "1" };
const ModelOptionDef OPT_MDPmaskYPosition =
  { MODOPT_ARG_INT, "MDPmaskYPosition", &MOC_MBARI, OPTEXP_MRV,
    "MaskYPosition: y position of the mask point of reference; ",
    "mbari-mask-yposition", '\0', "<int>", "1" };
const ModelOptionDef OPT_MDPmaskWidth =
  { MODOPT_ARG_INT, "MDPmaskWidth", &MOC_MBARI, OPTEXP_MRV,
    "MaskWidth: mask width; ",
    "mbari-mask-width", '\0', "<int>", "1" };
const ModelOptionDef OPT_MDPmaskHeight =
  { MODOPT_ARG_INT, "MDPmaskHeight", &MOC_MBARI, OPTEXP_MRV,
    "MaskHeight: mask height; ",
    "mbari-mask-height", '\0', "<int>", "1" };
const ModelOptionDef OPT_MDPminEventArea =
  { MODOPT_ARG_INT, "MDPBminEventArea", &MOC_MBARI, OPTEXP_MRV,
    "The minimum area an event must be to be candidate. When set to 0, defaults to multiplied factor of the foa size, which is derived from the image size.",
    "mbari-min-event-area", '\0', "<int>", "0" };
const ModelOptionDef OPT_MDPmaxEventArea =
  { MODOPT_ARG_INT, "MDPBmaxEventArea", &MOC_MBARI, OPTEXP_MRV,
    "The maximum area an event can be, to be candidate. When set to 0, defaults to a multiplied factor of the foa size, which is derived from the image size.",
    "mbari-max-event-area", '\0', "<int>", "0" };
const ModelOptionDef OPT_MDPminEventFrames =
  { MODOPT_ARG_INT, "MDPBminEventFrames", &MOC_MBARI, OPTEXP_MRV,
    "The minimum number of frames an event must be to be candidate",
    "mbari-min-event-frames", '\0', "<int>", "1" };
const ModelOptionDef OPT_MDPmaxEventFrames =
  { MODOPT_ARG_INT, "MDPBmaxEventFrames", &MOC_MBARI, OPTEXP_MRV,
    "The maximum number of frames an event can be; defaults to indefinite",
    "mbari-max-event-frames", '\0', "<int>", "-1" };
const ModelOptionDef OPT_MDPsizeAvgCache =
  { MODOPT_ARG_INT, "MDPBsizeAvgCache", &MOC_MBARI, OPTEXP_MRV,
    "The number of frames used to compute the running average",
    "mbari-cache-size", '\0', "<int>", "30" };
const ModelOptionDef OPT_MDPsaliencyFrameDist =
  { MODOPT_ARG_INT, "MDPBsaliencyFrameDist", &MOC_MBARI, OPTEXP_MRV,
    "The number of frames to delay between saliency map computations ",
    "mbari-saliency-dist", '\0', "<int>", "5" };
const ModelOptionDef OPT_MDPmaxEvolveTime =
  { MODOPT_ARG_INT, "MDPBmaxEvolveTime", &MOC_MBARI, OPTEXP_MRV,
    "Maximum amount of time in milliseconds to evolve the brain until stopping",
    "mbari-max-evolve-msec", '\0', "<int>", "500" };
const ModelOptionDef OPT_MDPmaxWTAPoints =
  { MODOPT_ARG_INT, "MDPBmaxWTAPoints", &MOC_MBARI, OPTEXP_MRV,
    "Maximum number of winner-take-all points to find in each frame",
    "mbari-max-WTA-points", '\0', "<int>", "20" };
const ModelOptionDef OPT_MDPkeepBoringWTAPoints =
  { MODOPT_FLAG, "OPT_MDPkeepBoringWTAPoints", &MOC_MBARI, OPTEXP_MRV,
    "Keep boring WTA points from saliency computation. Turning this on "
    "will increase the number of candidates, but can also increase the"
    "number of false detections",
    "mbari-keep-boring-WTA-points", '\0', "", "false" };
const ModelOptionDef OPT_MDPmaskDynamic =
  { MODOPT_FLAG, "OPT_MDPmaskDynamic", &MOC_MBARI, OPTEXP_MRV,
    "Generate dyamic mask for brain during saliency computation using segmented images ",
    "mbari-dynamic-mask", '\0', "", "false" };
const ModelOptionDef OPT_MDPmaskLasers =
  { MODOPT_FLAG, "OPT_MDPmaskLasers", &MOC_MBARI, OPTEXP_MRV,
    "Mask lasers commonly used for measurement in underwater video.",
    "mbari-mask-lasers", '\0', "", "false" };
const ModelOptionDef OPT_MDPsaveBoringEvents =
  { MODOPT_FLAG, "OPT_MDPsaveBoringEvents", &MOC_MBARI, OPTEXP_MRV,
    "Save boring events. Default is to remove boring (non-interesting) events, set to true to save",
    "mbari-save-boring-events", '\1', "", "false" };
const ModelOptionDef OPT_MDPsaveOriginalFrameSpec =
  { MODOPT_FLAG, "OPT_MDPsaveOriginalFrameSpec", &MOC_MBARI, OPTEXP_MRV,
    "Save events in original frame size specs, but run saliency computation on reduced frame size. This does nothing if the frames are not resized with the --rescale-input option. Default is set to false",
    "mbari-save-original-frame-spec", '\0', "", "false" };
const ModelOptionDef OPT_MDPminStdDev =
  { MODOPT_ARG_FLOAT, "OPT_MDPminStdDev", &MOC_MBARI, OPTEXP_MRV,
    "Minimum std deviation of input image required for processing. This is useful to remove black frames, or frames with high visual noise",
    "mbari-min-std-dev", '\0', "<float>", "0"};

// ####################

// #################### Version options:
const ModelOptionDef OPT_MbariVersion =
  { MODOPT_FLAG, "MBARIVersion", &MOC_MBARI, OPTEXP_MRV,
    "Print version",
    "version", 'v', "", "false" };
// ####################

// #################### Learning options:
const ModelOptionDef OPT_MLbayesPath =
  { MODOPT_ARG_STRING, "MLbayesPath", &MOC_MBARI, OPTEXP_MRV,
    "Bayes network: path to the Bayes classifier network file",
    "mbari-bayes-net-path", '\0', "<file>",
    "" };

const ModelOptionDef OPT_MLfeatureType =
  { MODOPT_ARG(FeatureType), "MLfeatureType", &MOC_MBARI, OPTEXP_MRV,
    "Name of the features used in the Bayes classifier network file",
    "mbari-bayes-net-features", '\0', "<HOG3|HOG8|MBH3|MBH8|JET>",
    "HOG8" };
// ####################
