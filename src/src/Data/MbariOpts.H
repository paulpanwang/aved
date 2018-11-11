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
 * David and Lucile Packard Foundation:q
 */

/*!@file MbariModelOptionDef.H Types for defining command-line options. */

#ifndef MBARIOPTIONDEF_H_
#define MBARIOPTIONDEF_H_

#include "Component/ModelComponent.H"

extern void REQUEST_OPTIONALIAS_MBARI(OptionManager& m);
struct ModelOptionCateg;
extern const ModelOptionCateg MOC_MBARI;

//! General command-line-options
extern const ModelOptionDef OPT_MMosaicStills;
extern const ModelOptionDef OPT_MTimeLapseStills;
extern const ModelOptionDef OPT_MMidwaterVideo;
extern const ModelOptionDef OPT_MEyeInTheSeaVideo;
extern const ModelOptionDef OPT_MBenthicVideo;
extern const ModelOptionDef OPT_MMidwaterVideo;
extern const ModelOptionDef OPT_MToolSledVideo;

//Extra option
extern const  ModelOptionDef OPT_LOGhavexml;

//! Command-line options for Logger
//@{
extern const ModelOptionDef OPT_LOGsaveEvents;
extern const ModelOptionDef OPT_LOGloadEvents;
extern const ModelOptionDef OPT_LOGsaveEventFeatures;
extern const ModelOptionDef OPT_LOGsaveProperties;
extern const ModelOptionDef OPT_LOGloadProperties;
extern const ModelOptionDef OPT_LOGsavePositions;
extern const ModelOptionDef OPT_LOGsaveOutput;
extern const ModelOptionDef OPT_LOGdisplayOutput;
extern const ModelOptionDef OPT_LOGsaveEventNums;
extern const ModelOptionDef OPT_LOGsaveSummaryEventsName;
extern const ModelOptionDef OPT_LOGsaveXMLEventSet;
extern const ModelOptionDef OPT_LOGmetadataSource;
extern const ModelOptionDef OPT_LOGpadEvents;

//! Command-line options for MbariResultViewer
//@{
extern const ModelOptionDef OPT_MRVmarkInteresting;
extern const ModelOptionDef OPT_MRVopacity;
extern const ModelOptionDef OPT_MRVmarkCandidate;
extern const ModelOptionDef OPT_MRVmarkPrediction;
extern const ModelOptionDef OPT_MRVmarkFOE;
extern const ModelOptionDef OPT_MRVsaveResults;
extern const ModelOptionDef OPT_MRVdisplayResults;
extern const ModelOptionDef OPT_MRVshowEventLabels;
extern const ModelOptionDef OPT_MRVrescaleDisplay;
//@}

//! Command-line options for DetectionParametersModelComponent
//@{
extern const ModelOptionDef OPT_MDPcontrastEnhance;
extern const ModelOptionDef OPT_MDPrescaleSaliency;
extern const ModelOptionDef OPT_MDPsaliencyInputImage;
extern const ModelOptionDef OPT_MDPsaliencyFrameDist;
extern const ModelOptionDef OPT_MDPsegmentAlgorithmInputImage;
extern const ModelOptionDef OPT_MDPsegmentAlgorithmType;
extern const ModelOptionDef OPT_MDPsegmentGraphParameters;
extern const ModelOptionDef OPT_MDPsegmentAdaptiveParameters;
extern const ModelOptionDef OPT_MDPcleanupSESize;
extern const ModelOptionDef OPT_MDPmaxEvolveTime;
extern const ModelOptionDef OPT_MDPmaxWTAPoints;
extern const ModelOptionDef OPT_MDPsaveBoringEvents;
extern const ModelOptionDef OPT_MDPsaveOriginalFrameSpec;
extern const ModelOptionDef OPT_MDPcolorSpace;
extern const ModelOptionDef OPT_MDPminStdDev;
extern const ModelOptionDef OPT_MDPeventExpirationFrames;
extern const ModelOptionDef OPT_MDPremoveOvelappingDetections;
extern const ModelOptionDef OPT_MDPkeepBoringWTAPoints;
extern const ModelOptionDef OPT_MDPtrackingMode;
extern const ModelOptionDef OPT_MDPmaskPath;
extern const ModelOptionDef OPT_MDPmaskXPosition;
extern const ModelOptionDef OPT_MDPmaskYPosition;
extern const ModelOptionDef OPT_MDPmaskWidth;
extern const ModelOptionDef OPT_MDPmaskHeight;
extern const ModelOptionDef OPT_MDPminEventArea;
extern const ModelOptionDef OPT_MDPmaxEventArea;
extern const ModelOptionDef OPT_MDPminEventFrames;
extern const ModelOptionDef OPT_MDPmaxEventFrames;
extern const ModelOptionDef OPT_MDPsizeAvgCache;
extern const ModelOptionDef OPT_MDPmaskDynamic;
extern const ModelOptionDef OPT_MDPmaskLasers;
extern const ModelOptionDef OPT_MDPXKalmanFilterParameters;
extern const ModelOptionDef OPT_MDPYKalmanFilterParameters;
//@}

//! Command-line options for Version
//@{
extern const ModelOptionDef OPT_MbariVersion;
//@}

//! Command-line options for OutputMbariFrameSeries
//@{
extern const ModelOptionDef OPT_OutputMbariShowFrames;
//@}

//! Command-line options for Version
//@{
extern const ModelOptionDef OPT_MLbayesPath;
extern const ModelOptionDef OPT_MLfeatureType;
//@}

#endif /*MBARIOPTIONDEF_H_*/
