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

/*!@file MbariFunctions.H miscellaneous functions
 */ 
#ifndef MBARI_FUNCTIONS_DEFINED
#define MBARI_FUNCTIONS_DEFINED

#include "Neuro/StdBrain.H"
#include "Simulation/SimEventQueueConfigurator.H"
#include "Image/Point2D.H"
#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "Data/Winner.H"
#include "Data/MbariMetaData.H"
#include "DetectionAndTracking/DetectionParameters.H"
#include "Image/Transforms.H"
#include "Parallel/pvisionTCP-defs.H"
#include "Util/Types.H"

#include <vector>
#include <string>

class Token;
class MbariResultViewer;
class Brain; 
class BitObject;
template <class T> class PixRGB;
template <class T> class Image;
namespace rutz { template <class T> class shared_ptr; }
namespace nub { template <class T> class soft_ref; }
#include <iostream>
#include <list>

//! compute input filename for current frame
std::string getInputFileName(const std::string &stem,
                             const int framenumber);

//! A method to test if an image is grayscale
/*! or not by checking whether r = g = b for every pixel
 * returns true if grayscale, otherwise false
 * !@param src the image to test
 */
bool isGrayscale(const Image<PixRGB<byte> > &src);

//! extract a set of BitObjects from bitImg, which intersect region
/*! bitImg is flooded starting from each point within region,
  and each BitObject exceeding the minSize is stored in the
  list of BitObjects that is returned. */
std::list <BitObject> extractBitObjectsGray(const Image<byte> &bitImg,
                                        Rectangle region,
                                        const int minSize,
                                        const int maxSize); 
                                        
//! remove overlapping bit objects
std::list <BitObject> removeOverlappingObjects(std::list<BitObject> &bosUnfiltered);
 
//! extract a set of BitObjects from a color labeled images, which intersect region
/*! Color labeled by graphcut segmentation algorithm. Image is then flooded starting
 * from each point within segmentRegion, and each BitObject within min/maxSize is kept*/
std::list <BitObject> extractBitObjectsGraphcut(nub::soft_ref<MbariResultViewer> &rv,
                                        const uint frameNum,
                                        const Image<PixRGB<byte> > &bImg,
                                        const Point2D<int> seed,
                                        const Rectangle searchRegion,
                                        const Rectangle segmentRegion,
                                        const int minSize,
                                        const int maxSize,
                                        const float minIntensity = 0.0F,
                                        const int iterations = 3);
    
//! returns list of SalientWinners of the most salient points
/*! @param manager the model manager
 @param clipMask image to apply as mask over the input image
 @param brain the brain to process saliency
 @param q the event queue
 @param maxEvolveTime the maximum time to evolve the brain before stopping
 @param maxNumSalSpots the maximum number of salient locations extracted before stopping
 @param framenum the frame number 
 sent to the brain 
 *
 *If either @param maxEolveTime or @param maxNumSalSpots
 *conditions are met, this function will return.s
 */
std::list <Winner> getSalientWinners(
        nub::soft_ref<MbariResultViewer> &rv,
        const Image<byte> &clipMask,
        const Image<PixRGB<byte> > &img,
        nub::soft_ref<StdBrain> brain,
        nub::soft_ref<SimEventQueue> q,
        float maxEvolveTime,
        int maxNumSalSpots,
        int framenum);


//! returns the winners in the graph based segmented color bit image
/*!*@param graphBitImg a colorized image for extracting BitObjects;
 * here we use it to extract winners based on colorized values
 */
std::list <WTAwinner> getGraphWinners(const Image<PixRGB<byte> > &graphBitImg,
                                      int framenum);

//! returns the largest BitObjects near the winning locations points
/*!*@param graphBitImg a color coded image used for extracting BitObjects at the
 * winner locations
 @param winners list of  winners*/
std::list <BitObject> getSalientObjects(nub::soft_ref<MbariResultViewer> &rv, const uint frameNum, 
const Image<PixRGB<byte> > &graphBitImg, const std::list <Winner> &winners);

//! returns the FOA BitObjects near the winning locations points
/*!*@param mask user supplied mask for equipment, etc.
 @param winners list of  winners*/
std::list<BitObject> getFOAObjects(const std::list<Winner> &winners, const Image< byte >& mask);

//! returns the largest BitObjects near the most winning locations points
// if bit object cannot be found in graphBitImg or bitImg, returns the foamask image
/*!*@param graphBitImg a color coded image used for extracting BitObjects at the
 * winner locations
 * @param bitImg a binary image used for extracting BitObjects at the winner locations
 @param winners list of Salient winners
 *@param mask user supplied mask for equipment, etc. */
std::list <BitObject> getSalientObjects(nub::soft_ref<MbariResultViewer> &rv, const uint frameNum, 
                                        const Image<PixRGB<byte> > &graphBitImg,
                                        const Image<byte> &bitImg, const std::list <Winner> &winners,
                                        const Image<byte> &mask);

//! Filters the graph winners with the clip Mask removing those that fall within the mask
std::list <Winner> filterGraphWinners(const Image<byte> &clipMask, const std::list <Winner> &winlist);

//! all BitObjects in objs are drawninto the return image
Image<byte> showAllObjects(const std::list <BitObject> &objs);

//! Draw circle around winning point in image and return annotated image
Image<PixRGB<byte> > showAllWinners(const std::list <Winner> winlist, const Image<PixRGB<byte> > &img, int maxDist);

// ! Run mask option: flag the area specified as background after the segmentation
Image<byte> maskArea(const Image<byte> &img, DetectionParameters *parms, byte maskval = byte(0));

// ! flag the area specified as background after the segmentation
Image<byte> maskArea(const Image<byte> &img, const Image<byte> &mask, const byte maskval = byte(0));

// ! Run mask option on a color image: flag the area specified as background after the segmentation
Image<PixRGB<byte> > maskArea(const Image<PixRGB<byte> > &img, DetectionParameters *parms,
                              const byte maskval = byte(0));

// ! flag the area specified as background after the segmentation
Image<PixRGB<byte> > maskArea(const Image<PixRGB<byte> > &img, const Image<byte> &mask,
                              const byte maskval = byte(0));

// ! Return the max value of a matrix
float getMax(const Image<float> matrix);

// ! Return the picture to add to the background Model: take in consideration if it's a fixed camera then
// objects detected are not included into it. Only use this for moving camera background image updates.
Image<PixRGB<byte> > getBackgroundImage(const Image<PixRGB<byte> > &img,
                                        const Image<PixRGB<byte> > &currentBackgroundMean,
                                        Image<PixRGB<byte> > savePreviousPicture,
                                        const std::list <BitObject> &bitObjectFrameList, PixRGB<byte> &avgVal);

// ! Return the image to add to the mask cache
Image<byte> getMaskImage(const Image<byte> &img, const std::list <BitObject> &bitObjectFrameList);

// ! Return largest BitObject with bounding box within @param maxDist and not intersecting with those in the @param bos
BitObject findBestBitObject(Rectangle r1, int maxDist, std::list <BitObject> &sobjs, std::list <BitObject> &bos);

// ! Return combined BitObjects with centroids within @param maxDist and not intersecting with those in the @param bos
BitObject findBestBitObject(Point2D<int> center, int maxDist, std::list <BitObject> &sobjs,
                            std::list <BitObject> &bos);

// ! Return the float parameters
std::vector<float> getFloatParameters(const std::string &str);
#endif
