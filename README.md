#  Automated Visual Event Detection Software for Underwater Images

Software to assist with automated detection and tracking of visually salient events in underwater 
video.  This can also be used for still images. 

This is based on modified version from Dirk Walther's work that originated at a  
Neuromorphic Engineering Workshop in Telluride, CO, USA, with a significant
contribution from [MBARI][http://www.mbari.org] engineers and many student projects.

These are command-line only tools, designed to be run in Docker containers.

# Requirements

- [Docker](https://docs.docker.com/installation/)
- (optional) X Windows for your OS (only needed if using display options, e.g. 
 --display-foa, --mbari-display-results)
    - If you are already using a GUI in your OS, e.g. Ubuntu, Centos, you have X Windows.
    - Mac OSX [XQuartz](https://www.xquartz.org/) and (optional, but recommended) [iTerm2](https://iterm2.com/)
   
-  Permission to use the iLab Neuromorphic Vision C++ Toolkit developed by the University of Southern California (USC) 
and the iLab at USC. See [iLab toolkit](http://ilab.usc.edu/bu/) for details on requesting access. 
 
# Overview 

There are two tools in this repository:

## *salientvision* 
This is a bottom-up algorithm that uses the visual attention model in the  [iLab toolkit](http://ilab.usc.edu/bu/)
to find visually interesting objects, which are referred to as *VisualEvents* in the code.  These *VisualEvents* are 
then tracked across frames using either a Kalman, Nearest-Neighbor, or a combined 
Hough-based deformable object tracker with a Kalman or NearestNeighbor tracker. 
  
This tool is useful for extracting useful information about *VisualEvents* e.g. 
their location, visual salience (in milliVolts), and various other features (e.g. shape, centroid, HOG, or Local Jets).

See [salientvision](doc/SALIENTVISION.md) for more details.
  

## *pascalvocvision* 
This combines PASCAL Visual Object Class (VOC) formatted bounding box classifications that might be
generated with modern convolutional object detectors with the object tracking system in *salientvision*.
This is still in the early stages of development, but showing promise as a combined top-down bottom-up approach.

## Building  

Get the password from http://ilab.usc.edu/toolkit/downloads.shtml for the toolkit, 
then run the Docker build with that password e.g.

```bash
    git clone https://github.com/underh20cv/aved.git
    build.sh <password>
```

## License

[Apache License 2.0](LICENSE)
 
 
## References
 
If you find this useful for your work, please cite:

Edgington, D.R., Cline, D.E., Davis, D., Kerkez, I., and Mariette, J. 2006, ‘Detecting, Tracking and Classifying Animals in Underwater Video’, in MTS/IEEE Oceans 2006 Conference Proceedings, Boston, MA, September, IEEE Press.

Itti, L., C. Koch, and E. Niebur, 1998. ‘A model of saliency-based event visual attention for rapid scene analyses. IEEE Transactions on Pattern Analysis and Machine Intelligence, 20(22): p 1254-1259.
 
