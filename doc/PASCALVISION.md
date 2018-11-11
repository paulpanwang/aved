## Running salientvision

This combines PASCAL Visual Object Class (VOC) formatted bounding box classifications with the object tracking system 
in *salientvision*. This is still in the early stages of development, but showing promise as a combined top-down bottom-up approach.

## Requirements
*  Data must be in XML files formatted in the same manner as the examples provided in midwater and benthic examples.
*  Currently there is no support to separate the XML files from the image input; the input images and XML must 
reside in the same directory

## Examples


###Midwater 

A small example is available in the data/midwater folder. These images were captured around 300 meters depth from 
an autonomous underwater vehicle at MBARI.  

First uncompress the example with:
 
```bash 
cd data/midwater
tar -vzxf midwaterdata.tar.gz
```
Process first 5 frames, saving output to events.xml file, contrast enhance the images and use those contrast enhanced
images for segmenting
```bash 
docker run -v $PWD:/data avedac/salientvision /usr/local/bin/salientvision --in=raster:/data/f#.png \
--input-frames=1-5@1 --output-frames=1-5@1 --out=raster:/data/  --mbari-save-events-xml=/data/events.xml \
--mbari-contrast-enhance-results=true --mbari-segment-algorithm-input-image=ContrastEnhance
```

To see displayed intermediate results as it's running. This can be useful for debugging and understanding the algorithm.
```bash 
host + 127.0.0.1
docker run -e DISPLAY=host.docker.internal:0 -v $PWD:/data avedac/salientvision /usr/local/bin/salientvision \
--in=raster:/data/f#.png --input-frames=1-5@1 --output-frames=1-5@1 --out=raster:/data/  \
--mbari-save-events-xml=/data/events.xml --mbari-midwater-video --mbari-display-results \
--mbari-rescale-display=640x480
```
  
###Benthic


There are many options in salientvision, some simple like setting the minimum and maximum allowable event areas.  See more [here](OPTIONS.md), or display the options with:

 ```bash 
docker run avedac/salientvision --help
```
