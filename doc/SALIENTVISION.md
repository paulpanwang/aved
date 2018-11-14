## Running salientvision

The software that runs the automated visual event detection and tracking is called *salientvision*.
It's a command-line tool that can either take a series of sequential frames, or a single frame.

 
## Examples


### Midwater 
![Midwater results](img/midwaterresults000003.jpg)

A small example is available in the data/midwater folder. These images were extracted a video clip recorded
at approximately 300 meters depth from an autonomous underwater vehicle.  

First uncompress the example with:
 
```bash 
cd data/midwater
tar -vzxf midwaterdata.tar.gz
```
Process first 5 frames, run saliency detection very frame on rescaled 960x540 images, and save the output to events.xml file.
Set minimum area to 50 square pixels
```bash 
docker run -v $PWD:/data aved/salientvision --in=raster:/data/f#.png  --input-frames=1-5@1 --output-frames=1-5@1 \
--out=raster:/data/  --mbari-save-events-xml=/data/events.xml --mbari-saliency-dist=1 --mbari-mark-interesting=Outline \
--mbari-min-event-area=50 --rescale-input=960x540 

#!/bin/bash
#export DISPLAY=localhost:0.0
#xhost + localhost
#DISPLAY=docker.for.mac.localhost:0
#DISPLAY=host.docker.internal:0
#DISPLAY=localhost:0

docker run -e DISPLAY=docker.for.mac.localhost:0 -v $PWD:/data aved/salientvision /usr/local/bin/salientvision \
--in=raster:/data/f#.png --input-frames=1000-1010@1 --output-frames=1000-1010@1 --out=raster:/data/  \
 --mbari-save-events-xml=/data/events.xml --mbari-display-results --mbari-save-output \
 --mbari-saliency-dist=0  \
 --mbari-mark-interesting=Outline \
 --mbari-remove-overlapping-detections=true \
 --mbari-mask-lasers \
 --logverb=Info  \
 --mbari-tracking-mode=KalmanFilter  \
 --rescale-input=960x540 \


```



To see displayed intermediate results as it's running. This can be useful for debugging and understanding the algorithm.
```bash 
xhost + 127.0.0.1
docker run -e DISPLAY=host.docker.internal:0 -v $PWD:/data aved/salientvision --in=raster:/data/f#.png \
--input-frames=1-5@1 --output-frames=1-5@1 --out=raster:/data/  --mbari-save-events-xml=/data/events.xml \
--mbari-mark-interesting=Outline --mbari-min-event-area=50 --rescale-input=960x540 --mbari-display-results --mbari-rescale-display=640x480
```
  
### Benthic

![Benthic results](img/benthicresults001010.jpg)
```bash 
cd data/benthic
tar -vzxf benthicdata.tar.gz
```
Process first 5 frames running saliency every 2 frames, starting at 1000, rescaled to 960x540,  save output to events.xml file 
```bash 
docker run -v $PWD:/data aved/salientvision --in=raster:/data/f#.png  --input-frames=1001-1010@1 \
--output-frames=1001-1010@1 --out=raster:/data/  --mbari-save-events-xml=/data/events.xml  --rescale-input=960x540
```

To see displayed intermediate results as it's running. This can be useful for debugging and understanding the algorithm.
```bash 
docker run -e DISPLAY=host.docker.interna:0  -v $PWD:/data aved/salientvision --in=raster:/data/f#.png  --input-frames=1001-1010@1 \
--output-frames=1001-1010@1 --out=raster:/data/  --mbari-save-events-xml=/data/events.xml -rescale-input=960x540
``` 

There are many [options](doc/SALIENTVISIONOPTS.md) in salientvision, some simple like setting the minimum and maximum allowable event areas.  See more [here](OPTIONS.md), or display the options with:

 ```bash 
docker run aved/salientvision --help
```
