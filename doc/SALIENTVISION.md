## Running salientvision

The software that runs the automated visual event detection and tracking is called *salientvision*.
It's a command-line tool that can either take a series of sequential frames, or a single frame.

 
## Examples


###Midwater 
[Example output](doc/img/midwaterresults000003.jpg) 

A small example is available in the data/midwater folder. These images were captured around 300 meters depth from 
an autonomous underwater vehicle at MBARI.  

First uncompress the example with:
 
```bash 
cd data/midwater
tar -vzxf midwaterdata.tar.gz
```
Process first 5 frames, rescaled to 960x540,  save output to events.xml file and allow events less than 50 square pixels
```bash 
docker run -v $PWD:/data aved/salientvision --in=raster:/data/f#.png  --input-frames=1-5@1 --output-frames=1-5@1 \
--out=raster:/data/  --mbari-save-events-xml=/data/events.xml   --mbari-mark-interesting=Outline \
--mbari-min-event-area=50 --rescale-input=960x540
```



To see displayed intermediate results as it's running. This can be useful for debugging and understanding the algorithm.
```bash 
xhost + 127.0.0.1
docker run -e DISPLAY=host.docker.internal:0 -v $PWD:/data aved/salientvision --in=raster:/data/f#.png \
--input-frames=1-5@1 --output-frames=1-5@1 --out=raster:/data/  --mbari-save-events-xml=/data/events.xml \
--mbari-mark-interesting=Outline --mbari-min-event-area=50 --rescale-input=960x540 --mbari-display-results --mbari-rescale-display=640x480
```
  
###Benthic

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
