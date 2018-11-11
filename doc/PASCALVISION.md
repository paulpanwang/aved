## Running pascalvocvision

This combines PASCAL Visual Object Class (VOC) formatted bounding box classifications with the object tracking system 
in *salientvision*. This is still in the early stages of development, but showing promise as a combined top-down bottom-up approach.

## Requirements
*  Data must be in XML files formatted in the same manner as the examples provided in midwater and benthic examples.
*  Currently there is no support to separate the XML files from the image input; the input images and XML must 
reside in the same directory

## Examples


### Midwater 

![Midwater results](img/pascalvocvisionresults000001.jpg)

A small example is available in the data/midwater folder. These images were captured around 300 meters depth from 
an autonomous underwater vehicle at MBARI.  

First uncompress the example with:
 
```bash 
cd data/midwater
tar -vzxf midwaterdata.tar.gz
```

Run with
```bash

docker run -e DISPLAY=host.docker.internal:0 -v $PWD:/data aved/pascalvocvision  \
--in=raster:/data/f#.png --input-frames=1-5@1 --output-frames=1-5@1 --out=raster:/data/  \
 --mbari-save-events-xml=/data/events.xml --mbari-display-results  --mbari-mark-interesting=Outline \
 --logverb=Info --rescale-input=960x540 --mbari-save-output --mbari-save-results

```
