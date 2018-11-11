## Notes on developing
 
## Running with X11/XQuartz
Launch XQuartz and open a terminal window (Mac shortcut for this is <kbd>&#8984;N</kbd>)

In the terminal window, ensure the host is allowing X forwarding 

```bash 
xhost + 127.0.0.1
```

* In XQuarts preferences, click allow connections from network clients
[![ Image link ](img/xquarts_allow.jpg)]
 
* Run docker in the background in a "detached" mode (-d). Expose port 22 on the host machine (-p)  

```bash 
CID=$(docker run -d -p 22 avedac/salientvision)
```

* If you have video to process, share your drive using the volume mount (-v) command. 

*Mac*
```bash 
CID=$(docker run -d -v /Users/dcline/Downloads:/tmp/Downloads -p 22 avedac/salientvision)
```

*Windows*
```bash 
CID=$(docker run -d -v c:\\Users\dcline\Downloads/:/tmp/Downloads -p 22 avedac/salientvision)
```
  
* You should now see the image running in the background. Note that it takes 50-60 seconds to completely boot.
```bash
docker ps
```

* Get the port number
```bash
echo $(docker port $CID 22 | cut -d ':' -f 2)
32768
```

* Using the password "saliency", port, and IP address, ssh directly to the docker container
```bash 
ssh -Y docker@localhost -p 32768 
```

* Set the display once in the container
```bash
export DISPLAY=docker.for.mac.localhost:0
```
or 
```bash
export DISPLAY=host.docker.internal:0
```
 
