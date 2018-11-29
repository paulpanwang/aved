#!/bin/bash
if [ -z "$1" ]; then
    echo "Missing password argument, e.g. ./build.sh <saliency password>"
    exit -1 
fi
if [ ! -d $PWD/saliency ]; then
    
    # Suppress warning about storing unencrypted password that prompts yes/no answer
    mkdir ~/.subversion
    echo "[global]" > ~/.subversion/config
    echo "store-plaintext-passwords=off" >> ~/.subversion/config

    # Check out saliency 
    svn checkout --username anonsvn --password $1 svn://isvn.usc.edu/software/invt/trunk/saliency 
fi

# Build everything
version=`cat VERSION`
docker-compose build --build-arg VERSION=$version
