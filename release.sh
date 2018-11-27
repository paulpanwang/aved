#!/usr/bin/env bash
set -ex
# ensure we're up to date
git pull
# bump version
docker run --rm -v "$PWD":/app treeder/bump patch
version=`cat VERSION`
echo "version: $version"
# run build
./build.sh
# tag it
git add -A
git commit -m "version $version"
git tag -a "$version" -m "version $version"
git push
git push --tags 
# push it
docker push aved/salientvision:$version
docker push aved/pascalvocvision:$version
build.sh
