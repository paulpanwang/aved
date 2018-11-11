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
#include "Image/OpenCVUtil.H"
#include "Component/ModelManager.H"
#include "Image/CutPaste.H"
#include "Image/ColorOps.H"
#include "Image/FilterOps.H"
#include "Raster/Raster.H"
#include "Learn/Bayes.H"
#include "Learn/FisherLDA.H"
#include "Media/FrameSeries.H"
#include "Util/StringUtil.H"
#include "rutz/rand.h"
#include "rutz/trace.h"

#include <boost/filesystem/operations.hpp>
#include <math.h>
#include <fcntl.h>
#include <limits>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <dirent.h>
#include <sstream>
#include <cstdio>
#include <fstream>

// File/path name manipulations
std::string dirname(const std::string& path)
{
   namespace fs = boost::filesystem ;
   return fs::path(path).branch_path().string() ;
}

// get the basename sans the extension
std::string basename(const std::string& path)
{
   namespace fs = boost::filesystem ;
   return fs::path(path).stem() ;
}

std::vector<std::string> readDir(std::string inName)
{
        DIR *dp = opendir(inName.c_str());
        if(dp == NULL)
        {
          LFATAL("Directory does not exist %s",inName.c_str());
        }
        dirent *dirp;
        std::vector<std::string> fList;
        while ((dirp = readdir(dp)) != NULL ) {
                if (dirp->d_name[0] != '.')
                        fList.push_back(inName + '/' + std::string(dirp->d_name));
        }
        LINFO("%" ZU " files in the directory %s", fList.size(), inName.c_str());
        //LINFO("file list : \n");
        //for (unsigned int i=0; i<fList.size(); i++)
        //                LINFO("\t%s", fList[i].c_str());
        std::sort(fList.begin(),fList.end());
        return fList;
}



int main(const int argc, const char **argv)
{

    MYLOGVERB = LOG_INFO;
    ModelManager manager("Train Bayesian Network");
    
    // Create classifier
    Bayes *bn;

    Dims cellSize = Dims(3,3); // if fixedHist is true, this is hist size, if false, this is cell size
    //8x8 = 1296 features
    //3x3 =  36 features
    // default network
    bn = new Bayes(324, 0);

    if (manager.parseCommandLine(
        (const int)argc, (const char**)argv, "<featurestr> <featuredir> <class1dir> ... <classNdir>", 3, 20) == false)
    return 0;
    
    manager.start();        
    
    int numClasses = manager.numExtraArgs() - 2;
    std::string featureStr = manager.getExtraArg(0);
    std::string featureDir = manager.getExtraArg(1);
    bool init = false;
    int comp = 3;
    int nSample = 0;
    std::vector<int> classTotal;
    std::vector<std::string> files;
    std::vector<std::string> classNames;
    int numFeatures = 0;
    int numSamples = 0;
    MatrixXd XX;

    // HOG_3/HOGMMAP_3 = 36
    // HOG_8/HOGMMAP_8 = 1296
    // MBH_3 = 288
    // MBH_8 = 10368
    for(uint i = 0; i < numClasses; i++) {
        int argIdx = i + 2;
        std::string classDir = manager.getExtraArg(argIdx);
        std::vector <std::string> list = readDir(classDir);
        std::string className = basename(classDir);

        numSamples += list.size();
        classTotal.push_back(list.size());
        classNames.push_back(className);
        files.insert(files.end(), list.begin(), list.end());
    }

    std::vector<std::string>::iterator it = files.begin();

    for(uint i = 0; i < numClasses; i++) {
        int argIdx = i + 2;
        int idx = 0;
        bool classInit = false;
        std::string className = classNames[i];
        std::string classDir = manager.getExtraArg(argIdx);
        int numFiles = classTotal[i];

        LINFO("Training %d files from dir %s classname %s feature %s", numFiles, classDir.c_str(), className.c_str(),
              featureStr.c_str());

        // For each file in a class directory extract the stored features and initialize Bayes network
        for (uint j = 0; j < numFiles; j++) {

            std::string filename = basename(*it++);
            //LINFO("Filename %s", filename.c_str());
            std::ostringstream ss;
            ss << featureDir << "/" << filename << featureStr << ".dat";
            std::string datFilename = ss.str();
            float f = 0;

            if (init == false) {
                delete bn;
                LINFO("Creating Bayes classifier with %d features and %d classes", comp, numClasses);
                bn = new Bayes(comp, 0);
                ifstream file(datFilename.c_str());
                while (file >> f) numFeatures++;
                file.close();
                LINFO("Resizing matrix to %dx%d", comp, numSamples);
                XX.resize(numFeatures, numSamples);
                init = true;
            }

            std::ifstream ifs(datFilename.c_str());
            if (ifs.good()) {
                int k = 0;
                while (1) {
                    ifs >> f;
                    if (ifs.eof() != true)
                        XX(k++, nSample) = f;
                    else
                        break;
                }
                ifs.close();
                nSample++;

                if (classInit == false) {
                    // add class by name and return its Id
                    idx = bn->addClass(className.c_str());
                    classInit = true;
                    LINFO("Adding class %s %d", className.c_str(), idx);
                }
            }
        }
    }

    // Make X zero mean
    LINFO("Normalizing features");

    VectorXd Xmean = XX.rowwise().mean();
    /*cout << "Here is the mean of each row:" << endl << Xmean << endl;*/
    XX.array().colwise() -= Xmean.array();

    // Reduce
    MatrixXd FLD = fisher_l_d(XX, comp, classTotal);
    //std::cout << "Here is the FLD matrix D:\n" << FLD << std::endl;

    XX = (FLD.adjoint()*XX).adjoint();
    std::cout << "Here is the input matrix reduced to " << comp << " dimensions XX:\n" << XX << std::endl;

    // Train
    nSample = 0;
    std::vector<double> feature(comp);
    for(std::vector<int>::size_type classIndx = 0; classIndx != classTotal.size(); classIndx++) {
        int ttl = classTotal[classIndx];
        std::cout << ttl << " samples " << " in class index " << classIndx << std::endl;
        for (int i = 0; i < ttl; i++) {
            for (int j = 0; j < comp; j++) {
                feature[j] = XX(nSample, j);
            }
            nSample++;
            std::cout << "learning class " << classIndx << " sample " << nSample << std::endl;
            bn->learn(feature, classIndx);
        }
    }

    LINFO("Classifier num classes: %d", bn->getNumClasses());

    // dump information about the bayes classifier
    for(uint i = 0; i < bn->getNumFeatures(); i++)
      LINFO("Feature %i: mean %f, stddevSq %f", i, bn->getMean(0, i), bn->getStdevSq(0, i));

    for(uint i = 0; i < bn->getNumClasses(); i++)
      LINFO("Trained class %s", bn->getClassName(i));

    bn->save("bayesLDA.net");
    LINFO("Use the bayes.net file to test the classification performance of this feature vector");
    manager.stop();

}



// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */



