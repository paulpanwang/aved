#include <numeric>
#include <list>
#include <vector>
#include <iostream>

#include "Image/OpenCVUtil.H"
#include "Component/ModelManager.H"
#include "Image/Image.H"
#include "Learn/Bayes.H"
#include "Learn/FisherLDA.H"

using namespace Eigen;
using namespace std;

int main()
{

    int numClasses = 2;
    int ttlFeatures = 4;
    int comp = 3;
    std::vector<int> numSamples;
    int n;
    MatrixXd XX(ttlFeatures, 10 * numClasses);

    numSamples.push_back(10);
    numSamples.push_back(10);

    Bayes bn(comp, numClasses); //construct a bayes network with comp features and 2 classes
    std::vector<double> FV(ttlFeatures), FV1(ttlFeatures), FV_FINAL(comp);

    //Class 0
    n = 0;
    FV[0] = 752; FV[1] = 265; FV[2] = 700; FV[3] = 271; //bn.learn(FV, 0u);
    for (int j=0; j < ttlFeatures; j++) XX(j, n) = FV[j];
    n++;
    FV[0] = 895; FV[1] = 355; FV[2] = 812; FV[3] = 288; //bn.learn(FV, 0u);
    for (int j=0; j < ttlFeatures; j++) XX(j, n) = FV[j];
    n++;
    FV[0] = 893; FV[1] = 352; FV[2] = 790; FV[3] = 298; //bn.learn(FV, 0u);
    for (int j=0; j < ttlFeatures; j++) XX(j, n) = FV[j];
    n++;
    FV[0] = 814; FV[1] = 326; FV[2] = 790; FV[3] = 296; //bn.learn(FV, 0u);
    for (int j=0; j < ttlFeatures; j++) XX(j, n) = FV[j];
    n++;
    FV[0] = 532; FV[1] = 405; FV[2] = 750; FV[3] = 401; //bn.learn(FV, 0u);
    for (int j=0; j < ttlFeatures; j++) XX(j, n) = FV[j];
    n++;
    FV[0] = 532; FV[1] = 405; FV[2] = 750; FV[3] = 401; //bn.learn(FV, 0u);
    for (int j=0; j < ttlFeatures; j++) XX(j, n) = FV[j];
    n++;
    FV[0] = 478; FV[1] = 385; FV[2] = 750; FV[3] = 394; //bn.learn(FV, 0u);
    for (int j=0; j < ttlFeatures; j++) XX(j, n) = FV[j];
    n++;
    FV[0] = 532; FV[1] = 405; FV[2] = 750; FV[3] = 401; //bn.learn(FV, 0u);
    for (int j=0; j < ttlFeatures; j++) XX(j, n) = FV[j];
    n++;
    FV[0] = 565; FV[1] = 47 ; FV[2] = 710; FV[3] = 142; //bn.learn(FV, 0u);
    for (int j=0; j < ttlFeatures; j++) XX(j, n) = FV[j];
    n++;
    FV[0] = 689; FV[1] = 127; FV[2] = 955; FV[3] = 162; //bn.learn(FV, 0u);
    for (int j=0; j < ttlFeatures; j++) XX(j, n) = FV[j];
    n++;


    //Class 1
    FV1[0] = 576; FV1[1] = 726; FV1[2] = 287; FV1[3] =719; //bn.learn(FV, 1);
    for (int j=0; j < ttlFeatures; j++) XX(j, n) = FV[j];
    n++;
    FV1[0] = 718; FV1[1] = 783; FV1[2] = 300; FV1[3] =536; //bn.learn(FV, 1);
    for (int j=0; j < ttlFeatures; j++) XX(j, n) = FV[j];
    n++;
    FV1[0] = 859; FV1[1] = 724; FV1[2] = 270; FV1[3] =480; //bn.learn(FV, 1);
    for (int j=0; j < ttlFeatures; j++) XX(j, n) = FV[j];
    n++;
    FV1[0] = 839; FV1[1] = 512; FV1[2] = 246; FV1[3] =657; //bn.learn(FV, 1);
    for (int j=0; j < ttlFeatures; j++) XX(j, n) = FV[j];
    n++;
    FV1[0] = 746; FV1[1] = 343; FV1[2] = 250; FV1[3] =710; //bn.learn(FV, 1);
    for (int j=0; j < ttlFeatures; j++) XX(j, n) = FV[j];
    n++;
    FV1[0] = 660; FV1[1] = 527; FV1[2] = 272; FV1[3] =763; //bn.learn(FV, 1);
    for (int j=0; j < ttlFeatures; j++) XX(j, n) = FV[j];
    n++;
    FV1[0] = 704; FV1[1] = 621; FV1[2] = 263; FV1[3] =713; //bn.learn(FV, 1);
    for (int j=0; j < ttlFeatures; j++) XX(j, n) = FV[j];
    n++;
    FV1[0] = 684; FV1[1] = 836; FV1[2] = 287; FV1[3] =213; //bn.learn(FV, 1);
    for (int j=0; j < ttlFeatures; j++) XX(j, n) = FV[j];
    n++;
    FV1[0] = 678; FV1[1] = 800; FV1[2] = 377; FV1[3] =220; //bn.learn(FV, 1);
    for (int j=0; j < ttlFeatures; j++) XX(j, n) = FV[j];
    n++;
    FV1[0] = 624; FV1[1] = 697; FV1[2] = 494; FV1[3] =238; //bn.learn(FV, 1);
    for (int j=0; j < ttlFeatures; j++) XX(j, n) = FV[j];
    n++;

    std::cout << "Here is the input matrix XX:\n" << XX << std::endl;

    MatrixXd FLD = fisher_l_d(XX, comp, numSamples);
    std::cout << "Here is the FLD matrix D:\n" << FLD << std::endl;

    XX = (FLD.adjoint()*XX).adjoint();
    std::cout << "Here is the input matrix reduced to " << comp << " dimensions XX:\n" << XX << std::endl;

    n = 0;
    for(std::vector<int>::size_type classIndx = 0; classIndx != numSamples.size(); classIndx++) {
        int num_samples = numSamples[classIndx];
        std::cout << num_samples << "samples " << " in class index " << classIndx << std::endl;
        for (int i = 0; i < num_samples; i++) {
            for (int j = 0; j < comp; j++) {
                FV_FINAL[j] = XX(n, j);
            }
            n++;
            std::cout << "learning class " << classIndx << " sample " << n << std::endl;
            bn.learn(FV_FINAL, classIndx);
        }
    }

    LINFO("Class 0");
    for(uint i=0; i<bn.getNumFeatures(); i++)
        LINFO("Feature %i: mean %f, stddevSq %f", i, bn.getMean(0, i), bn.getStdevSq(0, i));

    LINFO("Class 1");
    for(uint i=0; i<bn.getNumFeatures(); i++)
        LINFO("Feature %i: mean %f, stddevSq %f", i, bn.getMean(1, i), bn.getStdevSq(1, i));

    LINFO("Class 0 frq %i prob %f", bn.getClassFreq(0), bn.getClassProb(0));
    LINFO("Class 1 frq %i prob %f", bn.getClassFreq(1), bn.getClassProb(1));


    //New FV to classify
    FV[0] = 750; FV[1] = 269; FV[2] = 720; FV[3] = 291;
    int cls = bn.classify(FV); //classify a given FV
    LINFO("FV1 belongs to class %i", cls);

    FV[0] = 458; FV[1] = 381; FV[2] = 350; FV[3] = 392;
    cls = bn.classify(FV); //classify a given FV
    LINFO("FV2 belongs to class %i", cls);


    bn.save("Bayes.net");

    bn.load("Bayes.net");

    LINFO("Class 0");
    for(uint i=0; i<bn.getNumFeatures(); i++)
        LINFO("Feature %i: mean %f, stddevSq %f", i, bn.getMean(0, i), bn.getStdevSq(0, i));

    LINFO("Class 1");
    for(uint i=0; i<bn.getNumFeatures(); i++)
        LINFO("Feature %i: mean %f, stddevSq %f", i, bn.getMean(1, i), bn.getStdevSq(1, i));

    LINFO("Class 0 frq %i prob %f", bn.getClassFreq(0), bn.getClassProb(0));
    LINFO("Class 1 frq %i prob %f", bn.getClassFreq(1), bn.getClassProb(1));


    //New FV to classify
    FV[0] = 750; FV[1] = 269; FV[2] = 720; FV[3] = 291;
    cls = bn.classify(FV); //classify a given FV
    LINFO("FV1 belongs to class %i", cls);

    FV[0] = 458; FV[1] = 381; FV[2] = 350; FV[3] = 392;
    cls = bn.classify(FV); //classify a given FV
    LINFO("FV2 belongs to class %i", cls);
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
