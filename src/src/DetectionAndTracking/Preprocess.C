#include "DetectionAndTracking/Preprocess.H"
#include "Component/OptionManager.H"
#include "Component/ParamClient.H"
#include "Data/MbariMetaData.H"
#include "DetectionAndTracking/MbariFunctions.H"
#include "Data/MbariOpts.H"
#include "Image/ColorOps.H"
#include "Image/MathOps.H"
#include "Image/MbariImage.H"
#include "Image/MorphOps.H"
#include "Image/ShapeOps.H"
#include "Media/MediaOpts.H"
#include "SIFT/Histogram.H"

using namespace std;

// ######################################################################
// Preprocess member definitions:
// ######################################################################

// ######################################################################
Preprocess::Preprocess(OptionManager& mgr,
      const string& descrName,
      const string& tagName)
      : ModelComponent(mgr, descrName, tagName),
      itsFrameSource(&OPT_InputFrameSource, this),
      itsSizeAvgCache(&OPT_MDPsizeAvgCache, this),
      itsMinStdDev(&OPT_MDPminStdDev, this),
      itsMinFrame(0)
{

}

// ######################################################################
Preprocess::~Preprocess()
{  }

// ######################################################################
void Preprocess::start1()
{
}

// ######################################################################
Image< PixRGB<byte> >  Preprocess::contrastEnhance(const Image< PixRGB<byte> >& img)
{
    //if first frame update gamma correction curve
    if (itsAvgCache.size() == 0) {
        itscdfw = updateGammaCurve(img, itspdf, true);
    }
    
    return enhanceImage(img, itscdfw);
}

// ######################################################################
void Preprocess::adjustGamma(Image<byte>&lum, map<int, double> &cdfw, Image<PixHSV<float> >&hsvRes)
 {
  Image<byte>::iterator aptr = lum.beginw();
  Image<PixHSV<float> >::iterator itr = hsvRes.beginw();
  while(itr != hsvRes.endw())
    {
      int valint = (int) (*aptr);
      float gamma = 1 - cdfw[valint];
      // apply gamma in the value only, preserving hue and saturation
      itr->p[2] = pow(itr->p[2], gamma);
      ++aptr;
      ++itr;
    }
}

// ######################################################################
 Image<PixRGB<byte> > Preprocess::enhanceImage(const Image<PixRGB<byte> >& img, map<int, double> &cdfw)
{
    Image<byte> lumImg = luminance(img);
    Image<PixRGB<float> > input = img;
    Image<PixHSV<float> > hsvRes = static_cast< Image<PixHSV<float> > > (input/255);
    adjustGamma(lumImg, cdfw, hsvRes);
    Image< PixRGB<float> > foutput = static_cast< Image<PixRGB<float> > > (hsvRes);
    foutput = foutput * 255.0F;
    Image< PixRGB<byte> > rgbImg = static_cast< Image< PixRGB<int> > >(foutput);

    return rgbImg;
}

// ######################################################################
map<int, double> Preprocess::updateGammaCurve(const Image<PixRGB<byte> >& img, map<int, double> &pdf, bool init)
{
    LINFO("Updating gamma curve");
    map<int, double> pdfw,cdfw;
    float pdfmin = 1.f;
    float pdfmax = 0.f;

    if (init){
        Dims d = img.getDims();
        float ttl = d.w()*d.h();
        Histogram h(luminance(img));
        for(int i=0 ; i< 256; i++) {
            // fast pdf approximation
            pdf[i] = h.getValue(i)/ttl;
            if (pdf[i] < pdfmin)
                pdfmin = pdf[i];
            if (pdf[i] > pdfmax)
                pdfmax = pdf[i];
        }
    }
    else {
        for(int i=0 ; i< 256; i++) {
            if (pdf[i] < pdfmin)
                pdfmin = pdf[i];
            if (pdf[i] > pdfmax)
                pdfmax = pdf[i];
        }
    }

    // fast weighting distribution
    float alpha = 0.25f;
    float sumpdfw = 0.f;
    for(int i=0 ; i<  256; i++) {
        pdfw[i] = pdfmax * pow( (pdf[i] - pdfmin) / (pdfmax - pdfmin) , alpha);
        sumpdfw += pdfw[i];
    }

    // modified cumulative distribution function
    for(int i=0; i< 256; i++)
      for(int k=0; k< i; k++)
        cdfw[i] += pdfw[k]/sumpdfw;

    return cdfw;
}

// ######################################################################
float Preprocess::updateEntropyModel(const Image<PixRGB<byte> >& img, map<int, double> &pdf)
{
    Dims d = img.getDims();
    Histogram h(luminance(img));
    float ttl = d.w()*d.h();

    // fast pdf approximation
    for(int i=0 ; i< 256; i++)
        pdf[i] = h.getValue(i)/ttl;

    // calculate entropy
    float H = 0.f;
    for(int i=0 ; i< 256; i++) {
        if (pdf[i] > 0.F)
            H += pdf[i]*log(pdf[i]);
    }

    return -1*H;
}

// ######################################################################
Image< PixRGB<byte> > Preprocess::background(const Image< PixRGB<byte> >& img, const Image< PixRGB<byte> >& prevImg,
                                            const uint frameNum, const list<BitObject> bitObjectFrameList)
{
    PixRGB<byte> avgVal(0,0,0);
    // return background image which tries to erase all active bit objects
    if (!bitObjectFrameList.empty()){
        Image< PixRGB<byte> > bgndImg = getBackgroundImage(
                img, itsAvgCache.mean(),
                prevImg,
                bitObjectFrameList,avgVal);
                return lowPass5(bgndImg);
    }
    else
        return img;
}

// ######################################################################
MbariImage< PixRGB<byte> > Preprocess::update(const Image< PixRGB<byte> >& img,
                                              const Image< PixRGB<byte> >& prevImg,
                                              const uint frameNum,
                                              const list<BitObject> bitObjectFrameList)
{
    PixRGB<byte> avgVal(0,0,0);
    // update cache with background image only when the cache is completely initialized
    if (!bitObjectFrameList.empty() && frameNum >= itsMinFrame ) {
        Image< PixRGB<byte> > bgndImg = getBackgroundImage(
                img, itsAvgCache.mean(),
                prevImg,
                bitObjectFrameList,avgVal);
        update(bgndImg, frameNum);
    }
    else
        update(img, frameNum);

    // get the MBARI metadata from the frame (if it exists)
    MbariImage< PixRGB<byte> > mbariImg(itsFrameSource.getVal());

    // update data with enhanced image
    mbariImg.updateData(img, frameNum);
    MbariMetaData metadata = mbariImg.getMetaData();
    string tc = metadata.getTC();

    if (tc.length() > 0)
      LINFO("Caching frame %06d timecode: %s", frameNum, tc.c_str());
    else
      LINFO("Caching frame %06d", frameNum);

    return mbariImg;

}

// ######################################################################
void Preprocess::update(const Image< PixRGB<byte> >& img, const uint frameNum, bool updateModel) {

    LINFO("Updating cache for frame %d", frameNum);

    // if user specified minimum standard deviation
    if (itsMinStdDev.getVal() > 0.f) {
      float stddev = stdev(luminance(img));
      LINFO("Standard deviation in frame %d:  %f", frameNum, stddev);

      // get the standard deviation in the input image
      // if there is little deviation do not add to the average cache
      if (stddev <= itsMinStdDev.getVal() && itsAvgCache.size() > 0) {
          LINFO("Standard deviation in frame %d too low. Is this frame all black ? Not including this image in the cache", frameNum);
          itsAvgCache.push_back(itsAvgCache.mean());
      }
      else
          itsAvgCache.push_back(img);
    }
    else
      itsAvgCache.push_back(img);

    // if first frame update gamma correction curve
    if (itsAvgCache.size() == 0) {
        itscdfw = updateGammaCurve(img, itspdf, true);
    }
    else {
        if (updateModel) {
            float entrop = updateEntropyModel(img, itspdf);
            itsPrevEntropy = entrop;
            itscdfw = updateGammaCurve(img, itspdf, false);
        }
    }
}

// ######################################################################
void Preprocess::init(nub::soft_ref<InputFrameSeries> ifs, const Dims scaledDims)
{
    itsPrevEntropy = 0.F;
    FrameRange frameRange = ifs->getFrameRange();
    Image< PixRGB<byte> > img;

    for(int i=0; i < 256; i++) itspdf[i] = 0.F;

    while (itsAvgCache.size() < itsSizeAvgCache.getVal()) {
        if (ifs->frame() >= frameRange.getLast()) {
          LERROR("Less input frames than necessary for sliding average - "
                  "using all the frames for caching.");
          break;
        }
        ifs->updateNext();
        img = rescale(ifs->readRGB(), scaledDims);
        // TODO: add threshold on entropy gamma curve difference and flag true/false accordingly here
        update(img, ifs->frame(), true);

    }
    itsMinFrame = ifs->frame();
}

// ######################################################################
Image< PixRGB<byte> > Preprocess::absDiffMean(Image< PixRGB<byte> >& image)
{
    if (itsAvgCache.size() > 0)
        return itsAvgCache.absDiffMean(image);
    return image;
}

// ######################################################################
Image< PixRGB<byte> > Preprocess::clampedDiffMean(Image< PixRGB<byte> >& image)
{
    if (itsAvgCache.size() > 0)
        return itsAvgCache.clampedDiffMean(image);
    return image;
}

// ######################################################################
Image< PixRGB<byte> > Preprocess::mean()
{
    return itsAvgCache.mean();
}

// ######################################################################
void Preprocess::paramChanged(ModelParamBase* const param,
                                 const bool valueChanged,
                                 ParamClient::ChangeStatus* status)
{
    if (param == &itsSizeAvgCache)
        itsAvgCache.setMaxSize(itsSizeAvgCache.getVal());
}
 

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
