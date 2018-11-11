/******************************************************************************
 * Author: Martin Godec
 *         godec@icg.tugraz.at
 * Feature calculation from Juergen Gall
 *         gall@vision.ee.ethz.ch
 ******************************************************************************/

#ifndef FEATURES_H_
#define FEATURES_H_

#include "utilities.h"

class HoG {
public:
	HoG();
	~HoG()
	{
		delete[] ptGauss;
	}

	void extractOBin(cv::Mat &Iorient, cv::Mat &Imagn, std::vector<cv::Mat>& out, int off);

private:

	void calcHoGBin(uchar* ptOrient, uchar* ptMagn, int step, double* desc);
	void binning(float v, float w, double* desc, int maxb);

	int bins;
	float binsize;

	int g_w;

	// Gauss as vector
	float* ptGauss;
};

inline void HoG::calcHoGBin(uchar* ptOrient, uchar* ptMagn, int step, double* desc) {
	for(int i=0; i<bins;i++)
		desc[i]=0;

	uchar* ptO = &ptOrient[0];
	uchar* ptM = &ptMagn[0];
	int i=0;
	for(int y=0;y<g_w; ++y, ptO+=step, ptM+=step) {
		for(int x=0;x<g_w; ++x, ++i) {
			binning((float)ptO[x]/binsize, (float)ptM[x] * ptGauss[i], desc, bins);
		}
	}
}

inline void HoG::binning(float v, float w, double* desc, int maxb) {
	int bin1 = int(v);
	int bin2;
	float delta = v-bin1-0.5f;
	if(delta<0) {
		bin2 = bin1 < 1 ? maxb-1 : bin1-1;
		delta = -delta;
	} else
		bin2 = bin1 < maxb-1 ? bin1+1 : 0;
	desc[bin1] += (1-delta)*w;
	desc[bin2] += delta*w;
}

class Features
{
public:
	Features()
	{
        m_numChannels = 16;
		m_cols = 0;
		m_rows = 0;
    };

	~Features()
	{
		clear();
    }

	inline void clear()
	{
		m_channels.clear();
	}

	inline void setImage(cv::Mat& img)
	{
		m_channels.clear();
	    m_channels.resize(m_numChannels);
		m_cols = img.cols;
		m_rows = img.rows;
	    extractFeatureChannels(img, m_channels);
    };

	inline const cv::Mat getChannel(unsigned int idx) const
	{
    	return m_channels.at(idx);
    };

	inline unsigned int getNumChannels() const
	{
	    return m_numChannels;
    };

    inline cv::Size getSize() const
    {
		return cv::Size(m_cols, m_rows);
    };

private:
	unsigned int m_numChannels;
	unsigned int m_cols;
	unsigned int m_rows;
	std::vector<cv::Mat> m_channels;
	HoG hog;

	inline void extractFeatureChannels(const cv::Mat &img, std::vector<cv::Mat >& vImg)
	{
	    // 16 feature channels
	    // 7+9 channels: L, a, b, |I_x|, |I_y|, |I_xx|, |I_yy|, HOGlike features with 9 bins (weighted orientations 5x5 neighborhood)

	    vImg.resize(16);
	    for(unsigned int c=0; c<vImg.size(); ++c) {
			cv::Mat m(img.rows, img.cols, CV_8UC1, cv::Scalar(1));
			vImg[c] = m;
		}

		cv::cvtColor(img, vImg[0], CV_RGB2GRAY);
	    cv::medianBlur(vImg[0], vImg[0], 3);

	    // Temporary images for computing I_x, I_y (Avoid overflow for cvSobel)
	    cv::Mat I_x(img.rows, img.cols, CV_16S, cv::Scalar(1));
	    cv::Mat I_y(img.rows, img.cols, CV_16S, cv::Scalar(1));

	    // |I_x|, |I_y| 
	    cv::Sobel(vImg[0], I_x,CV_16S,1,0,3);
	    cv::Sobel(vImg[0], I_y,CV_16S,0,1,3);

		cv::convertScaleAbs( I_x, vImg[3], 0.25);
	    cv::convertScaleAbs( I_y, vImg[4], 0.25);

	    //#pragma omp sections nowait
	    {
		    //#pragma omp section
		    {

		    short* dataX;
		    short* dataY;
		    uchar* dataZ;
		    int stepX, stepY, stepZ;
		    CvSize size;
		    int x, y;

			IplImage tmp_x = I_x;
			IplImage tmp_y = I_y;
			IplImage tmp_v = vImg[1];
			cvGetRawData( &tmp_x, (uchar**)&dataX, &stepX, &size);
			cvGetRawData( &tmp_y, (uchar**)&dataY, &stepY);
			cvGetRawData( &tmp_v, (uchar**)&dataZ, &stepZ);
		    stepX /= sizeof(dataX[0]);
		    stepY /= sizeof(dataY[0]);
		    stepZ /= sizeof(dataZ[0]);

		    // Orientation of gradients
		    for( y = 0; y < size.height; y++, dataX += stepX, dataY += stepY, dataZ += stepZ  )
			    for( x = 0; x < size.width; x++ ) {
				    // Avoid division by zero
				    float tx = (float)dataX[x] + (0.000001f * sign((float)dataX[x]));
				    // Scaling [-pi/2 pi/2] -> [0 80*pi]
				    dataZ[x]=uchar( ( atan((float)dataY[x]/tx)+3.14159265f/2.0f ) * 80 );
			    }
		    }

		    //#pragma omp section
		    {
			    short* dataX;
			    short* dataY;
			    uchar* dataZ;
			    int stepX, stepY, stepZ;
				CvSize size;
			    int x, y;
				IplImage tmp_x = I_x;
				IplImage tmp_y = I_y;
				IplImage tmp_v = vImg[2];

				cvGetRawData( &tmp_x, (uchar**)&dataX, &stepX, &size);
				cvGetRawData( &tmp_y, (uchar**)&dataY, &stepY);
				cvGetRawData( &tmp_v, (uchar**)&dataZ, &stepZ);
			    stepX /= sizeof(dataX[0]);
			    stepY /= sizeof(dataY[0]);
			    stepZ /= sizeof(dataZ[0]);

			    // Magnitude of gradients
			    for( y = 0; y < size.height; y++, dataX += stepX, dataY += stepY, dataZ += stepZ  )
				    for( x = 0; x < size.width; x++ ) {
					    dataZ[x] = (uchar)( sqrt((float)dataX[x]*(float)dataX[x] + (float)dataY[x]*(float)dataY[x]) );
				    }
			    }

		    //#pragma omp section
		    {
			    // 9-bin HOG feature stored at vImg[7] - vImg[15]
			    hog.extractOBin(vImg[1], vImg[2], vImg, 7);
		    }

		    //#pragma omp section
		    {
			    // |I_xx|, |I_yy|

				cv::Sobel(vImg[0], I_x,CV_16S, 2,0,3);
				cv::convertScaleAbs(I_x, vImg[5], 0.25);

				cv::Sobel(vImg[0], I_y,CV_16S, 0,2,3);
				cv::convertScaleAbs( I_y, vImg[6], 0.25);
		    }

		    //#pragma omp section
		    {
			    // L, a, b
			    cv::Mat img_lab(img.rows,img.cols, CV_8UC3);
				cv::cvtColor( img, img_lab, cv::COLOR_RGB2Lab);
				std::vector<cv::Mat> rgb(3);
				cv::split( img_lab, rgb);
				vImg[0] = rgb[0]; vImg[1] = rgb[1]; vImg[2] = rgb[2];

				cv::medianBlur( vImg[0], vImg[0], 3);
				cv::medianBlur( vImg[1], vImg[1], 3);
				cv::medianBlur( vImg[2], vImg[2], 3);
		    }
	    }

    };


};

#endif //FEATURES_H_
