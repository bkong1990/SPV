#ifndef CIRCULARVOTING_H
#define CIRCULARVOTING_H
#include <vector>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>
//#include "opencv/cv.h"
//#include "opencv/highgui.h"
using namespace cv;
using namespace std;

/** Voting point */
struct VPoint2D {
	short x, y;
	short xc, yc; //< center point 
	short angIndex; //< angle index
	int pos;
	float mag; //< magnitude
};

/** Weighted point */
struct WPoint2D {    

	static int nx, ny;
	WPoint2D(int xx=0, int yy=0, float ww=0)
		: x(xx), y(yy), d(0), w(ww) {}

	static void setImageSize(int xsize, int ysize)
	{  
		nx = xsize;
		ny = ysize;
	}

	void setPos() 
	{  d = x + y*nx; }

	int x, y; //< coordinates
	int d; //< offset position
	float w; //< weight
};

/** An intersection of cone */
//  typedef vector<WPoint2D> ConePlane2D;  
struct ConePlane2D : public vector<WPoint2D> {
	static int nx, ny;

	ConePlane2D(int n, const value_type &v)
		: vector<WPoint2D>(n, v)
	{ }

	ConePlane2D()
		: vector<WPoint2D>()
	{ }

	static void setImageSize(int xsize, int ysize)
	{  
		nx = xsize;
		ny = ysize;
	}

	void setPos()
	{
		for(iterator it = begin(); it!=end(); it++)
			it->setPos();     
	}


	float center_weight;

};

/** \brief A 2D cone */
struct Cone2D : public vector<ConePlane2D> {

	static int nx, ny; //< image size

	Cone2D();
	void setDirection(float xx, float yy);
	static void setImageSize(int xsize, int ysize)
	{  
		nx = xsize;
		ny = ysize;
	}

	void init()
	{
		for(iterator it = begin(); it!=end(); it++) {
			it->setPos();
		}
	}

	void vote(Mat& sum_image, const VPoint2D& vp);
	void vote(float* p, float* pmask, const VPoint2D& vp);

	float dx, dy; //< direction

};

class CircularVoting
{
public:

	void compute( const Mat& img, Mat& grayimg, float reg_sign);

	int computeAngleIndex(float dx, float dy);

	void computeCones(int hmin, int hmax, int radius);

	static const float epsilon;
	static const float pi;
	vector<VPoint2D> _voting_points;

	CircularVoting();
	~CircularVoting();

protected:
private:

	vector<Cone2D> _cones;

	int ntheta;

	double delta_theta;


};

#endif
