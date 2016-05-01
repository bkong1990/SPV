#include "circularvoting.h"
#include <iostream>
using namespace std;


const float CircularVoting::epsilon(0.001);
const float CircularVoting::pi = 3.1415927;

int WPoint2D::nx = 0;
int WPoint2D::ny = 0;

int Cone2D::nx = 0;
int Cone2D::ny = 0;

int ConePlane2D::nx = 0;
int ConePlane2D::ny = 0;

// 2D cone
Cone2D::Cone2D()  : dx(1), dy(0) { }
void Cone2D::setDirection(float xx, float yy)
{ dx = xx; dy = yy; }

CircularVoting::CircularVoting(){

	ntheta = 256;
	delta_theta = 2*pi/ntheta;
}

void Cone2D::vote(Mat& sum_image, const VPoint2D& vp)
{
	iterator from = begin();
	iterator to = end();     
	for(iterator it = from; it != to; it++) 
	{
		for(value_type::iterator it1=it->begin(); it1!=it->end(); it1++) {  
			double *p = sum_image.ptr<double>(vp.y + it1->y);
			p[vp.x + it1->x] += vp.mag * it1->w;
			//p[vp.x] += it1->w;
		}
	}
}

CircularVoting::~CircularVoting()
{ }

int CircularVoting::computeAngleIndex(float dx, float dy)
{
	//int ntheta = 256;
	//double delta_theta = 2*CircularVoting::pi/ntheta;
	float a = acos(dx);
	if (dy<0) {
		a = 2*CircularVoting::pi-a;
	}

	int indx_theta = int(a/delta_theta + 0.5);
	return max(0, min(ntheta-1, indx_theta));
}

void CircularVoting::computeCones(int hmin, int hmax, int radius)
{
	_cones = vector<Cone2D>(ntheta);

	Cone2D cone;
	cone.reserve(2*hmax);

	cone.setDirection(1, 0);

	for(int x=hmin; x<=hmax; x++) {
		cone.push_back(ConePlane2D(1, WPoint2D(x, 0, 1)));
	}

	for(int x=hmin; x<=hmax; x++) {
		//int rmax = round_double(x * radius / hmax);
		int rmax = int(0.5 + x * radius / hmax);
		for(int y=-rmax; y<=rmax; y++)   if (y!=0) {

			float R = sqrt((double)(x*x+y*y));
			float a = fabs((float)x)/R;
			double center_x = (hmin+hmax)/2;
			double center_y = 0;
			double sigma = 3;

			WPoint2D wp;
			wp.x = x; 
			wp.y = y; 
			//wp.w = 1;// a;
			wp.w = exp(-((x-center_x)*(x-center_x) + (y-center_y)*(y-center_y))/2/sigma/sigma);
			int n = int(0.5 + R) - hmin;
			if (n>=0 && n<cone.size())
				cone[n].push_back(wp);
		}

	}
//	cout << cone[n].size() << endl;

	//if ( true) {

	//	Cone2D cone1;
	//	cone1.setDirection(1, 0);

	//	for(int x=hmin; x<=hmax; x++) {
	//		cone1.push_back(ConePlane2D(1, WPoint2D(-x, 0, 1)));
	//	}

	//	for(int x=hmin; x<=hmax; x++) {

	//		int rmax = int(0.5+x * radius / hmax);
	//		for(int y=-rmax; y<=rmax; y++)   if (y!=0) {

	//			float R = sqrt((double)(x*x+y*y));
	//			float a = fabs((float)x)/R;

	//			WPoint2D wp;
	//			wp.x = -x; 
	//			wp.y = y; 
	//			wp.w = 1;// a;
	//			int n = int(0.5+R) - hmin;
	//			if (n>=0 && n<cone1.size())
	//				cone1[n].push_back(wp);
	//		}
	//	}

	//	for(int i=0; i<cone1.size(); i++)
	//		cone.push_back(cone1[i]);
	//}


	cone.init();

	for(int i=0; i<ntheta; i++) {

		float theta = i*delta_theta;

		Cone2D rcone = cone; // rotated cone

		const float cos_theta = cos(theta);
		const float sin_theta = sin(theta);

		for(Cone2D::iterator it1=rcone.begin(); it1!=rcone.end(); it1++)
			for(ConePlane2D::iterator it2=it1->begin(); 
				it2!=it1->end(); it2++) {
					int x = it2->x;
					int y = it2->y;

					it2->x = int(0.5 + cos_theta * x - sin_theta * y );
					it2->y = int(0.5 + sin_theta * x + cos_theta * y );

			}

			rcone.setDirection(cos_theta,  sin_theta);
			rcone.init();
			_cones[i] = rcone;      

	}
}

void CircularVoting::compute( const Mat& img, Mat& out_img, float reg_sign)
{
	Mat grayimg;
	cvtColor(img, grayimg, CV_BGR2GRAY);
	GaussianBlur(grayimg, grayimg, Size(9,9), 0,0);


	/// Generate grad_x and grad_y
	Mat grad_x, grad_y, grad;
	//Mat abs_grad_x, abs_grad_y, grad;

	/// Gradient X
	//Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
	Sobel( grayimg, grad_x, CV_64FC1, 1, 0, 3, 1, 0, BORDER_DEFAULT );


	//convertScaleAbs( grad_x, abs_grad_x );
	/// Gradient Y
	//Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
	Sobel( grayimg, grad_y, CV_64FC1, 0, 1, 3, 1, 0, BORDER_DEFAULT );
	//convertScaleAbs( grad_y, abs_grad_y );


	double* p_x, *p_y, *p_g;
	int nRows = img.rows;
	int nCols = img.cols;
	double temp_grad;
	grad = grad_x.clone();
	double max_grad = 0;
	for(int i = 0; i < nRows; ++i)
	{
		p_x = grad_x.ptr<double>(i);
		p_y = grad_y.ptr<double>(i);
		p_g = grad.ptr<double>(i);
		for (int j = 0; j < nCols; ++j)
		{
			p_x[j] *= reg_sign;
			p_y[j] *= reg_sign;
			temp_grad = sqrt(p_x[j]*p_x[j] + p_y[j]*p_y[j]);
			p_g[j] = temp_grad;
			//cout << p_x[j] << '\t' << p_y[j] << '\t' << endl;;

			if (temp_grad>epsilon)
			{
				p_x[j] /= temp_grad;
				p_y[j] /= temp_grad;
			}

			if (max_grad < temp_grad)
			{
				max_grad = temp_grad;
			}
		}//cout << endl;
	}

	if (max_grad > epsilon)
	{
		for(int i = 0; i < nRows; ++i)
		{
			p_g = grad.ptr<double>(i);
			for (int j = 0; j < nCols; ++j)
			{
				if (p_g[j] < 50)
				{
					p_g[j] = 0;
				}
			}
		}
	}

	double diameter = 20;
	double hmax = 1.5 * diameter;
	double hmin = 0.1 * diameter;
	int radius = 40;
	int bw = sqrt((double)(radius*radius + hmax*hmax)) + 3;
	const int bw2 = 2*bw;

	// Initialize structure for storing voting points
	int npix = nRows * nCols;
	_voting_points = vector<VPoint2D>();
	_voting_points.reserve(npix/2);
	VPoint2D vp;
	int indx;

	// push every large gradient into vector and add offset to avoid border problems
	for(int i = 0; i < nRows; ++i)
	{
		p_x = grad_x.ptr<double>(i);
		p_y = grad_y.ptr<double>(i);
		p_g = grad.ptr<double>(i);
		for (int j = 0; j < nCols; ++j)
		{
			if (p_g[j] > epsilon)
			{
				if ((indx=computeAngleIndex(p_x[j], p_y[j])) >= 0 ) 
				{
					vp.x = j + bw;
					vp.y = i + bw;
					vp.mag = p_g[j]; 
					vp.angIndex = indx;
					_voting_points.push_back(vp);
				}
			}
		}
	}

	WPoint2D::setImageSize(nCols + bw2, nRows + bw2);
	ConePlane2D::setImageSize(nCols + bw2, nRows + bw2);
	Cone2D::setImageSize(nCols + bw2, nRows + bw2);
	// Vote
	computeCones(hmin, hmax, radius);
	//for (int i = 0; i < _cones.size(); ++i)
	//{
	//	Cone2D cone = _cones[i];
	//	for (int j = 0; j < cone.size(); ++j)
	//	{
	//		cout << cone[j].nx << '\t';
	//	}

	//	cout << endl;
	//}

	vector<VPoint2D>::iterator voting_points_begin = _voting_points.begin();
	vector<VPoint2D>::iterator voting_points_end = _voting_points.end();
	//for(vector<VPoint2D>::iterator it=voting_points_begin; 
	//	it!=voting_points_end; it++) {	
	//		it->pos = 0;
	//}


	Mat sum_image(nRows + bw2, nCols + bw2, CV_64FC1, Scalar(0));

	for(vector<VPoint2D>::iterator it=voting_points_begin; 
		it!=voting_points_end; it++) {
			//cout << it->x << '\t' << it->y << endl;
			//_cones[it->angIndex].vote(psum+it->pos, *it);
			_cones[it->angIndex].vote(sum_image, *it);
	}

	sum_image(Rect(bw,bw,nCols,nRows)).copyTo(out_img);

	/*for (int i = 0; i < _voting_points.size(); ++i){
		cout << _voting_points[i].x << '\t' << _voting_points[i].y << endl;
	}*/

	/*
	/// Total Gradient (approximate)
	Mat abs_img;
	convertScaleAbs( grad, abs_img);
	//addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

	imshow( "Gradient", abs_img );
	waitKey(0);
	*/


}
