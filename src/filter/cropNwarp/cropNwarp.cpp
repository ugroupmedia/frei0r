
#include "frei0r.hpp"

#include <math.h>
// Other includes used for the examples below.
// Can be removed on copy/paste.

// Limits (min/max values) of various data types
//#include <limits>
// For the CHAR_BIT constant
//#include <climits>
// pow() and other mathematical functions
//#include <cmath>

#include <opencv2/opencv.hpp>
//#include <opencv/highgui.h>
//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

#include <iostream>

using namespace cv;
using namespace std;

int method[5] = {INTER_NEAREST, INTER_LINEAR, INTER_AREA, INTER_CUBIC, INTER_LANCZOS4};

Scalar okScalar = Scalar( 0, 255, 255, 255 );
Scalar errScalar = Scalar( 255, 0, 0, 255 );

/**
The CMakeLists.txt needs to be adjusted as well (both in this directory and in the parent directory).
Also, don't forget src/Makefile.am.
*/

#define LINEWIDTH 2


class cropNwarp : public frei0r::filter
{

public:
    f0r_param_double  x1;
    f0r_param_double  y1;
    f0r_param_double  x2;
    f0r_param_double  y2;
    f0r_param_double  x3;
    f0r_param_double  y3;
    f0r_param_double  x4;
    f0r_param_double  y4;

    f0r_param_double  smooth;

    f0r_param_double  left;
    f0r_param_double  top;
    f0r_param_double  right;
    f0r_param_double  bottom;

    f0r_param_double  warp;
    f0r_param_double  istatic;

    f0r_param_double  feather;
    f0r_param_double  trans; // if true, input contains modulated alpha

    f0r_param_double  uniqueID; // 
 

    cropNwarp(unsigned int width, unsigned int height)
    { 

	//cout << "cropNwarp ctor" << endl;


	W = width;
	H = height;
 
	maxx = (double) (W - 1);
	maxy = (double) (H - 1);

	x1=0.33333;
	y1=0.33333;
	x2=0.66666;
	y2=0.33333;
	x3=0.66666;
	y3=0.66666;
	x4=0.33333;
	y4=0.66666;
	feather=0.0;
	trans=0.0;
	warp=0.0;
	smooth=0.0;
	istatic=1.0;
	uniqueID=0.0;

	left=0.0;
	top=0.0;
	right=1.0;
	bottom=1.0;

        register_param(x1, "x1", "upper left X");	// 0
        register_param(y1, "y1", "upper left Y");	// 1
        register_param(x2, "x2", "upper right X");	// 2
        register_param(y2, "y2", "upper right Y");	// 3
        register_param(x3, "x3", "lower right X");	// 4
        register_param(y3, "y3", "lower right Y");	// 5
        register_param(x4, "x4", "lower left X");	// 6
        register_param(y4, "y4", "lower left Y");	// 7

        register_param(smooth, "Smooth", "smoothing method"); // 8

        register_param(left, "left", "left X");		// 9
        register_param(top, "top", "top Y");		// 10
        register_param(right, "right", "right X");	// 11
        register_param(bottom, "bottom", "bottom Y");	// 12

        register_param(warp, "Warp", "warp active");	// 13
        register_param(istatic, "Static", "input is a PNG"); //14

        register_param(feather, "Feather", "amount of edge alpha feather"); //15
        register_param(trans, "Transparency", "Use input alpha channel"); //16

        register_param(uniqueID, "cropNwarpID", "Unique ID number"); //17

        lambda = Mat( 2, 4, CV_32FC1 );

        destinationQuad[0] = Point2f( 0.0, 0.0 );
        destinationQuad[1] = Point2f( (float) maxx, 0.0);
        destinationQuad[2] = Point2f( (float) maxx, (float) maxy);
        destinationQuad[3] = Point2f( 0.0, (float) maxy );

        isize = W * H * sizeof(uint32_t);

	region_coord_err = false;

	FIRST = true;

	osmooth = -1.0;

	ofeather = -1.0;
	otrans = -1.0;

	ua = (unsigned char *) 	malloc((W/2) * sizeof(unsigned char));	// even with region same size as inframe, the smallest dim / 3.0 will be smaller than W/2 
	fa = (double *) 	malloc((W/2) * sizeof(double));
    }

    ~cropNwarp()
    {
	//cout << "cropNwarp dtor" << endl;


        // Delete member variables if necessary.
        input.release();
        output.release();
        region.release();
        lambda.release();

	free( ua );
	free( fa );
    }
    
    virtual void update_feather_arrays(Mat& mat, double f, int& r)
    {
	int rows, cols, x, y, rr;
	double af, fr;

	double a;

	if( f < 0.01 )
	{
		r = 0;
		return;
	}

	rows = mat.rows;
	cols = mat.cols;

	if(cols > rows)	// use smallest dimension to prevent overflow
	{
		a = ((double) rows / 3.0) * f;	// limit rows/cols to smallest dim / 3.0	
	}
	else 
	{
		a = ((double) cols / 3.0) * f;		
	}


	if(a < 1.0)
	{
		r = 0;
		return;
	}

	af = 1.0 / a;

	r = (int) a;

	for(int i = 0; i < r; i++)
	{
		a = (double) (i) * af;
		ua[i] = 255 * a;
		fa[i] = a;
	} 
    }

    virtual void do_feather(Mat& mat, bool transparency)
    {
	int rows, cols, x, y;

	if( r < 1 )
		return;

	rows = mat.rows - 1;
	cols = mat.cols - 1;

	if( transparency == false )
	{
		unsigned char a;
		for(y = 0; y < r; y++)	// <=
		{
			a = ua[y];
			for(x = y; x <= cols-y; x++)
			{
				mat.at<Vec4b>(y, x)[3] = a;
				mat.at<Vec4b>(rows-y, x)[3] = a;
			}
		}
		for(x = 0; x < r; x++)
		{
			a = ua[x];
			for(y = x; y <= rows-x; y++)
			{
				mat.at<Vec4b>(y, x)[3] = a;
				mat.at<Vec4b>(y, cols-x)[3] = a;
			}
		}
	}
	else
	{
		double a;
		for(y = 0; y < r; y++)	// <=
		{
			a = fa[y];
			for(x = y; x <= cols-y; x++)
			{
				mat.at<Vec4b>(y, x)[3] *= a;
				mat.at<Vec4b>(rows-y, x)[3] *= a;
			}
		}
		for(x = 0; x < r; x++)
		{
			a = fa[x];
			for(y = x+1; y <= rows-x; y++)
			{
				mat.at<Vec4b>(y, x)[3] *= a;
				mat.at<Vec4b>(y, cols-x)[3] *= a;
			}
		}
	}
    }


    virtual void update(double time, uint32_t* out, const uint32_t* in)
    {
        double mx1;
        double mx2;
        double mx3;
        double mx4;
        double my1;
        double my2;
        double my3;
        double my4;
        
        int method_index;

	double mID;

	Scalar color = okScalar;

	feather_changed = false;
	trans_changed = false;

//	cout << "cropNwarp update" << endl;

	if( FIRST )	// first invocation (probably editing...)
	{
		mleft =   (left * maxx);
		mtop =    (top * maxy);
		mright =  (right * maxx);
		mbottom = (bottom * maxy);

		mwidth =  mright-mleft;
		mheight = mbottom-mtop;

		input = Mat(Size(W,H), CV_8UC4, (unsigned char *) in, W*4);
		output = Mat(Size(W,H), CV_8UC4, (unsigned char *) out, W*4);

	        method_index = (int) round(smooth * 4.0); // compatible with c0rners
	        if(method_index < 0 || method_index > 4)
		       method_index=0;

	        METHOD = method[method_index];

		osmooth = smooth;
		otrans = trans;
		ofeather = feather;

		region_coord_err = false;

		ox1 = x1;
		oy1 = y1;
		ox2 = x2;
		oy2 = y2;
		ox3 = x3;
		oy3 = y3;
		ox4 = x4;
		oy4 = y4;
		// must do this math if FIRST, compatible with c0rners		
	       	mx1 = (((x1 * 3.0) - 1.0) * maxx);
	       	my1 = (((y1 * 3.0) - 1.0) * maxy);
	       	mx2 = (((x2 * 3.0) - 1.0) * maxx);
	       	my2 = (((y2 * 3.0) - 1.0) * maxy);
	       	mx3 = (((x3 * 3.0) - 1.0) * maxx);
	       	my3 = (((y3 * 3.0) - 1.0) * maxy);
	       	mx4 = (((x4 * 3.0) - 1.0) * maxx);
	       	my4 = (((y4 * 3.0) - 1.0) * maxy);
       				
	        destinationQuad[0] = Point2f( mx1,my1 );
	        destinationQuad[1] = Point2f( mx2,my2 );
	        destinationQuad[2] = Point2f( mx3,my3 );
	        destinationQuad[3] = Point2f( mx4,my4 );

		if( otrans < 0.5 )
			has_alpha = false;
		else
			has_alpha = true;
				
		if( warp < 0.5 )
		{
			WarpWasOFF = true;
			memcpy(output.data, input.data, isize);

			if( mleft >= 0.0 && mright <= maxx && mtop >= 0.0)
				line(output, Point( mleft, mtop ), Point( mright, mtop ), color,  LINEWIDTH  );

			if( mleft >= 0.0 && mright <= maxx && mbottom <= maxy )
				line(output, Point( mright, mbottom ), Point( mleft, mbottom ), color,  LINEWIDTH  );

			if( mright <= maxx && mtop >= 0.0 && mbottom <= maxy )
				line(output, Point( mright, mtop ), Point( mright, mbottom ), color,  LINEWIDTH  );

			if( mleft >= 0.0 && mtop >= 0.0 && mbottom <= maxy )
				line(output, Point( mleft, mbottom ), Point( mleft, mtop ), color,  LINEWIDTH  );
		}
		else
		{
			WarpWasOFF = false;

			input(Rect(mleft, mtop, mwidth, mheight)).copyTo(region);
			
			if( ofeather > 0.0 )
			{
				update_feather_arrays(region, ofeather, r);				
				if( r > 0 )
					do_feather( region, has_alpha );
			}

			//cout << "r=" << r << endl;

			sourceQuad[0] = Point2f( 0.0, 0.0 );
			sourceQuad[1] = Point2f( (float) (mwidth-1.0), 0.0 );
	        	sourceQuad[2] = Point2f( (float) (mwidth-1.0), (float) (mheight-1.0) );
	        	sourceQuad[3] = Point2f( 0.0, (float) (mheight-1.0) );

		        // Get the Perspective Transform Matrix i.e. lambda
		        lambda = getPerspectiveTransform( sourceQuad, destinationQuad );

			output.setTo(0); 

		        // Apply the Perspective Transform just found to the src image
		        warpPerspective(region, output, lambda, output.size(), METHOD, BORDER_TRANSPARENT);
		}
		FIRST = false;
		return;
	}

	input.data = (unsigned char *) in;

	if( warp < 0.5 )	// we are in no_warp mode (set region)
	{
		mleft =   (left * maxx);
		mtop =    (top * maxy);
		mright =  (right * maxx);
		mbottom = (bottom * maxy);

		mwidth =  mright-mleft;
		mheight = mbottom-mtop;

		if(mleft >= mright || mtop >= mbottom) // dont draw rectangle when errors
		{
			region_coord_err = true;
			return;	
		}
		else
			region_coord_err = false; // only clear when values are OK

	
		if( WarpWasOFF == false ) // toggled ON to OFF
			WarpWasOFF = true;

		output.data = (unsigned char *) out;

		memcpy(output.data, input.data, isize); // draw over input image copied to output

		if( mleft >= 0.0 && mright <= maxx && mtop >= 0.0)
			line(output, Point( mleft, mtop ), Point( mright, mtop ), color,  LINEWIDTH  );

		if( mleft >= 0.0 && mright <= maxx && mbottom <= maxy )
			line(output, Point( mright, mbottom ), Point( mleft, mbottom ), color,  LINEWIDTH  );

		if( mright <= maxx && mtop >= 0.0 && mbottom <= maxy )
			line(output, Point( mright, mtop ), Point( mright, mbottom ), color,  LINEWIDTH  );

		if( mleft >= 0.0 && mtop >= 0.0 && mbottom <= maxy )
			line(output, Point( mleft, mbottom ), Point( mleft, mtop ), color,  LINEWIDTH  );

		return;	// skip warping	
	}

	if(region_coord_err == true) // if warp was turned ON but with invalid dest coords
		return;

	// check for trans change
	if( trans != otrans )
	{
		otrans = trans;
		trans_changed = true;
		if( otrans < 0.5 )
			has_alpha = false;
		else
			has_alpha = true;			
	}			
	// check feather change
	if( feather != ofeather )
	{
		ofeather = feather;
		feather_changed = true;
	}

	// warp is ON
	if( WarpWasOFF == true )	// warp is ON, check if toggled from OFF to ON?
	{
		WarpWasOFF = false;	// we caught the change...

		input(Rect(mleft, mtop, mwidth, mheight)).copyTo(region);

		if ( ofeather > 0.0 )
		{
			update_feather_arrays(region, ofeather, r);
			if( r > 0 )
				do_feather( region, has_alpha ) ;
		}

		sourceQuad[0] = Point2f( 0.0, 0.0 );
		sourceQuad[1] = Point2f( (float) (mwidth-1.0), 0.0 );
        	sourceQuad[2] = Point2f( (float) (mwidth-1.0), (float) (mheight-1.0) );
        	sourceQuad[3] = Point2f( 0.0, (float) (mheight-1.0) );

        	lambda = getPerspectiveTransform( sourceQuad, destinationQuad );
	}
	else // warp already ON
	{
		if ( feather_changed == true || trans_changed == true ) // reload region 
		{
			feather_changed = false;
			trans_changed = false;

			input(Rect(mleft, mtop, mwidth, mheight)).copyTo(region);

			if ( ofeather > 0.0 )
			{
				update_feather_arrays(region, ofeather, r);
				if( r > 0 )
					do_feather( region, has_alpha ) ;
				//cout << "r=" << r << endl;
			
			}
		}
		else if( istatic < 0.5 ) // must be video, input each frame into region
		{
			input(Rect(mleft, mtop, mwidth, mheight)).copyTo(region);

			if ( ofeather > 0.0 )
			{
				if( r > 0 )
					do_feather( region, has_alpha ) ;
			}
		}			
	}

	if( smooth != osmooth ) // change of method in UI
	{
		osmooth = smooth;
                method_index = (int) round(osmooth * 4.0);
                if(method_index < 0 || method_index > 4)
                    method_index=0;
		//cout << "method=" << method_index << endl; 
                METHOD = method[method_index];
	}

	// did coords change since last update?
	if( x1 != ox1 || y1 != oy1 || x2 != ox2 || y2 != oy2 || x3 != ox3 || y3 != oy3  || x4 != ox4 || y4 != oy4 )
	{
		ox1 = x1;
		oy1 = y1;
		ox2 = x2;
		oy2 = y2;
		ox3 = x3;
		oy3 = y3;
		ox4 = x4;
		oy4 = y4;

	       	mx1 = (((x1 * 3.0) - 1.0) * maxx);
	       	my1 = (((y1 * 3.0) - 1.0) * maxy);
	       	mx2 = (((x2 * 3.0) - 1.0) * maxx);
	       	my2 = (((y2 * 3.0) - 1.0) * maxy);
	       	mx3 = (((x3 * 3.0) - 1.0) * maxx);
	       	my3 = (((y3 * 3.0) - 1.0) * maxy);
	       	mx4 = (((x4 * 3.0) - 1.0) * maxx);
	       	my4 = (((y4 * 3.0) - 1.0) * maxy);
       				
	        destinationQuad[0] = Point2f( mx1,my1 );
	        destinationQuad[1] = Point2f( mx2,my2 );
	        destinationQuad[2] = Point2f( mx3,my3 );
	        destinationQuad[3] = Point2f( mx4,my4 );

	        lambda = getPerspectiveTransform( sourceQuad, destinationQuad );
	}

	output.data = (unsigned char *) out; // now we need to set output ptr before actual use
	output.setTo(0); // blank output before pasting warped input region
	warpPerspective(region, output, lambda, output.size(), METHOD, BORDER_TRANSPARENT);
    }

    
    
private:
	bool FIRST, WarpWasOFF, has_alpha, feather_changed, trans_changed;
	bool region_coord_err;

	int METHOD;

	double mleft, mtop, mright, mbottom, mwidth, mheight;	

	double maxx, maxy;

	Point2f destinationQuad[4];
	Point2f sourceQuad[4];

	int r;	// number of rows/cols of feathering

	//int trows, tcols;

	double osmooth;
	double ofeather;
	double otrans;

	unsigned char *ua;
	double  *fa;

	// Lambda Matrix
	Mat lambda;

	Mat input, output, region;
    
	int isize; // rows*cols*bpp (input and output mat memory sizes in bytes

	double W, H;
    
	double ox1, oy1,
	       ox2, oy2,
               ox3, oy3,
               ox4, oy4;
};


frei0r::construct<cropNwarp> plugin("cropNwarp",
                "openCV replacement for c0rners with crop",
                "RG-PNP",
                0,2,
                F0R_COLOR_MODEL_RGBA8888);



