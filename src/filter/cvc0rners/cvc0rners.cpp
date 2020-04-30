/*
 * Copyright (C) 2010-2011 Simon Andreas Eugster (simon.eu@gmail.com)
 * This file is not a Frei0r plugin but a collection of ideas.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */


/*
	This version tries to optimize calculations by:
	1- see if input image is a static or dynamic (video or png seq.)
	   if static, and the corners not changed, previously warped content is copied to output
	   if corners have changed, warp matrix is recalculated and applied to input
	2- to use TRANSPARENT borders (to prevent antialiasing to black), the output data
           is cleared if corners have changed.	
*/



#include "frei0r.hpp"

// Other includes used for the examples below.
// Can be removed on copy/paste.

// Limits (min/max values) of various data types
//#include <limits>
// For the CHAR_BIT constant
//#include <climits>
// pow() and other mathematical functions
//#include <cmath>

#include <opencv2/opencv.hpp>

//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

//#include <iostream>

using namespace cv;
using namespace std;

int method[5] = {INTER_NEAREST,INTER_LINEAR,INTER_AREA,INTER_CUBIC,INTER_LANCZOS4};

/**
The CMakeLists.txt needs to be adjusted as well (both in this directory and in the parent directory).
Also, don't forget src/Makefile.am.
*/


class cvc0rners : public frei0r::filter
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
    //f0r_param_double static_content;

    cvc0rners(unsigned int width, unsigned int height)
    { 
	W = width;
	H = height;
 
	maxx = (double) (width - 1);
	maxy = (double) (height - 1);
        
	x1 = 0.333333;
	y1 = 0.333333;
	x2 = 0.666666;
	y2 = 0.333333;
	x3 = 0.666666;
	y3 = 0.666666;
	x4 = 0.333333;
	y4 = 0.666666;
        register_param(x1, "x1", "upper left X");
        register_param(y1, "y1", "upper left Y");
        register_param(x2, "x2", "upper right X");
        register_param(y2, "y2", "upper right Y");
        register_param(x3, "x3", "lower right X");
        register_param(y3, "y3", "lower right Y");
        register_param(x4, "x4", "lower left X");
        register_param(y4, "y4", "lower left Y");

        register_param(smooth, "Smooth", "smoothing method");

        lambda = Mat( 2, 4, CV_32FC1 );
	//input = Mat(Size(W, H), CV_8UC4);
	//output = Mat(Size(width,height), CV_8UC4);
#if 1        
        sourceQuad[0] = Point2f( 0.0, 0.0 );
        sourceQuad[1] = Point2f( (float) maxx, 0.0);
        sourceQuad[2] = Point2f( (float) maxx, (float) maxy);
        sourceQuad[3] = Point2f( 0.0, (float) maxy  );
#else
        sourceQuad[0] = Point2f( 0, 0 );
        sourceQuad[1] = Point2f( (float) maxx/2, 0.0);
        sourceQuad[2] = Point2f( (float) maxx/2, (float) maxy/2);
        sourceQuad[3] = Point2f( 0.0, (float) maxy/2  );
#endif

        destinationQuad[0] = Point2f( 0.0, 0.0 );
        destinationQuad[1] = Point2f( (float) maxx, 0.0);
        destinationQuad[2] = Point2f( (float) maxx, (float) maxy);
        destinationQuad[3] = Point2f( 0.0, (float) maxy );

        //isize = width * height * 4;

	FIRST = true;
    }

    ~cvc0rners()
    {
        // Delete member variables if necessary.
        input.release();
        output.release();
        lambda.release();
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
        
        int index;


	if( FIRST ) // do full process (but store lambda, input and output data)
	{
		input = Mat(Size(W,H), CV_8UC4, (unsigned char *) in, W*4);
		output = Mat(Size(W,H), CV_8UC4, (unsigned char *) out, W*4);

                index = (int) round(smooth * 4.0);
                if(index < 0 || index > 4)
                    index=0;

                METHOD = method[index];
		SMOOTH = smooth;
		
		ox1 = x1;
		oy1 = y1;
		ox2 = x2;
		oy2 = y2;
		ox3 = x3;
		oy3 = y3;
		ox4 = x4;
		oy4 = y4;
		// must do this math if FIRST		
	       	mx1 = ((x1 * 3.0) - 1.0) * maxx;
	       	my1 = ((y1 * 3.0) - 1.0) * maxy;
	       	mx2 = ((x2 * 3.0) - 1.0) * maxx;
	       	my2 = ((y2 * 3.0) - 1.0) * maxy;
	       	mx3 = ((x3 * 3.0) - 1.0) * maxx;
	       	my3 = ((y3 * 3.0) - 1.0) * maxy;
	       	mx4 = ((x4 * 3.0) - 1.0) * maxx;
	       	my4 = ((y4 * 3.0) - 1.0) * maxy;
       		
	        destinationQuad[0] = Point2f( mx1,my1 );
	        destinationQuad[1] = Point2f( mx2,my2 );
	        destinationQuad[2] = Point2f( mx3,my3 );
	        destinationQuad[3] = Point2f( mx4,my4 );

	        // Get the Perspective Transform Matrix i.e. lambda
	        lambda = getPerspectiveTransform( sourceQuad, destinationQuad );
	        // Apply the Perspective Transform just found to the src image
	        warpPerspective(input, output, lambda, output.size(), METHOD, BORDER_TRANSPARENT);
		FIRST = false;
		return;
	}

	input.data = (uchar *) in; 
	output.data = (uchar *) out; 
       
	output.setTo(0);
	if( smooth != SMOOTH ) // change of method in UI
	{
                index = (int) round(smooth * 4.0);
                if(index < 0 || index > 4)
                    index=0;

                METHOD = method[index];
		SMOOTH = smooth;
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

	       	mx1 = ((x1 * 3.0) - 1.0) * maxx;
	       	my1 = ((y1 * 3.0) - 1.0) * maxy;
	       	mx2 = ((x2 * 3.0) - 1.0) * maxx;
	       	my2 = ((y2 * 3.0) - 1.0) * maxy;
	       	mx3 = ((x3 * 3.0) - 1.0) * maxx;
	       	my3 = ((y3 * 3.0) - 1.0) * maxy;
	       	mx4 = ((x4 * 3.0) - 1.0) * maxx;
	       	my4 = ((y4 * 3.0) - 1.0) * maxy;
       		
	        destinationQuad[0] = Point2f( mx1,my1 );
	        destinationQuad[1] = Point2f( mx2,my2 );
	        destinationQuad[2] = Point2f( mx3,my3 );
	        destinationQuad[3] = Point2f( mx4,my4 );

	        // Get the Perspective Transform Matrix i.e. lambda
	        lambda = getPerspectiveTransform( sourceQuad, destinationQuad );
	}
	warpPerspective(input, output, lambda, output.size(), METHOD, BORDER_TRANSPARENT);
    }

    
    
private:

    bool FIRST;
//    bool SC; // static_content (1 when true)
    int IM;
    int METHOD;
    double SMOOTH;

    double maxx, maxy;

    /*cv::*/Point2f destinationQuad[4];
    // Output Quadilateral or World plane coordinates
    /*cv::*/Point2f sourceQuad[4];

    // Lambda Matrix
    /*cv::*/Mat lambda;

    /*cv::*/Mat input, output;
    
    std::size_t isize;

    double W, H;
    
    double ox1, oy1,
           ox2, oy2,
           ox3, oy3,
           ox4, oy4;
};



frei0r::construct<cvc0rners> plugin("cvc0rners",
                "openCV replacement for c0rners",
                "RG-PNP",
                0,2,
                F0R_COLOR_MODEL_RGBA8888);


//       <parameter type="bool" name="mode" default="0">
//                <name>use cos instead of linear</name>
//        </parameter>


