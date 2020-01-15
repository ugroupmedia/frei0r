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
#include <math.h>


// Other includes used for the examples below.
// Can be removed on copy/paste.

// Limits (min/max values) of various data types
#include <limits>
// For the CHAR_BIT constant
#include <climits>
// pow() and other mathematical functions
#include <cmath>

#include <opencv/cv.h>
//#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

#include <iostream>

double PI = 3.14159265359; 
double pixelScale = 255.9;

using namespace cv;
using namespace std;

/**
The CMakeLists.txt needs to be adjusted as well (both in this directory and in the parent directory).
Also, don't forget src/Makefile.am.
*/

class cvemboss : public frei0r::filter
{

public:
    f0r_param_double  Azimuth;
    f0r_param_double  Elevation;
    f0r_param_double  Width45;
    f0r_param_double  Strength;
    f0r_param_double  static_content;
    f0r_param_double  colorize;

    cvemboss(unsigned int width, unsigned int height)
    {  
        register_param(Azimuth, "Azimuth", "direction");
        register_param(Elevation, "Elevation", "elevation");
        register_param(Width45, "Width45", "width45");
        register_param(Strength, "Strength", "strength");

	W = width;
	H = height;
	L = W * H; // number of pixels

	bumpPixels.resize(L);
	alphaVals.resize(L);

        isize = L * 4; //rgba

	FIRST = true;
    }

    ~cvemboss()
    {
    }
    
    virtual void update(double time, uint32_t* out, const uint32_t* in)
    {
  	unsigned int index, r, g, b, a = 0;
  	const unsigned char* src = (unsigned char*) in;
	unsigned char* dst = (unsigned char*) out;

	int len;
	int s1, s2, s3 = 0;
  	int x, y = 0;
	int shade;
	int bumpIndex = 0;
  	float fshade = 0.0, p = 0.0, scaler = 0.0;
	int minV = 1000;	// impossible
	int maxV = -1;		// impossible

	azimuth = (Azimuth * 360.0) * PI / 180.0; 	//input range 0 - 1 will be interpreted as angle 0 - 360
        elevation = (Elevation * 90.0) * PI / 180.0;	//input range 0 - 1 will be interpreted as lighness value 0 - 90
        width45 = Width45 * 40.0;	//input range 0 - 1 will be interpreted as bump height value 1 - 40

	strength = Strength;

        Lx = (int)(cos(azimuth) * cos(elevation) * pixelScale);
        Ly = (int)(sin(azimuth) * cos(elevation) * pixelScale);
        Lz = (int)(sin(elevation) * pixelScale);

        Nz = (int)(6 * 255 / width45);
        Nz2 = Nz * Nz;
        NzLz = Nz * Lz;

        background = Lz;

	// generate gray
	len = L;
  	while (len--)
  	{
		int v;
  	  	r = *src++;
  	  	g = *src++;
  	  	b = *src++;
  	  	a = *src++;
			
		v=(r + g + b) / 3;

  		bumpPixels[index++] = v;
  		alphaVals[index - 1] = a;
  	}
		
	// reset src to top
	src = (unsigned char*) in;

  	for (y = 0; y < H; y++, bumpIndex += W) 
  	{
	    s1 = bumpIndex;
	    s2 = s1 + W;
	    s3 = s2 + W;
	    for (x = 0; x < W; x++, s1++, s2++, s3++) 
	    {
	      	if (y != 0 && y < H-2 && x != 0 && x < W-2) 
	      	{
		    Nx = bumpPixels[s1-1] + bumpPixels[s2-1] + bumpPixels[s3-1] - bumpPixels[s1+1] - bumpPixels[s2+1] - bumpPixels[s3+1];
		    Ny = bumpPixels[s3-1] + bumpPixels[s3] + bumpPixels[s3+1] - bumpPixels[s1-1] - bumpPixels[s1] - bumpPixels[s1+1];
		    if (Nx == 0 && Ny == 0)
			    shade = background;
		    else if ((NdotL = Nx*Lx + Ny*Ly + NzLz) < 0)
			    shade = 0;
		    else
			    shade = (int)(NdotL / sqrt(Nx*Nx + Ny*Ny + Nz2));
		} 
		else
		{
		    shade = background;
		}
		fshade = ((float) shade / 255.0) * strength;
		p = (float) *src++;
		*dst++ = (unsigned char) (p - (p * fshade));

		p = (float) *src++;
		*dst++ = (unsigned char) (p - (p * fshade));

		p = (float) *src++;
		*dst++ = (unsigned char) (p - (p * fshade));
	
		*dst++ = alphaVals[s1]; //copy alpha
		*src++; // skip over src alpha
	    }  
	}
    }
    
private:

    bool FIRST;
    bool SC; // static_content (1 when true)
    bool COLOR;

    double azimuth; //input range 0 - 1 will be interpreted as angle 0 - 360
    double elevation;//input range 0 - 1 will be interpreted as lighness value 0 - 90
    double width45;//input range 0 - 1 will be interpreted as bump height value 1 - 40
    double strength;
	
    int W, H, L; 

    int Nx, Ny, Nz, Lx, Ly, Lz, Nz2, NzLz, NdotL;
    unsigned char background;

    std::vector< unsigned char > bumpPixels;
    std::vector< unsigned char > alphaVals;

//    Mat input, output;
    
    std::size_t isize;
};



frei0r::construct<cvemboss> plugin("cvemboss",
                "openCV replacement for c0rners",
                "RG-PNP",
                0,2,
                F0R_COLOR_MODEL_RGBA8888);


//       <parameter type="bool" name="mode" default="0">
//                <name>use cos instead of linear</name>
//        </parameter>

