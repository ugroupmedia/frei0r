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

#include "frei0r.hpp"
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

using namespace cv;
using namespace std;

#define NBYTES 4
#define ALPHA 3

static int SEED = 0;

static int perlin_hash[] = {208,34,231,213,32,248,233,56,161,78,24,140,71,48,140,254,245,255,247,247,40,
                     185,248,251,245,28,124,204,204,76,36,1,107,28,234,163,202,224,245,128,167,204,
                     9,92,217,54,239,174,173,102,193,189,190,121,100,108,167,44,43,77,180,204,8,81,
                     70,223,11,38,24,254,210,210,177,32,81,195,243,125,8,169,112,32,97,53,195,13,
                     203,9,47,104,125,117,114,124,165,203,181,235,193,206,70,180,174,0,167,181,41,
                     164,30,116,127,198,245,146,87,224,149,206,57,4,192,210,65,210,129,240,178,105,
                     228,108,245,148,140,40,35,195,38,58,65,207,215,253,65,85,208,76,62,3,237,55,89,
                     232,50,217,64,244,157,199,121,252,90,17,212,203,149,152,140,187,234,177,73,174,
                     193,100,192,143,97,53,145,135,19,103,13,90,135,151,199,91,239,247,33,39,145,
                     101,120,99,3,186,86,99,41,237,203,111,79,220,135,158,42,30,154,120,67,87,167,
                     135,176,183,191,253,115,184,21,233,58,129,233,142,39,128,211,118,137,139,255,
                     114,20,218,113,154,27,127,246,250,1,8,198,250,209,92,222,173,21,88,102,219};

int noise2(int x, int y)
{
    int tmp = perlin_hash[(y + SEED) % 256];
    return perlin_hash[(tmp + x) % 256];
}

double lin_inter(double x, double y, double s)
{
    return x + s * (y-x);
}

double smooth_inter(double x, double y, double s)
{
    return lin_inter(x, y, s * s * (3-2*s));
}

double noise2d(double x, double y)
{
    int x_int = x;
    int y_int = y;
    double x_frac = x - x_int;
    double y_frac = y - y_int;
    int s = noise2(x_int, y_int);
    int t = noise2(x_int+1, y_int);
    int u = noise2(x_int, y_int+1);
    int v = noise2(x_int+1, y_int+1);
    double low = smooth_inter(s, t, x_frac);
    double high = smooth_inter(u, v, x_frac);
    return smooth_inter(low, high, y_frac);
}

double perlin2d(double x, double y, double freq, int depth)
{
    double xa = x*freq;
    double ya = y*freq;
    double amp = 1.0;
    double fin = 0;
    double div = 0.0;

    int i;
    for(i=0; i<depth; i++)
    {
        div += 256 * amp;
        fin += noise2d(xa, ya) * amp;
        amp /= 2;
        xa *= 2;
        ya *= 2;
    }

    return fin/div;
}


class perlin : public frei0r::filter
{

public:
    f0r_param_double  Frequency;
    f0r_param_double  Depth;
    f0r_param_double  Amplitude;

    perlin(unsigned int width, unsigned int height)

    {
        register_param(Frequency, "Frequency", "the Size");
        register_param(Depth, "Depth", "the Depth");
        register_param(Amplitude, "Amplitude", "the Amplitude");

        isize = width * height * 4;

	W = width;
	H = height;
    }

    ~perlin()
    {
    }


    
    virtual void update(double time, uint32_t* out, const uint32_t* in)
    {
	frequency = Frequency / 100.0;
	depth = ( int ) Depth;
	amplitude = Amplitude / 100.0;	

	double minV = 1000, maxV = -1;

        const uint8_t *src = reinterpret_cast<const uint8_t*>(in);

        uint8_t *dst = reinterpret_cast<uint8_t*>(out);

	for (int y = 0; y < H; ++y)
	{
		for (int x = 0; x < W; ++x)
		{
			double v;

			v = perlin2d((double) x, (double) y, frequency, depth) * amplitude;	
			
			dst[0] = src[0] - (src[0] * v);
			dst[1] = src[1] - (src[1] * v) ;
			dst[2] = src[2] - (src[2] * v);
			dst[3] = src[3];

			dst += NBYTES;
			src += NBYTES;
		}
	}
    }
    
private:
    double frequency, amplitude;
    int depth;

    int W, H;

    std::size_t isize;
};

frei0r::construct<perlin> plugin("perlin noise generator",
                "Generates a coherent noise image",
                "RG (UGM)",
                0,2,
                F0R_COLOR_MODEL_RGBA8888);
