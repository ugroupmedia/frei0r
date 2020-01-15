/* alphamatte.cpp
 * Copyright (C) 2006 Jean-Sebastien Senecal (js@drone.ws)
 * This file is a Frei0r plugin.
 * The code is a modified version of code from the Gimp.
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
#include "frei0r_math.h"

#define NBYTES 4
#define ALPHA 3

class alphamatte : public frei0r::mixer2
{
public:
  alphamatte(unsigned int width, unsigned int height)
  {
	W = width;
	H = height;
	L = W * H; // number of pixels
  }

  /**
   * luma of src1 is used as alpha mask for src2 (black=0,0,0,0 to out, white=src2 to out, gray src2*luma to out  
   **/
  void update(double time,
              uint32_t* out,
              const uint32_t* in1,
              const uint32_t* in2)
  {
    const uint8_t *src1 = reinterpret_cast<const uint8_t*>(in1);
    const uint8_t *src2 = reinterpret_cast<const uint8_t*>(in2);
    uint8_t *dst = reinterpret_cast<uint8_t*>(out);

    uint32_t tmp;
//    uint32_t w = size;
    uint32_t a; // luma of src1
  
    for( uint32_t i = 0; i < L; i++, src1+=NBYTES, src2+=NBYTES, dst+=NBYTES)
    {
      	a = src1[0]; // use R channel of grayscale as alpha of matte PNG

        if(a == 0) // matte is 0, black (none of src2)
        {
	    *(out+i) = 0;
        }
        else 
        {
          if(a == 255) // matte is 255, white (original src2 only)
          {
	     *(out+i) = *(in2+i);
          }
          else // gray (src2 modulated by src1, but with src2 alpha value)
          {
//              for (uint32_t b = 0; b < NBYTES; b++)
//                  dst[b] = INT_MULT(src2[b], a, tmp);
                 *dst     = INT_MULT(*src2, a, tmp);
		 *(dst+1) = INT_MULT(*(src2+1), a, tmp);
		 *(dst+2) = INT_MULT(*(src2+2), a, tmp);
		 *(dst+3) = INT_MULT(*(src2+3), a, tmp);
          }            
        }
      }
    }  
private:

    unsigned int W, H, L; 
};

frei0r::construct<alphamatte> plugin("alphamatte",
                                "Perform a alphamatte operation between two sources",
                                "RG",
                                0,2,
                                F0R_COLOR_MODEL_RGBA8888);

