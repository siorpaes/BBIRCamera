#include <stdint.h>
#include "bilinear.h"

int resizeBilinearGrey(int16_t* pixelsIn, int16_t* pixelsOut, int w, int h, int w2, int h2)
{
	int A, B, C, D, x, y, index, gray ;
	float x_ratio = ((float)(w-1))/w2 ;
	float y_ratio = ((float)(h-1))/h2 ;
	float x_diff, y_diff;
	int offset = 0 ;
	for (int i=0;i<h2;i++) {
		for (int j=0;j<w2;j++) {
			x = (int)(x_ratio * j) ;
			y = (int)(y_ratio * i) ;
			x_diff = (x_ratio * j) - x ;
			y_diff = (y_ratio * i) - y ;
			index = y*w+x ;
			
			// range is 0 to 255 thus bitwise AND with 0xff
			A = pixelsIn[index] & 0xffff;
			B = pixelsIn[index+1] & 0xffff;
			C = pixelsIn[index+w] & 0xffff;
			D = pixelsIn[index+w+1] & 0xffff;
         
			// Y = A(1-w)(1-h) + B(w)(1-h) + C(h)(1-w) + Dwh
			gray = (int)(
				A*(1-x_diff)*(1-y_diff) +  B*(x_diff)*(1-y_diff) +
				C*(y_diff)*(1-x_diff)   +  D*(x_diff*y_diff)
							 ) ;
			
			pixelsOut[offset++] = gray ;                                   
		}
	}
	return 0;
}
