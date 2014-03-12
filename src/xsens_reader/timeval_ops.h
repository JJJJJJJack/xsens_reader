/*The MIT License (MIT)

Copyright (c) 2014 HeXiang

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/
#ifndef _TIMEVAL_OPS_H
#define _TIMEVAL_OPS_H

#include <sys/time.h>

#define ONE_MILLION 1000000

///see http://www.xorp.org/release-0.2/docs/kdoc/html/libxorp/timeval_hh.html
///for a c++ implementation

#ifdef __cplusplus
extern "C" {
#endif

///checks for illigal values (like usec < 0 or usec >= 1000000)
///corrects this if needed!
void tvu_correct_timeval (struct timeval* t);
	
double tvu_timeval_to_double(const struct timeval* t);

struct timeval tvu_double_to_timeval(const double* v);


///calculate result = t2-t1
void tvu_subtract (struct timeval* result, struct timeval* t2, struct timeval* t1);

///calculate result = t1+t2
void tvu_add (struct timeval* result, struct timeval* t1, struct timeval* t2);

void tvu_add_ms(struct timeval* result, struct timeval* t1, int ms);


#ifdef __cplusplus
}
#endif

#endif //_TIMEVAL_OPS_H
