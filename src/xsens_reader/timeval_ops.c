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
#include "timeval_ops.h"

void tvu_correct_timeval (struct timeval* t){
	if (t->tv_usec < 0){
		t->tv_usec += ONE_MILLION;
		t->tv_sec -= 1;	
	} else {
		if (t->tv_usec >= ONE_MILLION){
			t->tv_usec -= ONE_MILLION;
			t->tv_sec += 1;	
		}
	}
}
	
	
double tvu_timeval_to_double(const struct timeval* t){
	return t->tv_sec * 1.0 + t->tv_usec * 1.0e-6;
}

struct timeval tvu_double_to_timeval(const double* v){
	struct timeval t;
	t.tv_sec = (int)*v;
	t.tv_usec = (int)(*v - ((double)t.tv_sec)) * ONE_MILLION;
	return t;
}


///calc t2-t1
void tvu_subtract (struct timeval* result, struct timeval* t2, struct timeval* t1){
	/* Perform the carry for the later subtraction by updating y. */
	result->tv_sec = t2->tv_sec - t1->tv_sec;
	result->tv_usec = t2->tv_usec - t1->tv_usec;
	tvu_correct_timeval(result);
// 	if (result->tv_usec < 0){
// 		result->tv_usec += ONE_MILLION;
// 		result->tv_sec -= 1;
// 	}
}

void tvu_add (struct timeval* result, struct timeval* t1, struct timeval* t2){
	result->tv_sec = t2->tv_sec + t1->tv_sec;
	result->tv_usec = t2->tv_usec + t1->tv_usec;
	tvu_correct_timeval(result);
// 	if (result->tv_usec >= ONE_MILLION){
// 		result->tv_usec -= ONE_MILLION;
// 		result->tv_sec += 1;
// 	}
}

void tvu_add_ms(struct timeval* result, struct timeval* t1, int ms){
	struct timeval t2;
	t2.tv_sec = ms / 1000;
	t2.tv_usec = (ms % 1000)*1000;
	return tvu_add (result, t1, &t2);
}
