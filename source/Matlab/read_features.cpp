// ----------------------------------------------------------------------------
// -                       Fast Global Registration                           -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2016
// Jordi Pont-Tuset <jponttuset@vision.ee.ethz.ch>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include <stdio.h>
#include "mex.h"
#include "Eigen/Dense"

void mexFunction( int nlhs, mxArray *plhs[],
        		  int nrhs, const mxArray*prhs[] )
{
    char* filepath = mxArrayToString(prhs[0]);

	FILE* fid = fopen(filepath, "rb");
	int nvertex;
	fread(&nvertex, sizeof(int), 1, fid);
	int ndim;
	fread(&ndim, sizeof(int), 1, fid);

    plhs[0] = mxCreateDoubleMatrix(nvertex,3, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(nvertex,ndim, mxREAL);
    double * pt1 = mxGetPr(plhs[0]);
    double * pt2 = mxGetPr(plhs[1]);

	// Read from feature file and fill out pts and feat
	for (int v = 0; v < nvertex; v++)
    {
    	Eigen::Vector3f pts_v;
		fread(&pts_v(0), sizeof(float), 3, fid);
        for(std::size_t ii=0; ii<3; ++ii)
            *(pt1+(ii*nvertex)+v) = pts_v[ii];

		Eigen::VectorXf feat_v(ndim);
		fread(&feat_v(0), sizeof(float), ndim, fid);
        for(std::size_t ii=0; ii<ndim; ++ii)
            *(pt2+(ii*nvertex)+v) = feat_v[ii];
	}
	fclose(fid);
}
