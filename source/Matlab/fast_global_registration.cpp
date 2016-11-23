// ----------------------------------------------------------------------------
// -                       Fast Global Registration                           -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) Intel Corporation 2016
// Qianyi Zhou <Qianyi.Zhou@gmail.com>
// Jaesik Park <syncle@gmail.com>
// Vladlen Koltun <vkoltun@gmail.com>
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
#include "app.h"
#include "mex.h"

void mexFunction( int nlhs, mxArray *plhs[], 
        		  int nrhs, const mxArray*prhs[] )
{
    /* Some checks */
    if(nrhs!=4)
        mexErrMsgTxt("FastGlobalRegistration(points_01, features_01, points_02, features_02)\n");
    if(mxGetN(prhs[0])!=3 || mxGetN(prhs[2])!=3)
        mexErrMsgTxt("Points must have three columns\n");

    /* Input points */
    Eigen::Map<Eigen::Array<double,Eigen::Dynamic,3> >
            points_01((double*)mxGetData(prhs[0]),mxGetM(prhs[0]),mxGetN(prhs[0]));
    Eigen::Map<Eigen::Array<double,Eigen::Dynamic,3> >
            points_02((double*)mxGetData(prhs[2]),mxGetM(prhs[2]),mxGetN(prhs[2]));
    
    /* Input features */
    Eigen::Map<Eigen::Array<double,Eigen::Dynamic,Eigen::Dynamic> >
            features_01((double*)mxGetData(prhs[1]),mxGetM(prhs[1]),mxGetN(prhs[1]));
    Eigen::Map<Eigen::Array<double,Eigen::Dynamic,Eigen::Dynamic> >
            features_02((double*)mxGetData(prhs[3]),mxGetM(prhs[3]),mxGetN(prhs[3]));
    
    /* More checks */
    if(points_01.rows()!=features_01.rows() || points_02.rows()!=features_02.rows())
        mexErrMsgTxt("Number of points and features must be the same\n");
    if(features_01.cols()!=features_02.cols())
        mexErrMsgTxt("Feature dimensions must be the same\n");
    
    /* Copy to the expected container (vector of Eigen vectors instead of full Eigen matrix) */
    Points pts1(points_01.rows());
    Feature feat1(features_01.rows());
    for (std::size_t ii=0; ii<points_01.rows(); ++ii)
    {
        pts1[ii] = points_01.row(ii).cast<float>();
        feat1[ii] = features_01.row(ii).cast<float>();
    }
    Points pts2(points_02.rows());
    Feature feat2(features_02.rows());
    for (std::size_t ii=0; ii<points_02.rows(); ++ii)
    {
        pts2[ii] = points_02.row(ii).cast<float>();
        feat2[ii] = features_02.row(ii).cast<float>();
    }    

    /* Call the actual code */
	CApp app;
    app.LoadFeature(pts1,feat1);
    app.LoadFeature(pts2,feat2);
	app.NormalizePoints();
	app.AdvancedMatching();
	app.OptimizePairwise(true, ITERATION_NUMBER);
	Eigen::Matrix4f transf = app.GetTrans();
    
    /* Return the transformation */
    plhs[0] = mxCreateDoubleMatrix(4,4, mxREAL);
    double * pt = mxGetPr(plhs[0]);
    for(std::size_t ii=0; ii<4; ++ii)
        for(std::size_t jj=0; jj<4; ++jj)
            *(pt++) = (double)transf(jj,ii);
}
