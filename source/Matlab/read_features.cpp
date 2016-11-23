

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