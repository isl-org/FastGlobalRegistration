
// dependency FLANN and Eigen?

#include <vector>
#include <flann/flann.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#define DIM_FPFH			33	// FPFH feature dimension
#define DIV_FACTOR			1.4 // 2.0 originally
#define MAX_CORR_DIST		0.025 // todo: should check
#define ITERATION_NUMBER	64
#define TUPLE_SCALE			0.95
#define TUPLE_MAX_CNT		300


using namespace Eigen;
using namespace std;

typedef vector<Vector3f> Points;
typedef vector<VectorXf> Feature;

class CApp{
public:
	
	void ReadFeature(char* filepath);
	void NormalizePoints();
	void AdvancedMatching();
	void WriteTrans(char* filepath);
	double OptimizePairwise(double mu_, bool decrease_mu_, int numIter_);

	// test
	void TestFlann();


private:

	// containers
	vector<Points> pointcloud_;
	vector<Feature> features_;
	Matrix4f TransOutput_;
	vector<pair<int, int>> corres_;

	// for normalization
	Points Means;
	float GlobalScale;


	// some internal functions
	void ReadFeature(char* filepath, Points& pts, Feature& feat);

	//flann::Index<flann::L2<float>>* BuildFLANNTree(Feature& input);
	//void GetFLANNMatrix(Feature);
	void SearchFLANNTree(flann::Index<flann::L2<float>>* index,
		VectorXf& input,
		std::vector<int>& indices,
		std::vector<float>& dists,
		int nn);



};