
#include <stdio.h>
#include "app.h"

int main(int argc, char *argv[])
{

	if (argc != 4)
	{
		printf("FastGlobalRegistration.exe [feature_01] [feature_02] [output_txt]");
		return 0;
	}

	CApp app;
	app.ReadFeature(argv[1]);
	app.ReadFeature(argv[2]);
	app.NormalizePoints();
	app.AdvancedMatching();
	app.OptimizePairwise(1.0f, true, ITERATION_NUMBER);
	app.WriteTrans(argv[3]);

	return 0;
}

