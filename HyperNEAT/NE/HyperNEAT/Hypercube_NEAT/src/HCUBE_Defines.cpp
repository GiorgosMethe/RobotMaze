#include "HCUBE_Defines.h"

namespace HCUBE
{
#if defined(_DEBUG) || defined(USE_GPU)
	int NUM_THREADS = 1;
#else
    int NUM_THREADS = 1;
#endif

	const int EXPERIMENT_BLOCKS_GUI  = 0;
}
