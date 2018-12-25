#ifndef TSUTIL_H
#define TSUTIL_H

#include <vector>
#include <cstdint>

namespace apollo {
	namespace perception {
		float CalculateEuclideanDistance(float x, float y);
		float CalculateDynamicTimeWarpedDistance(std::vector<float> t0, std::vector<float> t1);
	}
}
#endif
