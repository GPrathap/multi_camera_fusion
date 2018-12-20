#include <vector>
#include <cmath>
#include <algorithm>
#include "dtw.h"

namespace apollo {
	namespace perception {

		float CalculateEuclideanDistance(float x, float y) {
			return std::sqrt(std::pow((x - y), 2));
		}

		float CalculateDynamicTimeWarpedDistance(std::vector<float> t0, std::vector<float> t1) {
			size_t m = t0.size();
			size_t n = t1.size();
			std::vector <std::vector<float>> cost(m, std::vector<float>(n));
			cost[0][0] = CalculateEuclideanDistance(t0[0], t1[0]);
			for (size_t i = 1; i < m; i++) {
				cost[i][0] = cost[i - 1][0] + CalculateEuclideanDistance(t0[i], t1[0]);
			}
			for (size_t j = 1; j < n; j++) {
				cost[0][j] = cost[0][j - 1] + CalculateEuclideanDistance(t0[0], t1[j]);
			}
			for (size_t i = 1; i < m; i++) {
				for (size_t j = 1; j < n; j++) {
					cost[i][j] = std::min(cost[i - 1][j], std::min(cost[i][j - 1], cost[i - 1][j - 1]))
								 + CalculateEuclideanDistance(t0[i], t1[j]);
				}
			}
			return cost[m - 1][n - 1];
		}
	}
}
