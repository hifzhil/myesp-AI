#include "mouth_openness.h"

std::pair<int, int> get_mouth_center(std::list<dl::detect::result_t> &results)
{
    for (auto &prediction : results)
    {
        if (prediction.keypoint.size() == 10)
        {
            int left_mouth_x = prediction.keypoint[2];
            int left_mouth_y = prediction.keypoint[3];
            int right_mouth_x = prediction.keypoint[8];
            int right_mouth_y = prediction.keypoint[9];

            int center_x = (left_mouth_x + right_mouth_x) / 2;
            int center_y = (left_mouth_y + right_mouth_y) / 2;

            return {center_x, center_y}; 
        }
    }
    return {-1, -1};
}
