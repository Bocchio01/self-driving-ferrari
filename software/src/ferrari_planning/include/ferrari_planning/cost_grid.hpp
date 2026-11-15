#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

namespace ferrari_planning
{
    struct CostGrid
    {
        int width = 0;
        int height = 0;
        double resolution = 0.05;
        double origin_x = 0.0;
        double origin_y = 0.0;
        std::vector<int8_t> data;

        bool valid() const { return width > 0 && height > 0 && data.size() == static_cast<size_t>(width) * height; }

        bool toCell(double x, double y, int &ix, int &iy) const
        {
            ix = static_cast<int>(std::floor((x - origin_x) / resolution));
            iy = static_cast<int>(std::floor((y - origin_y) / resolution));
            return ix >= 0 && iy >= 0 && ix < width && iy < height;
        }

        int cost(double x, double y) const
        {
            int ix, iy;
            if (!toCell(x, y, ix, iy))
            {
                return -1;
            }
            return data[static_cast<size_t>(iy) * width + ix];
        }
    };
}
