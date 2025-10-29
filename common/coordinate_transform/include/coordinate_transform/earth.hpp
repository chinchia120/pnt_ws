#pragma once

#include <cmath>

namespace earth
{   
    extern double loc_x, loc_y, loc_z;

    void geo2loc(double lat, double lon, double ell);
    double getloc_x();
    double getloc_y();
    double getloc_z();
} // namespace earth