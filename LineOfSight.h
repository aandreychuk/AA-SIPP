#ifndef LINEOFSIGHT_H
#define LINEOFSIGHT_H
#include <cMap.h>

class LineOfSight
{
public:
    bool checkLine(int i1, int j1, int i2, int j2, const cMap& map)
    {
        int delta_i = std::abs(i1 - i2);
        int delta_j = std::abs(j1 - j2);
        int step_i = (i1 < i2 ? 1 : -1);
        int step_j = (j1 < j2 ? 1 : -1);
        int error = 0;
        int i = i1;
        int j = j1;
        int sep_value = delta_i*delta_i + delta_j*delta_j;
        if(delta_i == 0)
        {
            for(; j != j2; j += step_j)
                if(map.CellIsObstacle(i, j))
                    return false;
        }
        else if(delta_j == 0)
        {
            for(; i != i2; i += step_i)
                if(map.CellIsObstacle(i, j))
                    return false;
        }
        else if(delta_i > delta_j)
        {
            for(; i != i2; i += step_i)
            {
                if(map.CellIsObstacle(i, j))
                    return false;
                if(map.CellIsObstacle(i, j + step_j))
                    return false;
                error += delta_j;
                if(error > delta_i)
                {
                    if(((error << 1) - delta_i - delta_j)*((error << 1) - delta_i - delta_j) < sep_value)
                        if(map.CellIsObstacle(i + step_i, j))
                            return false;
                    if((3*delta_i - ((error << 1) - delta_j))*(3*delta_i - ((error << 1) - delta_j)) < sep_value)
                        if(map.CellIsObstacle(i, j + 2*step_j))
                            return false;
                    j += step_j;
                    error -= delta_i;
                }
            }
            if(map.CellIsObstacle(i, j))
                return false;
            if(map.CellIsObstacle(i, j + step_j))
                return false;
        }
        else
        {
            for(; j != j2; j += step_j)
            {
                if(map.CellIsObstacle(i, j))
                    return false;
                if(map.CellIsObstacle(i + step_i, j))
                    return false;
                error += delta_i;
                if(error > delta_j)
                {
                    if(((error << 1) - delta_i - delta_j)*((error << 1) - delta_i - delta_j) < sep_value)
                        if(map.CellIsObstacle(i, j + step_j))
                            return false;
                    if((3*delta_j - ((error << 1) - delta_i))*(3*delta_j - ((error << 1) - delta_i)) < sep_value)
                        if(map.CellIsObstacle(i + 2*step_i, j))
                            return false;
                    i += step_i;
                    error -= delta_j;
                }
            }
            if(map.CellIsObstacle(i, j))
                return false;
            if(map.CellIsObstacle(i + step_i, j))
                return false;
        }
        return true;
    }
};

#endif // LINEOFSIGHT_H
