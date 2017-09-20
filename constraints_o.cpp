#include "Constraints_o.h"

Constraints_o::Constraints_o()
{

}
void Constraints_o::init(int width, int height)
{
    safe_intervals.resize(height);
    for(int i = 0; i < height; i++)
    {
        safe_intervals[i].resize(width);
        for(int j = 0; j < width; j++)
        {
            safe_intervals[i][j].resize(0);
            safe_intervals[i][j].push_back({0,CN_INFINITY});
        }
    }
    constraints.resize(height);
    for(int i = 0; i < height; i++)
    {
        constraints[i].resize(width);
        for(int j = 0; j < width; j++)
            constraints[i][j].resize(0);
    }
}

bool sort_function_o(std::pair<double, double> a, std::pair<double, double> b)
{
    return a.first < b.first;
}

void Constraints_o::updateSafeIntervals(const std::vector<std::pair<int, int> > &cells, section sec, bool goal)
{
    int i0(sec.i1), j0(sec.j1), i1(sec.i2), j1(sec.j2), i2, j2;
    for(int i = 0; i < cells.size(); i++)
    {
        i2 = cells[i].first;
        j2 = cells[i].second;
        std::pair<double,double> ps, pg, interval;
        ps = {i0, j0};
        pg = {i1, j1};
        double dist = fabs((ps.first - pg.first)*j2 + (pg.second - ps.second)*i2 + (ps.second*pg.first - ps.first*pg.second))
                /sqrt(pow(ps.first - pg.first, 2) + pow(ps.second - pg.second, 2));
        if(dist >= 1.0)
            continue;
        int da = (i0 - i2)*(i0 - i2) + (j0 - j2)*(j0 - j2);
        int db = (i1 - i2)*(i1 - i2) + (j1 - j2)*(j1 - j2);
        double ha = sqrt(da - dist*dist);
        double size = sqrt(1.0 - dist*dist);
        if(da == 0 && db == 0)
        {
            interval.first = sec.g1;
            interval.second = sec.g2;
        }
        else if(da != 0.0 && db != 0.0)
        {
            interval.first = sec.g1 + ha - size;
            interval.second = sec.g1 + ha + size;
        }
        else if(da == 0.0)
        {
            double hb = sqrt(db - dist*dist);
            interval.first = sec.g1;
            interval.second = sec.g2 - hb + size;
        }
        else
        {
            interval.first = sec.g1 + ha - size;
            interval.second = sec.g2;
            if(goal)
                interval.second = CN_INFINITY;
        }
        for(int j = 0; j < safe_intervals[i2][j2].size(); j++)
        {
            if(safe_intervals[i2][j2][j].first <= interval.first && safe_intervals[i2][j2][j].second >= interval.first)
            {
                if(safe_intervals[i2][j2][j].first == interval.first)
                {
                    safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, {safe_intervals[i2][j2][j].first,safe_intervals[i2][j2][j].first});
                    j++;
                    if(safe_intervals[i2][j2][j].second < interval.second)
                        safe_intervals[i2][j2].erase(safe_intervals[i2][j2].begin() + j);
                    else
                        safe_intervals[i2][j2][j].first = interval.second;
                }
                else if(safe_intervals[i2][j2][j].second < interval.second)
                    safe_intervals[i2][j2][j].second = interval.first;
                else
                {
                    std::pair<double,double> new1, new2;
                    new1.first = safe_intervals[i2][j2][j].first;
                    new1.second = interval.first;
                    new2.first = interval.second;
                    new2.second = safe_intervals[i2][j2][j].second;
                    safe_intervals[i2][j2].erase(safe_intervals[i2][j2].begin() + j);
                    safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, new2);
                    safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, new1);
                }
            }
            else if(safe_intervals[i2][j2][j].first >= interval.first && safe_intervals[i2][j2][j].first < interval.second)
            {
                if(safe_intervals[i2][j2][j].first == interval.first)
                {
                    safe_intervals[i2][j2].insert(safe_intervals[i2][j2].begin() + j, {safe_intervals[i2][j2][j].first,safe_intervals[i2][j2][j].first});
                    j++;
                }
                if(safe_intervals[i2][j2][j].second < interval.second)
                    safe_intervals[i2][j2].erase(safe_intervals[i2][j2].begin() + j);
                else
                    safe_intervals[i2][j2][j].first = interval.second;
            }
        }
    }
}

std::vector<std::pair<int,int>> Constraints_o::findConflictCells(Node begin, Node end)
{
    std::vector<std::pair<int,int>> cells(0);
    int i1 = begin.i, j1 = begin.j, i2 = end.i, j2 = end.j;
    int delta_i = std::abs(i1 - i2);
    int delta_j = std::abs(j1 - j2);
    int step_i = (i1 < i2 ? 1 : -1);
    int step_j = (j1 < j2 ? 1 : -1);
    int error = 0;
    int i = i1;
    int j = j1;
    int sep_value = delta_i*delta_i + delta_j*delta_j;
    if((delta_i + delta_j) == 0)//this situation is possible after modification of hppath and is needed for addConstraints function
        cells.push_back({i,j});
    else if(delta_i == 0)
        for(; j != j2+step_j; j += step_j)
            cells.push_back({i,j});
    else if(delta_j == 0)
        for(; i != i2+step_i; i += step_i)
            cells.push_back({i,j});
    else if(delta_i > delta_j)
    {
        for(; i != i2; i += step_i)
        {
            cells.push_back({i,j});
            cells.push_back({i,j+step_j});
            error += delta_j;
            if(error > delta_i)
            {
                j += step_j;
                if(((error << 1) - delta_i - delta_j)*((error << 1) - delta_i - delta_j) < sep_value)
                    cells.push_back({i + step_i,j - step_j});
                if((3*delta_i - ((error << 1) - delta_j))*(3*delta_i - ((error << 1) - delta_j)) < sep_value)
                    cells.push_back({i,j + step_j});
                error -= delta_i;
            }
        }
        cells.push_back({i,j});
        cells.push_back({i,j+step_j});
    }
    else
    {
        for(; j != j2; j += step_j)
        {
            cells.push_back({i,j});
            cells.push_back({i+step_i,j});
            error += delta_i;
            if(error > delta_j)
            {
                i += step_i;
                if(((error << 1) - delta_i - delta_j)*((error << 1) - delta_i - delta_j) < sep_value)
                    cells.push_back({i-step_i,j+step_j});
                if((3*delta_j - ((error << 1) - delta_i))*(3*delta_j - ((error << 1) - delta_i)) < sep_value)
                    cells.push_back({i+step_i,j});
                error -= delta_j;
            }
        }
        cells.push_back({i,j});
        cells.push_back({i+step_i,j});
    }
    return cells;
}

void Constraints_o::addConstraints(const std::vector<Node> &sections)
{
    std::vector<std::pair<int,int>> cells;
    section sec(sections.back(), sections.back());
    sec.g2 = CN_INFINITY;
    constraints[sec.i1][sec.j1].push_back(sec);
    if(sections.size() == 1)
        safe_intervals[sec.i1][sec.j1].clear();
    for(int a = 1; a < sections.size(); a++)
    {
        cells = findConflictCells(sections[a-1], sections[a]);
        sec = section(sections[a-1], sections[a]);
        for(int i = 0; i < cells.size(); i++)
            constraints[cells[i].first][cells[i].second].push_back(sec);
        if(a+1 == sections.size())
            updateSafeIntervals(cells,sec,true);
        else
            updateSafeIntervals(cells,sec,false);
    }
}

int Constraints_o::checkIntersection(Point A, Point B, Point C, Point D, Point &intersec)
{
    double denom  = (D.j - C.j)*(B.i - A.i) - (D.i - C.i)*(B.j - A.j);
    double nume_a = (D.i - C.i)*(A.j - C.j) - (D.j - C.j)*(A.i - C.i);
    double nume_b = (B.i - A.i)*(A.j - C.j) - (B.j - A.j)*(A.i - C.i);
    if(denom == 0.0)
    {
        if(nume_a == 0.0 && nume_b == 0.0)
            return CN_COINCIDENT;
        return CN_PARALLEL;
    }
    double ua = nume_a / denom;
    double ub = nume_b / denom;
    if(ua >= 0.0 && ua <= 1.0 && ub >= 0.0 && ub <= 1.0)
    {
        intersec = Point{A.i + ua*(B.i - A.i), A.j + ua*(B.j - A.j)};
        return CN_INTERSECTING;
    }
    return CN_NONINTERSECTING;
}

double Constraints_o::findEAT(const Node &curNode)
{
    std::vector<std::pair<double,double>> safeIntervals(0);
    double dist(sqrt(pow(curNode.i - curNode.Parent->i,2)+pow(curNode.j - curNode.Parent->j,2))), EAT(curNode.g);
    safeIntervals.push_back({EAT, curNode.interval_end});
    std::vector<section> sections(0);
    section sec;
    std::pair<double, double> badInterval;
    std::vector<std::pair<int,int>> cells = findConflictCells(*curNode.Parent, curNode);
    for(int i = 0; i < cells.size(); i++)
        for(int j = 0; j < constraints[cells[i].first][cells[i].second].size(); j++)
        {
            sec = constraints[cells[i].first][cells[i].second][j];
            if(sec.g2 <= (safeIntervals.begin()->first - dist) || sec.g1 >= (std::min(curNode.Parent->interval_end + dist, safeIntervals.back().second)))
                continue;
            if(std::find(sections.begin(), sections.end(), sec) == sections.end())
            {
                sections.push_back(sec);
                badInterval = this->countInterval(sec, curNode);
                if(badInterval.second >= 0)
                    for(int k=0; k<safeIntervals.size(); k++)
                        if(badInterval.first < safeIntervals[k].first)
                        {
                            if(badInterval.second > safeIntervals[k].second)
                            {
                                if(safeIntervals.size() == 1)
                                    return CN_INFINITY;
                                safeIntervals.erase(safeIntervals.begin()+k);
                                k--;
                            }
                            else if(badInterval.second > safeIntervals[k].first)
                            {
                                safeIntervals[k].first = badInterval.second;
                                if(safeIntervals[k].first > curNode.Parent->interval_end + dist || safeIntervals[k].first >= curNode.best_g)
                                {
                                    if(safeIntervals.size() == 1)
                                        return CN_INFINITY;
                                    safeIntervals.erase(safeIntervals.begin()+k);
                                    k--;
                                }
                            }
                        }
                        else if(safeIntervals[k].first<badInterval.first && safeIntervals[k].second>badInterval.first)
                        {
                            safeIntervals[k].second = badInterval.first;
                            if(safeIntervals[k].second>badInterval.second && badInterval.second < curNode.best_g && badInterval.second < curNode.Parent->interval_end + dist)
                                safeIntervals.insert(safeIntervals.begin()+k+1,{badInterval.second,safeIntervals[k].second});
                        }
            }
        }
    return safeIntervals[0].first;
}
double Constraints_o::minDist(Point A, Point C, Point D)
{
    int classA=A.classify(C,D);
    if(classA==3)
        return sqrt(pow(A.i-C.i,2)+pow(A.j-C.j,2));
    else if(classA==4)
        return sqrt(pow(A.i-D.i,2)+pow(A.j-D.j,2));
    else
        return fabs((C.i-D.i)*A.j+(D.j-C.j)*A.i+(C.j*D.i-D.j*C.i))/sqrt(pow(C.i-D.i,2)+pow(C.j-D.j,2));
}

std::pair<double,double> Constraints_o::countInterval(section sec, Node curNode)
{
    Point intersec, A(curNode.Parent->i, curNode.Parent->j), B(curNode.i, curNode.j), C(sec.i1, sec.j1), D(sec.i2, sec.j2);
    int pos = checkIntersection(A, B, C, D, intersec);
    int A1(A.j - B.j), A2(C.j - D.j), B1(A.i - B.i), B2(C.i - D.i);
    double lengthAB = sqrt(pow(A.i - B.i,2)+pow(A.j-B.j,2));
    double lengthCD = sec.g2 - sec.g1;
    if(A2 == 0 && B2 == 0)//if we collide with a section, that represents wait action (or goal)
    {
        double dist_to_AB = (B1*D.j - A1*D.i + A.j*B.i - A.i*B.j)/lengthAB;
        if(dist_to_AB >= 1.0)
            return {-1, -1};
        double gap = sqrt(1.0 - pow(dist_to_AB, 2));
        double offset = sqrt(pow(dist(B, C), 2) - pow(dist_to_AB, 2));
        return {sec.g1 + offset - gap, sec.g2 + offset + gap};
    }
    if(pos == CN_COINCIDENT || pos == CN_PARALLEL)
    {
        if(pos == CN_PARALLEL)
            if(abs(B1*sec.j2 - A1*sec.i2 + curNode.Parent->j*curNode.i - curNode.Parent->i*curNode.j) >= lengthAB)//if the distance between sections is not less than 1.0 (2r), collision is immpossible
                return {-1, -1};
        double BC = dist(B, C);
        if(A1*A2 >= 0 && B1*B2 >= 0)//if sections are co-directional
            return {sec.g1 + BC - 1, sec.g1 + BC + 1};
        if((A.i - C.i)*(A.i - D.i) <= 0 && (A.j - C.j)*(A.j - D.j) <= 0)//A inside CD
        {
            if((B.i - C.i)*(B.i - D.i) <= 0 && (B.j - C.j)*(B.j - D.j) <= 0)//B inside CD => AB is fully in CD
                return {sec.g1 + BC - 2, sec.g1 + BC + 2*lengthAB + 2};
            else
                return {sec.g1 + BC - 2, sec.g1 + BC + 2*(lengthAB - BC) + 2};
        }
        else//A outside of CD
        {
            if((B.i - C.i)*(B.i - D.i) <= 0 && (B.j - C.j)*(B.j - D.j) <= 0)//B inside CD
                return {sec.g1 + BC - 2, sec.g1 + BC + 2*(lengthCD - BC) + 2};
            else
                return {sec.g1 + BC - 2, sec.g1 + BC + 2*lengthCD + 2};
        }
    }
    else if(pos == CN_NONINTERSECTING)
    {
        double A_CD(minDist(A, C, D)), B_CD(minDist(B, C, D)), C_AB(minDist(C, A, B)), D_AB(minDist(D, A, B));
        if(std::min(std::min(A_CD, B_CD), std::min(C_AB, D_AB)) >= 1.0)
            return {-1,-1};

        intersec.i = ((C.i*D.j - C.j*D.i)*B1 - B2*(A.i*B.j - A.j*B.i))/((C.i - D.i)*(A.j - B.j) - (C.j - D.j)*(A.i - B.i));
        intersec.j = ((C.i*D.j - C.j*D.i)*A1 - A2*(A.i*B.j - A.j*B.i))/((C.i - D.i)*(A.j - B.j) - (C.j - D.j)*(A.i - B.i));
        int classAB = intersec.classify(A, B);
        int classCD = intersec.classify(C, D);
        double span = sqrt(2.0/((A1*A2 + B1*B2)/(sqrt(A1*A1 + B1*B1)*sqrt(A2*A2 + B2*B2)) + 1.0));
        std::pair<double, double> interval, interval2(-1,-1);
        if(classAB == 3 && classCD == 4)//intersection point is behind AB and beyond CD
        {
            double dist_A(sqrt(pow(A.i - intersec.i,2) + pow(A.j - intersec.j,2))),
                   dist_B(sqrt(pow(B.i - intersec.i,2) + pow(B.j - intersec.j,2))),
                   dist_C(sqrt(pow(C.i - intersec.i,2) + pow(C.j - intersec.j,2))),
                   dist_D(sqrt(pow(D.i - intersec.i,2) + pow(D.j - intersec.j,2))),
                   gap, offset;
            if(dist_A > dist_D)
            {
                gap = sqrt(1.0 - pow(A_CD, 2));
                offset = sqrt(pow(A.i - C.i, 2) + pow(A.j - C.j, 2) - pow(A_CD, 2)) + lengthAB;
                interval = {sec.g1 + offset - gap, sec.g1 + offset + gap};
            }
            else
            {
                gap = sqrt(1.0 - pow(D_AB, 2));
                offset = sqrt(pow(B.i - D.i, 2) + pow(B.j - D.j, 2) - pow(D_AB, 2));
                interval = {sec.g2 + offset - gap, sec.g2 + offset + gap};
            }
            if(std::min(dist_B,dist_C)*2>span)
                return {sec.g1 + dist_C + dist_B - span, interval.second};
            else if(dist_B<dist_C)
            {
                gap = sqrt(1.0 - pow(B_CD, 2));
                offset = sqrt(pow(B.i - C.i,2) + pow(B.j - C.j, 2) - pow(B_CD, 2));
                interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
            }
            else
            {
                gap = sqrt(1 - pow(C_AB, 2));
                offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(C_AB, 2));
                interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
            }
        }
        else if(classAB == 4 && classCD == 3)
        {
            double dist_A(sqrt(pow(A.i - intersec.i, 2) + pow(A.j - intersec.j, 2))),
                   dist_B(sqrt(pow(B.i - intersec.i, 2) + pow(B.j - intersec.j, 2))),
                   dist_C(sqrt(pow(C.i - intersec.i, 2) + pow(C.j - intersec.j, 2))),
                   dist_D(sqrt(pow(D.i - intersec.i, 2) + pow(D.j - intersec.j, 2))),
                   gap, offset;
            if(dist_B > dist_C)
            {
                gap = sqrt(1.0 - pow(B_CD, 2));
                offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(B_CD, 2));
                interval = {sec.g1 + offset - gap, sec.g1 + offset + gap};
            }
            else
            {
                gap = sqrt(1.0 - pow(C_AB, 2));
                offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(C_AB, 2));
                interval = {sec.g1 + offset - gap, sec.g1 + offset + gap};
            }
            if(std::min(dist_A, dist_D)*2 > span)
                return {interval.first, sec.g1 - dist_C - dist_B + span};
            else if(dist_A < dist_D)
            {
                gap = sqrt(1.0 - pow(A_CD, 2));
                offset = sqrt(pow(dist(A, C), 2) - pow(A_CD, 2)) + lengthAB;
                interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
            }
            else
            {
                gap = sqrt(1.0 - pow(D_AB, 2));
                offset = sqrt(pow(dist(B, D), 2) - pow(D_AB, 2));
                interval2 = {sec.g2 + offset - gap, sec.g2 + offset + gap};
            }
        }
        else if(classAB==3 && classCD==3)//intersection point is before both sections
        {
            double dist_A(sqrt(pow(A.i - intersec.i, 2) + pow(A.j - intersec.j, 2))),
                   dist_B(sqrt(pow(B.i - intersec.i, 2) + pow(B.j - intersec.j, 2))),
                   dist_C(sqrt(pow(C.i - intersec.i, 2) + pow(C.j - intersec.j, 2))),
                   dist_D(sqrt(pow(D.i - intersec.i, 2) + pow(D.j - intersec.j, 2))),
                   gap, offset;
            if(dist_A>dist_C)
            {
                gap = sqrt(1.0 - pow(A_CD, 2));
                offset = sqrt(pow(A.i - C.i, 2) + pow(A.j - C.j, 2) - pow(A_CD, 2)) + lengthAB;
                interval = {sec.g1 + offset - gap, sec.g1 + offset + gap};
            }
            else
            {
                gap = sqrt(1.0 - pow(C_AB, 2));
                offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(C_AB, 2));
                interval = {sec.g1 + offset - gap, sec.g1 + offset + gap};
            }
            if(std::min(dist_B, dist_D)*2 > span)
                return {sec.g1 - dist_C + dist_B - span, interval.second};
            else if(dist_B < dist_D)
            {
                gap = sqrt(1.0 - pow(B_CD, 2));
                offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(B_CD, 2));
                interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
            }
            else
            {
                gap = sqrt(1.0 - pow(D_AB, 2));
                offset = sqrt(pow(B.i - D.i, 2) + pow(B.j - D.j, 2) - pow(D_AB, 2));
                interval2 = {sec.g2 + offset - gap, sec.g2 + offset + gap};
            }
        }
        else if(classAB == 4 && classCD == 4)//intersection point is beyond both sections
        {
            double dist_A(sqrt(pow(A.i - intersec.i, 2) + pow(A.j - intersec.j, 2))),
                   dist_B(sqrt(pow(B.i - intersec.i, 2) + pow(B.j - intersec.j, 2))),
                   dist_C(sqrt(pow(C.i - intersec.i, 2) + pow(C.j - intersec.j, 2))),
                   dist_D(sqrt(pow(D.i - intersec.i, 2) + pow(D.j - intersec.j, 2))),
                   gap, offset;
            if(dist_B > dist_D)
            {
                gap = sqrt(1.0 - pow(B_CD, 2));
                offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(B_CD, 2));
                interval = {sec.g1 + offset - gap, sec.g1 + offset + gap};
            }
            else
            {
                gap = sqrt(1.0 - pow(D_AB, 2));
                offset = sqrt(pow(B.i - D.i, 2) + pow(B.j - D.j, 2) - pow(D_AB, 2));
                interval = {sec.g2 + offset - gap, sec.g2 + offset + gap};
            }
            if(std::min(dist_A, dist_C)*2>span)
                return {sec.g2 + dist_D - dist_B - span, interval.second};
            else if(dist_A < dist_C)
            {
                gap = sqrt(1.0 - pow(A_CD, 2));
                offset = sqrt(pow(A.i - C.i, 2) +pow(A.j - C.j, 2) - pow(A_CD, 2)) + lengthAB;
                interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
            }
            else
            {
                gap = sqrt(1 - pow(C_AB, 2));
                offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(C_AB, 2));
                interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
            }
        }
        else if(classAB == 4)//BEYOND (AFTER B)
        {
            double gap = sqrt(1.0 - pow(B_CD, 2));
            double offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(B_CD, 2));
            interval = {sec.g1 + offset - gap, sec.g1 + offset + gap};

            if((span - sqrt(2.0)) < CN_EPSILON)
            {
                double dist_A = sqrt(pow(A.i - intersec.i, 2) + pow(A.j - intersec.j, 2));
                double dist_C = sqrt(pow(C.i - intersec.i, 2) + pow(C.j - intersec.j, 2));
                if(std::min(dist_A, dist_C)*2> span)
                {
                    if(dist(B, intersec)*2 < span)
                        return {interval.first, sec.g1 + dist_C - dist(B, intersec) + span};
                    else
                        return interval;
                }
                else if(dist_A < dist_C)
                {
                    gap = sqrt(1.0 - pow(A_CD, 2));
                    offset = sqrt(pow(A.i - C.i, 2) + pow(A.j - C.j, 2) - pow(A_CD, 2)) + lengthAB;
                    interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
                }
                else
                {
                    gap = sqrt(1 - pow(C_AB, 2));
                    offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(C_AB, 2));
                    interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
                }
            }
            else
            {
                double dist_A = sqrt(pow(A.i - intersec.i, 2) + pow(A.j - intersec.j, 2));
                double dist_D = sqrt(pow(D.i - intersec.i, 2) + pow(D.j - intersec.j, 2));
                if(std::min(dist_A, dist_D)*2 > span)
                {
                    if(dist(B, intersec)*2 < span)
                        return {interval.first, sec.g1 + dist(C, intersec) - dist(B, intersec) + span};
                    else
                        return interval;
                }
                else if(dist_A < dist_D)
                {
                    gap = sqrt(1.0 - pow(A_CD, 2));
                    offset = sqrt(pow(A.i - C.i, 2) + pow(A.j - C.j, 2) - pow(A_CD, 2)) + lengthAB;
                    interval2 = {sec.g1 + offset - gap,sec.g1 + offset + gap};
                }
                else
                {
                    gap = sqrt(1.0 - pow(D_AB, 2));
                    offset = sqrt(pow(B.i - D.i, 2) + pow(B.j - D.j, 2) - pow(D_AB, 2));
                    interval2 = {sec.g2 + offset - gap,sec.g2 + offset + gap};
                }
            }

        }
        else if(classAB == 3)//BEHIND (BEFORE A)
        {
            double gap = sqrt(1.0 - pow(A_CD, 2));
            double offset = sqrt(pow(A.i - C.i, 2) + pow(A.j - C.j, 2) - pow(A_CD, 2)) + lengthAB;
            interval = {sec.g1 + offset - gap, sec.g1 + offset + gap};
            if((span - sqrt(2.0)) < CN_EPSILON)
            {
                double dist_B = sqrt(pow(B.i - intersec.i, 2) + pow(B.j - intersec.j, 2));
                double dist_D = sqrt(pow(D.i - intersec.i, 2) + pow(D.j - intersec.j, 2));
                if(std::min(dist_B, dist_D)*2 > span)
                {
                    if(dist(A, intersec)*2 < span)
                        return {sec.g1 + dist(C, intersec) + dist_B - span, interval.second};
                    else
                        return interval;
                }
                else if(dist_B < dist_D)
                {
                    gap = sqrt(1.0 - pow(B_CD, 2));
                    offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(B_CD, 2));
                    interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
                }
                else
                {
                    gap = sqrt(1.0 - pow(D_AB, 2));
                    offset = sqrt(pow(B.i - D.i, 2) + pow(B.j - D.j, 2) - pow(D_AB, 2));
                    interval2 = {sec.g2 + offset - gap, sec.g2 + offset + gap};
                }
            }
            else
            {
                double dist_B = sqrt(pow(B.i - intersec.i, 2) + pow(B.j - intersec.j, 2));
                double dist_C = sqrt(pow(C.i - intersec.i, 2) + pow(C.j - intersec.j, 2));
                if(std::min(dist_B, dist_C)*2 > span)
                {
                    if(dist(A, intersec)*2 < span)
                        return {sec.g1 + dist_C + dist_B - span, interval.second};
                    else
                        return interval;
                }
                else if(dist_B < dist_C)
                {
                    gap = sqrt(1.0 - pow(B_CD, 2));
                    offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(B_CD, 2));
                    interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
                }
                else
                {
                    gap = sqrt(1.0 - pow(C_AB, 2));
                    offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(C_AB, 2));
                    interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
                }
            }
        }
        else if(classCD == 4)//BEYOND (AFTER D)
        {
            double gap = sqrt(1.0 - pow(D_AB, 2));
            double offset = sqrt(pow(B.i - D.i, 2) + pow(B.j - D.j, 2) - pow(D_AB, 2));
            interval = {sec.g2 + offset - gap, sec.g2 + offset + gap};

            if((span - sqrt(2.0)) < CN_EPSILON)
            {
                double dist_A = sqrt(pow(A.i - intersec.i, 2) + pow(A.j - intersec.j, 2));
                double dist_C = sqrt(pow(C.i - intersec.i, 2) + pow(C.j - intersec.j, 2));
                if(std::min(dist_A, dist_C)*2 > span)
                {
                    if(dist(D, intersec)*2 < span)
                        return {sec.g2 + dist(D, intersec) + dist(B, intersec) - span, interval.second};
                    else
                        return interval;
                }
                else if(dist_A < dist_C)
                {
                    gap = sqrt(1.0 - pow(A_CD, 2));
                    offset = sqrt(pow(A.i - C.i, 2) + pow(A.j - C.j, 2) - pow(A_CD, 2)) + lengthAB;
                    interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
                }
                else
                {
                    gap = sqrt(1.0 - pow(C_AB, 2));
                    offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(C_AB, 2));
                    interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
                }
            }
            else
            {
                double dist_B = sqrt(pow(B.i - intersec.i, 2) + pow(B.j - intersec.j, 2));
                double dist_C = sqrt(pow(C.i - intersec.i, 2) + pow(C.j - intersec.j, 2));
                if(std::min(dist_B, dist_C)*2 > span)
                {
                    if(dist(D, intersec)*2 < span)
                        return {sec.g2 + dist(D, intersec) + dist_B - span, interval.second};
                    else
                        return interval;
                }
                else if(dist_B < dist_C)
                {
                    gap = sqrt(1.0 - pow(B_CD, 2));
                    offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(B_CD, 2));
                    interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
                }
                else
                {
                    gap = sqrt(1.0 - pow(C_AB, 2));
                    offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(C_AB, 2));
                    interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
                }
            }
        }
        else if(classCD == 3)//BEHIND (BEFORE C)
        {
            double gap = sqrt(1.0 - pow(C_AB, 2));
            double offset = sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(C_AB, 2));
            interval = {sec.g1 + offset - gap, sec.g1 + offset + gap};
            if((span - sqrt(2.0)) < CN_EPSILON)//if sections are co-directional
            {
                double dist_B = sqrt(pow(B.i - intersec.i, 2) + pow(B.j - intersec.j, 2));
                double dist_D = sqrt(pow(D.i - intersec.i, 2) + pow(D.j - intersec.j, 2));
                if(std::min(dist_B, dist_D)*2 > span)
                {
                    if(dist(C, intersec)*2 < span)
                        return {interval.first, sec.g1 - dist(C, intersec) + dist_B + span};
                    else
                        return interval;
                }
                else if(dist_B < dist_D)
                {
                    gap = sqrt(1.0 - pow(B_CD, 2));
                    offset = sqrt(pow(B.i - C.i,2) + pow(B.j - C.j, 2) - pow(B_CD, 2));
                    interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
                }
                else
                {
                    gap = sqrt(1.0 - pow(D_AB, 2));
                    offset = sqrt(pow(B.i - D.i, 2) + pow(B.j - D.j, 2) - pow(D_AB, 2));
                    interval2 = {sec.g2 + offset - gap, sec.g2 + offset + gap};
                }
            }
            else
            {
                double dist_A = sqrt(pow(A.i - intersec.i, 2) + pow(A.j - intersec.j, 2));
                double dist_D = sqrt(pow(D.i - intersec.i,2) + pow(D.j - intersec.j, 2));
                if(std::min(dist_A, dist_D)*2 > span)
                {
                    if(dist(C, intersec)*2 < span)
                        return {interval.first, sec.g1 - dist(C, intersec) + dist(B, intersec) + span};
                    else
                        return interval;
                }
                else if(dist_A<dist_D)
                {
                    gap = sqrt(1.0 - pow(A_CD, 2));
                    offset = sqrt(pow(A.i - C.i,2) + pow(A.j - C.j, 2) - pow(A_CD, 2)) + lengthAB;
                    interval2 = {sec.g1 + offset - gap, sec.g1 + offset + gap};
                }
                else
                {
                    gap = sqrt(1.0 - pow(D_AB, 2));
                    offset = sqrt(pow(B.i - D.i, 2) + pow(B.j - D.j, 2) - pow(D_AB, 2));
                    interval2 = {sec.g2 + offset - gap, sec.g2 + offset + gap};
                }
            }
        }
        return {std::min(interval.first, interval2.first), std::max(interval.second, interval2.second)};
    }
    else//have intersection point
    {

        double dist_A(sqrt(pow(A.i - intersec.i, 2) + pow(A.j - intersec.j, 2))),
               dist_B(sqrt(pow(B.i - intersec.i, 2) + pow(B.j - intersec.j, 2))),
               dist_C(sqrt(pow(C.i - intersec.i, 2) + pow(C.j - intersec.j, 2))),
               dist_D(sqrt(pow(D.i - intersec.i, 2) + pow(D.j - intersec.j, 2))),
               span = sqrt(2.0/((A1*A2 + B1*B2)/(sqrt(A1*A1 + B1*B1)*sqrt(A2*A2 + B2*B2)) + 1.0)),
               dist;
        std::pair<double,double> interval(sec.g1 + dist_C + dist_B - span, sec.g1 + dist_C + dist_B + span);
        if(std::min(dist_A, dist_D)*2 < span)
        {
            if(dist_A < dist_D)
            {
                dist = ((C.i - D.i)*A.j + (D.j - C.j)*A.i + (C.j*D.i - D.j*C.i))/lengthCD;
                interval.second = sec.g1 + sqrt(pow(A.i - C.i, 2) + pow(A.j - C.j, 2) - pow(dist, 2)) + lengthAB + sqrt(1.0 - pow(dist, 2));
            }
            else
            {
                dist = ((A.i - B.i)*D.j + (B.j - A.j)*D.i + (A.j*B.i - A.i*B.j))/lengthAB;
                interval.second = sec.g2 + sqrt(pow(B.i - D.i, 2) + pow(B.j - D.j, 2) - pow(dist, 2)) + sqrt(1.0 - pow(dist, 2));
            }
        }
        if(std::min(dist_B, dist_C)*2 < span)
        {
            if(dist_B < dist_C)
                dist = ((C.i - D.i)*B.j + (D.j - C.j)*B.i + (C.j*D.i - D.j*C.i))/lengthCD;
            else
                dist = ((A.i - B.i)*C.j + (B.j - A.j)*C.i + (A.j*B.i - A.i*B.j))/lengthAB;
            interval.first = sec.g1 + sqrt(pow(B.i - C.i, 2) + pow(B.j - C.j, 2) - pow(dist, 2)) - sqrt(1.0 - pow(dist, 2));
        }
        return interval;
    }
}
