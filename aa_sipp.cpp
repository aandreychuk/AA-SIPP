#include "aa_sipp.h"


AA_SIPP::AA_SIPP(double weight, bool breakingties)
{
    this->weight = weight;
    this->breakingties = breakingties;
    closeSize = 0;
    openSize = 0;
}

AA_SIPP::~AA_SIPP()
{
}


bool AA_SIPP::lineOfSight(int i1, int j1, int i2, int j2, const cMap &map)
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

double AA_SIPP::calculateDistanceFromCellToCell(double start_i, double start_j, double fin_i, double fin_j)
{
    return sqrt(double((start_i - fin_i)*(start_i - fin_i) + (start_j - fin_j)*(start_j - fin_j)));
}

SearchResult AA_SIPP::startSearch(cLogger *Log, cMap &Map)
{
#ifdef __linux__
    timeval begin, end;
    gettimeofday(&begin, NULL);
#else
    LARGE_INTEGER begin, end, freq;
    QueryPerformanceCounter(&begin);
    QueryPerformanceFrequency(&freq);
#endif
    constraints.init(Map.width, Map.height);
    sresult.pathInfo.resize(0);
    sresult.agents = Map.agents;
    sresult.agentsSolved = 0;
    for(int i = 0; i < Map.agents; i++)
    {
        Map.addConstraint(Map.start_i[i], Map.start_j[i]);
        Map.addConstraint(Map.goal_i[i], Map.goal_j[i]);
    }
    for(int numOfCurAgent = 0; numOfCurAgent < Map.agents; numOfCurAgent++)
    {
        std::cout<<numOfCurAgent<<"\n";
        for(int i = numOfCurAgent; i < Map.agents; i++)
        {
            Map.addConstraint(Map.start_i[i], Map.start_j[i]);
            Map.addConstraint(Map.goal_i[i], Map.goal_j[i]);
        }
        Map.removeConstraint(Map.start_i[numOfCurAgent], Map.start_j[numOfCurAgent]);
        Map.removeConstraint(Map.goal_i[numOfCurAgent], Map.goal_j[numOfCurAgent]);
        if(findPath(numOfCurAgent, Map))
            constraints.addConstraints(sresult.pathInfo.back().sections);
    }
#ifdef __linux__
    gettimeofday(&end, NULL);
    sresult.time = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
    QueryPerformanceCounter(&end);
    sresult.time = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
#endif
    std::vector<conflict> confs = CheckConflicts();
    for(int i = 0; i < confs.size(); i++)
        std::cout<<confs[i].i<<" "<<confs[i].j<<" "<<confs[i].g<<" "<<confs[i].agent1<<" "<<confs[i].agent2<<"\n";
    return sresult;
}

void AA_SIPP::initStates(int numOfCurAgent, const cMap &Map)
{
    std::vector<std::pair<double, double>> begins(0);
    double g, h(calculateDistanceFromCellToCell(Map.start_i[numOfCurAgent], Map.start_j[numOfCurAgent], Map.goal_i[numOfCurAgent], Map.goal_j[numOfCurAgent]));
    Node n(Map.start_i[numOfCurAgent], Map.start_j[numOfCurAgent],h,0,h,true,0,CN_INFINITY,1);
    states.insert(n);
    auto parent = states.getParentPtr();
    for(int i = 0; i < Map.height; i++)
        for(int j = 0; j < Map.width; j++)
        {
            begins = constraints.getSafeBegins(i, j);
            g = calculateDistanceFromCellToCell(i, j, Map.start_i[numOfCurAgent], Map.start_j[numOfCurAgent]);
            h = calculateDistanceFromCellToCell(i, j, Map.goal_i[numOfCurAgent], Map.goal_j[numOfCurAgent]);
            n = Node(i,j,g+h,g,h,false);
            n.Parent = parent;
            for(int k = 0; k < begins.size(); k++)
            {
                n.interval_begin = begins[k].first;
                n.interval_end = begins[k].second;
                if(n.interval_begin > n.g)
                {
                    n.g = n.interval_begin;
                    n.F = n.g + n.h;
                }
                n.parents.clear();
                n.parents.push_back({parent, n.g + n.h});
                if(n.g <= n.interval_end)
                    states.insert(n);
            }
        }
    h = (calculateDistanceFromCellToCell(Map.start_i[numOfCurAgent], Map.start_j[numOfCurAgent], Map.goal_i[numOfCurAgent], Map.goal_j[numOfCurAgent]));
    n = Node(Map.start_i[numOfCurAgent], Map.start_j[numOfCurAgent],h,0,h,true,0,CN_INFINITY);
    n.best_g = h;
    states.expand(n);
}

bool AA_SIPP::findPath(int numOfCurAgent, const cMap &Map)
{

#ifdef __linux__
    timeval begin, end;
    gettimeofday(&begin, NULL);
#else
    LARGE_INTEGER begin, end, freq;
    QueryPerformanceCounter(&begin);
    QueryPerformanceFrequency(&freq);
#endif
    ResultPathInfo resultPath;
    states.clear();
    initStates(numOfCurAgent, Map);
    Node curNode(states.getMin());
    Node newNode;
    bool pathFound(false);
    while(!curNode.expanded && curNode.g < CN_INFINITY)//if expanded => there is no any non-expanded node, i.e. OPEN is empty //if curNode.g=CN_INFINITY => there are only unreachable non-consistent states => path cannot be found
    {
        newNode = curNode;
        newNode.g = constraints.findEAT(newNode, states);
        if(newNode.g > newNode.best_g || newNode.g == CN_INFINITY)
        {
            states.update(newNode, false);
        }
        else
        {
            newNode.best_g = newNode.g;
            newNode.best_Parent = newNode.Parent;
            states.update(newNode, true);
        }
        curNode = states.getMin();
        if(newNode.best_g + newNode.h - curNode.F < CN_EPSILON && newNode.best_g < CN_INFINITY)
        {
            states.expand(newNode);
            states.updateNonCons(newNode);
            if(newNode.i == Map.goal_i[numOfCurAgent] && newNode.j == Map.goal_j[numOfCurAgent] && newNode.interval_end == CN_INFINITY)
            {
                pathFound = true;
                break;
            }
            if(curNode.i == newNode.i && curNode.j == newNode.j && curNode.interval_begin == curNode.interval_begin)
                curNode = states.getMin();
        }
    }
    if(pathFound)
    {
        makePrimaryPath(newNode);
        for(int i = 1; i < hppath.size(); i++)
            if((hppath[i].g - (hppath[i - 1].g + calculateDistanceFromCellToCell(hppath[i].i, hppath[i].j, hppath[i - 1].i, hppath[i - 1].j))) > CN_EPSILON)
            {
                Node add = hppath[i - 1];
                add.g = hppath[i].g - calculateDistanceFromCellToCell(hppath[i].i, hppath[i].j, hppath[i - 1].i,hppath[i - 1].j);
                hppath.emplace(hppath.begin() + i, add);
                i++;
            }

#ifdef __linux__
        gettimeofday(&end, NULL);
        resultPath.time = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
        QueryPerformanceCounter(&end);
        resultPath.time = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
#endif
        resultPath.sections = hppath;
        makeSecondaryPath(newNode);
        resultPath.nodescreated = openSize + closeSize;
        resultPath.pathfound = true;
        resultPath.path = lppath;
        resultPath.numberofsteps = closeSize;
        resultPath.pathlength = newNode.g;
        sresult.pathfound = true;
        sresult.pathlength += newNode.g;
        sresult.nodescreated += openSize + closeSize;
        sresult.numberofsteps += closeSize;
        sresult.pathInfo.push_back(resultPath);
        sresult.agentsSolved++;
    }
    else
    {
#ifdef __linux__
        gettimeofday(&end, NULL);
        resultPath.time = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
        QueryPerformanceCounter(&end);
        resultPath.time = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
#endif
        std::cout<<numOfCurAgent<<" PATH NOT FOUND!\n";
        //sresult.pathfound = false;
        sresult.nodescreated += closeSize;
        sresult.numberofsteps += closeSize;
        resultPath.nodescreated = closeSize;
        resultPath.pathfound = false;
        resultPath.path.clear();
        resultPath.sections.clear();
        resultPath.pathlength = 0;
        resultPath.numberofsteps = closeSize;
        sresult.pathInfo.push_back(resultPath);
    }
    return resultPath.pathfound;
}

std::vector<conflict> AA_SIPP::CheckConflicts()
{
    std::vector<conflict> conflicts(0);
    conflict conf;
    Node cur, check;
    std::vector<std::vector<conflict>> positions;
    positions.resize(sresult.agents);
    for(int i = 0; i < sresult.agents; i++)
    {
        if(!sresult.pathInfo[i].pathfound)
            continue;
        positions[i].resize(0);
        int k = 0;
        double part = 1;
        for(int j = 1; j<sresult.pathInfo[i].sections.size(); j++)
        {
            cur = sresult.pathInfo[i].sections[j];
            check = sresult.pathInfo[i].sections[j-1];
            int di = cur.i - check.i;
            int dj = cur.j - check.j;
            double dist = (cur.g - check.g)*10;
            int steps = (cur.g - check.g)*10;
            if(dist - steps + part >= 1)
            {
                steps++;
                part = dist - steps;
            }
            else
                part += dist - steps;
            double stepi = double(di)/dist;
            double stepj = double(dj)/dist;
            double curg = double(k)*0.1;
            double curi = check.i + (curg - check.g)*di/(cur.g - check.g);
            double curj = check.j + (curg - check.g)*dj/(cur.g - check.g);
            conf.i = curi;
            conf.j = curj;
            conf.g = curg;
            if(curg <= cur.g)
            {
                positions[i].push_back(conf);
                k++;
            }
            while(curg <= cur.g)
            {
                if(curg + 0.1 > cur.g)
                    break;
                curi += stepi;
                curj += stepj;
                curg += 0.1;
                conf.i = curi;
                conf.j = curj;
                conf.g = curg;
                positions[i].push_back(conf);
                k++;
            }
        }
        if(double(k - 1)*0.1 < sresult.pathInfo[i].sections.back().g)
        {
            conf.i = sresult.pathInfo[i].sections.back().i;
            conf.j = sresult.pathInfo[i].sections.back().j;
            conf.g = sresult.pathInfo[i].sections.back().g;
            positions[i].push_back(conf);
        }
    }
    int max = 0;
    for(int i = 0; i < positions.size(); i++)
        if(positions[i].size() > max)
            max = positions[i].size();
    for(int i = 0; i < sresult.agents; i++)
    {
        for(int k = 0; k < max; k++)
        {
            for(int j = i + 1; j < sresult.agents; j++)
            {
                if(!sresult.pathInfo[j].pathfound || !sresult.pathInfo[i].pathfound)
                    continue;
                conflict a, b;
                if(positions[i].size() > k)
                    a = positions[i][k];
                else
                    a = positions[i].back();
                if(positions[j].size() > k)
                    b = positions[j][k];
                else
                    b = positions[j].back();
                if(sqrt((a.i - b.i)*(a.i - b.i) + (a.j - b.j)*(a.j - b.j)) + CN_EPSILON < 1.0)
                {
                   // std::cout<<a.i<<" "<<a.j<<" "<<b.i<<" "<<b.j<<" "<<sqrt((a.i - b.i)*(a.i - b.i) + (a.j - b.j)*(a.j - b.j))<<"\n";
                    conf.i = b.i;
                    conf.j = b.j;
                    conf.agent1 = i;
                    conf.agent2 = j;
                    conf.g = b.g;
                    conflicts.push_back(conf);
                }
            }
        }
    }
    return conflicts;
}

void AA_SIPP::makePrimaryPath(Node curNode)
{
    hppath.clear();
    hppath.shrink_to_fit();
    std::list<Node> path;
    Node n(curNode.i, curNode.j, curNode.F, curNode.g);
    path.push_front(n);
    if(curNode.Parent != nullptr)
    {
        curNode = *curNode.Parent;
        if(curNode.Parent != nullptr)
        {
            do
            {
                Node n(curNode.i, curNode.j, curNode.F, curNode.g);
                path.push_front(n);
                curNode = *curNode.Parent;
            }
            while(curNode.Parent != nullptr);
        }
        Node n(curNode.i, curNode.j, curNode.F, curNode.g);
        path.push_front(n);
    }
    for(auto it = path.begin(); it != path.end(); it++)
        hppath.push_back(*it);
    return;
}

void AA_SIPP::makeSecondaryPath(Node curNode)
{
    lppath.clear();
    if(curNode.Parent != nullptr)
    {
        std::vector<Node> lineSegment;
        do
        {
            calculateLineSegment(lineSegment, *curNode.Parent, curNode);
            lppath.insert(lppath.begin(), ++lineSegment.begin(), lineSegment.end());
            curNode = *curNode.Parent;
        }
        while(curNode.Parent != nullptr);
        lppath.push_front(*lineSegment.begin());
    }
    else
        lppath.push_front(curNode);
}

void AA_SIPP::calculateLineSegment(std::vector<Node> &line, const Node &start, const Node &goal)
{
    int i1 = start.i;
    int i2 = goal.i;
    int j1 = start.j;
    int j2 = goal.j;

    int delta_i = std::abs(i1 - i2);
    int delta_j = std::abs(j1 - j2);
    int step_i = (i1 < i2 ? 1 : -1);
    int step_j = (j1 < j2 ? 1 : -1);
    int error = 0;
    int i = i1;
    int j = j1;
    if (delta_i > delta_j)
    {
        for (; i != i2; i += step_i)
        {
            line.push_back(Node(i,j));
            error += delta_j;
            if ((error << 1) > delta_i)
            {
                j += step_j;
                error -= delta_i;
            }
        }
    }
    else
    {
        for (; j != j2; j += step_j)
        {
            line.push_back(Node(i,j));
            error += delta_i;
            if ((error << 1) > delta_j)
            {
                i += step_i;
                error -= delta_j;
            }
        }
    }
    return;
}
