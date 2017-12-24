#ifndef AA_SIPP_O_H
#define AA_SIPP_O_H

#include "cSearch.h"
#include "constraints_o.h"
#include "constraints.h"
#include "StatesContainer.h"
#include <iomanip>
class AA_SIPP_O
{

public:

    AA_SIPP_O(double weight, bool breakingties);
    ~AA_SIPP_O();
    ResultPathInfo findPath(cLogger *Log, const SearchResult &sresult, const cMap &Map);
    void copy_constraints(const SectionConstraints &cons){constraints.copy(cons.safe_intervals, cons.constraints);}

private:

    double calculateDistanceFromCellToCell(double start_i, double start_j, double fin_i, double fin_j);
    void findSuccessors(const Node curNode, const cMap &Map, std::list<Node> &succs, int numOfCurAgent){}
    void makePrimaryPath(Node curNode);
    void makeSecondaryPath(Node curNode);
    void calculateLineSegment(std::vector<Node> &line, const Node &start, const Node &goal);
    void initStates(int numOfCurAgent, const cMap &Map);
    void addConstraints(){}
    std::vector<conflict> CheckConflicts();//bruteforce checker. It splits final(already built) trajectories into sequences of points and checks distances between them
    double weight;
    bool breakingties;
    int constraints_type;
    unsigned int closeSize, openSize;
    std::list<Node> lppath;
    std::vector<Node> hppath;
    Constraints_o constraints;
    StatesContainer states;

};

#endif // AA_SIPP_O_H
