#ifndef multi_index_H
#define multi_index_H
#include "structs.h"
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/composite_key.hpp>
#include <boost/tuple/tuple.hpp>
using namespace boost::multi_index;
struct by_ij;
struct by_fg;
struct by_cons;
typedef multi_index_container<
    Node,
    indexed_by<
        ordered_unique<
            tag<by_ij>,
            composite_key<
                Node,
                member<Node, int, &Node::i>,
                member<Node, int, &Node::j>,
                member<Node, double, &Node::interval_begin>
            >
        >,
        ordered_non_unique<
            tag<by_fg>,
            composite_key<
                Node,
                member<Node, bool,   &Node::expanded>,
                member<Node, double, &Node::F>,
                member<Node, double, &Node::h>
            >
        >,
        ordered_non_unique<
            tag<by_cons>,
            member<Node, int, &Node::consistent>
        >

    >
> multi_index;

class StatesContainer
{
public:
    multi_index states;
    struct updateExpand
    {
        updateExpand(){}
        void operator()(Node& n)
        {
            n.expanded = true;
            n.consistent = 1;
            n.g = n.best_g;
            n.Parent = n.best_Parent;
        }
    };
    struct addParent
    {
        addParent(const Node* parent, double new_g):parent(parent), new_g(new_g){}
        void operator()(Node& n)
        {
            if(n.parents.empty())
                n.parents.push_front({parent, new_g});
            else if(n.parents.back().second - new_g < CN_EPSILON)
                n.parents.push_back({parent, new_g});
            else
                for(auto it = n.parents.begin(); it != n.parents.end(); it++)
                    if(it->second - new_g > CN_EPSILON)
                    {
                        n.parents.insert(it,{parent, new_g});
                        break;
                    }
        }
        private:
        const Node* parent;
        double new_g;
    };

    struct updateFG
    {
        updateFG(double g, const Node* parent):g(g), parent(parent){}
        void operator()(Node& n)
        {
            n.F = g + n.h;
            n.g = g;
            if(n.consistent == 0)
                n.consistent = 2;
            n.Parent = parent;
        }

    private:
        double g;
        const Node* parent;
    };

    struct updateBest
    {
        updateBest(double g, const Node* parent):g(g),parent(parent){}
        void operator()(Node& n)
        {
            n.best_Parent = parent;
            n.best_g = g;
        }
    private:
        double g;
        const Node* parent;
    };


    struct pop_parent
    {
        pop_parent(){}
        void operator()(Node& n)
        {
            if(!n.parents.empty())
                n.parents.pop_front();
        }
    };

    Node getMin()
    {
        typedef multi_index::index<by_fg>::type fg_index;
        fg_index & fg = states.get<by_fg>();
        return *(fg.begin());
    }

    void expand(const Node& curNode)
    {
        typedef multi_index::index<by_ij>::type ij_index;
        ij_index & ij = states.get<by_ij>();
        auto it = ij.find(boost::tuple<int, int, double>(curNode.i, curNode.j, curNode.interval_begin));
        ij.modify(it, updateExpand());
    }

    const Node* getParentPtr()
    {
        typedef multi_index::index<by_fg>::type fg_index;
        fg_index & fg = states.get<by_fg>();
        return &(*(fg.begin()));
    }

    void insert(const Node& curNode)
    {
        states.insert(curNode);
    }

    void update(const Node &curNode, bool best)
    {
        typedef multi_index::index<by_ij>::type ij_index;
        ij_index & ij = states.get<by_ij>();
        std::list<std::pair<const Node*, double>> parents(0);
        auto it = ij.find(boost::tuple<int, int, double>(curNode.i, curNode.j, curNode.interval_begin));
        ij.modify(it, pop_parent());
        if(best)
            ij.modify(it, updateBest(curNode.g, curNode.Parent));
        if(curNode.consistent == 0)
        {
            parents = this->findParents(curNode);
            for(auto pit = parents.begin(); pit!= parents.end(); pit++)
                ij.modify(it, addParent(&(*pit->first), pit->second));
        }
        if(!it->parents.empty() && it->parents.begin()->second < std::min(curNode.g, curNode.best_g))
            ij.modify(it, updateFG(it->parents.begin()->second, it->parents.begin()->first));
        else if(best)
            ij.modify(it, updateFG(curNode.g, curNode.Parent));
        else
            ij.modify(it, updateFG(curNode.best_g, curNode.best_Parent));
    }

    std::list<std::pair<const Node*, double>> findParents(const Node& curNode)
    {
        std::list<std::pair<const Node*, double>> parents;
        parents.clear();
        typedef multi_index::index<by_cons>::type cons_index;
        cons_index & cons = states.get<by_cons>();
        auto range = cons.equal_range(1);
        double dist;
        bool found = false;
        for(auto it = range.first; it != range.second; it++)
        {
            if(!found)
                if(it->i == curNode.Parent->i && it->j == curNode.Parent->j && fabs(it->interval_begin - curNode.Parent->interval_begin) < CN_EPSILON)
                {
                    found = true;
                    it++;
                    if(it == range.second)
                        break;
                }
            dist = sqrt(pow(it->i - curNode.i,2) + pow(it->j - curNode.j,2));
            if(it->g + dist < curNode.g)
                if(it->g + dist >= curNode.interval_begin)
                {
                    if(it->g + dist <= curNode.interval_end)
                        parents.push_back({&(*it), it->g + dist});
                }
                else if(it->interval_end + dist >= curNode.interval_begin)
                    parents.push_front({&(*it), curNode.interval_begin});
        }
        return parents;
    }

    void updateNonCons(const Node& curNode)
    {
        typedef multi_index::index<by_ij>::type ij_index;
        ij_index & ij = states.get<by_ij>();
        auto parent = &(*ij.find(boost::tuple<int, int, double>(curNode.i, curNode.j, curNode.interval_begin)));
        typedef multi_index::index<by_cons>::type nc_index;
        nc_index & non_cons = states.get<by_cons>();
        auto range = non_cons.equal_range(2);
        double dist, new_g;
        for(auto it = range.first; it != range.second; it++)
        {
            dist = sqrt(pow(it->i - curNode.i,2) + pow(it->j - curNode.j,2));
            new_g = dist + curNode.best_g;
            if(new_g < it->best_g)
                if(new_g >= it->interval_begin)
                {
                    if(new_g <= it->interval_end)
                    {
                        non_cons.modify(it, addParent(parent, new_g));
                        if(it->g - new_g > CN_EPSILON)
                            non_cons.modify(it, updateFG(new_g, parent));
                    }
                }
                else if(curNode.interval_end + dist >= it->interval_begin)
                {
                    non_cons.modify(it, addParent(parent, it->interval_begin));
                    non_cons.modify(it, updateFG(it->interval_begin, parent));
                }
        }

    }
    void findAndPrint(int i, int j)
    {
        typedef multi_index::index<by_ij>::type ij_index;
        ij_index & ij = states.get<by_ij>();
        auto range = ij.equal_range(boost::make_tuple(i,j));
        for(auto it = range.first; it != range.second; it++)
        {
            std::cout<<it->i<<" "<<it->j<<" "<<it->g<<" "<<it->F<<" "<<it->best_g<<"\n";
            for(auto p_it = it->parents.begin(); p_it!=it->parents.end(); p_it++)
                std::cout<<p_it->first->i<<" "<<p_it->first->j<<" "<<p_it->first->g<<" "<<p_it->first->F<<" "<<p_it->first->best_g<<" "<<p_it->second<<" parent\n";
        }
    }

    void printByFG()
    {
        typedef multi_index::index<by_fg>::type fg_index;
        fg_index & fg = states.get<by_fg>();
        for(auto it=fg.begin(); it!=fg.end(); it++)
            std::cout<<it->i<<" "<<it->j<<" "<<it->interval_begin<<" "<<it->interval_end<<" "<<it->g<<" "<<it->F<<" "<<it->best_g<<" "<<it->parents.size()<<" "<<it->expanded<<" "<<it->consistent<<"\n";
    }
    void printStats()
    {
        typedef multi_index::index<by_cons>::type cons_index;
        cons_index & cons = states.get<by_cons>();
        std::cout<<cons.count(0)<<" "<<cons.count(1)<<" "<<cons.count(2)<<" ";
    }

    void clear()
    {
        states.clear();
    }

    int size()
    {
        return states.size();
    }
};


#endif // multi_index_H
