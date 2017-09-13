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
        updateExpand(double best_F, bool expanded):best_F(best_F),expanded(expanded){}
        void operator()(Node& n)
        {
            n.expanded = expanded;
            n.Parent = n.best_Parent;
            n.F = best_F;
            n.best_F = best_F;
            n.consistent = 1;
        }

    private:
        bool expanded;
        double best_F;
    };
    struct addParent
    {
        addParent(const Node* parent, double new_g):parent(parent), new_g(new_g){}
        void operator()(Node &n)
        {
            if(n.parents.empty() || n.parents.back().second <= new_g)
            {
                n.parents.push_back({parent, new_g});
                return;
            }
            if(n.parents.begin()->second >= new_g)
            {
                n.F = n.F - n.g + new_g;
                n.g = new_g;
                n.Parent = parent;
                n.parents.push_front({parent, new_g});
            }
            bool inserted = false;
            for(auto it = n.parents.begin(); it!= n.parents.end(); it++)
                if(it->second >= new_g)
                {
                    inserted = true;
                    n.parents.insert(it,{parent, new_g});
                    break;
                }
            if(!inserted)
                n.parents.push_back({parent, new_g});
        }
        private:
        const Node* parent;
        double new_g;
    };

    struct updateFG
    {
        updateFG(const double& g, const Node* parent, bool best):g(g), parent(parent), best(best){}
        void operator()(Node& n)
        {
            n.F = n.F - n.g + g;
            n.g = g;
            n.consistent = 2;
            n.Parent = parent;
            n.parents.pop_front();
            if(best)
            {
                n.best_Parent = parent;
                n.best_F = n.F;
            }
        }

    private:
        bool best;
        double g;
        const Node* parent;
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
        ij.modify(it, updateExpand(curNode.best_F, true));
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

    void update(const Node& curNode, bool best)
    {
        typedef multi_index::index<by_ij>::type ij_index;
        ij_index & ij = states.get<by_ij>();
        auto it = ij.find(boost::tuple<int, int, double>(curNode.i, curNode.j, curNode.interval_begin));
        if(best)
            ij.modify(it, updateFG(curNode.g, curNode.Parent, true));
        else
        {
            if(curNode.consistent == 0)
                this->findParents(curNode);

            if(curNode.parents.size()>1)
            {
                auto c_it = curNode.parents.begin();
                c_it++;
                if(c_it->second < curNode.g)
                {
                    ij.modify(it, updateFG(c_it->second, c_it->first, false));
                }
                else
                {
                    ij.modify(it, updateFG(curNode.g, curNode.Parent, false));
                }
            }
            else
            {
                ij.modify(it, updateFG(curNode.g, curNode.Parent, false));
            }
        }
    }
    void findParents(const Node& curNode)
    {
        typedef multi_index::index<by_fg>::type fg_index;
        fg_index & fg = states.get<by_fg>();
        auto range = fg.equal_range(true);
        typedef multi_index::index<by_ij>::type ij_index;
        ij_index & ij = states.get<by_ij>();
        auto c_it = ij.find(boost::tuple<int, int, double>(curNode.i, curNode.j, curNode.interval_begin));
        double dist;
        for(auto it = range.first; it != range.second; it++)
        {
            dist = sqrt(pow(it->i - curNode.i,2)+pow(it->j - curNode.j,2));
            if(it->g + dist >= curNode.interval_begin)
            {
                if(it->g + dist <= curNode.interval_end)
                    ij.modify(c_it, addParent(&(*it), it->g + dist));
            }
            else if(it->interval_end + dist >= curNode.interval_begin)
                ij.modify(c_it, addParent(&(*it), curNode.interval_begin));
        }
    }

    void updateNonCons(const Node& curNode)
    {
        typedef multi_index::index<by_ij>::type ij_index;
        ij_index & ij = states.get<by_ij>();
        auto p_it = ij.find(boost::tuple<int, int, double>(curNode.i, curNode.j, curNode.interval_begin));
        typedef multi_index::index<by_cons>::type nc_index;
        nc_index & non_cons = states.get<by_cons>();
        auto range = non_cons.equal_range(2);
        double dist, new_g;
        for(auto it = range.first; it != range.second; it++)
        {
            dist = sqrt(pow(it->i - curNode.i,2)+pow(it->j - curNode.j,2));
            new_g = dist + curNode.best_F - curNode.h;
            if(new_g >= it->interval_begin)
            {
                if(new_g <= it->interval_end)
                    non_cons.modify(it, addParent(&(*p_it), new_g));
            }
            else if(curNode.interval_end + dist >= it->interval_begin)
                non_cons.modify(it, addParent(&(*p_it), it->interval_begin));
        }
    }
    void findAndPrint(int i, int j)
    {
        typedef multi_index::index<by_ij>::type ij_index;
        ij_index & ij = states.get<by_ij>();
        auto range = ij.equal_range(boost::make_tuple(i,j));
        for(auto it = range.first; it != range.second; it++)
        {
            std::cout<<it->i<<" "<<it->j<<" "<<it->g<<" "<<it->F<<" "<<it->best_F<<"\n";
            for(auto p_it = it->parents.begin(); p_it!=it->parents.end(); p_it++)
                std::cout<<p_it->first->i<<" "<<p_it->first->j<<" "<<p_it->first->g<<" "<<p_it->first->F<<" "<<p_it->first->best_F<<" "<<p_it->second<<" parent\n";
        }
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
