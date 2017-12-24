#include"cMission.h"
#include <iostream>

int main(int argc, char* argv[])
{
    /*if (argc==2)
    {
        cMission Mission(argv[1]);
        if (!Mission.getConfig())
            return 0;
        else
            std::cout<<"CONFIG LOADED\n";

        if (!Mission.getMap())
        {
            std::cout<<"Program terminated.\n";
            return 0;
        }
        else
            std::cout<<"MAP LOADED\n";

        Mission.createSearch();
        Mission.createLog();
        Mission.startSearch();
        Mission.printSearchResultsToConsole();
        Mission.saveSearchResultsToLog();
    }
    return 1;*/
    std::vector<std::string> names = {"brc202d","den520d","ost003d"};
    std::vector<int> agents = {32,64,96,128,160};
    //for(int j=0; j<3; j++)
        for(int k=4; k<agents.size(); k++)
            for(int i=0; i<500; i++)
            {
                //int k = 4;
                //int i = 16;
                std::cout<<i<<" ";
                std::string path="D:/Users/andreychuk/Documents/GitHub/xml/Warehouse_WFI/"+std::to_string(agents[k])+"/"+std::to_string(i)+".xml";
                cMission Mission(path.c_str());
                if (!Mission.getConfig())
                    return 0;

                if (!Mission.getMap())
                {
                    std::cout<<"Program terminated.\n";
                    return 0;
                }
                Mission.createSearch();
                Mission.createLog();
                Mission.startSearch();
                Mission.saveSearchResultsToLog();
            }
}
