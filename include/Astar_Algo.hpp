#ifndef ASTAR_ALGO_HPP
#define ASTAR_ALGO_HPP

#include<iostream>
#include<string>
#include<fstream>
#include<sstream>
#include<vector>
#include<algorithm>
#include<cmath>

//enum class state {kopen,kclose,kobstacle,kpath};    
    
namespace PathPlanning{

enum class state {kclose,kpath,kempty,kobstacle};   


class AstarAlgo{
    public:
        AstarAlgo(std::vector<int> start,std::vector<int> goal);
        virtual ~AstarAlgo();
        void Visualizer();
        void parseInputData(std::string filename);
        int calculateHueristic(std::vector<int> node, std::vector<int> goal);
        void sortMinPick();
        void findValidSuccessor(std::vector<int> curr, std::vector<int>goal);
        int calculatePath();  
    private:
        std::vector<std::vector<state>> board;  // board is the environment. 
        std::vector<std::vector<int>> openList; //  We maintain two lists: open and close lists
        std::vector<std::vector<int>> closedList;
        std::vector<std::vector<int>> direction {{0,1},{0,-1},{1,0},{-1,0}};  
        std::vector<int> goal;
        std::vector<int> start;
         
};

}

#endif