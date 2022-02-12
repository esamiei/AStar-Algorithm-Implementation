#include<iostream>
#include "Astar_Algo.hpp"

int main(){
    std::string filename="../Inputdata/a.txt";
    std::vector<int> start={4,5};
    std::vector<int> goal={0,0};

    PathPlanning::AstarAlgo astar(start,goal);
    astar.parseInputData(filename);
    astar.calculatePath();
    return 0;
}