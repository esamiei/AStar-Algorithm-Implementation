#include "Astar_Algo.hpp"

namespace PathPlanning{

AstarAlgo:: AstarAlgo(std::vector<int> start,std::vector<int> goal): start(start),goal(goal){}
AstarAlgo::~AstarAlgo()=default;

void AstarAlgo::Visualizer(){
    std::cout<<"*******Input Maze******\n";
    for (int i=0;i<board.size();++i){
        for(int j=0;j<board[i].size();++j){    
            if (board[i][j]==state::kobstacle) std::cout<<"1"<<" ";  
            else if (i==start[0] && j==start[1]) std::cout<<"S"<<" ";
            else if (i==goal[0] && j==goal[1]) std::cout<<"G"<<" "; 
            else if ((board[i][j]==state::kpath)) std::cout<<"x"<<" ";
            else if (board[i][j]==state::kclose) std::cout<<"c"<<" "; 
            else std::cout<<"0"<<" "; 
  
        }
        std::cout<<"\n";
    }
    std::cout<<"\n*************\n";
}

void AstarAlgo::parseInputData(std::string filename){
    std::ifstream input(filename);
    std::string line;
    //std::vector<std::vector<state>> board;
    std::vector<state> row_temp;
    if(!input.is_open()){
        std::cout<<"input file is not open\n";
    }
    while (std::getline(input, line)) {
        for (int i=0;i<line.size();++i){
            if(line[i]!=','){
                if (line[i]=='0') row_temp.push_back(state::kempty);
                else row_temp.push_back(state::kobstacle);
            }
        }
        board.push_back(row_temp);
        row_temp.clear();
    }
    Visualizer();
}



bool compareCost(const  std::vector<int> a, const std::vector<int> b){
    return a[2]>b[2];
}



int AstarAlgo::calculateHueristic(std::vector<int> node, std::vector<int> goal){
    return std::abs(node[0]-goal[0])+std::abs(node[1]-goal[1]);  //Manhatan distance
}



void AstarAlgo::sortMinPick(){
     std::sort(openList.begin(),openList.end(), compareCost);
     if (openList.size()>1 && openList[openList.size()-1][2]==openList[openList.size()-2][2]){
         if (openList[openList.size()-1][4]>openList[openList.size()-2][4]){
            std::vector<int> temp=openList[openList.size()-1];  
            openList[openList.size()-1]=openList[openList.size()-2];
            openList[openList.size()-2]=temp;
         }
     }
}


void AstarAlgo::findValidSuccessor(std::vector<int> currNode, std::vector<int> goal){
    std::vector<int> successorNode;
    for (int i=0;i<4;++i){
        int temp_x=currNode[0]+direction[i][0];
        int temp_y=currNode[1]+direction[i][1];

        if(temp_x>=0 && temp_x<board.size() && temp_y>=0 && temp_y<board[0].size() \
            && board[temp_x][temp_y]!=state::kobstacle && board[temp_x][temp_y]!=state::kpath &&\
            board[temp_x][temp_y]!=state::kclose){   //
            successorNode={temp_x,temp_y};
            int h=calculateHueristic(successorNode,goal);
            int g=currNode[3]+1;                                    // Assuming that the cost of going from the curr node to the sucessor node is 1
            int f=g+h;
            std::vector<int> successor_node_param={successorNode[0],successorNode[1],f,g,h};      //define node[0] node[1] f g
            
            //if(std::find(openList.begin(),openList.end(),successor_node_param)!=openList.end()){
            //    auto it=std::find(openList.begin(),openList.end(),successor_node_param);
            //    int ele=(int)(it-openList.begin());
            //    if (successor_node_param[3]>openList[ele][3]) continue;
            //}
            //else
            //{
            openList.push_back(successor_node_param);
            board[successorNode[0]][successorNode[1]]=state::kclose;
            //}
        }
    }
}




int AstarAlgo::calculatePath(){

    int h=calculateHueristic(start,goal);
    int g=0;
    int f=g+h;
    std::vector<int> node_param={start[0],start[1],f,g,h};   //define x=nod[0] which represets row and y=node[1] which represents column f g
    openList.push_back(node_param);

    while(openList.size()>0){      
        sortMinPick();
        std::vector<int> currNode=openList.back();          //pick the node with minimum f value
        board[currNode[0]][currNode[1]]=state::kpath;      // tag it as the kpath. note that y is row and x is column
        closedList.insert(closedList.end(), openList.begin(), openList.end());   
        openList.clear();
        std::cout<<"\n";
        Visualizer();
        if (currNode[0]==goal[0] && currNode[1]==goal[1]){
            return 0;        
        }
        else{
            findValidSuccessor(currNode, goal);
        }
    }
return 0;
}


}

