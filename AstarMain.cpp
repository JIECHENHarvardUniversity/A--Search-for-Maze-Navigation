#include <iostream>  
#include "Astar.h"  
using namespace std;  
  
int main()  
{  
    //initialize the map of the maze
    //where the map is represented as a matrix
    //"1" represents the obstacle
    //"0" represents the path
    vector<vector<int>> maze={  
        {0,1,1,1,1,1,1,1,1,1,1,1},  
        {1,0,0,1,1,0,1,0,0,0,0,1},  
        {1,0,0,1,1,0,0,0,0,0,0,0},  
        {1,0,0,0,0,0,1,0,0,1,1,1},  
        {1,1,1,0,0,0,0,0,1,1,0,0},  
        {1,1,0,1,0,0,0,0,0,0,0,1},  
        {1,0,1,0,0,0,0,1,0,0,0,1},  
        {1,1,1,1,1,1,1,0,1,0,1,0}  
    };  
    Astar astar;  
    astar.InitAstar(maze);  
  
    //set the start point and the end point  
    Point start(1,1);  
    Point end(6,10);  
    //run the A* algorithm  
    list<Point *> path=astar.GetPath(start,end,false);  
    //print the output coordinates of the found path  
    for(auto &p:path)  
        cout<<'('<<p->x<<','<<p->y<<')'<<endl;  
  
    system("pause");  
    return 0;  
}  

