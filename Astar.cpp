#include <math.h>  
#include "Astar.h"  
  
void Astar::InitAstar(std::vector<std::vector<int>> &_maze)  
{  
    maze=_maze;  
}  
  
int Astar::calcG(Point *temp_start,Point *point)  
{  
    int extraG=(abs(point->x-temp_start->x)+abs(point->y-temp_start->y))==1?kCost1:kCost2;  
    int parentG=point->parent==NULL?0:point->parent->G; //if it is the initial node, then its parent mode is void
    return parentG+extraG;  
}  
  
int Astar::calcH(Point *point,Point *end)  
{  
    //H is calculate d with Euclid distance
    return sqrt((double)(end->x-point->x)*(double)(end->x-point->x)+(double)(end->y-point->y)*(double)(end->y-point->y))*kCost1;  
}  
  
int Astar::calcF(Point *point)  
{  
    return point->G+point->H;  
}  
  
Point *Astar::getLeastFpoint()  
{  
    if(!openList.empty())  
    {  
        auto resPoint=openList.front();  
        for(auto &point:openList)  
            if(point->F<resPoint->F)  
                resPoint=point;  
        return resPoint;  
    }  
    return NULL;  
}  
  
Point *Astar::findPath(Point &startPoint,Point &endPoint,bool isIgnoreCorner)  
{  
    openList.push_back(new Point(startPoint.x,startPoint.y));
    //put the initial node into the openList, create a new node by copy, and separate the two nodes
    while(!openList.empty())  
    {  
        auto curPoint=getLeastFpoint(); //find the curPoint with the least F value  
        openList.remove(curPoint); //delete the curPoint from the openList  
        closeList.push_back(curPoint); //store the curPoint in the closeList  
        //1. find the eight accessible nodes in the neighborhood of the current node  
        auto surroundPoints=getSurroundPoints(curPoint,isIgnoreCorner);  
        for(auto &target:surroundPoints)  
        {  
            //2. for any node, put it into the openList is it is not included in the openList
            //set the current node as its parent node, and calculate the corresponding F, G, and H
            if(!isInList(openList,target))  
            {  
                target->parent=curPoint;  
  
                target->G=calcG(curPoint,target);  
                target->H=calcH(target,&endPoint);  
                target->F=calcF(target);  
  
                openList.push_back(target);  
            }  
            //3. for any node, if it has already been included in the openList then calculate the G value
            //if the G value is larger than the previous one, leave it unchnaged\
            //otherwise set its parent node as the current node and update the G and F
            else  
            {  
                int tempG=calcG(curPoint,target);  
                if(tempG<target->G)  
                {  
                    target->parent=curPoint;  
  
                    target->G=tempG;  
                    target->F=calcF(target);  
                }  
            }  
            Point *resPoint=isInList(openList,&endPoint);  
            if(resPoint)  
                return resPoint;
                //return pointer of node in the list
                //do not use the previously passed endpoint pointer because it has been deep copied
        }  
    }  
  
    return NULL;  
}  
  
std::list<Point *> Astar::GetPath(Point &startPoint,Point &endPoint,bool isIgnoreCorner)  
{  
    Point *result=findPath(startPoint,endPoint,isIgnoreCorner);  
    std::list<Point *> path;  
    //return the path, return a void list if no path has been found  
    while(result)  
    {  
        path.push_front(result);  
        result=result->parent;  
    }  
    return path;  
}  
  
Point *Astar::isInList(const std::list<Point *> &list,const Point *point) const  
{  
    //determine whether a given node has already been included in tyhe openList
    for(auto p:list)  
        if(p->x==point->x&&p->y==point->y)  
            return p;  
    return NULL;  
}  
  
bool Astar::isCanreach(const Point *point,const Point *target,bool isIgnoreCorner) const  
{  
    if(target->x<0||target->x>maze.size()-1  
        ||target->y<0&&target->y>maze[0].size()-1  
        ||maze[target->x][target->y]==1  
        ||target->x==point->x&&target->y==point->y  
        ||isInList(closeList,target))
        //return false 
        //when the point coincides with current node
        //when the point is out of the map
        //when the point is an belongs to any obstacle
        //when the point is in the closeList
        return false;  
    else  
    {  
        if(abs(point->x-target->x)+abs(point->y-target->y)==1) //when the agent is not moving diagonally, it is OK   
            return true;  
        else  
        {  
            //determine whether the agent is stucked once it is moving diagonally  
            if(maze[point->x][target->y]==0&&maze[target->x][point->y]==0)  
                return true;  
            else  
                return isIgnoreCorner;  
        }  
    }  
}  
  
std::vector<Point *> Astar::getSurroundPoints(const Point *point,bool isIgnoreCorner) const  
{  
    std::vector<Point *> surroundPoints;  
  
    for(int x=point->x-1;x<=point->x+1;x++)  
        for(int y=point->y-1;y<=point->y+1;y++)  
            if(isCanreach(point,new Point(x,y),isIgnoreCorner))  
                surroundPoints.push_back(new Point(x,y));  
      
    return surroundPoints;  
}  
