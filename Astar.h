//define the object containing the A* algorithm
#include <vector>  
#include <list>  
  
const int kCost1=10; //define the cost of agent when moving one step axially  
const int kCost2=14; //define the cost of agent when moving one step diagonally  
  
struct Point  
{  
    int x,y; //coordinates of the agent 
    int F,G,H; //F=G+H  
    Point *parent; //coordinates of the parent node  
    Point(int _x,int _y):x(_x),y(_y),F(0),G(0),H(0),parent(NULL)  //initialize the variables  
    {  
    }  
};  
  
  
class Astar  
{  
public:  
    void InitAstar(std::vector<std::vector<int>> &_maze);  
    std::list<Point *> GetPath(Point &startPoint,Point &endPoint,bool isIgnoreCorner);  
  
private:  
    Point *findPath(Point &startPoint,Point &endPoint,bool isIgnoreCorner);  
    std::vector<Point *> getSurroundPoints(const Point *point,bool isIgnoreCorner) const;  
    bool isCanreach(const Point *point,const Point *target,bool isIgnoreCorner) const; 
    //determine whether a node can be used in the next step  
    Point *isInList(const std::list<Point *> &list,const Point *point) const; 
    //determine whether a node has been included in the openList or the closeList  
    Point *getLeastFpoint(); //return node in the openList with the least F value  
    //calculate the values of F, G, and H  
    int calcG(Point *temp_start,Point *point);  
    int calcH(Point *point,Point *end);  
    int calcF(Point *point);  
private:  
    std::vector<std::vector<int>> maze;  
    std::list<Point *> openList;  //define the openList  
    std::list<Point *> closeList; //define the closeList 
};  
