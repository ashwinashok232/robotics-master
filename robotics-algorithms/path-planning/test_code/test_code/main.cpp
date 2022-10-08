#include <iostream>
#include <stack>

using namespace std;


stack<int>* fun()
{
    stack<int> *stk = new stack<int>;
 
    /* Some operations on arr[] */
    stk->push(0);
    stk->push(1);
    stk->push(2);
    stk->push(3);
 
    return stk;
}
 
int main()
{
    stack<int>* stk = fun();
    while (!stk->empty()){
        cout << stk->top() << endl;
        stk->pop();
    }
    
    bool abc = 1<2;
    if (abc){
        cout << "Yes" << endl;
    }
    
    //delete[] ptr; //allocated memory must be deleted
    return 0;
}



//
//  main.cpp
//  a-star
//
//  Created by Ashwin Ashok on 8/4/22.
//

/*
#include <iostream>
#include <stack>
#include <vector>
#include <tuple>
#include <cmath>

class Node {
public:
    float g;
    float h;
    float f;
    int x;
    int y;
};

class Astar {

public:
    
    int x0,y0,x1,y1,curx,cury;
    int grid[10][10]
    = { { 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 },
        { 1, 1, 1, 0, 1, 1, 1, 0, 1, 1 },
        { 1, 1, 1, 0, 1, 1, 0, 1, 0, 1 },
        { 0, 0, 1, 0, 1, 0, 0, 0, 0, 1 },
        { 1, 1, 1, 0, 1, 1, 1, 0, 1, 0 },
        { 1, 0, 1, 1, 1, 1, 0, 1, 0, 0 },
        { 1, 0, 0, 0, 0, 1, 0, 0, 0, 1 },
        { 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 },
        { 1, 1, 1, 0, 0, 0, 1, 0, 0, 1 } };
    
    int gridx = 10;
    int gridy = 10;
    
    Astar(int startx,int starty,int goalx,int goaly) {
        x0 = startx;
        y0 = starty;
        x1 = goalx;
        y1 = goaly;
        curx = startx;
        cury = starty;
    }
    
    std::stack<Node>* open_nodes() {
        std::stack<Node> *openList = new std::stack<Node>;
        
        for (int i=-1; i<2; i++){
            for(int j=-1; j<2; j++){
                try {
//                    int* newx = &curx;
//                    int* newy = &cury;
                    
                    int newx = curx+i;
                    int newy = cury+j;
//                    if (grid[*newx][*newy] == 1 && std::make_tuple(i,j)!=std::make_tuple(0,0)){
                    if (grid[newx][newy] == 1 && std::make_tuple(i,j)!=std::make_tuple(0,0)){
//                        if (*newx>=0 && *newy>=0){
                        if (newx>=0 && newy>=0){
                            Node newNode;
//                            newNode.x = *newx;
//                            newNode.y = *newy;
                            newNode.x = newx;
                            newNode.y = newy;
                            
                            newNode.g = Astar::g_cost_calc(newNode);
                            newNode.h = Astar::h_cost_calc(newNode);
                            newNode.f = Astar::f_cost_calc(newNode);
                            openList->push(newNode);
                        }
                    }
                }
                catch (...){continue;}
            }
        }
        return openList;
    }
    
    float g_cost_calc(Node node){
        return std::pow(std::pow(curx-node.x,2)+std::pow(cury-node.y,2),0.5);
    }
    
    float h_cost_calc(Node node){
        return std::pow(std::pow(x1-node.x,2)+std::pow(y1-node.y,2),0.5);
    }
     
    float f_cost_calc(Node node){
        return node.g+node.h;
    }
    
    Node best_node(){
        float minCost = 10000.f;
        Node best;
        std::stack<Node>* openList = open_nodes();
        
        while (!openList->empty()) {
            if ((openList->top()).f < minCost) {
                minCost = (openList->top()).f;
                best = openList->top();
            }
            openList->pop();
        }
        return best;
    }
    
    bool infinite_loop(int count){
        return count > gridx*gridy/2;
    }
    
    std::vector<std::tuple<int,int>>* find_path() {
        int count = 0;
        std::vector<std::tuple<int,int> >* traversed = new std::vector<std::tuple<int,int> >;
        
        while (curx!=x1 && cury!=y1) {
            if (infinite_loop(count)) {
                break;
            }
            Node best = best_node();
            curx = best.x;
            cury = best.y;
            traversed->push_back(std::make_tuple(curx,cury));
            count+=1;
            std::cout << curx << " " << cury << std::endl;
        }
        return traversed;
    }
};

int main() {
    Astar run1(0,0,7,5);
    std::vector<std::tuple<int,int>>* traversed= run1.find_path();
    
    return 0;
}
*/
