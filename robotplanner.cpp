//Edited by Josh Song
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>

using namespace std;

struct Point
{
	int x,y;		//the (x,y) vector for 2-D coordinate system
};

struct Cell
{
	Point p;		//the coordinate of the cell 
	int state;		//0: cell is empty, 1: cell has obstacle, 2: robot is in the cell
};

struct Node
{
	Point p,father;		//p: the current point (x,y),father : the point (x',y') before the previous move 
	int g,h;	//the cost value G and H

};

class PathFinder
{
private: 
	vector<vector<Cell> > map;			// save the map information
	Point start_point,end_point;		// the start and end point
	int row_len,col_len;		//the length of the map in row and column
	vector<Node> openlist;		//the open lsit and close list for A* algorithm
	vector<Node> closelist;
	Node target;		//the target point node
	vector<char> track; //to save track(in diverse order)

public:
	PathFinder(Point start,Point end, char* filename)	//find path
	{
		Initialize(start, end, filename);	//do some initialization jobs
		FindPath(start,end);		//A* algorithm to find path
	}

	void Initialize(Point start,Point end, char* filename)
	{
		start_point.x=start.x;		start_point.y=start.y;		//set start and end point coordinate
		end_point.x=end.x;		end_point.y=end.y;
		LoadMap(filename);			//load map form file		
		target.p.x=end.x;	target.p.y=end.y;		
		
	}

	void LoadMap(char* filename)		//load map form file	
	{
		ifstream f;
		f.open(filename);
		if(!f.is_open())
		{
			printf("file %s open failed!\n",filename);
			exit(0);
		}
		f>>col_len>>row_len;		//get the length of the map in row and column for first line
		Cell cell_t;
		vector<Cell> cell_row_t;
		for(int i=0;i<row_len;i++)	//read the state of the map for each cell
		{
			cell_row_t.clear();
			for(int j=0;j<col_len;j++)
			{
				cell_t.p.x=j;
				cell_t.p.y=i;
				f>>cell_t.state;
				cell_row_t.push_back(cell_t);
			}
			map.push_back(cell_row_t);
		}		
	}

	void display(vector<Node> list)			//show the results
	{
		Node cur;
		int dx,dy,ret,index=list.size()-1;
		char move;
		
		bool is_first=1;
		cur.p.x=list[index].p.x;	cur.p.y=list[index].p.y;		// find the track from the target node
		cur.father.x=list[index].father.x;	cur.father.y=list[index].father.y;
		cur.g=list[index].g;	cur.h=list[index].h;
		while(1)
		{
			if(cur.father.x==-1&&cur.father.y==-1)		//if reach the start node,complete finding
			{
				break;
			}
			dx=cur.father.x-cur.p.x;		//check the move operation
			dy=cur.father.y-cur.p.y;
			if(dx==0)
			{
				if(dy==1)move='U';
				else move='D';
			}
			else if(dx==1)move='L';
			else move='R';
			track.push_back(move);			//put into the track vector

			cur.p.x=cur.father.x;	cur.p.y=cur.father.y;
			ret=IsInNodeList(list,cur);		//find for the previous node
			cur.p.x=list[ret].p.x;	cur.p.y=list[ret].p.y;	
			cur.father.x=list[ret].father.x;	cur.father.y=list[ret].father.y;
			cur.g=list[ret].g;	cur.h=list[ret].h;
			//system("pause");
		}
		for(int i=track.size()-1;i>=0;i--)		//output the track in the right order
		{
			if(is_first)
			{
				cout<<track[i];
				is_first=0;
			}
			else cout<<" "<<track[i];
		}
		cout<<endl;
		//system("pause");
	}

	int IsInNodeList(vector<Node> list,Node node)		//check if the node(x,y) in the Node list
	{
		bool is_find=0;
		int ret=-1;
		for(int i=0;i<list.size();i++)
		{
			if((list[i].p.x==node.p.x)&&(list[i].p.y==node.p.y))	//if find ,return the index
			{
				is_find=1;
				ret=i;
				break;
			}
		}
		return ret;		//if not find, return -1
	}

	int Hamming_dist(Point c1,Point c2)		//compute Hamming distance :H[(x1,y1),(x2,y2)]=|x1-x2|+|y1-y2|
	{
		int d1=c1.x-c2.x;
		int d2=c1.y-c2.y;
		if(d1<0)d1*=-1;
		if(d2<0)d2*=-1;
		return d1+d2;
	}

	void FindPath(Point start,Point end)		//A* algorithm to find path
	{
		Node current_node,node_t;
		current_node.p.x=start.x;	current_node.p.y=start.y;			//begin from the start node
		current_node.g=0; current_node.h=Hamming_dist(start,end);
		current_node.father.x=-1;	current_node.father.y=-1;
		//closelist.push_back(current_node);
		//Point current_point;
		int ret;
		while(1)
		{
			if((current_node.p.y-1)>=0)						//for 'U' operation, check the move not exceed the boundary
			{
				if(map[current_node.p.y-1][current_node.p.x].state==0)	//if the up cell is empty
				{
					node_t.p.x=current_node.p.x;	node_t.p.y=current_node.p.y-1;		//set the temp node point as (x,y-1)
					
					if(IsInNodeList(closelist,node_t)<0)		//if the temp node is not in the closelist
					{
						if((ret=IsInNodeList(openlist,node_t))>=0)	//if the temp node is in the openlist
						{
							if(openlist[ret].g>current_node.g)		//set the smaller G value
							{
								openlist[ret].g=current_node.g+1;
								openlist[ret].father.x=current_node.p.x;		//if a better new route found, set the new father point
								openlist[ret].father.y=current_node.p.y;
							}
						}
						else			//if the temp node is not in the openlist, add it to openlist
						{
							node_t.father.x=current_node.p.x;	node_t.father.y=current_node.p.y; 
							node_t.g=current_node.g+1;	node_t.h=Hamming_dist(node_t.p,end);
							openlist.push_back(node_t);
						}
					}
				}
			}

			if((current_node.p.y+1)<row_len)						//for 'D' operation, check the move not exceed the boundary
			{
				if(map[current_node.p.y+1][current_node.p.x].state==0)	//if the down cell is empty
				{
					node_t.p.x=current_node.p.x;	node_t.p.y=current_node.p.y+1;		//set the temp node point as (x,y+1)
					
					if(IsInNodeList(closelist,node_t)<0)		//if the temp node is not in the closelist
					{
						if((ret=IsInNodeList(openlist,node_t))>=0)	//if the temp node is in the openlist
						{
							if(openlist[ret].g>current_node.g)		//set the smaller G value
							{
								openlist[ret].g=current_node.g+1;
								openlist[ret].father.x=current_node.p.x;		//if a better new route found, set the new father point
								openlist[ret].father.y=current_node.p.y;
							}
						}
						else			//if the temp node is not in the openlist, add it to openlist
						{
							node_t.father.x=current_node.p.x;	node_t.father.y=current_node.p.y; 
							node_t.g=current_node.g+1;	node_t.h=Hamming_dist(node_t.p,end);
							openlist.push_back(node_t);
						}
					}
				}
			}

			if((current_node.p.x-1)>=0)						//for 'L' operation, check the move not exceed the boundary
			{
				if(map[current_node.p.y][current_node.p.x-1].state==0)	//if the left cell is empty
				{
					node_t.p.x=current_node.p.x-1;	node_t.p.y=current_node.p.y;		//set the temp node point as (x-1,y)
					
					if(IsInNodeList(closelist,node_t)<0)		//if the temp node is not in the closelist
					{
						if((ret=IsInNodeList(openlist,node_t))>=0)	//if the temp node is in the openlist
						{
							if(openlist[ret].g>current_node.g)		//set the smaller G value
							{
								openlist[ret].g=current_node.g+1;
								openlist[ret].father.x=current_node.p.x;		//if a better new route found, set the new father point
								openlist[ret].father.y=current_node.p.y;
							}
						}
						else			//if the temp node is not in the openlist, add it to openlist
						{
							node_t.father.x=current_node.p.x;	node_t.father.y=current_node.p.y; 
							node_t.g=current_node.g+1;	node_t.h=Hamming_dist(node_t.p,end);
							openlist.push_back(node_t);
						}
					}
				}
			}

			if((current_node.p.x+1)<col_len)						//for 'R' operation, check the move not exceed the boundary
			{
				if(map[current_node.p.y][current_node.p.x+1].state==0)	//if the right cell is empty
				{
					node_t.p.x=current_node.p.x+1;	node_t.p.y=current_node.p.y;		//set the temp node point as (x+1,y)
					
					if(IsInNodeList(closelist,node_t)<0)		//if the temp node is not in the closelist
					{
						if((ret=IsInNodeList(openlist,node_t))>=0)	//if the temp node is in the openlist
						{
							if(openlist[ret].g>current_node.g)		//set the smaller G value
							{
								openlist[ret].g=current_node.g+1;
								openlist[ret].father.x=current_node.p.x;		//if a better new route found, set the new father point
								openlist[ret].father.y=current_node.p.y;
							}
						}
						else			//if the temp node is not in the openlist, add it to openlist
						{
							node_t.father.x=current_node.p.x;	node_t.father.y=current_node.p.y; 
							node_t.g=current_node.g+1;	node_t.h=Hamming_dist(node_t.p,end);
							openlist.push_back(node_t);
						}
					}
				}
			}

			if(openlist.size()<=0)		//if the openlist is empty, which means no path can reach the end point
			{
				cout<<"Impossible to find a path"<<endl;
				//system("pause");
				break;
			}

			closelist.push_back(current_node);		//put the current node into closelist

			if((ret=IsInNodeList(openlist,target))>=0)	//if the target node is in the openlist, which means the path has successfully found, show results and return
			{
				closelist.push_back(openlist[ret]);	//put the target node into closelist
				display(closelist);
				break;
			}

			int min_index=-1,min_cost=-1,cost_temp;			//find the min cost_f (F=G+H) in the openlist 
			for(int i=0;i<openlist.size();i++)
			{
				cost_temp=openlist[i].g+openlist[i].h;
				if(cost_temp<min_cost||min_cost==-1)
				{
					min_cost=cost_temp;
					min_index=i;
				}
			}			
												//and set the min cost_f node as the current node
			current_node.p.x=openlist[min_index].p.x;	current_node.p.y=openlist[min_index].p.y;
			current_node.father.x=openlist[min_index].father.x;	current_node.father.y=openlist[min_index].father.y;
			current_node.g=openlist[min_index].g;	current_node.h=openlist[min_index].h;

			//cout<<current_node.p.x<<" "<<current_node.p.y<<" "<<current_node.g<<" "<<current_node.h<<" "<<endl;

			vector<Node>::iterator iter;		//delete the node in the openlist
			iter=openlist.begin()+min_index;
			openlist.erase(iter);
			
			
		}

		
	}
};

int main(int argc,char** argv)
{
	if(argc<6)	//check for the numbers of arguments
	{
		cout<<"Invalid command arguments"<<endl;
		exit(1);
	}

	Point start,end;
	start.x=atoi(argv[2]);	start.y=atoi(argv[3]);		//get arguments from command line
	end.x=atoi(argv[4]);	end.y=atoi(argv[5]);

	PathFinder pathfinder=PathFinder(start,end,argv[1]); //solve the result

	return 0;
}
