#include <iostream>
#include <array>
#include <vector>
#include <algorithm>	//For std::stable_sort()
#include <functional>
#include <memory>	//For std::shared_ptr
#include <deque>
#include <chrono>
#include <cmath>		//For sqrt function
#include <thread>
using namespace std::chrono;


typedef std::array<std::array<int, 3>, 3> TileGrid;

typedef struct BoardState
{
	TileGrid currentLayout;
	double distanceFromStart;		//g(n)
	double estDistanceFromGoal;		//h'(n)
	double totalCost;				//f'(n)
	std::vector<std::shared_ptr<BoardState>> successorList;
	std::shared_ptr<BoardState> parent;
}
BoardState;
typedef std::shared_ptr<BoardState> NodePtr;

//All of the data types returned from a call to A*.
//If it ends up being simple, we can just delete the struct.
typedef struct AStarResult
{
	bool foundIt;
	std::size_t numNodes;
}
AStarResult;

//declare two initial states
const TileGrid init1{
	{
		{2,8,3},
		{1,6,4},
		{0,7,5}
	}
};

const TileGrid init2
{
	{
		{2,1,6},
		{4,0,8},
		{7,5,3}
	}
};

//declare goal variables
const TileGrid goal{
	{
		{1,2,3},
		{8,0,4},
		{7,6,5}
	}
};

//Default child generator declaration.
//Returns a vector containing the children of node.
//Defining this funciton separately from A* may be important.
std::vector<TileGrid> defaultChildGen(TileGrid node);


//Heuristic function declarations
//Functions discussed in-class

//Simply counts the number of tiles out of place.
double inClassH1(TileGrid currentState);

//Takes the sum of the distances each tile is from its correct location.
//One for each square the tile must be moved to reach the correct
//position
double inClassH2(TileGrid currentState);

//Non-admissible heuristic from SCP-5 Slide 38
double inClassH3(TileGrid currentState);

//Funcitons developed by group members
double hEthan(TileGrid currentState);
double hBryce(TileGrid currentState);
double hAndy(TileGrid currentState);
double S(TileGrid currentState);


//Parameters: Initial state, represented as a 2-d array of ints (initial)
//Heuristic function that accepts a TileGrid as an argument (h)
AStarResult AStar(TileGrid initial, double (*h)(TileGrid));


//Recursive helper function that propogates a cost improvement down
//the tree when one is found.
void propogateCost(NodePtr curNode);

//helper function to print tilegrids
void printBoard(TileGrid b);

void printPath(NodePtr b, TileGrid init);

int main() {
	auto result = AStar(init2, hEthan);
	std::cout << (result.foundIt ? "Found it!\n" : "Failure.\n");
	std::cout << "Generated " << result.numNodes << " nodes.\n";
}


AStarResult AStar(TileGrid initial, double (*h)(TileGrid))
{
	auto start = high_resolution_clock::now();

	/*Declare variables we can't initialize with RAII.*/
	NodePtr bestNode = nullptr;
	std::vector<TileGrid> childList;
	NodePtr old = nullptr;
	std::size_t nodesGenerated = 1;
	std::size_t nodesExpanded = 0;
	std::size_t depth = 0;


	/*Start with OPEN containing only the initial Node.
	 *Set node's g value to zero, h' value to whatever that is,
	 *and its f' value to h'.*/
	
	//First, create the initial node.
	NodePtr initialState(new BoardState{initial, 0, 0, 0, {}, nullptr});
	
	//Then, initialize h' and f' values using h function.
	initialState->estDistanceFromGoal = h(initial);
	initialState->totalCost = initialState->estDistanceFromGoal;

	//Lastly, initialize `open` and store initialState in it.
	std::deque<NodePtr> open;
	open.push_back(initialState);

	//Helper function for finding nodes in open
	auto findNodeInOpen = [&open](NodePtr target)
	{
		for (NodePtr item : open)
		{
			if (item->currentLayout == target->currentLayout)
			{
				return item;
			}
		}

		//If code reaches this point, target wasn't in there.
		NodePtr notFound(nullptr);
		return notFound;
	};
	
	//Create empty list CLOSED.
	std::vector<NodePtr> closed;

	//Helper function for finding nodes in closed
	auto findNodeInClosed = [&closed](NodePtr target)
	{
		for (NodePtr item : closed)
		{
			if (item->currentLayout == target->currentLayout)
			{
				return item;
			}
		}

		//If code reaches this point, target wasn't in there.
		NodePtr notFound(nullptr);
		return notFound;
	};

	/*Until goal is found, do the following...*/
	do
	{
		if (open.empty())
		{
			//return failure
			AStarResult failed{0, nodesGenerated};
			return failed;
		}

		/*Pick node on OPEN with lowest f' value.
		 * Call it bestNode and remove it from open.
		 * Place it on closed.*/
		
		//Open is sorted in non-ascending order so that the cheapest
		//node is in the back.
		//We sort open here so that we don't need to sort every time
		//we add, remove, or change something.
		std::stable_sort(open.begin(), open.end(), std::greater<NodePtr>());

		bestNode = open.back();
		open.pop_back();
		closed.push_back(bestNode);
		std::cout << "bestNode:\n";
		printBoard(bestNode->currentLayout);
		std::cout << "g:  " << bestNode->distanceFromStart << "\n";
		std::cout << "h': " << bestNode->estDistanceFromGoal << "\n";
		std::cout << "f': " << bestNode->totalCost << "\n";
		std::cout << "\n";
		std::chrono::duration<int, std::milli> duration(200);
    	std::this_thread::sleep_for(duration);


		if (goal == bestNode->currentLayout)
		{
			//Exit and report solution
			auto stop = high_resolution_clock::now();
			AStarResult success{1, nodesGenerated};
			std::cout << nodesExpanded << " nodes expanded.\n";
			std::cout << "Max depth: " << depth << "\n";
			std::cout << "Effective branching factor: " << nodesGenerated/depth;
			std::cout << "\nTrace to root node: \n";
			printPath(bestNode,initial);
			auto duration = duration_cast<microseconds>(stop - start);
			std::cout << "Execution time: " << duration.count() <<" microseconds\n";
			return success;
		}

		depth = (depth > bestNode->distanceFromStart+1? depth : bestNode->distanceFromStart+1);
		nodesExpanded++;
		childList = defaultChildGen(bestNode->currentLayout);
		/* std::cout << "Children of current node:\n"; */
		for (auto&& successor: childList)
		{
			//Do nothing if the child is the same as
			//bestNode's parent.
			if (bestNode->parent && successor == bestNode->parent->currentLayout)
			{
				continue;
			}

			//Reset betterPath for each child.
			bool betterPath = false;

			//Create NodePtr newNode from successor.
			NodePtr newNode(new BoardState{successor, 0, 0, 0, {}, nullptr});

			//Set gval
			newNode->distanceFromStart = bestNode->distanceFromStart
				+ 1;
			/* std::cout << "g:  " << newNode->distanceFromStart << "\n"; */
			
			//Set h'val
			newNode->estDistanceFromGoal = h(successor);
			/* std::cout << "h': " << newNode->estDistanceFromGoal << "\n"; */
			
			//Set f'val
			newNode->totalCost = newNode->distanceFromStart 
				+ newNode->estDistanceFromGoal;
			/* std::cout << "f': " << newNode->totalCost << "\n"; */
			/* std::cout << "\n"; */

			old = findNodeInOpen(newNode);
			if (old)
			{
				//Add old to bestNode->successorList
				bestNode->successorList.push_back(old);

				//If the new successor is cheaper than the old one...
				if (newNode->distanceFromStart < old->distanceFromStart)
				{
					//Change `old`'s parent to point to bestNode
					old->parent = bestNode;

					//Record new cheaper cost in g(old) and
					//update f'(old)
					old->distanceFromStart = newNode->distanceFromStart;
					old->totalCost = old->distanceFromStart+old->estDistanceFromGoal;
					
					//Add newNode to open.
					open.push_back(newNode);

					//Remember: We've found a better path.
					betterPath = true;
				}
			}

			//Check that the new node wasn't in open before looking
			//through closed.
			if (!(old) && (old = findNodeInClosed(newNode)))
			{
				//Add old to bestNode->successorList
				bestNode->successorList.push_back(old);
				
				//If this new path is cheaper than the old path...
				if (newNode->distanceFromStart < old->distanceFromStart)
				{
					//Change `old`'s parent to point to bestNode
					old->parent = bestNode;

					//Record new cheaper cost in g(old) and
					//update f'(old)
					old->distanceFromStart = bestNode->distanceFromStart+1;
					old->totalCost = old->distanceFromStart+old->estDistanceFromGoal;

					//Add newNode to open.
					open.push_back(newNode);

					betterPath = true;
				}
			}

			//If we have found a better path to `old`, then propogate
			//the improvement among `old`'s successors.
			if (betterPath)
			{
				propogateCost(old);
			}

			//If this child is in either open nor closed,
			//then it is completely new.
			else
			{
				//Increase the counter.
				nodesGenerated++;

				//Change newNode's parent link to bestNode
				newNode->parent = bestNode;

				//Put newNode on open and reorder.
				open.push_back(newNode);
			}
		}
		/* std::cout << "\n"; */
	} while (!(open.empty()));

	//If code somehow reaches this point, Goal node could not be
	//reached.
	AStarResult failure{0, nodesGenerated};
	return failure;
}


//Returns the number of tiles that are out of place.
double inClassH1(TileGrid currentState)
{
	double outOfPlace{0.0};
	for (int row = 0; row < 3; row++)
	{
		for (int col = 0; col < 3; col++)
		{
			if (currentState[row][col] && currentState[row][col] != goal[row][col])
			{
				outOfPlace++;
			}
		}
	}
	return outOfPlace;
}


// currentState is initilized
// returns sum of distances to correct tiles
double inClassH2(TileGrid currentState)
{
	int ans = 0;

	//Translation tables to help find correct tile
	int rtrans[8] = {0,0,0,1,2,2,2,1};
	int ctrans[8] = {0,1,2,2,2,1,0,0}; 
	for (int row = 0; row < 3; row++)
	{
		for (int col = 0; col < 3; col++)
		{			
			if(currentState[row][col]!=0){
			// gets the correct row and column of given number 
				int r = rtrans[currentState[row][col]-1];
				int c = ctrans[currentState[row][col]-1];
				ans+= std::abs(c-col) + std::abs(r-row);
			}
		}
	}
	return ans;
}


//The non-admissible heuristic shown to us in class.
//Returns the sum of the Manhattan distances between the Tile and its
//proper location (inClassH2), plus 3 times the value of S(n) (hBryce).
double inClassH3(TileGrid currentState)
{
	return inClassH2(currentState) + 3 * S(currentState);
}


//S(n)
//For every Tile in currentState:
//	If a Tile is not in the center and
//	the Tile is not followed by the proper successor and
//	the Tile is not zero:
//		Add 2 to ans.
//	else: if the Tile is in the center and the Tile is not zero:
//		Add 1 to ans.
//	else: Do nothing.
double S(TileGrid currentState)
{
double ans = 0;
int nrow = -10;
int ncol = -10;
	for (int row = 0; row < 3; row++)
	{
		//gets the additions to row and col index to find the text tile
		for (int col = 0; col < 3; col++)
		{
			if(row*3+col < 2){ 
				nrow = 0;
				ncol = 1;
			}else if(row*3+col >6){
				nrow = 0;
				ncol = -1;
			}else{
				nrow = col-1;
				ncol = 0;
			}

			int currentTile = currentState[row][col];

			//If center tile is not zero, add 1 to ans.
			if(row==1 && col==1){
				if(currentTile != 0){
					ans++;
				}
			}
			
			//The outer tiles work in a repeating sequence of 1-8
			//If the current tile is not empty and it is not followed
			//by the next number in the sequence, add 2.
			else if (currentTile != 0 &&
					currentState[row+nrow][col+ncol] != (currentTile%8)+1)
			{
				ans+= 2;
			}
		}
	}
	return ans;
}



double hBryce(TileGrid currentState)
{
    double resultRow = 0;
    double resultColumn = 0;
    int rowTranslation[8] = {0,0,0,1,2,2,2,1};
    int colTranslation[8] = {0,1,2,2,2,1,0,0};   
	for (int row = 0; row < 3; row++) {
         for (int col = 0; col < 3; col++) {
            if(currentState[row][col]!=0){
                 int r = rowTranslation[currentState[row][col]-1];
                 int c = colTranslation[currentState[row][col]-1];
                 if(row!=r){
                      resultRow++;
                 }
                 if(col!=c){
                     resultColumn++;
                 }
               }
         }
     }
    return resultColumn+resultRow;
//return 0.0;
}

//returns the number of digits in the wrong row + the number in the wrong col + s(n)
double hEthan(TileGrid currentState)
{
	double result = 0;

	//Translation tables to help find correct tile.
	int rtrans[8] = {0,0,0,1,2,2,2,1};
	int ctrans[8] = {0,1,2,2,2,1,0,0}; 
	for (int row = 0; row < 3; row++)
	{
		for (int col = 0; col < 3; col++)
		{
			if(currentState[row][col]!=0){
			// gets the correct row and column of given number 
				int r = rtrans[currentState[row][col]-1];
				int c = ctrans[currentState[row][col]-1];
				if(row!=r){
					result++;
				}if(col!=c){
					result++;
				}
			}
		}
	}
	return S(currentState) + 1.5*result;
}


//Returns the sum of straight-line/"as-the-crow-flies" distances
//between each tile and its correct tile.
double hAndy(TileGrid currentState)
{
	double result = 0.0;

	//Translation tables to help find correct tile.
	int findRow[8] = {0,0,0,1,2,2,2,1};
	int findCol[8] = {0,1,2,2,2,1,0,0};

	for (int row = 0; row < 3; row++)
	{
		for (int col = 0; col < 3; col++)
		{
			int curTile = currentState[row][col];
			if (curTile /*!=0*/)
			{
				//Figure out the straight-line distance between
				//curTile and its correct spot.
				int correctRow = findRow[curTile-1];
				int correctCol = findCol[curTile-1];

				double distance = std::sqrt(
						std::pow(correctRow-row, 2)+
						std::pow(correctCol-col, 2)
						);

				result += distance;
			}
		}
	}
	return result;
}


//PRE node is an initilized tilegrid with a 0 marking the empty spaces
//POST returns a vector with tilegrids for each of the possible moves from node with 0 showing empty spcace
std::vector<TileGrid> defaultChildGen(TileGrid node)
{
	std::vector<TileGrid> result;

	//find row and column of empty space r and c
	int r = -1;
	int c = -1;
	for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            if(node[i][j]==0)
        	{
				r=i;
				c=j;
        	}
        }
    }

	//checks all 4 directions and adds child to result if possible
	int temp;
	if(r>0){
		TileGrid upTile = node;
		temp = upTile[r-1][c];
		upTile[r-1][c] = 0;
		upTile[r][c] = temp;
		result.push_back(upTile);
	}
	if(r<2){
		TileGrid downTile = node;
		temp = downTile[r+1][c];
		downTile[r+1][c] = 0;
		downTile[r][c] = temp;
		result.push_back(downTile);
	}
	if(c>0){
		TileGrid leftTile = node;
		temp = leftTile[r][c-1];
		leftTile[r][c-1] = 0;
		leftTile[r][c] = temp;
		result.push_back(leftTile);
	}
	if(c<2){
		TileGrid rightTile = node;
		temp = rightTile[r][c+1];
		rightTile[r][c+1] = 0;
		rightTile[r][c] = temp;
		result.push_back(rightTile);
	}

	return result;
}


/*Weird stuff we need to do for C++ purposes*/
bool operator>(const NodePtr lhs, const NodePtr rhs)
{
	return lhs->totalCost > rhs->totalCost;
}


//Helper function for A*.
void propogateCost(NodePtr curNode)
{
	std::function<void(NodePtr, NodePtr)> recursiveBody;
	recursiveBody = [&recursiveBody](NodePtr targetNode, NodePtr prevNode)->void
	{
		//If targetNode already lists prevNode as its parent,
		//simply continue propogation.
		if (targetNode->parent == prevNode)
		{
			//Change g and h values to reflect changes in parent.
			targetNode->distanceFromStart = prevNode->distanceFromStart+1;
			targetNode->totalCost = targetNode->distanceFromStart+targetNode->estDistanceFromGoal;

			//Continue propogation throughout targetNode's children.
			for (auto successor : targetNode->successorList)
			{
				recursiveBody(targetNode, successor);
			}
		}
		
		//If targetNode doesn't list prevNode as its parent, then its
		//current parent USED TO be cheaper, at the very least.
		//Compare its currently listed parent to previous node.
		else if (prevNode->distanceFromStart < targetNode->parent->distanceFromStart)
		{
			//If prevNode is cheaper than current parent,
			//set parent to prevNode and continue propogation.
			targetNode->parent = prevNode;
			targetNode->distanceFromStart = prevNode->distanceFromStart+1;
			targetNode->totalCost = targetNode->distanceFromStart+targetNode->estDistanceFromGoal;

			for (auto successor : targetNode->successorList)
			{
				recursiveBody(targetNode, successor);
			}
		}
		//If prevNode is not cheaper than targetNode's listed parent,
		//do nothing and return.
	};

	for (auto successor : curNode->successorList)
	{
		recursiveBody(successor, curNode);
	}
}


//PRE TileGrid is initilzed 
//POST Tilegrid has been printed and not modified
void printBoard(TileGrid b){
	for(int i=0; i<3; i++){
		for(int j=0; j<3; j++){
			std::cout << b[i][j] << " ";
		}
		std::cout << "\n";
	}
}


//recursive helper funtion for path
void printPath(NodePtr b, TileGrid init){
	if(b->currentLayout != init){
		printPath(b->parent,init);
	}
	printBoard(b->currentLayout);
	std::cout<<"\n";
}
