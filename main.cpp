//Cs323 - Project#10: Traveling Salesman AStar Alg, by Xiaomin Chen

#include<iostream>
#include<fstream>
using namespace std;

class AstarNode{
	private:
		int ID;
		int numEdgeFromStart;
		int gStar;															//total cost from START node to this node
		int hStar;															//estimated cost from this node to a goal node
		int fStar;															//gStar + hStar
		AstarNode* parent;							
		AstarNode* next;													//used for OpenList and CloseList
		
	public:
		AstarNode(int num){
			ID = num;
			numEdgeFromStart = 0;
			parent = NULL;
			next = NULL;
		}
		
		AstarNode(int num, int edgesFromStart, AstarNode* p){
			ID = num;
			numEdgeFromStart = edgesFromStart;
			parent = p;
			next = NULL;
		}	
		
		void setnumEdgesFromStart(int edge){
			numEdgeFromStart = edge;
		}
		
		void setGstar(int g){
			gStar = g;
		}
		
		void setHstar(int h){
			hStar = h;
		}
		
		void setFstar(int f){
			fStar = f;
		}
		
		void setParent(AstarNode* p){
			parent = p;
		}
		
		void setNext(AstarNode* n){
			next = n;
		}
		
		int getID(){
			return ID;
		}
		
		int getGstar(){
			return gStar;
		}
		
		int getHstar(){
			return hStar;
		}
		
		int getFstar(){
			return fStar;
		}
		
		int getNumEdgesTraveled(){
			return numEdgeFromStart;
		}
		
		AstarNode* getParent(){
			return parent;
		}
		
		AstarNode* getNext(){
			return next;
		}
};

class AstarSearch{
	private:
		int numNodes;
		int start;
		int HFunction;
		int sumEdges;
		int numEdges;
		int** costMatrix;
		AstarNode* OpenList;
		int* childAry;
		
	public:
		AstarSearch(int n){
			sumEdges = 0;
			numEdges = 0;
			numNodes = n;
			OpenList = new AstarNode(0);						//dummy node for the convenience of inserting to the front
			
			//Asking for the starting node
			do{
				cout<<"Please state the starting node which must be between 1-"<<numNodes<<": ";
				cin>>start;
			}while(start<1 || start>numNodes);
			
			//Asking for the Heuristic Function
			do{
				cout<<"Please choose H-Function 1, 2, or 3 for estimation: ";
				cin>>HFunction;
			}while(HFunction<1 || HFunction>3);
			
			//Initialization of Cost Matrix: where costs are initially -1(infinity) and [i][i] is 0.				
			costMatrix = new int*[numNodes];
			for(int row=0; row<numNodes; row++){
				costMatrix[row] = new int[numNodes]();
			}
			for(int i=0; i<numNodes; i++){
				for(int j=0; j<numNodes; j++){
					if(i!=j){
						costMatrix[i][j] = -1;
					}
				}
			}
			
			childAry = new int[numNodes];
		}

		//Loading values into costMatrix with the input (Ni, Nj, cost) undirected(bidirectional) edge with cost 
		void loadMatrix(ifstream& read, ofstream& write1, ofstream& write2){
			int Ni, Nj, cost;
			write1<<"Below is the input graph for this program.\n";
			while(read>>Ni && read>>Nj && read>>cost){
				write1<<Ni<<"  "<<Nj<<"  "<<cost<<endl;
				costMatrix[Ni-1][Nj-1] = cost;
				costMatrix[Nj-1][Ni-1] = cost;
				sumEdges += cost;
				numEdges++;
				
			}
			
			write2<<"[Cost Matrix]"<<endl;
			for(int i=0; i<numNodes; i++){
				for(int j=0; j<numNodes; j++){
					write2<<costMatrix[i][j]<<"\t";
				}
				write2<<endl;
			}
			write2<<endl;
		}
		
		//Performs algorithm to find the "best" path of estimated lowest cost for the Traveling Salesman
		void performAlg(ofstream& write1, ofstream& write2){
			int iteration = 0;

			//initialization of starting node
			AstarNode* SNode = new AstarNode(start);
			SNode->setGstar(0);
			SNode->setHstar(0);				
			SNode->setFstar(0);
			openInsert(SNode);
			write2<<"[Starting Node]\n";
			printOpen(write2);
			
			//Expansion
			AstarNode* parent = openRemove();
			AstarNode* childNode;
			
			//While current path ending in parentNode is not an eligible path yet
			while(parent->getID() != start || !eligiblePath(parent)){	
				
				//if we returned to start node without passing all nodes, check next parent
				if(parent->getID() == start && parent->getParent()!= NULL){
					parent = openRemove();
					continue;
				}		
				
				copyChildList(parent->getID());
				write2<<"\n[Expansion #"<<++iteration<<" under Parent "<<parent->getID()<<"]\n";
				for(int child=0; child<numNodes; child++){
					//if there's a path to this child and have not been visited yet, create a childNode and compute G*, H*, F* for it
					if(childAry[child] > 0 && !nodeVisited(parent, child+1)){																	
						childNode = new AstarNode(child+1, parent->getNumEdgesTraveled()+1, parent);
						computeGstar(childNode);
						computeHstar(childNode);
						computeFstar(childNode);
						openInsert(childNode);
						write2<<"Child "<<childNode->getID()<<": g*="<<childNode->getGstar()<<"\th*="<<childNode->getHstar()<<"\tf*="<<childNode->getFstar()<<endl;
					}			
				}
				printOpen(write2);
				parent = openRemove();
			}	
			tracePath(parent, write1);
		}
		
		//copy costMatrix[nodeID][i] to childAry to find adjacent nodes
		void copyChildList(int nodeID){
			for(int i=0; i<numNodes; i++){
				childAry[i] = costMatrix[nodeID-1][i];
			}
		}
		
		void computeGstar(AstarNode* node){
			int G;
			int pastG = node->getParent()->getGstar();
			int newCost = costMatrix[node->getID()-1][node->getParent()->getID()-1];
			G = pastG + newCost;
			node->setGstar(G);
			
		}
		
		void computeHstar(AstarNode* node){
			int H;
			int nodesRemain = numNodes - node->getNumEdgesTraveled();
			if(HFunction == 1){
				H = ((sumEdges - node->getGstar()) / (numEdges - node->getNumEdgesTraveled())) * nodesRemain;
				node->setHstar(H);
			}
			else if(HFunction == 2){
				H = (node->getGstar()/node->getNumEdgesTraveled()) * nodesRemain;
				node->setHstar(H);
			}
			else{
				int a = ((sumEdges - node->getGstar()) / (numEdges - node->getNumEdgesTraveled())) * nodesRemain;
				int b = (node->getGstar()/node->getNumEdgesTraveled()) * nodesRemain;
				H = (a*b)/2;
				node->setHstar(H);
			}
		}
		
		void computeFstar(AstarNode* node){
			node->setFstar(node->getGstar() + node->getHstar());
		}
		
		//Insertion sort in the order of ascending F* value
		void openInsert(AstarNode* node){
			AstarNode* current = OpenList;
			if(OpenList->getNext() == NULL){
				OpenList->setNext(node);
			}
			else{
				while(current->getNext() != NULL && current->getNext()->getFstar() < node->getFstar()){
					current = current->getNext();
				}
				AstarNode* temp = current->getNext();
				current->setNext(node);
				node->setNext(temp);
			}
		}
		
		void printOpen(ofstream& write){
			AstarNode* current = OpenList->getNext();
			write<<"OpenList:  ";
			while(current != NULL){
				write<<current->getID()<<"\t";
				current = current->getNext();
			}
			write<<endl;
		}
		
		//Removes the node of lowest F* value from the head of linked list
		AstarNode* openRemove(){
			if(OpenList->getNext() != NULL){
				AstarNode* head = OpenList->getNext();
				OpenList->setNext(head->getNext());
				return head;
			}
			return NULL;
		}
		
		//Path is eligible is every node have been visited only once
		bool eligiblePath(AstarNode* node){
			int instance[numNodes] = {0};								
			AstarNode* current = node;
			while(current->getParent() != NULL){
				current = current->getParent();
				++instance[current->getID()-1];					
			}
			//checks that every node occured only once in the path
			for(int i=0; i<numNodes; i++){				
				if(instance[i] != 1){
					return false;
				}
			}
			return true;
		}
		
		//checks if current path already visited this node
		bool nodeVisited(AstarNode* node, int id){
			AstarNode* current = node->getParent();
			while(current!=NULL){
				if(current->getID() == id && start != id){
					return true;
				}
				current = current->getParent();
			}
			return false;	
		}
		
		//Prints the founded path
		void tracePath(AstarNode* end, ofstream& write){
			AstarNode* current = end;
			int nodes[numNodes+1];
			for(int n=numNodes; n>=0; n--){
				nodes[n]=current->getID();
				current = current->getParent();
			}
			write<<"\nThe starting node is "<<start<<"."<<endl;
			write<<"The HFunction used is "<<HFunction<<endl<<endl;
			write<<"Below is the result of the simple-path with a list of edges starting from node "<<start<<"."<<endl;
			write<<"START:";
			for(int i=0; i<=numNodes; i++){
				write<<"-->"<<nodes[i];
			}
			write<<endl;
			int x, y;
			for(int i=0; i<numNodes; i++){
				x = nodes[i];
				y = nodes[i+1];
				write<<x<<"\t"<<y<<"\t"<<costMatrix[x-1][y-1]<<endl;
			}
			write<<endl<<"The total cost of the simple-path is "<<end->getFstar()<<".";
		}
		
};

int main(int argc, char** argv){
	ifstream inFile1;
	ofstream outFile1;
	ofstream outFile2;
	inFile1.open(argv[1]);
	outFile1.open(argv[2]);
	outFile2.open(argv[3]);
	int nodes;

	//check the number of arguments
	if (argc != 4) {																
        cerr << "Error: Number of arguments unsatisfied.\n";
        exit(1);
    }
    
    if(inFile1>>nodes){
    	AstarSearch findPath(nodes);
    	findPath.loadMatrix(inFile1, outFile1, outFile2);
    	findPath.performAlg(outFile1, outFile2);
	}
    
	inFile1.close();
	outFile1.close();
	outFile2.close();
}
