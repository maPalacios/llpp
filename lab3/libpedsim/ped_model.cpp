
#include "ped_model.h"
#include "ped_waypoint.h"
#include "cuda_dummy.h"
#include <pthread.h>
#include <nmmintrin.h>
#include <iostream>
#include <stack>
#include <algorithm>
#include <limits.h>
#include <atomic>



// Comparator used to identify if two agents differ in their position
bool cmp(Ped::Tagent *a, Ped::Tagent *b) {
	return (a->getX() < b->getX()) || ((a->getX() == b->getX()) && (a->getY() < b->getY()));
}

enum IMPLEMENTATION {CUDA, VECTOR, OMP, PTHREAD, SEQ};
struct interval {
	int left, right;
	Ped::Model *model;
	std::vector<Ped::Tagent*> *agents;
};

/// Set number of processors and type of parallelization
/// \date    2012-02-25
/// \param   inPar - the type of parallelization to use {CUDA, VECTOR, OMP, PTHREAD, SEQ} 0-4
/// \param   numProcs - number of processors to employ
void Ped::Model::setPar(int inPar, int numProcs){
	implementation = (IMPLEMENTATION)inPar;
	np = numProcs;
}

/// Get the type of parallelization used
/// \date    2012-02-25
/// \return  The type of parallelization used
int Ped::Model::getPar(){
return implementation;
}
/// Get the type number of processors employed
/// \date    2012-02-25
/// \return  The number of processors employed
int Ped::Model::getNumProcs(){
				return np;
}

/// TODO
/// \date    2012-02-25
/// \param	 agentsInScenario - 
void Ped::Model::setup(vector<Ped::Tagent*> agentsInScenario)
{
				agents = agentsInScenario;
				implementation = SEQ;

				// Hack! do not allow agents to be on the same position. Remove duplicates from scenario.
				bool (*fn_pt)(Ped::Tagent*, Ped::Tagent*) = cmp;
				std::set<Ped::Tagent*, bool(*)(Ped::Tagent*, Ped::Tagent*)> agentsWithUniquePosition (fn_pt);
				std::copy(agentsInScenario.begin(), agentsInScenario.end(), std::inserter(agentsWithUniquePosition, agentsWithUniquePosition.begin()));
				agents = std::vector<Ped::Tagent*>(agentsWithUniquePosition.begin(), agentsWithUniquePosition.end());

				treehash = new std::map<const Ped::Tagent*, Ped::Ttree*>();
				tree = new Ttree(NULL, treehash, 0, treeDepth, 0, 0, 1000, 800);



				int size = agents.size();
				data.ax = (double*)malloc(sizeof(double)*size);
				data.ay = (double*)malloc(sizeof(double)*size);
				data.wpx = (double*)malloc(sizeof(double)*size);
				data.wpy = (double*)malloc(sizeof(double)*size);
				data.wpr = (double*)malloc(sizeof(double)*size);
				data.desx = (double*)malloc(sizeof(double)*size);
				data.desy = (double*)malloc(sizeof(double)*size);
				vector<Ped::Tagent*> newAgents;
				for (int i = 0;i<size;i++){
								Ped::Tagent * a = new Ped::Tagent( &(data.ax)[i],&(data.ay)[i],&(data.wpx)[i],&(data.wpy)[i],&(data.wpr)[i],&(data.desx)[i], &(data.desy)[i]);
								*a = *(agents[i]);
								newAgents.push_back(a);
				}
				agents = newAgents;

				for (int i=0;i<agents.size();i++){
								agents[i]->whereToGo();
				}
				for (std::vector<Ped::Tagent*>::iterator it = agents.begin(); it != agents.end(); ++it)
				{
					grid[(int)(*it)->getX()+OFFSET][(int)(*it)->getY()+OFFSET] = true;
								tree->addAgent(*it);
				}

}

/// Get the list of agents
/// \date    2012-02-25
/// \return  The list of agents 
const std::vector<Ped::Tagent*> Ped::Model::getAgents() const{
				return agents;
}

/// TODO
/// \date    2012-02-25
/// \param   arg - TODO 
void* tickHelp(void *arg){
				struct interval myInterval = *((struct interval*)arg);
				for (int i=myInterval.left; i<myInterval.right; i++){
								(*myInterval.agents)[i]->whereToGo();
								(*myInterval.agents)[i]->go();
				}
				pthread_exit(NULL);
}

/// TODO
/// \date    2012-02-25 
void Ped::Model::tick(){
				int numAgents = agents.size();
				int numProc = np;

				if (getPar() == PTHREAD){
								int blocksize = agents.size()/numProc;
								pthread_t threads[numProc];
								struct interval * intervals[numProc];

								for (int i=0;i<numProc;i++){
												intervals[i] = new struct interval();
												intervals[i]->left = i*blocksize;
												intervals[i]->right = (i+1)*blocksize;
												intervals[i]->agents = &agents;
												pthread_create(&threads[i], NULL, &tickHelp,(void*) intervals[i]);
								}
								
								void * result;
								for (int i=0; i< numProc;i++)
												pthread_join(threads[i], &result);

								callPartition();
				} else if (getPar() == OMP) {
#pragma omp parallel for num_threads(np)
								for(int i=0;i<numAgents;i++){
												agents[i]->whereToGo();
												agents[i]->go();
								}
								callPartition();
				} else if (getPar() == SEQ) {
								for(int i=0;i<numAgents;i++){
												agents[i]->whereToGo();
												agents[i]->go();
								}
								callPartition();

				}
				else if (getPar() == CUDA){
								whereToGoCUDA(&agents);
								callPartition();
				} else if (getPar() == VECTOR){
#pragma omp parallel for num_threads(np)
								for(int i=0;i<numAgents-1;i+=2){
																__m128d *x = (__m128d*)agents[i]->getPosX();
																__m128d *y = (__m128d*)agents[i]->getPosY();
																__m128d *wx = (__m128d*)agents[i]->getPosWX();
																__m128d *wy = (__m128d*)agents[i]->getPosWY();
																__m128d *wr = (__m128d*)agents[i]->getPosWR();
																__m128d *desx = (__m128d*)agents[i]->getDesX();
																__m128d *desy = (__m128d*)agents[i]->getDesY();
																__m128d diffx, diffy, length, diffpx, diffpy, dist;
																

																diffx = _mm_sub_pd(*wx, *x);
																diffy = _mm_sub_pd(*wy, *y);
																dist = _mm_add_pd(_mm_mul_pd(diffx, diffx), _mm_mul_pd(diffy, diffy));

																double dres[2];
																__m128d *res = (__m128d*)dres;

																*res = _mm_sub_pd(dist, _mm_mul_pd(*wr,*wr));

																if (dres[0] > 0 && dres[1] > 0){
																				length = _mm_add_pd(_mm_mul_pd(diffx, diffx), _mm_mul_pd(diffy, diffy));
																				dist = _mm_sqrt_pd(dist);
																				*desx = _mm_add_pd(*x, _mm_div_pd(diffx,dist));
																				*desy = _mm_add_pd(*y, _mm_div_pd(diffy,dist));
																				agents[i]->roundDes();
																				agents[i+1]->roundDes();
														} else {
																				agents[i]->whereToGo();
																				agents[i]->go();
																				agents[i+1]->whereToGo();
																				agents[i+1]->go();
																}
								}
								callPartition();
				}
}
/// Returns the median of values
/// \date    2012-02-25
/// \return  The median of values
/// \param   values vector of ints
double getMedian(vector<int> values)
{
				int size = values.size();
				sort(values.begin(), values.end());
				if (size  % 2 == 0)
								return (values[size / 2 - 1] + values[size / 2]) / 2;
				else
								return values[size / 2];

}
pthread_barrier_t barrier;

/// TODO
/// \date    2012-02-25
/// \param   arg - TODO
void * partition(void * arg){
	struct interval *interv = (interval*)arg;
	int left = interv->left;
	int right = interv->right;
	vector<int> myAgents;
	for (int j=0;j<(*interv->agents).size();j++){
		int x = (*interv->agents)[j]->getX();
	if (x >= left && x < right){
	myAgents.push_back(j);
	}
	}
	pthread_barrier_wait(&barrier);
	for (int i=0;i<myAgents.size();i++){
		int x = (*interv->agents)[myAgents[i]]->getX();
		if (x >= left && x < right)
			(interv->model)->doSafeMovement(left, right,(*interv->agents)[myAgents[i]]);
		}
	pthread_exit(NULL);
}
/*
void Ped::Model::callPartition(){
				vector<struct interval*> intervals;

					vector<int> numbers;
					for (int j=0;j<agents.size();j++)
									numbers.push_back( agents[j]->getX());
				sort(numbers.begin(), numbers.end());				
				int min = numbers.front();
				int max = numbers.back()+1;
				int num = numbers.size();
	
				for (int i=0;i<4;i++){
								struct interval *   interval = new struct interval;
								interval->left = numbers[i*num/4];
								if (i !=3)
									interval->right = numbers[(i+1)*(num/4)+1];
								else
									interval->right = max;
								interval->agents = &agents;
								interval->model = this;
								intervals.push_back(interval);
				}
		//		cout << intervals.size() << endl;
				pthread_t thread[intervals.size()];
				pthread_barrier_init (&barrier, NULL, intervals.size());
				void * result;
				for (int i=0;i<intervals.size();i++)
								pthread_create(&thread[i], NULL, &partition, (void*) intervals[i]);
				for (int i=0;i<intervals.size();i++)
								pthread_join(thread[i], &result);

}
*/

/// Assign doSafeMovement(0, 0, agents[i]) for each agent in np number of threads
/// \date    2012-02-25
void Ped::Model::callPartition(){
	#pragma omp parallel for num_threads(np) 
	for (int i=0;i<agents.size();i++){
		doSafeMovement(0,0, agents[i]);
	}
}

/// TODO
/// \date    2012-02-25
/// \param   left -  TODO
/// \param   right - TODO
/// \param   agent - TODO
void  Ped::Model::doSafeMovement(int left, int right, Ped::Tagent *agent)
{
				std::vector<std::pair<int, int> > prioritizedAlternatives;
				std::pair<int, int> pDesired(agent->getDesiredX(), agent->getDesiredY());
				prioritizedAlternatives.push_back(pDesired);

				int diffX = pDesired.first - agent->getX();
				int diffY = pDesired.second - agent->getY();
				std::pair<int, int> p1, p2;
				if (diffX == 0 || diffY == 0)
				{
								// Agent wants to walk straight to North, South, West or East
								p1 = std::make_pair(pDesired.first + diffY, pDesired.second + diffX);
								p2 = std::make_pair(pDesired.first - diffY, pDesired.second - diffX);
				}
				else {
								// Agent wants to walk diagonally
								p1 = std::make_pair(pDesired.first, agent->getY());
								p2 = std::make_pair(agent->getX(), pDesired.second);
				}
				prioritizedAlternatives.push_back(p1);
				prioritizedAlternatives.push_back(p2);

				// Find the first empty alternative position
				for (std::vector<pair<int, int> >::iterator it = prioritizedAlternatives.begin(); it != prioritizedAlternatives.end(); ++it) {

								// If the current position is not yet taken by any neighbor
//								if (std::find(takenPositions.begin(), takenPositions.end(), *it) == takenPositions.end()) {

												// Set the agent's position
												bool exp = false;
												if (grid[(*it).first+OFFSET][(*it).second+OFFSET].compare_exchange_strong(exp, true)){
												bool exp = true;
												grid[(int)agent->getX()+OFFSET][(int)agent->getY()+OFFSET].compare_exchange_strong(exp, false);			
												//grid[(int)agent->getX()+100][(int)agent->getY()+100] = false;			
												agent->setX((*it).first);
												agent->setY((*it).second);
}
												else
														continue;
												break;
				}

}

/// Returns the list of neighbors within dist of the point x/y. This
/// can be the position of an agent, but it is not limited to this.
/// \date    2012-01-29
/// \return  The list of neighbors
/// \param   x the x coordinate
/// \param   y the y coordinate
/// \param   dist the distance around x/y that will be searched for agents (search field is a square in the current implementation)
set<const Ped::Tagent*> Ped::Model::getNeighbors(int x, int y, int dist) const {
				// if there is no tree, return all agents
				if(tree == NULL)
								return set<const Ped::Tagent*>(agents.begin(), agents.end());
				// create the output list
				list<const Ped::Tagent*> neighborList;
				getNeighbors(neighborList, x, y, dist);

				// copy the neighbors to a set
				return set<const Ped::Tagent*>(neighborList.begin(), neighborList.end());

}

/// Populates the list of neighbors that can be found around x/y./// This triggers a cleanup of the tree structure. Unused leaf nodes are collected in order to
/// save memory. Ideally cleanup() is called every second, or about every 20 timestep.
/// \date    2012-01-28
void Ped::Model::cleanup() {
				if(tree != NULL)
								tree->cut();
}

/// \date    2012-01-29
/// \param   the list to populate
/// \param   x the x coordinate
/// \param   y the y coordinate
/// \param   dist the distance around x/y that will be searched for agents (search field is a square in the current implementation)
void Ped::Model::getNeighbors(list<const Ped::Tagent*>& neighborList, int x, int y, int dist) const {
				stack<Ped::Ttree*> treestack;

				treestack.push(tree);
				while(!treestack.empty()) {
								Ped::Ttree *t = treestack.top();
								treestack.pop();
								if (t->isleaf) {
												t->getAgents(neighborList);
								}
								else {
												if (t->tree1->intersects(x, y, dist)) treestack.push(t->tree1);
												if (t->tree2->intersects(x, y, dist)) treestack.push(t->tree2);
												if (t->tree3->intersects(x, y, dist)) treestack.push(t->tree3);
												if (t->tree4->intersects(x, y, dist)) treestack.push(t->tree4);
								}
				}
}

Ped::Model::~Model()
{
				if(tree != NULL)
				{
								delete tree;
								tree = NULL;
				}
				if(treehash != NULL)
				{
								delete treehash;
								treehash = NULL;
				}
}
