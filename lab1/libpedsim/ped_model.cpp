#include "ped_model.h"
#include "ped_waypoint.h"
#include "cuda_dummy.h"
#include <pthread.h>
#include <iostream>


void Ped::Model::setup(vector<Ped::Tagent*> agentsInScenario)
{
  agents = agentsInScenario;
  implementation = SEQ;
}

const std::vector<Ped::Tagent*> Ped::Model::getAgents() const
{
  return agents;
}

void tickHelp(std::vector<Ped::Tagent*> agents2)
{
  for(std::vector<Ped::Tagent*>::iterator it = agents2.begin(); it != agents2.end();++it){
    Ped::Tagent* temp = *it;
    temp->whereToGo();
    temp->go();
  }
  // EDIT HERE
}

void Ped::Model::tick(){
  int numProc = 2;
  int blocksize = agents.size()/numProc;
  int n=0;
  pthread_t threads[numProc];
  for (int i=0;i<numProc;i++){
    std::vector<Ped::Tagent*> agents2;
    std::cout << "a" << endl;    
    for(std::vector<Ped::Tagent*>::iterator it = agents.begin()+i*blocksize; !(it == agents.end() || it==agents.begin()+(i+1)*blocksize);++it){
      n++;
 std::cout << n << endl;       
	 agents2.push_back(*it);
  }
    void * result;
    pthread_create(&threads[i], NULL, &tickHelp,NULL);
    pthread_join(threads[i], &result);
 }

}

