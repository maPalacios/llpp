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

void* tickHelp(void *arg)
{
  std::vector<Ped::Tagent*> agents2 = *((std::vector<Ped::Tagent*> *)arg);
  for(std::vector<Ped::Tagent*>::iterator it = agents2.begin(); it != agents2.end();++it){
    (*it)->whereToGo();
    (*it)->go();
  }
}

void Ped::Model::tick(){
  int numProc = 1;
  int blocksize = agents.size()/numProc;
  pthread_t threads[numProc];
  std::vector<Ped::Tagent*> agents2[numProc];

  for (int i=0;i<numProc;i++){
    for(std::vector<Ped::Tagent*>::iterator it = agents.begin()+i*blocksize;
	!(it == agents.end() || it==agents.begin()+(i+1)*blocksize);
	++it)
      {
    	agents2[i].push_back(*it);
      }
    pthread_create(&threads[i], NULL, &tickHelp,(void*) &agents2[i]);
  }
  void * result;
  for (int i=0; i< numProc;i++)
    pthread_join(threads[i], &result);
     
}

