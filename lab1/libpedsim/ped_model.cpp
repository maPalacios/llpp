#include "ped_model.h"
#include "ped_waypoint.h"
#include "cuda_dummy.h"
#include <pthread.h>
#include <iostream>


#define SERIAL 1
#define OPENMP 2
#define PTHREADS 3;

struct interval {
  int left, right;
  Ped::Model * model;
};

void Ped::Model::setPar(int inPar){
  par = inPar;
}

int Ped::Model::getPar(){
  return par;
}

void Ped::Model::setup(vector<Ped::Tagent*> agentsInScenario)
{
  agents = agentsInScenario;
  implementation = SEQ;
}

const std::vector<Ped::Tagent*> Ped::Model::getAgents() const
{
  return agents;
}

void* tickHelp(void *arg){
  struct interval myInterval;// = new struct interval();
  myInterval = *((struct interval*)arg);
  int left = myInterval.left;
  int right = myInterval.right;

  std::vector<Ped::Tagent*> agents = (myInterval.model)->getAgents();

  for (int i=left; i<right; i++){
    agents[i]->whereToGo();
    agents[i]->go();
  }
}

void Ped::Model::tick(){
  int numAgents = agents.size();
  if (getPar() == 2){
    int numProc = 1;
    int blocksize = agents.size()/numProc;
    pthread_t threads[numProc];
    struct interval * intervals[numProc];

    for (int i=0;i<numProc;i++){
      intervals[i] = new struct interval();
      intervals[i]->left = i*blocksize;
      intervals[i]->right = (i+1)*blocksize;
      intervals[i]->model = this;
      pthread_create(&threads[i], NULL, &tickHelp,(void*) intervals[i]);
    }
    void * result;
    for (int i=0; i< numProc;i++)
      pthread_join(threads[i], &result);
  } else if (getPar() == 1) {
#pragma omp parallel for
    for(int i=0;i<numAgents;i++){
      agents[i]->whereToGo();
      agents[i]->go();
    }
  } else {
    for(int i=0;i<numAgents;i++){
      agents[i]->whereToGo();
      agents[i]->go();
    }
  }
}
