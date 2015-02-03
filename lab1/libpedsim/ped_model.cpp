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
  std::vector<Ped::Tagent*> *agents;
};

void Ped::Model::setPar(int inPar, int numProcs){
  par = inPar;
  np = numProcs;
}

int Ped::Model::getPar(){
  return par;
}

int Ped::Model::getNumProcs(){
  return np;
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
  struct interval myInterval = *((struct interval*)arg);
  for (int i=myInterval.left; i<myInterval.right; i++){
    (*myInterval.agents)[i]->whereToGo();
    (*myInterval.agents)[i]->go();
  }
}

void Ped::Model::tick(){
  int numAgents = agents.size();
  if (getPar() == PTHREAD){
    int numProc = np;
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
  } else if (getPar() == OMP) {
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
