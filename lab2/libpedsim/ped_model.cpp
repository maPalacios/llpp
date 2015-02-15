#include "ped_model.h"
#include "ped_waypoint.h"
#include "cuda_dummy.h"
#include <pthread.h>
#include <nmmintrin.h>
#include <iostream>

enum IMPLEMENTATION {CUDA, VECTOR, OMP, PTHREAD, SEQ};
struct interval {
  int left, right;
  std::vector<Ped::Tagent*> *agents;
};

void Ped::Model::setPar(int inPar, int numProcs){
  implementation = (IMPLEMENTATION)inPar;
  np = numProcs;
}

int Ped::Model::getPar(){
  return implementation;
}

int Ped::Model::getNumProcs(){
  return np;
}

void Ped::Model::setup(vector<Ped::Tagent*> agentsInScenario)
{
  agents = agentsInScenario;
  implementation = SEQ;
  for (int i=0;i<agents.size();i++){
      agents[i]->whereToGo();
    }

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
  } else if (getPar() == OMP) {
#pragma omp parallel for num_threads(np)
    for(int i=0;i<numAgents;i++){
      agents[i]->whereToGo();
      agents[i]->go();
    }
  } else if (getPar() == SEQ) {
    for(int i=0;i<numAgents;i++){
      agents[i]->whereToGo();
      agents[i]->go();
    }
  }
  else if (getPar() == CUDA){

    whereToGoCUDA(&agents);
  } else if (getPar() == VECTOR){
#pragma omp parallel for num_threads(np)
    for(int i=0;i<numAgents-1;i+=2){
      if (*(agents[i]->getVisited()) == true && *(agents[i+1]->getVisited()) == true){
      __m128d *x = (__m128d*)agents[i]->getPosX();
      __m128d *y = (__m128d*)agents[i]->getPosY();
      __m128d *wx = (__m128d*)agents[i]->getPosWX();
      __m128d *wy = (__m128d*)agents[i]->getPosWY();
      __m128d *wr = (__m128d*)agents[i]->getPosWR();
      __m128d *lwx = (__m128d*)agents[i]->getPosLWX();
      __m128d *lwy = (__m128d*)agents[i]->getPosLWY();
      __m128d diffx, diffy, length, diffpx, diffpy, dist;


      diffx = _mm_sub_pd(*wx, *lwx);
      diffy = _mm_sub_pd(*wy, *lwy);
      diffpx = _mm_sub_pd(*x, *wx);
      diffpy = _mm_sub_pd(*y, *wy);
      dist = _mm_add_pd(_mm_mul_pd(diffpx, diffpx), _mm_mul_pd(diffpy, diffpy));

      double dres[2];
      __m128d *res = (__m128d*)dres;

      *res = _mm_sub_pd(dist, _mm_mul_pd(*wr,*wr));

      if (dres[0] > 0 && dres[1] > 0){
        length = _mm_add_pd(_mm_mul_pd(diffx, diffx), _mm_mul_pd(diffy, diffy));
        length = _mm_sqrt_pd(length);
        *x = _mm_add_pd(*x, _mm_div_pd(diffx,length));
        *y = _mm_add_pd(*y, _mm_div_pd(diffy,length));
      } else {
          agents[i]->whereToGo();
          agents[i]->go();
          agents[i+1]->whereToGo();
          agents[i+1]->go();
      }
    } else {
        agents[i]->whereToGo();
        agents[i]->go();
        agents[i+1]->whereToGo();
        agents[i+1]->go();
    }

    }
  }
}
