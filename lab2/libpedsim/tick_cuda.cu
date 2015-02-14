#include "ped_agent.h"
#include "ped_waypoint.h"
#include <stdio.h>
#include <iostream>

struct CUDA_DATA{
  double * ax,*ay, *wpx, *wpy, *wpr, *lwpx, *lwpy;
};

#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
   if (code != cudaSuccess)
   {
      fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
      if (abort) exit(code);
   }
}

using namespace Ped;

__global__ void dummyKernel(double *x, double *y, double *wx, double* wy, double * wr, double * lwx, double * lwy, bool *reached)  {
	int i = blockIdx.x * 40+ threadIdx.x;	// This gives every thread a unique ID.
  double diffx, diffy, length;
  diffx = wx[i]-lwx[i];
  diffy = wy[i]-lwy[i];

  if ((x[i]-wx[i])*(x[i]-wx[i]) + (y[i]-wy[i])*(y[i]-wy[i]) > wr[i]*wr[i]) {
    length = sqrt(diffx*diffx+diffy*diffy);
    x[i] = x[i]+diffx/length; // round!
    y[i] = y[i]+diffy/length; // round!
    reached[i] = false;
  } else {
    x[i] = x[i];
    y[i] = y[i];
    reached[i] = true;
  }
}


void whereToGoCUDA(vector<Tagent*> *agents){
  double *x,*y,*wx,*wy,*wr,*lwx,*lwy;
  bool *reached, *hreached;
  int NUM = (*agents).size();

  hreached     = (bool *)malloc(sizeof(bool) * NUM);

  cudaMalloc( (void **)&x, sizeof(double) * NUM);
  cudaMalloc( (void **)&y, sizeof(double) * NUM);
  cudaMalloc( (void **)&wx, sizeof(double) * NUM);
  cudaMalloc( (void **)&wy, sizeof(double) * NUM);
  cudaMalloc( (void **)&wr, sizeof(double) * NUM);
  cudaMalloc( (void **)&lwx, sizeof(double) * NUM);
  cudaMalloc( (void **)&lwy, sizeof(double) * NUM);

  cudaMalloc( (void **)&reached, sizeof(bool) * NUM);

  cudaMemcpy(x, (*agents)[0]->getPosX(), sizeof(double) * NUM, cudaMemcpyHostToDevice);
  cudaMemcpy(y, (*agents)[0]->getPosY(), sizeof(double) * NUM, cudaMemcpyHostToDevice);
  cudaMemcpy(wx, (*agents)[0]->getPosWX(), sizeof(double) * NUM, cudaMemcpyHostToDevice);
  cudaMemcpy(wy, (*agents)[0]->getPosWY(), sizeof(double) * NUM, cudaMemcpyHostToDevice);
  cudaMemcpy(wr, (*agents)[0]->getPosWR(), sizeof(double) * NUM, cudaMemcpyHostToDevice);
  cudaMemcpy(lwx, (*agents)[0]->getPosLWX(), sizeof(double) * NUM, cudaMemcpyHostToDevice);
  cudaMemcpy(lwy, (*agents)[0]->getPosLWY(), sizeof(double) * NUM, cudaMemcpyHostToDevice);

  dummyKernel<<<50, 40>>>(x,y,wx,wy,wr,lwx,lwy,reached);
  cudaThreadSynchronize();

  cudaMemcpy((*agents)[0]->getPosX(), x, sizeof(double) * NUM, cudaMemcpyDeviceToHost);
  cudaMemcpy((*agents)[0]->getPosY(), y, sizeof(double) * NUM, cudaMemcpyDeviceToHost);
  cudaMemcpy(hreached, reached, sizeof(bool) * NUM, cudaMemcpyDeviceToHost);


  for (int i=0;i<NUM;i++){
    if (hreached[i]) {
      (*agents)[i]->whereToGo();
      (*agents)[i]->go();
    }
  }
/*
This is what seems to take time
  cudaFree(x);
	cudaFree(y);
	cudaFree(wx);
  cudaFree(wy);
  cudaFree(reached);
  free(hreached);*/
}
