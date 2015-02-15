#include "ped_agent.h"
#include "ped_waypoint.h"
#include <stdio.h>
#include <iostream>


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

__global__ void dummyKernel(double *x, double *y, double *wx, double* wy, double * wr, double * lwx, double * lwy, bool *visited, bool *reached)  {
	int i = blockIdx.y*128*128 + blockIdx.x * 128+ threadIdx.x;	// This gives every thread a unique ID.
	double diffx, diffy, length;
	if (visited[i]){
		diffx = wx[i]-lwx[i];
		diffy = wy[i]-lwy[i];
	} else {
		diffx = wx[i]-x[i];
		diffy = wy[i]-y[i];
	}
	if ((x[i]-wx[i])*(x[i]-wx[i]) + (y[i]-wy[i])*(y[i]-wy[i]) > wr[i]*wr[i]) {
		length = sqrt(diffx*diffx+diffy*diffy);
		x[i] = round(x[i]+diffx/length); // round!
		y[i] = round(y[i]+diffy/length); // round!
		reached[i] = false;
	} else {
		x[i] = x[i];
		y[i] = y[i];
		reached[i] = true;
		visited[i] = true;
	}
}


void whereToGoCUDA(vector<Tagent*> *agents){
	double *x,*y,*wx,*wy,*wr,*lwx,*lwy;
	bool *reached, *hreached, *visited;
	int NUM = (*agents).size();
	int CNUM = 0;
	int blockGridWidth, blockGridHeight;

	if (NUM > 128*128){
	blockGridWidth = 128;
	blockGridHeight = NUM/(128*128);
	} else {
	blockGridWidth = NUM/128;
	blockGridHeight = 1;
}

	dim3 blockGridRows(blockGridWidth, blockGridHeight);
	dim3 threadBlockRows(128, 1);

	CNUM = NUM;// + 128*128*(blockGridHeight-1);
	hreached     = (bool *)malloc(sizeof(bool) * NUM);

	cudaMalloc( (void **)&x, sizeof(double) * NUM);
	cudaMalloc( (void **)&y, sizeof(double) * NUM);
	cudaMalloc( (void **)&wx, sizeof(double) * NUM);
	cudaMalloc( (void **)&wy, sizeof(double) * NUM);
	cudaMalloc( (void **)&wr, sizeof(double) * NUM);
	cudaMalloc( (void **)&lwx, sizeof(double) * NUM);
	cudaMalloc( (void **)&lwy, sizeof(double) * NUM);
	cudaMalloc( (void **)&visited, sizeof(bool) * NUM);
	cudaMalloc( (void **)&reached, sizeof(bool) * NUM);


	cudaMemcpy(x, (*agents)[0]->getPosX(), sizeof(double) * NUM, cudaMemcpyHostToDevice);
	cudaMemcpy(y, (*agents)[0]->getPosY(), sizeof(double) * NUM, cudaMemcpyHostToDevice);
	cudaMemcpy(wx, (*agents)[0]->getPosWX(), sizeof(double) * NUM, cudaMemcpyHostToDevice);
	cudaMemcpy(wy, (*agents)[0]->getPosWY(), sizeof(double) * NUM, cudaMemcpyHostToDevice);
	cudaMemcpy(wr, (*agents)[0]->getPosWR(), sizeof(double) * NUM, cudaMemcpyHostToDevice);
	cudaMemcpy(lwx, (*agents)[0]->getPosLWX(), sizeof(double) * NUM, cudaMemcpyHostToDevice);
	cudaMemcpy(lwy, (*agents)[0]->getPosLWY(), sizeof(double) * NUM, cudaMemcpyHostToDevice);
	cudaMemcpy(visited, (*agents)[0]->getVisited(), sizeof(bool) * NUM, cudaMemcpyHostToDevice);



	dummyKernel<<<blockGridRows, threadBlockRows>>>(x,y,wx,wy,wr,lwx,lwy,visited, reached);
	cudaThreadSynchronize();

	gpuErrchk(cudaMemcpy((*agents)[0]->getPosX(), x, sizeof(double) * NUM, cudaMemcpyDeviceToHost));
	cudaMemcpy((*agents)[0]->getPosY(), y, sizeof(double) * NUM, cudaMemcpyDeviceToHost);
	cudaMemcpy((*agents)[0]->getVisited(), visited, sizeof(bool) * NUM, cudaMemcpyDeviceToHost);
	cudaMemcpy(hreached, reached, sizeof(bool) * NUM, cudaMemcpyDeviceToHost);


	for (int i=0;i<CNUM;i++){
		if (hreached[i]) {
			//	cout << (*agents)[i]->getX() << endl;
			(*agents)[i]->whereToGo();
			(*agents)[i]->go();
		}
	}

	for (int i=CNUM;i<NUM;i++){
		(*agents)[i]->whereToGo();
		(*agents)[i]->go();
	}

	cudaFree(x);
	cudaFree(y);
	cudaFree(wx);
	cudaFree(wy);
	cudaFree(wr);
	cudaFree(lwy);
	cudaFree(lwx);
	cudaFree(reached);

	free(hreached);
}
