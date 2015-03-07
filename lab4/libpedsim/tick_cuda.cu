#include "ped_agent.h"
#include "ped_waypoint.h"
#include "ped_model.h"
#include <stdio.h>
#include <iostream>

#define DIM 256

double totfade = 0;
double totblur =0;
double totupdate = 0;
double totscale = 0;

using namespace Ped;

__global__ void dummyKernel(double *x, double *y, double *wx, double* wy, double * wr, double * lwx, double * lwy, bool *reached)  {
	int i = blockIdx.y*DIM*DIM + blockIdx.x * DIM+ threadIdx.x;	// This gives every thread a unique ID.
	double diffx, diffy, length;
	diffx = wx[i]-x[i];
	diffy = wy[i]-y[i];
	if ((x[i]-wx[i])*(x[i]-wx[i]) + (y[i]-wy[i])*(y[i]-wy[i]) > wr[i]*wr[i]) {
		length = sqrt(diffx*diffx+diffy*diffy);
		x[i] = round(x[i]+diffx/length); // round!
		y[i] = round(y[i]+diffy/length); // round!
		reached[i] = false;
	} else {
		x[i] = x[i];
		y[i] = y[i];
		reached[i] = true;
	}
}


__global__ void fadeKernel(int * heatmap){
	int i = blockIdx.y*DIM*DIM + blockIdx.x * DIM+ threadIdx.x;
	heatmap[i] *= 0.8;
}

__global__ void updateKernel(int * heatmap, double * desx, double* desy){
	int i = blockIdx.y*DIM*DIM + blockIdx.x * DIM+ threadIdx.x;

	int index = SIZE*desy[i]+desx[i];
	if (!(desx[i] < 0 || desx[i] > SIZE || desy[i] < 0 || desy[i] > SIZE))
		heatmap[index] += 40;
	if (heatmap[index] > 255)
		heatmap[index] = 255;
}


__global__ void scaleKernel(int * heatmap, int * scalemap){
	int index = blockIdx.y*DIM*DIM + blockIdx.x * DIM+ threadIdx.x;
	heatmap[index] *=0.8;

	int x = index%SIZE;
	int y = index/SIZE;
	int value = heatmap[index];
	for (int cellY=0;cellY< CELLSIZE; cellY++)
		for (int cellX=0;cellX<CELLSIZE;cellX++){
		int tmpIndex = (CELLSIZE*CELLSIZE*SIZE*y)+cellY*SIZE*CELLSIZE+x*CELLSIZE+cellX;
			scalemap[tmpIndex] = value;
	}
}

__global__ void blurKernel(int * scalemap, int * blurmap){
	long long index = blockIdx.y*DIM*DIM + blockIdx.x * blockDim.x+ threadIdx.x;
	long long x = index%SCALED_SIZE;
	long long y = index/SCALED_SIZE;
	const int w[5][5] = {
		{1,4,7,4,1},
		{4, 16,26,16,4},
		{7,26,41,26,7},
		{4, 16,26,16,4},
		{1,4,7,4,1}
	};
	int sum = 0;
	if (x < 2 && y < 2 && x > SCALED_SIZE-2 && y > SCALED_SIZE-2) {
		blurmap[index] = 0x00FF0000 | (1<<24);
  	} else {
		for (int k=-2;k<3; k++)
			for (int l=-2;l<3;l++){
				sum += w[2+k][2+l]*scalemap[index+l*SCALED_SIZE+k];
			}
			blurmap[index] = 0x00FF0000 | ((sum/273)<<24);
	}
}

void whereToGoCUDA(vector<Tagent*> *agents){
	double *x,*y,*wx,*wy,*wr,*lwx,*lwy;
	int NUM = (*agents).size();
	int CNUM = 0;
	int blockGridWidth, blockGridHeight;

	if (NUM > DIM*DIM){
		blockGridWidth = DIM;
		blockGridHeight = NUM/(DIM*DIM);
	} else {
		blockGridWidth = NUM/DIM;
		blockGridHeight = 1;
	}

	dim3 blockGridRows(blockGridWidth, blockGridHeight);
	dim3 threadBlockRows(DIM, 1);

	bool *reached, *hreached;
	CNUM = NUM;// + DIM*DIM*(blockGridHeight-1);
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



	dummyKernel<<<blockGridRows, threadBlockRows>>>(x,y,wx,wy,wr,lwx,lwy,reached);
	cudaThreadSynchronize();

	cudaMemcpy((*agents)[0]->getDesX(), x, sizeof(double) * NUM, cudaMemcpyDeviceToHost);
	cudaMemcpy((*agents)[0]->getDesY(), y, sizeof(double) * NUM, cudaMemcpyDeviceToHost);
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



void fadeHeatmap(int ** heatmap){
	int numpositions = SIZE*SIZE;
	int blockGridWidth, blockGridHeight;

	if (numpositions > DIM*DIM){
		blockGridWidth = DIM;
		blockGridHeight = numpositions/(DIM*DIM);
	} else {
		blockGridWidth = numpositions/DIM;
		blockGridHeight = 1;
	}

	dim3 blockGridRows(blockGridWidth, blockGridHeight);
	dim3 threadBlockRows(DIM, 1);

	int * hm;
	cudaMalloc( (void **)&hm, sizeof(int) * numpositions);
	cudaMemcpy(hm, heatmap[0], sizeof(int) * numpositions, cudaMemcpyHostToDevice);


	cudaEvent_t start, stop;
	cudaEventCreate(&start);
	cudaEventCreate(&stop);
	cudaEventRecord(start);
	fadeKernel<<<blockGridRows, threadBlockRows>>>(hm);
	cudaThreadSynchronize();
	cudaEventRecord(stop);
	cudaEventSynchronize(stop);
	float ms = 0;
	cudaEventElapsedTime(&ms, start, stop),
	totfade +=ms;
	printf("fade: %f ms\n", totfade);

	cudaMemcpy(heatmap[0], hm, sizeof(int) * numpositions, cudaMemcpyDeviceToHost);
	cudaFree(hm);
}


void updateHeatmap(vector<Tagent*> *agents, int ** heatmap){
	int numagents = agents->size(); // heatmapsize
	int numpositions = SIZE*SIZE;
	int blockGridWidth, blockGridHeight;


	if (numagents > DIM*DIM){
		blockGridWidth = DIM;
		blockGridHeight = numagents/(DIM*DIM);
	} else {
		blockGridWidth = numagents/DIM;
		blockGridHeight = 1;
	}
	dim3 blockGridRows(blockGridWidth, blockGridHeight);
	dim3 threadBlockRows(DIM, 1);

	int * hm;
	double *desx,*desy;
	cudaMalloc( (void **)&hm, sizeof(int) * numpositions);
	cudaMalloc( (void **)&desx, sizeof(double) * numagents);
	cudaMalloc( (void **)&desy, sizeof(double) * numagents);
	cudaMemcpy(hm, heatmap[0], sizeof(int) * numpositions, cudaMemcpyHostToDevice);
	cudaMemcpy(desx, (*agents)[0]->getDesX(), sizeof(double) * numagents, cudaMemcpyHostToDevice);
	cudaMemcpy(desy, (*agents)[0]->getDesY(), sizeof(double) * numagents, cudaMemcpyHostToDevice);

	cudaEvent_t start, stop;
	cudaEventCreate(&start);
	cudaEventCreate(&stop);
	cudaEventRecord(start);
	updateKernel<<<blockGridRows, threadBlockRows>>>(hm,desx,desy);
	cudaThreadSynchronize();
	cudaEventRecord(stop);
	cudaEventSynchronize(stop);
	float ms = 0;
	cudaEventElapsedTime(&ms, start, stop),
	totupdate +=ms;
	printf("update: %f ms\n", totupdate);
	

cudaMemcpy(heatmap[0], hm, sizeof(int) * numpositions, cudaMemcpyDeviceToHost);
	cudaFree(hm);

}

void scaleHeatmap(int ** heatmap, int ** scalemap){
	int numpositions = SIZE*SIZE;
	int blockGridWidth, blockGridHeight;
	if (numpositions > DIM*DIM){
		blockGridWidth = DIM;
		blockGridHeight = numpositions/(DIM*DIM);
	} else {
		blockGridWidth = numpositions/DIM;
		blockGridHeight = 1;
	}

	dim3 blockGridRows(blockGridWidth, blockGridHeight);
	dim3 threadBlockRows(DIM, 1);

	int * hm, * sm;

	cudaMalloc( (void **)&hm, sizeof(int) * numpositions);
	cudaMalloc( (void **)&sm, sizeof(int) * numpositions*CELLSIZE*CELLSIZE);

	cudaMemcpy(hm, heatmap[0], sizeof(int) * numpositions, cudaMemcpyHostToDevice);

	cudaEvent_t start, stop;
	cudaEventCreate(&start);
	cudaEventCreate(&stop);
	cudaEventRecord(start);
	scaleKernel<<<blockGridRows, threadBlockRows>>>(hm,sm);
	cudaThreadSynchronize();
	cudaEventRecord(stop);
	cudaEventSynchronize(stop);
	float ms = 0;
	cudaEventElapsedTime(&ms, start, stop),
	totscale +=ms;
	printf("scale: %f ms\n", totscale);



	cudaMemcpy(scalemap[0], sm, sizeof(int) * SCALED_SIZE*SCALED_SIZE, cudaMemcpyDeviceToHost);

	cudaFree(hm);
	cudaFree(sm);

}
void blurHeatmap(int ** scalemap, int ** blurmap){
	int numpositions = SCALED_SIZE*SCALED_SIZE;
	int blockGridWidth, blockGridHeight;

	if (numpositions > DIM*DIM){
		blockGridWidth = DIM;
		blockGridHeight = numpositions/(DIM*DIM);
	} else {
		blockGridWidth = numpositions/DIM;
		blockGridHeight = 1;
	}

	dim3 blockGridRows(blockGridWidth, blockGridHeight);
	dim3 threadBlockRows(DIM, 1);
	int * sm, *bm;

	cudaMalloc( (void **)&sm, sizeof(int) * numpositions);
	cudaMalloc( (void **)&bm, sizeof(int) * numpositions);
	
	cudaMemcpy(sm, scalemap[0], sizeof(int) * numpositions, cudaMemcpyHostToDevice);
	
	cudaEvent_t start, stop;
	cudaEventCreate(&start);
	cudaEventCreate(&stop);
	cudaEventRecord(start);
	blurKernel<<<blockGridRows, threadBlockRows>>>(sm, bm);
	cudaThreadSynchronize();
	cudaEventRecord(stop);
	cudaEventSynchronize(stop);
	float ms = 0;
	cudaEventElapsedTime(&ms, start, stop),
	totblur +=ms;
	printf("blur: %f ms\n", totblur);


	cudaMemcpy(blurmap[0], bm, sizeof(int) * numpositions, cudaMemcpyDeviceToHost);
	cudaFree(sm);
	cudaFree(bm);
}
