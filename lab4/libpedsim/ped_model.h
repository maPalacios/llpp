#ifndef _ped_model_h_
#define _ped_model_h_
#include <vector>
#include "ped_agent.h"
#include "ped_tree.h"
#include <map>
#include <set>
#include <iostream>
#include <atomic>

#define OFFSET 400

namespace Ped{
	class Tagent;
	class Ttree;
  enum IMPLEMENTATION {CUDA, VECTOR, OMP, PTHREAD, SEQ};

struct CUDA_DATA{
  double * ax,*ay, *wpx, *wpy, *wpr, *lwpx, *lwpy, *desx, *desy;
  bool *visited;
};

  class Model
  {
  public:

	atomic<bool> **grid;


    void setup(std::vector<Tagent*> agentsInScenario);
    void setPar(int type, int np);
    void setCollisionMode(int type){collisionMode = type;}
    int getPar();
    int getNumProcs();
    void tick();
    const std::vector<Tagent*> getAgents() const;
CUDA_DATA data;
	void callPartition();
	void setResponsibleTree(Ped::Ttree *tree, const Ped::Tagent * agent);

	void placeAgent(const Ped::Tagent *a);

	void cleanup();

	~Model();
void doSafeMovement(int left, int right, Ped::Tagent *agent);

	int const * const * getHeatmap() const;
	int getHeatmapSize() const;

  private:
    int np;

    IMPLEMENTATION implementation;
    std::vector<Tagent*> agents;

	// New

	static const int treeDepth = 10;

	Ped::Ttree *tree;

	std::map<const Ped::Tagent*, Ped::Ttree*> *treehash;

	set<const Ped::Tagent*> getNeighbors(int x, int y, int dist) const;
	void getNeighbors(list<const Ped::Tagent*>& neighborList, int x, int y, int d) const;
	int collisionMode;

	#define SIZE 1024
	#define CELLSIZE 5
	#define SCALED_SIZE SIZE*CELLSIZE

	int ** heatmap;
	int ** scaled_heatmap;
	int ** blurred_heatmap;
	void setupHeatmapSeq();
	void updateHeatmapSeq();
 };
}
#endif
