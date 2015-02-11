#ifndef _ped_model_h_
#define _ped_model_h_
#include <vector>
#include "ped_agent.h"

namespace Ped{
  enum IMPLEMENTATION {CUDA, VECTOR, OMP, PTHREAD, SEQ};
  class Model
  {
  public:
    void setup(std::vector<Tagent*> agentsInScenario);
    void setPar(int type, int np);
    int getPar();
    int getNumProcs();
    void tick();
    const std::vector<Tagent*> getAgents() const;
  private:
    int par;
    int np;
    IMPLEMENTATION implementation;
    double *Ax; // x-position of agent
    double *Ay; // y-position of agent
    double *Az; // z-position of agent

    double *Wx; // x-position of waypoint
    double *Wy; // y-position of waypoint
    double *Wz; // z-position of waypoint
    
    Vector<Waypoint> *waypointlist;
  };
}
#endif
