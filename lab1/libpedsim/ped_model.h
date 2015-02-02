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
    void setPar(int arg);
    int getPar();
    void tick();
    const std::vector<Tagent*> getAgents() const;
  private:
    int par; // 0 = serial, 1 = openmp, 2 = pthreads
    IMPLEMENTATION implementation;
    std::vector<Tagent*> agents;
  };
}
#endif
