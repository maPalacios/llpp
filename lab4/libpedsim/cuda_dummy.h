#ifndef _tick_cuda_h_
#define _tick_cuda_h_
#include "ped_agent.h"
#include <iostream>
/**
 * These functions are placeholders for your functions 
 * implementing CUDA solutions. 
 * They are weakly linked. If there is any other function with the same signature at
 * link-time the conflict is resolved by scrapping the weak one.
 *
 * You should only care if you want to compile on a non-CUDA machine.
 */


void __attribute__((weak)) whereToGoCUDA(vector<Ped::Tagent*> *agents) {
  std::cerr << "Notice: calling a dummy function" << __FUNCTION__ << std::endl;
}
void __attribute__((weak)) fadeHeatmap(int ** heatmap) {
  std::cerr << "Notice: calling a dummy function" << __FUNCTION__ << std::endl;
}
void __attribute__((weak)) updateHeatmap(vector<Ped::Tagent*> *agents, int ** heatmap) {
  std::cerr << "Notice: calling a dummy function" << __FUNCTION__ << std::endl;
}
void __attribute__((weak)) scaleHeatmap(int ** heatmap, int ** scalemap) {
  std::cerr << "Notice: calling a dummy function" << __FUNCTION__ << std::endl;
}
void __attribute__((weak)) blurHeatmap(int ** heatmap, int ** blurmap) {
  std::cerr << "Notice: calling a dummy function" << __FUNCTION__ << std::endl;
}





#endif
