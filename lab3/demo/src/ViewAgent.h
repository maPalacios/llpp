#ifndef _view_agent_h
#define _view_agent_h

#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include "ped_agent.h"


class ViewAgent{
 public:
  ViewAgent(Ped::Tagent * agent,QGraphicsScene * scene);
  void paint();

 private:
  const Ped::Tagent * model ;
  double *x;
  double *y;
  QGraphicsRectItem * rect;

 };





#endif
