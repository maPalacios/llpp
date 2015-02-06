#include "ViewAgent.h"

#include "MainWindow.h"

#include <QGraphicsItemAnimation>

ViewAgent::ViewAgent(Ped::Tagent * agent,QGraphicsScene * scene) :model(agent), pos(agent->getPosition()) 
{

  QBrush blueBrush(Qt::green);
  QPen outlinePen(Qt::black);
  outlinePen.setWidth(2);
  
  

  rect =  scene->addRect(MainWindow::cellToPixel(pos.pos[0]),MainWindow::cellToPixel(pos.pos[1]),MainWindow::cellsizePixel-1 ,MainWindow::cellsizePixel-1 , outlinePen, blueBrush);

}


void ViewAgent::paint(){
  rect->setRect(MainWindow::cellToPixel(pos.pos[0]),MainWindow::cellToPixel(pos.pos[1]),MainWindow::cellsizePixel-1,MainWindow::cellsizePixel-1);

  //Todo: animate movement
}
