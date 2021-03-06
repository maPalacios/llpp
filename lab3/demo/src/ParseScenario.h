//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2015
//




#ifndef _parsescenario_h_
#define _parsescenario_h_

#include "ped_agent.h"
#include "ped_waypoint.h"
#include <QtCore>
#include <QXmlStreamReader>
#include <vector>

struct CUDA_DATA{
  double * ax,*ay, *wpx, *wpy, *wpr, *lwpx, *lwpy, *desx, *desy;
  bool *visited;
};


using namespace std;

class ParseScenario : public QObject
{
  Q_OBJECT

public:
  ParseScenario(QString file, CUDA_DATA *dat);
  vector<Ped::Tagent*> getAgents() const;

  private slots:
  void processXmlLine(QByteArray data, CUDA_DATA *dat);






private:
  QXmlStreamReader xmlReader;

  map<QString, Ped::Twaypoint*> waypoints;
  vector<Ped::Tagent*> agents;
  vector<Ped::Tagent*> tempAgents;

  void handleWaypoint();
  void handleAgent(CUDA_DATA *dat);
  void handleAddWaypoint();
  void handleXmlStartElement(CUDA_DATA *dat);
  void handleXmlEndElement();

  QString readString(const QString &tag);
  double readDouble(const QString &tag);
};

#endif
