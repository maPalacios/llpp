//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2015
//

#include "ParseScenario.h"
#include <string>
#include <iostream>


/// object constructor
/// \date    2011-01-03
ParseScenario::ParseScenario(QString filename, CUDA_DATA *data) : QObject(0)
{
  QFile file(filename);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
  {
    return;
  }

  while (!file.atEnd())
  {
    QByteArray line = file.readLine();
    processXmlLine(line,data);
  }
}

vector<Ped::Tagent*> ParseScenario::getAgents() const
{
  return agents;
}

/// Called for each line in the file
void ParseScenario::processXmlLine(QByteArray dataLine,CUDA_DATA *data)
{
  xmlReader.addData(dataLine);

  while (!xmlReader.atEnd())
  {
    xmlReader.readNext();
    if (xmlReader.isStartElement())
    {
      handleXmlStartElement(data);
    }
    else if (xmlReader.isEndElement())
    {
      handleXmlEndElement();
    }
  }
}

void ParseScenario::handleXmlStartElement(CUDA_DATA *data)
{
  if (xmlReader.name() == "waypoint")
  {
    handleWaypoint();
  }
  else if (xmlReader.name() == "agent")
  {
    handleAgent(data);
  }
  else if (xmlReader.name() == "addwaypoint")
  {
    handleAddWaypoint();
  }
  else
  {
    // nop, unknown, ignore
  }
}

void ParseScenario::handleXmlEndElement()
{
  if (xmlReader.name() == "agent") {
    Ped::Tagent *a;
    foreach (a, tempAgents)
    {
      agents.push_back(a);
    }
  }
}

void ParseScenario::handleWaypoint()
{
  QString id = readString("id");
  double x = readDouble("x");
  double y = readDouble("y");
  double r = readDouble("r");

  Ped::Twaypoint *w = new Ped::Twaypoint(x, y, r);
  waypoints[id] = w;
}

void ParseScenario::handleAgent(CUDA_DATA *data)
{
  double x = readDouble("x");
  double y = readDouble("y");
  int n = readDouble("n");
  double dx = readDouble("dx");
  double dy = readDouble("dy");


  int getNum = tempAgents.size();
  std::cout << getNum << std::endl;
  tempAgents.clear();

  for (int i = 0; i < n; ++i)
  {
    (data->ax)[getNum+i] = x + qrand()/(RAND_MAX/dx) -dx/2;
    (data->ay)[getNum+i] = y + qrand()/(RAND_MAX/dy) -dy/2;

    Ped::Tagent *a = new Ped::Tagent(&(data->ax)[getNum+i],&(data->ay)[getNum+i], &(data->wpx)[getNum+i], &(data->wpy)[getNum+i], &(data->wpr)[getNum+i], &(data->lwpx)[getNum+i], &(data->lwpy)[getNum+i], &(data->visited)[getNum+i]);
    tempAgents.push_back(a);
  }

}

void ParseScenario::handleAddWaypoint()
{
  QString id = readString("id");
  Ped::Tagent *a;
  foreach (a, tempAgents)
  {
    a->addWaypoint(waypoints[id]);
  }
}

double ParseScenario::readDouble(const QString &tag)
{
  return readString(tag).toDouble();
}

QString ParseScenario::readString(const QString &tag)
{
  return xmlReader.attributes().value(tag).toString();
}
