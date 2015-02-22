//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2015
//
#include "ped_agent.h"
#include "ped_waypoint.h"
#include <iostream>
#include <math.h>

using namespace std;

void Ped::Tagent::setX(double nx) { *x = nx; }
void Ped::Tagent::setY(double ny) { *y = ny; }


void Ped::Tagent::whereToGo() {
  computeForces();
}

void Ped::Tagent::go() {
/*
  *x = round(*x + waypointForce.x);
  *y = round(*y + waypointForce.y);
*/
	*desx = round(*x+ waypointForce.x);
	*desy = round(*y +waypointForce.y);

}



Ped::Tagent::Tagent(const Tagent &obj){
  cout << "copy" << endl;
  this->x = obj.x;
  this->y = obj.y;
  this->wpx = obj.wpx;
  this->wpy = obj.wpy;
  this->wpr = obj.wpr;
  this->lwpx = obj.lwpx;
  this->lwpy = obj.lwpy;
  this->visited = obj.visited;
  this->destination = obj.destination;
  this->lastDestination = obj.lastDestination;
  this->desiredPosition = obj.desiredPosition;
  this->waypoints = obj.waypoints;
  this->waypointBehavior = obj.waypointBehavior;
  this->pos = obj.pos;
  this->waypointForce = obj.waypointForce;
}

Ped::Tagent & Ped::Tagent::operator=(const Tagent& obj){
  *x = *(obj.x);
  *y = *(obj.y);
  *wpx = *(obj.wpx);
  *wpy = *(obj.wpy);
  *wpr = *(obj.wpr);
  *lwpx = *(obj.lwpx);
  *lwpy = *(obj.lwpy);
//  *desx = *(obj.desx);
//  *desy = *(obj.desy);
  *visited = *(obj.visited);
  desiredPosition = obj.desiredPosition;
  waypoints = obj.waypoints;
  waypointBehavior = obj.waypointBehavior;
  pos = obj.pos;
  waypointForce = obj.waypointForce;
  return *this;
}




void Ped::Tagent::addWaypoint(Twaypoint* wp) {
  waypoints.push_back(wp);
}

bool Ped::Tagent::removeWaypoint(const Twaypoint* wp) {
  if (destination == wp)
    destination = NULL;
  if (lastDestination == wp)
    lastDestination = NULL;

  bool removed = false;
  for (int i = waypoints.size(); i > 0; --i) {
    Twaypoint* currentWaypoint = waypoints.front();
    waypoints.pop_front();
    if (currentWaypoint != wp) {
      waypoints.push_back(currentWaypoint);
      removed = true;
    }
  }

  return removed;
}

void Ped::Tagent::clearWaypoints() {
  destination = NULL;
  lastDestination = NULL;

  for (int i = waypoints.size(); i > 0; --i) {
    waypoints.pop_front();
  }
}

void Ped::Tagent::computeForces() {
  waypointForce = computeWaypointForce();
}

Ped::Tvector Ped::Tagent::computeWaypointForce() {
  destination = getNextDestination();
  Tvector waypointDirection = computeDirection();
  Tvector force = waypointDirection.normalized();

  return force;
}

Ped::Tvector Ped::Tagent::computeDirection() {
  if (destination == NULL) {
    // Don't move, if nowhere to go
    return Ped::Tvector(0, 0, 0);
  }

  Tvector direction;
  bool reachesDestination = false; // if agent reaches destination in n
  if (lastDestination == NULL) {
    Twaypoint tempDestination(destination->getx(), destination->gety(), destination->getr());

    tempDestination.settype(Ped::Twaypoint::TYPE_POINT);
    direction = tempDestination.getForce(*x, *y, 0, 0, &reachesDestination);
  }
  else {
    direction = destination->getForce(*x, *y,
				      lastDestination->getx(),
				      lastDestination->gety(),
				      &reachesDestination);
  }

  if (reachesDestination == true) {

    // Circular waypoint chasing
    waypoints.push_back(destination);


    lastDestination = destination;
    destination = NULL;
    *visited = true;
  }

  return direction;
}

Ped::Twaypoint* Ped::Tagent::getNextDestination() {
  Ped::Twaypoint* nextDestination = NULL;
  if (destination != NULL) {
    nextDestination = destination; // agent hasn't arrived yet
  }
  else if (!waypoints.empty()) {
    *lwpx = *wpx;
    *lwpy = *wpy;
    nextDestination = getNextWaypoint();

    *wpx = nextDestination->getx();
    *wpy = nextDestination->gety();
    *wpr = nextDestination->getr();
  }

  return nextDestination;
}

Ped::Twaypoint* Ped::Tagent::getNextWaypoint() {
  Twaypoint* waypoint = waypoints.front();
  waypoints.pop_front();
  return waypoint;
}
