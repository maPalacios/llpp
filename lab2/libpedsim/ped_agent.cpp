
//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2015
//
#include "ped_agent.h"
#include "ped_waypoint.h"
#include <nmmintrin.h>
#include <math.h>



Ped::Tagent::Tagent(int posX, int posY) {
  Ped::Tagent::init(posX, posY);
}

Ped::Tagent::Tagent(double posX, double posY) {
  Ped::Tagent::init((int) round(posX), (int) round(posY));
}

void Ped::Tagent::init(int posX, int posY) {
  position.pos[0] = posX;
  position.pos[1] = posY;
  destination = NULL;
  lastDestination = NULL;
}

void Ped::Tagent::whereToGo() {
  computeForces();
}

void Ped::Tagent::go() {
  Tvector moveForce = waypointForce;

    __m128d *p = (__m128d*)position.pos;
    *p = _mm_add_pd(*p, *(__m128d*)moveForce.pos);

  // position.pos[0] = round(position.pos[0] + moveForce.pos[0]);
    //  position.pos[1] = round(position.pos[1] + moveForce.pos[1]);
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
    direction = tempDestination.getForce(position.pos[0], position.pos[1], 0, 0, &reachesDestination);
  }  else {
    direction = destination->getForce(position.pos[0], position.pos[1],
				      lastDestination->getx(),
				      lastDestination->gety(),
				      &reachesDestination);
  }

  if (reachesDestination == true) {

    // Circular waypoint chasing
    waypoints.push_back(destination);

    
    lastDestination = destination;
    destination = NULL;
  }

  return direction;
}

Ped::Twaypoint* Ped::Tagent::getNextDestination() {
  Ped::Twaypoint* nextDestination = NULL;
  if (destination != NULL) {
    nextDestination = destination; // agent hasn't arrived yet
  }
  else if (!waypoints.empty()) {
    nextDestination = getNextWaypoint();
  }
  return nextDestination;
}

Ped::Twaypoint* Ped::Tagent::getNextWaypoint() {
  Twaypoint* waypoint = waypoints.front();
  waypoints.pop_front();
  return waypoint;
}
