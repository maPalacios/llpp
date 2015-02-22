//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2015
//
#ifndef _ped_agent_h_
#define _ped_agent_h_ 1


// g++ does not understand cuda stuff. This makes it ignore them. (use this if you want)
#ifndef __CUDACC__
#define __device__
#define __host__
#endif

#include "ped_vector.h"
#include <vector>
#include <deque>

using namespace std;

namespace Ped {
  class Twaypoint;

  class Tagent {
  public:
    Tagent(double *posX, double *posY, double *wpX, double * wpY, double *wpR, double *lwpX, double * lwpY, double * desX, double * desY, bool*vis){
      x = posX; y= posY; wpx = wpX; wpy = wpY; wpr = wpR, lwpx = lwpX, lwpy = lwpY, desx = desX, desy = desY;
      visited = vis;
      *visited = false;
      destination = NULL; lastDestination = NULL;
    };



    // Computes forces that determine the next position
    void whereToGo();
    void whereToGoCUDA();

    // Update the position according to computed forces
    void go();

//    const Tvector& getPosition() const { pos = Tvector(*x, *y, 0); return &pos;}
    double getX() const { return *x; };
    double getY() const { return *y; };
    double* getPosX() const { return x; };
    double* getPosY() const { return y; };
    void setX(double nx);
    void setY(double ny);
    double* getPosWX() const { return wpx; };
    double* getPosWY() const { return wpy; };
    double* getPosWR() const { return wpr; };
    double* getPosLWX() const { return lwpx; };
    double* getPosLWY() const { return lwpy; };
    double* getDesX() const { return desx; };
    double* getDesY() const { return desy; };
    bool* getVisited() const { return visited; };


	// 	NEW!
	  int getDesiredX() const {return *desx;}
	  int getDesiredY() const {return *desy;}





    void addWaypoint(Twaypoint* wp);
    bool removeWaypoint(const Twaypoint* wp);
    void clearWaypoints();
    void setWaypointBehavior(int mode) { waypointBehavior = mode; };

    bool reachedDestination() { return (destination == NULL); };

    Twaypoint* getNextDestination();

Tagent(const Tagent &obj);
Tagent & operator=(const Tagent& obj);

  private:
    Tagent() {};

    double *x, *y, *wpx, *wpy, *wpr, *lwpx, *lwpy, *desx, *desy;
    bool * visited;
    // The current position

    Twaypoint* destination;
    Twaypoint* lastDestination;

	 Tvector desiredPosition;

    deque<Twaypoint*> waypoints;
    int waypointBehavior;
    Tvector pos;
    Tvector waypointForce;
    // The force towards the current destination

   // Computes the forces that determine the next position
    void computeForces();

    // Computing waypoint force and directions
    Tvector computeWaypointForce();
    Tvector computeDirection();
    Twaypoint* getNextWaypoint();
  };
}

#endif
