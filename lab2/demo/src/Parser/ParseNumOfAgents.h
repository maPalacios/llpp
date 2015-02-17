#ifndef _parsesnumofagents_h_
#define _parsesnumofagents_h_

#include "tinyxml.h"


using namespace std;

class ParseNumOfAgents
{

public:
  ParseNumOfAgents(const char* file);
  int getNumOfAgents(); 
	
private:
  int numOfAgents;
};

#endif
