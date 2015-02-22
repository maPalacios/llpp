#include"ParseNumOfAgents.h"

ParseNumOfAgents::ParseNumOfAgents(const char* filename)
{
  numOfAgents = 0;
  TiXmlDocument doc(filename);
  if(doc.LoadFile())
  {
    TiXmlElement* root = doc.RootElement();
    for(TiXmlNode * item = root -> FirstChild("agent");item;item = item ->NextSibling("agent"))
      {
        TiXmlElement *element = item -> ToElement();
        int num;
	element->QueryIntAttribute("n", &num);
        numOfAgents += num;
      }
  }
  else
  {
    //do nothing
  }
}

int ParseNumOfAgents::getNumOfAgents()
{
  return numOfAgents;
}
