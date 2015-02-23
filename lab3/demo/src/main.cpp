//#include "ped_agent.h"
#include "ped_model.h"
#include "MainWindow.h"
#include "ParseScenario.h"
#include "Parser/ParseNumOfAgents.h"
#include "Parser/tinyxml.h"
#include <string>
#include <atomic>

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QApplication>
#include <QTimer>
#include <thread>

#include <unistd.h>

#include "Timer.h"
#include <iostream>
#include <chrono>
#include <ctime>
#include <cstring>
#define SIMULATION_STEPS 1000000
enum IMPLEMENTATION {CUDA, VECTOR, OMP, PTHREAD, SEQ};



int main(int argc, char*argv[]) {
  Ped::Model model;
  bool timing_mode = 0;
  int i = 1;
  QString scenefile = "scenario.xml";
  string scenefileString = "scenario.xml"; 
  int par = SEQ;
  int np = 1;
  // 1.02
  // Argument handling
  while(i < argc)
    {
      if(argv[i][0] == '-' && argv[i][1] == '-') {
	  if(strcmp(&argv[i][2],"timing-mode") == 0){
	      cout << "Timing mode on\n";
	      timing_mode = true;
	    }
	  else if(strcmp(&argv[i][2],"openmp") == 0){
	    par = OMP;
	    cout << "Using openmp parallelisation.\n";
	    }
	  else if(strcmp(&argv[i][2],"pthreads") == 0){
	    par = PTHREAD;
	    cout << "Using pthreads parallelisation.\n";
	    }
	  else if(strcmp(&argv[i][2],"cuda") == 0){
	    par = CUDA;
	    cout << "Using CUDA parallelisation.\n";
	    }
	  else if(strcmp(&argv[i][2],"vector") == 0 || strcmp(&argv[i][2],"simd") == 0){
	    par = VECTOR;
	    cout << "Using vector parallelisation.\n";
    }
    else if(strcmp(&argv[i][2],"np") == 0){
      i++;
      if (i < argc)
        np = atoi(argv[i]);
      else
        cerr << "Too few arguments, --np needs a number." << endl;
    } else {
	      cerr << "Unrecognized command: \"" << argv[i] << "\". Ignoring ..." << endl;
    }
	}
      else // Assume it is a path to scenefile
	{
	  scenefile = argv[i];
          scenefileString = argv[i]; //(under scenefile = argv[i];)
	}
      i+=1;
    }
  ParseNumOfAgents numOfAgent(scenefileString.c_str());
  int size = numOfAgent.getNumOfAgents();
  CUDA_DATA data;
  data.ax = (double*)malloc(sizeof(double)*size);
  data.ay = (double*)malloc(sizeof(double)*size);
  data.wpx = (double*)malloc(sizeof(double)*size);
  data.wpy = (double*)malloc(sizeof(double)*size);
  data.wpr = (double*)malloc(sizeof(double)*size);
  data.lwpx = (double*)malloc(sizeof(double)*size);
  data.lwpy = (double*)malloc(sizeof(double)*size);
  data.visited = (bool*)malloc(sizeof(bool)*size);
  ParseScenario parser(scenefile, &data);

        int WIDTH = 1000;
        int HEIGHT = 1000;
        atomic<bool> * rows[WIDTH];
        for (int i=0;i<WIDTH;i++){
          rows[i] = (atomic<bool>*)malloc(sizeof(atomic<bool>)*HEIGHT);
        }
        for (int i=0;i<WIDTH;i++)
          for (int j=0;j<HEIGHT;j++)
            rows[i][j] = false;
    
        model.grid = rows;



  model.setup(parser.getAgents());
  model.setPar(par, np);

  QApplication app(argc, argv);
  MainWindow mainwindow(model);

  const int delay_ms = 10;
  Timer *timer;
  #define TICK_LIMIT 10000
  #define AS_FAST_AS_POSSIBLE 0
  if(timing_mode)
    {
      timer = new Timer(model,mainwindow,AS_FAST_AS_POSSIBLE);
      timer->setTickLimit(TICK_LIMIT);
    }
  else
    {
      timer = new Timer(model,mainwindow,delay_ms);
     mainwindow.show();

    }
  cout << "Demo setup complete, running ..." << endl;
  int retval = 0;
  std::chrono::time_point<std::chrono::system_clock> start,stop;
  start = std::chrono::system_clock::now();

  // If timing mode, just go as fast as possible without delays or graphical updates
  if(timing_mode)
  {
    timer->busyTick();
  }
  else
  {
    timer->qtTimerTick();
    retval = app.exec();
  }

  stop = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = stop-start;
  cout << "Time: " << elapsed_seconds.count() << " seconds." << endl;

  cout << "Done" << endl;
  delete (timer);
  return retval;
}
