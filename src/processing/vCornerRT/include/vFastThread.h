/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Valentina Vasco
 * email:  valentina.vasco@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/// \defgroup Modules Modules
/// \defgroup vCorner vCorner
/// \ingroup Modules
/// \brief detects corner events using the Harris method

#ifndef __VFASTTHREAD__
#define __VFASTTHREAD__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <iCub/eventdriven/all.h>
#include <filters.h>
#include <fstream>
#include <math.h>

class vComputeFastThread : public yarp::os::Thread
{
private:

    ev::collectorPort *outthread;
    yarp::os::Stamp *ystamp_p;
    ev::temporalSurface *cSurf_p;

    yarp::os::Semaphore *mutex;

    yarp::os::Mutex *mutex_writer;
    yarp::os::Mutex *trytoread;
    yarp::os::Mutex *mutex_reader;
    int *readcount;
    unsigned int patch3[16];
    unsigned int patch4[20];

    ev::event<ev::AddressEvent> aep;

//    int circle3[16][2] =
//    {
//        {0, 3},
//        {1, 3},
//        {2, 2},
//        {3, 1},
//        {3, 0},
//        {3, -1},
//        {2, -2},
//        {1, -3},
//        {0, -3},
//        {-1, -3},
//        {-2, -2},
//        {-3, -1},
//        {-3, 0},
//        {-3, 1},
//        {-2, 2},
//        {-1, 3}
//    };

//    int circle4[20][2] =
//    {
//        {0, 4},
//        {1, 4},
//        {2, 3},
//        {3, 2},
//        {4, 1},
//        {4, 0},
//        {4, -1},
//        {3, -2},
//        {2, -3},
//        {1, -4},
//        {0, -4},
//        {-1, -4},
//        {-2, -3},
//        {-3, -2},
//        {-4, -1},
//        {-4, 0},
//        {-4, 1},
//        {-3, 2},
//        {-2, 3},
//        {-1, 4}
//    };


    bool suspended;

    bool detectcornerfast(unsigned int patch[16], unsigned int patch4[20]);

public:

    vComputeFastThread(ev::collectorPort *outthread, yarp::os::Mutex *mutex_writer, yarp::os::Mutex *mutex_reader,
                   int *readcount);
    void assignTask(ev::event<ev::AddressEvent> ae, ev::temporalSurface *cSurf, yarp::os::Stamp *ystamp);
    void suspend();
    void wakeup();
    bool available();
    bool threadInit() { return true; }
    void run();
    void threadRelease() {}
    void onStop();
};

class vFastThread : public yarp::os::Thread
{
private:

    //thread for queues of events
    ev::queueAllocator allocatorCallback;

    //data structures
//    ev::temporalSurface *surfaceleft;
//    ev::temporalSurface *surfaceright;
    ev::temporalSurface *surfaceOnL;
    ev::temporalSurface *surfaceOfL;
    ev::temporalSurface *surfaceOnR;
    ev::temporalSurface *surfaceOfR;

    //output port for the vBottle with the new events computed by the module
    yarp::os::BufferedPort<ev::vBottle> vBottleOut;
    yarp::os::BufferedPort<yarp::os::Bottle> debugPort;

    //list of thread for processing
//    std::vector<vComputeFastThread *> computeThreads;

    //to protect the writing
    yarp::os::Mutex *mutex_writer;
    yarp::os::Mutex *mutex_reader;
    int readcount;

    //thread for the output
    ev::collectorPort outthread;

    //synchronising value
    yarp::os::Stamp yarpstamp;

    ev::vtsHelper unwrapper;

//    int k;

    int circle3[16][2] =
    {
        {0, 3},
        {1, 3},
        {2, 2},
        {3, 1},
        {3, 0},
        {3, -1},
        {2, -2},
        {1, -3},
        {0, -3},
        {-1, -3},
        {-2, -2},
        {-3, -1},
        {-3, 0},
        {-3, 1},
        {-2, 2},
        {-1, 3}
    };

    int circle4[20][2] =
    {
        {0, 4},
        {1, 4},
        {2, 3},
        {3, 2},
        {4, 1},
        {4, 0},
        {4, -1},
        {3, -2},
        {2, -3},
        {1, -4},
        {0, -4},
        {-1, -4},
        {-2, -3},
        {-3, -2},
        {-4, -1},
        {-4, 0},
        {-4, 1},
        {-3, 2},
        {-2, 3},
        {-1, 4}
    };

    //parameters
    unsigned int height;
    unsigned int width;
    std::string name;
    bool strict;
    int nthreads;

    bool detectcornerfast(unsigned int patch3[16], unsigned int patch4[20]);

public:

    vFastThread(unsigned int height, unsigned int width, std::string name, bool strict, int nthreads);
    bool threadInit();
    bool open(std::string portname);
    void onStop();
    void run();

};


#endif
//empty line to make gcc happy
