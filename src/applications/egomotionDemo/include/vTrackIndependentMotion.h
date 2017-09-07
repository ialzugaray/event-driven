/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Valentina.Vasco@iit.it
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

// \defgroup RobotIO RobotIO
// \defgroup vTrackToRobot vTrackToRobot
// \ingroup RobotIO
// \brief perform gaze control given cluster track events

#ifndef __ICUB_VTRACKINDEPENDENTMOTION_H__
#define __ICUB_VTRACKINDEPENDENTMOTION_H__

#include <yarp/os/all.h>
#include <iCub/eventdriven/all.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/PolyDriver.h>
#include <deque>

using namespace ev;

/*//////////////////////////////////////////////////////////////////////////////
  VBOTTLE READER/PROCESSOR
  ////////////////////////////////////////////////////////////////////////////*/

class positionReader : public yarp::os::BufferedPort<ev::vBottle>
{
private:

    yarp::sig::Vector target;

public:

    positionReader()
    {
        target.resize(3);

        useCallback();
    }

    void onRead(vBottle &vBottleIn)
    {

        //get the Q
        vQueue q = vBottleIn.get<LabelledAE>();
        if(q.empty()) {
//            yWarning() << "q empty in callback function?";
            return;
        }

        //update our current best position of the object
        double cmx;
        double cmy;
        int count = 0;
        for(ev::vQueue::iterator qi = q.begin(); qi != q.end(); qi++) {
            auto v = is_event<LabelledAE>(*qi);
            if(v->ID == 2) {
                cmx += v->x;
                cmy += 240 - v->y;
                count++;
            }
        }
        cmx /= count;
        cmy /= count;
        target[0] = cmx;
        target[1] = cmy;
        target[2] = count;
    }

    void getTarget(yarp::sig::Vector &target_)
    {
        target_ = target;
    }


};

/*//////////////////////////////////////////////////////////////////////////////
  MODULE
  ////////////////////////////////////////////////////////////////////////////*/

class vTrackModule : public yarp::os::RFModule
{
private:

    ev::resolution res;
    //the event bottle input and output handler
    positionReader inputPort;

    //the remote procedure port
    yarp::os::RpcServer rpcPort;

    yarp::dev::PolyDriver gazedriver;
    yarp::dev::IGazeControl *gazecontrol;

    yarp::sig::Vector headhomepos, headhomerot;

    bool gazingActive;

    double period;


public:

    //the virtual functions that need to be overloaded
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();
    virtual bool close();
    virtual double getPeriod();
    virtual bool updateModule();

    virtual bool respond(const yarp::os::Bottle &command,
                         yarp::os::Bottle &reply);

};


#endif
//empty line to make gcc happy
