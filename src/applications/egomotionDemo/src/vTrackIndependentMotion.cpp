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

#include "vTrackIndependentMotion.h"
#include "yarp/math/Math.h"
#include <algorithm>
#include <cmath>

using namespace yarp::math;
using namespace ev;

int main(int argc, char * argv[])
{
    /* initialize yarp network */
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()) {
        yError() << "Could not connect to yarp";
        return -1;
    }

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setDefaultContext( "eventdriven" );
    rf.setDefaultConfigFile( "vEgomotionDemo.ini" );
    rf.configure( argc, argv );

    /* create the module */
    vTrackModule module;

    /* run the module: runModule() calls configure first and, if successful, it then runs */
    return module.runModule(rf);
}


/*//////////////////////////////////////////////////////////////////////////////
  MODULE
  ////////////////////////////////////////////////////////////////////////////*/
bool vTrackModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    setName((rf.check("name", yarp::os::Value("/vEgomotionDemo")).asString()).c_str());
    period = rf.check("period", yarp::os::Value(0.01)).asDouble();
    gazingActive = rf.check("start", yarp::os::Value(false)).asBool();
    res.height = rf.check("height", yarp::os::Value(240)).asDouble();
    res.width = rf.check("width", yarp::os::Value(304)).asDouble();

    if(!rpcPort.open(getName() + "/control"))
        return false;
    this->attach(rpcPort);

    //inputPort.setDemo(rf.check("demo", yarp::os::Value("gaze")).asString());

    if(!inputPort.open(getName() + "/vBottle:i"))
        return false;

    yarp::os::Property options;
    options.put("device", "gazecontrollerclient");
    options.put("local", getName());
    options.put("remote", "/iKinGazeCtrl");
    gazedriver.open(options);
    if(gazedriver.isValid()) {
        gazedriver.view(gazecontrol);
        gazecontrol->getHeadPose(headhomepos, headhomerot);
    } else {
        yWarning() << "Gaze Driver not opened and will not be used";
    }

    return true ;
}

bool vTrackModule::updateModule()
{

    static double htimeout = yarp::os::Time::now();
    if(yarp::os::Time::now() - htimeout > 3.0) {
        htimeout = yarp::os::Time::now();
    }


    yarp::sig::Vector target;
    inputPort.getTarget(target);

    if(target[2] <= 3) {
//        std::cout << "Weak signal " << std::endl;
        return true;
    }

    if(!gazingActive || !gazedriver.isValid()) {
        //yInfo() << "Gaze valid (gazing blocked)";
        return true;
    }

    htimeout = yarp::os::Time::now();

    yInfo() << "Doing gaze";
    yarp::sig::Vector tp;

//    if(tp[0] < -0.10) {
    std::cout << target[0] << " " << target[1] << " " << target[2] << std::endl;
    gazecontrol->lookAtMonoPixel(0, target, 0.5);
//    }

    return !isStopping();
}

double vTrackModule::getPeriod()
{
    return period;
}

bool vTrackModule::respond(const yarp::os::Bottle &command,
                                  yarp::os::Bottle &reply)
{
    reply.clear();

    if(command.get(0).asString() == "start") {
        reply.addString("starting");
        gazingActive = true;
    } else if(command.get(0).asString() == "stop") {
        reply.addString("stopping");
        gazingActive = false;
    } else {
        return false;
    }

    return true;


}

bool vTrackModule::interruptModule()
{

    inputPort.interrupt();
    return yarp::os::RFModule::interruptModule();
}

bool vTrackModule::close()
 {

    if(gazedriver.isValid()) {
        gazecontrol->stopControl();
        gazedriver.close();
    }

    inputPort.close();
    return yarp::os::RFModule::close();
}

