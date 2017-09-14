#include "vCornerTrackingThread.h"

using namespace ev;

vCornerTrackingThread::vCornerTrackingThread(unsigned int height, unsigned int width, std::string name, bool strict,
                                             double mindistance, double maxdistance, double trefresh, int maxsize, int minevts)
{
    this->height = height;
    this->width = width;
    this->name = name;
    this->strict = strict;
    this->mindistance = mindistance;
    this->maxdistance = maxdistance;
    this->trefresh = trefresh;
    this->maxsize = maxsize;
    this->minevts = minevts;
    clusterSet = new clusterPool(mindistance, maxdistance, trefresh, maxsize, minevts);

}

bool vCornerTrackingThread::threadInit()
{

    if(!inputPort.open("/" + name + "/vBottle:i")) {
    std::cout << "could not open vBottleIn port " << std::endl;
        return false;
    }

    if(!outthread.open("/" + name + "/vBottle:o")) {
        std::cout << "could not open vBottleOut port" << std::endl;
        return false;
    }
    if(!outthread.start())
        return false;

    std::string debugPortName = "/" + name + "/dist:o";
    if(!debugPort.open(debugPortName)) {
        std::cout << "could not open debug port" << std::endl;
        return false;
    }

    std::cout << "Thread initialised" << std::endl;
    return true;
}


void vCornerTrackingThread::onStop()
{
    inputPort.close();
    inputPort.releaseDataLock();
    outthread.stop();
    delete clusterSet;
}


void vCornerTrackingThread::run()
{

    while(true) {

        ev::vQueue *q = 0;
        while(!q && !isStopping()) {
            q = inputPort.getNextQ(yarpstamp);
        }
        if(isStopping()) break;

        std::pair <double, double> vel;
//        double vx = 0.0;
//        double vy = 0.0;
//        int count = 0;

        for(ev::vQueue::iterator qi = q->begin(); qi != q->end(); qi++) {

            //current corner event
            auto cep = is_event<LabelledAE>(*qi);

            //unwrap timestamp
            double currt = vtsHelper::tsscaler * unwrapper(cep->stamp);
//            std::cout << currt << std::endl;

            //update cluster velocity
            vel = clusterSet->update(cep, currt);
//            vel = clusterSet->updateNew(cep, currt);

            //we output velocity if they are both non-zero
            if(vel.first && vel.second) {
                //create new flow event and assign to it the velocity of the current cluster
                auto fe = make_event<FlowEvent>(cep);
                fe->vx = vel.first;
                fe->vy = vel.second;

                outthread.pushevent(fe, yarpstamp);

                if(debugPort.getOutputCount()) {
                    yarp::os::Bottle &distbottleout = debugPort.prepare();
                    distbottleout.clear();
                    distbottleout.addDouble(inputPort.queryDelayN());
//                    distbottleout.addDouble(deltat);
                    debugPort.write();
                }

//                vx += vel.first;
//                vy += vel.second;
//                count++;
            }
        }

//        vx = vx / count;
//        vy = vy / count;

        inputPort.scrapQ();

    }

}
