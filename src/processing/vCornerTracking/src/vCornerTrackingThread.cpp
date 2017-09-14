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
//    int minAcceptableDelay =  5120;
    while(true) {

        ev::vQueue *q = 0;
        while(!q && !isStopping()) {
            q = inputPort.getNextQ(yarpstamp);
        }
        if(isStopping()) break;

//        unsigned int delay_n = inputPort.queryDelayN();
////        double increment = ((double)delay_n)/maxV;

//        double increment = 1.0 * (delay_n - q->size()) / minAcceptableDelay;
//        if(increment < 1.0)
//            increment = 1.0;

//        double currCount;
//        int currSkip,lastSkip = 0;
//        currCount = 0.0;
//        currSkip = (int)currCount;

//        int countProcessed = 0;
//        bool firstChecked = false;
//        ev::vQueue::iterator qi;
        std::pair <double, double> vel;
//        while(currSkip < q->size())  {

//            if(!firstChecked) {
//                qi = q->begin();
//                firstChecked = true;
//            } else {
//                qi = qi + (currSkip - lastSkip);
//                lastSkip = currSkip;
//            }

//            lastSkip = currSkip;
//            currCount += increment;
//            currSkip = (int)currCount;


        for(ev::vQueue::iterator qi = q->begin(); qi != q->end(); qi++) {

            //current corner event
            auto cep = is_event<LabelledAE>(*qi);

            //unwrap timestamp
            double currt = vtsHelper::tsscaler * unwrapper(cep->stamp);

            //update cluster velocity
            vel = clusterSet->update(cep, currt);

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
            }
        }

        inputPort.scrapQ();

    }

}
