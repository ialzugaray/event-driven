#include "vCornerRTThread_FAST.h"

using namespace ev;

vCornerThread::vCornerThread(unsigned int height, unsigned int width, std::string name, bool strict, int nthreads)
{
    this->height = height;
    this->width = width;
    this->name = name;
    this->strict = strict;
    this->nthreads = nthreads;

    std::cout << "Creating surfaces..." << std::endl;
    surfaceleft  = new temporalSurface(width, height);
    surfaceright = new temporalSurface(width, height);

    mutex_writer = new yarp::os::Mutex();
    mutex_reader = new yarp::os::Mutex();
    readcount = 0;

    for(int i = 0; i < nthreads; i ++) {
        computeThreads.push_back(new vComputeThread(&outthread, mutex_writer, mutex_reader, &readcount));
        computeThreads[i]->start();
    }
    std::cout << "Using " << nthreads << " threads for computation " << std::endl;

//    this->k = 0;

}

bool vCornerThread::threadInit()
{

    if(!allocatorCallback.open("/" + name + "/vBottle:i")) {
    std::cout << "could not open vBottleIn port " << std::endl;
        return false;
    }

    if(!outthread.open("/" + name + "/vBottle:o")) {
        std::cout << "could not open vBottleOut port" << std::endl;
        return false;
    }
    if(!outthread.start())
        return false;

    std::string debugPortName = "/" + name + "/score:o";
    if(!debugPort.open(debugPortName)) {
        std::cout << "could not open debug port" << std::endl;
        return false;
    }

    std::cout << "Thread initialised" << std::endl;
    return true;
}


void vCornerThread::onStop()
{
    allocatorCallback.close();
    allocatorCallback.releaseDataLock();

    for(int i = 0; i < nthreads; i++)
        delete computeThreads[i];

    delete surfaceleft;
    delete surfaceright;

    delete mutex_writer;
    delete mutex_reader;

}

void vCornerThread::run()
{
    while(true) {

        ev::vQueue *q = 0;
        while(!q && !isStopping()) {
            q = allocatorCallback.getNextQ(yarpstamp);
        }
        if(isStopping()) break;

//        int maxV = 1000000;
//        unsigned int delay_n = allocatorCallback.queryDelayN();
//        double acceptableRatio = ((double)delay_n)/maxV;
//        if(acceptableRatio <= 1.0)
//            acceptableRatio = 1;

//        double currCount;
//        int currSkip,lastSkip = 0;
//        currCount = 0.0;
//        currSkip = (int)currCount;

        int countProcessed = 0;
//        bool firstChecked = false;
//        ev::vQueue::iterator qi;
//        while(currSkip < q->size())  {

//            if(!firstChecked) {
//                qi = q->begin();
//                firstChecked = true;
//            } else {
//                qi = qi + (currSkip - lastSkip);
//                lastSkip = currSkip;
//            }

//            lastSkip = currSkip;
//            currCount += acceptableRatio;
//            currSkip = (int)currCount;

//            auto ae = ev::is_event<ev::AE>(*qi);
//            ev::temporalSurface *cSurf;
//            if(ae->getChannel() == 0)
//                cSurf = surfaceleft;
//            else
//                cSurf = surfaceright;

//            mutex_writer->lock();
//            cSurf->fastAddEvent(*qi);
//            mutex_writer->unlock();


        for(ev::vQueue::iterator qi = q->begin(); qi != q->end(); qi++) {

            auto ae = ev::is_event<ev::AE>(*qi);
            ev::temporalSurface *cSurf;
            if(ae->getChannel() == 0)
                cSurf = surfaceleft;
            else
                cSurf = surfaceright;
            cSurf->fastAddEvent(*qi);

            mutex_writer->lock();
            cSurf->fastAddEvent(*qi);
            mutex_writer->unlock();

//            for(int k = 0; k < nthreads; k++) {

//                //assign a task to a thread that is not managing a task
//                if(computeThreads[k]->available()) {
//                    computeThreads[k]->assignTask(ae, cSurf, &yarpstamp);
//                    countProcessed++;
//                    break;
//                }
//            }

            int k = 0;
            while(true) {

                //assign a task to a thread that is not managing a task
                if(computeThreads[k]->available()) {
                    computeThreads[k]->assignTask(ae, cSurf, &yarpstamp);
                    countProcessed++;
                    break;
                }
                if(++k == nthreads)
                    k = 0;
            }
        }

        if(debugPort.getOutputCount()) {
            yarp::os::Bottle &scorebottleout = debugPort.prepare();
            scorebottleout.clear();
            scorebottleout.addDouble((double)countProcessed/q->size());
            debugPort.write();
        }

        allocatorCallback.scrapQ();

    }

}


/*////////////////////////////////////////////////////////////////////////////*/
//threaded computation
/*////////////////////////////////////////////////////////////////////////////*/
vComputeThread::vComputeThread(collectorPort *outthread, yarp::os::Mutex *mutex_writer,
                               yarp::os::Mutex *mutex_reader, int *readcount)
{
    this->outthread = outthread;
    mutex = new yarp::os::Semaphore(0);
    suspended = true;

    this->mutex_writer = mutex_writer;
    this->mutex_reader = mutex_reader;
    this->readcount = readcount;
}

void vComputeThread::assignTask(ev::event<AddressEvent> ae, ev::temporalSurface *cSurf, yarp::os::Stamp *ystamp)
{
    cSurf_p = cSurf;
    ystamp_p = ystamp;
    aep = ae;
    wakeup();
}

void vComputeThread::suspend()
{
    suspended = true;
}

void vComputeThread::wakeup()
{
    suspended = false;
    mutex->post();
}

bool vComputeThread::available()
{
    return suspended;
}

void vComputeThread::run()
{

    while(true) {

        //if no task is assigned, wait
        if(suspended) {
            mutex->wait();
        }
        else {

            mutex_reader->lock();
            (*readcount)++;
            if(*readcount == 1)
                mutex_writer->lock();
            mutex_reader->unlock();

            cSurf_p->getEventsOnCircle3(patch3, aep->x, aep->y, circle3);
            cSurf_p->getEventsOnCircle4(patch4, aep->x, aep->y, circle4);

            mutex_reader->lock();
            (*readcount)--;
            if(*readcount == 0)
                mutex_writer->unlock();
            mutex_reader->unlock();

            if(detectcornerfast(patch3, patch4)) {
                auto ce = make_event<LabelledAE>(aep);
                ce->ID = 1;
                outthread->pushevent(ce, *ystamp_p);
            }
            suspend();

        }

    }
}

void vComputeThread::onStop()
{
    wakeup();
}

/**********************************************************/
bool vComputeThread::detectcornerfast(unsigned int patch3[16], unsigned int patch4[20])
{
    bool found_streak = false;

    for(int i = 0; i < 16; i++)
    {
        unsigned int ti = patch3[i];

        for (int streak_size = 3; streak_size <= 6; streak_size++)
        {
            //find the minimum timestamp in the current arc
            unsigned int min_t = ti;
            for (int j = 1; j < streak_size; j++)
            {
                int curri = (i+j)%16;
                unsigned int tj = patch3[curri];

                if (tj < min_t)
                    min_t = tj;
            }

            bool did_break = false;
            for (int j = streak_size; j < 16; j++)
            {
                int curri = (i+j)%16;
                unsigned int tj = patch3[curri];

                if (tj >= min_t)
                {
                    did_break = true;
                    break;
                }
            }

            if(did_break == false)
            {
                found_streak = true;
                break;
            }

        }
        if (found_streak)
        {
            break;
        }
    }

    if (found_streak)
    {
        found_streak = false;
        for (int i = 0; i < 20; i++)
        {
            unsigned int ti = patch4[i];

            for (int streak_size = 4; streak_size<= 8; streak_size++)
            {

                unsigned int min_t = ti;
                for (int j = 1; j < streak_size; j++)
                {
                    int curri = (i+j)%20;
                    unsigned int tj = patch4[curri];

                    if (tj < min_t)
                        min_t = tj;
                }

                bool did_break = false;
                for (int j = streak_size; j < 20; j++)
                {

                    int curri = (i+j)%20;
                    unsigned int tj = patch4[curri];

                    if (tj >= min_t)
                    {
                        did_break = true;
                        break;
                    }
                }

                if (!did_break)
                {
                    found_streak = true;
                    break;
                }
            }
            if (found_streak)
            {
                break;
            }
        }
    }

    return found_streak;

}

