#include "vFastThread.h"

using namespace ev;

vFastThread::vFastThread(unsigned int height, unsigned int width, std::string name, bool strict, int nthreads)
{
    std::cout << "Using FAST implementation..." << std::endl;

    this->height = height;
    this->width = width;
    this->name = name;
    this->strict = strict;
    this->nthreads = nthreads;

//    surfaceleft  = new temporalSurface(width, height);
//    surfaceright = new temporalSurface(width, height);

    surfaceOfR = new ev::temporalSurface(width, height);
    surfaceOnR = new ev::temporalSurface(width, height);
    surfaceOfL = new ev::temporalSurface(width, height);
    surfaceOnL = new ev::temporalSurface(width, height);

//    mutex_writer = new yarp::os::Mutex();
//    mutex_reader = new yarp::os::Mutex();
//    readcount = 0;

//    for(int i = 0; i < nthreads; i ++) {
//        computeThreads.push_back(new vComputeFastThread(&outthread, mutex_writer, mutex_reader, &readcount));
//        computeThreads[i]->start();
//    }
//    std::cout << "...with " << nthreads << " threads for computation " << std::endl;

//    this->k = 0;

}

bool vFastThread::threadInit()
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


void vFastThread::onStop()
{
    allocatorCallback.close();
    allocatorCallback.releaseDataLock();

//    for(int i = 0; i < nthreads; i++)
//        delete computeThreads[i];

//    delete surfaceleft;
//    delete surfaceright;

    delete surfaceOnL;
    delete surfaceOfL;
    delete surfaceOnR;
    delete surfaceOfR;

//    delete mutex_writer;
//    delete mutex_reader;

}

void vFastThread::run()
{
    while(true) {

        ev::vQueue *q = 0;
        while(!q && !isStopping()) {
            q = allocatorCallback.getNextQ(yarpstamp);
        }
        if(isStopping()) break;

        int maxV = 50000;
        unsigned int delay_n = allocatorCallback.queryDelayN();
        double acceptableRatio = ((double)delay_n)/maxV;
        if(acceptableRatio <= 1.0)
            acceptableRatio = 1;

        double currCount;
        int currSkip,lastSkip = 0;
        currCount = 0.0;
        currSkip = (int)currCount;

//        int countQi = 0;
        int countProcessed = 0;
//        for(ev::vQueue::iterator qi = q->begin(); qi != q->end(); qi++) {

//            auto ae = ev::is_event<ev::AE>(*qi);
//            ev::temporalSurface *cSurf;
//            if(ae->getChannel()) {
//                if(ae->polarity)
//                    cSurf = surfaceOfR;
//                else
//                    cSurf = surfaceOnR;
//            } else {
//                if(ae->polarity)
//                    cSurf = surfaceOfL;
//                else
//                    cSurf = surfaceOnL;
//            }

//            //unwrap stamp and add the event to the surface
//            (*qi)->stamp = unwrapper(ae->stamp);
//            cSurf->fastAddEvent(*qi);

//            if((countQi != (currSkip - lastSkip))) {

//                unsigned int patch3[16];
//                unsigned int patch4[20];
//                cSurf->getEventsOnCircle3(patch3, ae->x, ae->y, circle3);
//                cSurf->getEventsOnCircle4(patch4, ae->x, ae->y, circle4);
//                bool isc = detectcornerfast(patch3, patch4);
//                countProcessed++;

//                //if it's a corner, add it to the output bottle
//                if(isc) {
//                    auto ce = make_event<LabelledAE>(ae);
//                    ce->ID = 1;
//                    outthread.pushevent(ce, yarpstamp);
//                }

//                lastSkip = currSkip;
//                currCount += acceptableRatio;
//                currSkip = (int)currCount;
//            }

//            countQi++;
//        }

        bool firstChecked = false;
        ev::vQueue::iterator qi;
        while(currSkip < q->size())  {

            if(!firstChecked) {
                qi = q->begin();
                firstChecked = true;
            } else {
                qi = qi + (currSkip - lastSkip);
                lastSkip = currSkip;
            }

            lastSkip = currSkip;
            currCount += acceptableRatio;
            currSkip = (int)currCount;

            auto ae = ev::is_event<ev::AE>(*qi);
            ev::temporalSurface *cSurf;
            if(ae->getChannel()) {
                if(ae->polarity)
                    cSurf = surfaceOfR;
                else
                    cSurf = surfaceOnR;
            } else {
                if(ae->polarity)
                    cSurf = surfaceOfL;
                else
                    cSurf = surfaceOnL;
            }

            //unwrap stamp and add the event to the surface
            (*qi)->stamp = unwrapper(ae->stamp);
            cSurf->fastAddEvent(*qi);

            unsigned int patch3[16];
            unsigned int patch4[20];
            cSurf->getEventsOnCircle3(patch3, ae->x, ae->y, circle3);
            cSurf->getEventsOnCircle4(patch4, ae->x, ae->y, circle4);
            bool isc = detectcornerfast(patch3, patch4);
            countProcessed++;

            //if it's a corner, add it to the output bottle
            if(isc) {
                auto ce = make_event<LabelledAE>(ae);
                ce->ID = 1;
                outthread.pushevent(ce, yarpstamp);
            }

        }


//        for(ev::vQueue::iterator qi = q->begin(); qi != q->end(); qi++) {

//            auto ae = ev::is_event<ev::AE>(*qi);
//            ev::temporalSurface *cSurf;
//            if(ae->getChannel()) {
//                if(ae->polarity)
//                    cSurf = surfaceOfR;
//                else
//                    cSurf = surfaceOnR;
//            } else {
//                if(ae->polarity)
//                    cSurf = surfaceOfL;
//                else
//                    cSurf = surfaceOnL;
//            }

//            //unwrap stamp and add the event to the surface
//            (*qi)->stamp = unwrapper(ae->stamp);

////            mutex_writer->lock();
//            cSurf->fastAddEvent(*qi);
////            mutex_writer->unlock();

//            unsigned int patch3[16];
//            unsigned int patch4[20];
//            cSurf->getEventsOnCircle3(patch3, ae->x, ae->y, circle3);
//            cSurf->getEventsOnCircle4(patch4, ae->x, ae->y, circle4);
//            bool isc = detectcornerfast(patch3, patch4);

//            //if it's a corner, add it to the output bottle
//            if(isc) {
//                auto ce = make_event<LabelledAE>(ae);
//                ce->ID = 1;
//                outthread.pushevent(ce, yarpstamp);
//            }
//        }

        if(debugPort.getOutputCount()) {
            yarp::os::Bottle &scorebottleout = debugPort.prepare();
            scorebottleout.clear();
            scorebottleout.addDouble(allocatorCallback.queryDelayN());
            scorebottleout.addDouble((double)countProcessed/q->size());
            debugPort.write();
        }

        allocatorCallback.scrapQ();

    }

}

/**********************************************************/
bool vFastThread::detectcornerfast(unsigned int patch3[16], unsigned int patch4[20])
{
    bool found_streak = false;

    for(int i = 0; i < 16; i++)
    {
        unsigned int ti = patch3[i];

        for (int streak_size = 3; streak_size <= 6; streak_size++)
        {
            if(ti < patch3[(i-1+16)%16])
                continue;

            if(patch3[(i+streak_size-1)%16] < patch3[(i+streak_size)%16])
                continue;

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
                if(ti < patch4[(i-1+20)%20])
                    continue;

                if(patch3[(i+streak_size-1)%20] < patch3[(i+streak_size)%20])
                    continue;

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


///*////////////////////////////////////////////////////////////////////////////*/
////threaded computation
///*////////////////////////////////////////////////////////////////////////////*/
//vComputeFastThread::vComputeFastThread(collectorPort *outthread, yarp::os::Mutex *mutex_writer,
//                               yarp::os::Mutex *mutex_reader, int *readcount)
//{
//    this->outthread = outthread;
//    mutex = new yarp::os::Semaphore(0);
//    suspended = true;

//    this->mutex_writer = mutex_writer;
//    this->mutex_reader = mutex_reader;
//    this->readcount = readcount;
//}

//void vComputeFastThread::assignTask(ev::event<AddressEvent> ae, ev::temporalSurface *cSurf, yarp::os::Stamp *ystamp)
//{
//    cSurf_p = cSurf;
//    ystamp_p = ystamp;
//    aep = ae;
//    wakeup();
//}

//void vComputeFastThread::suspend()
//{
//    suspended = true;
//}

//void vComputeFastThread::wakeup()
//{
//    suspended = false;
//    mutex->post();
//}

//bool vComputeFastThread::available()
//{
//    return suspended;
//}

//void vComputeFastThread::run()
//{

//    while(true) {

//        //if no task is assigned, wait
//        if(suspended) {
//            mutex->wait();
//        }
//        else {

//            mutex_reader->lock();
//            (*readcount)++;
//            if(*readcount == 1)
//                mutex_writer->lock();
//            mutex_reader->unlock();

//            cSurf_p->getEventsOnCircle3(patch3, aep->x, aep->y, circle3);
//            cSurf_p->getEventsOnCircle4(patch4, aep->x, aep->y, circle4);

//            mutex_reader->lock();
//            (*readcount)--;
//            if(*readcount == 0)
//                mutex_writer->unlock();
//            mutex_reader->unlock();

//            if(detectcornerfast(patch3, patch4)) {
//                auto ce = make_event<LabelledAE>(aep);
//                ce->ID = 1;
//                outthread->pushevent(ce, *ystamp_p);
//            }
//            suspend();

//        }

//    }
//}

//void vComputeFastThread::onStop()
//{
//    wakeup();
//}

///**********************************************************/
//bool vComputeFastThread::detectcornerfast(unsigned int patch3[16], unsigned int patch4[20])
//{
//    bool found_streak = false;

//    for(int i = 0; i < 16; i++)
//    {
//        unsigned int ti = patch3[i];

//        for (int streak_size = 3; streak_size <= 6; streak_size++)
//        {
//            //find the minimum timestamp in the current arc
//            unsigned int min_t = ti;
//            for (int j = 1; j < streak_size; j++)
//            {
//                int curri = (i+j)%16;
//                unsigned int tj = patch3[curri];

//                if (tj < min_t)
//                    min_t = tj;
//            }

//            bool did_break = false;
//            for (int j = streak_size; j < 16; j++)
//            {
//                int curri = (i+j)%16;
//                unsigned int tj = patch3[curri];

//                if (tj >= min_t)
//                {
//                    did_break = true;
//                    break;
//                }
//            }

//            if(did_break == false)
//            {
//                found_streak = true;
//                break;
//            }

//        }
//        if (found_streak)
//        {
//            break;
//        }
//    }

//    if (found_streak)
//    {
//        found_streak = false;
//        for (int i = 0; i < 20; i++)
//        {
//            unsigned int ti = patch4[i];

//            for (int streak_size = 4; streak_size<= 8; streak_size++)
//            {

//                unsigned int min_t = ti;
//                for (int j = 1; j < streak_size; j++)
//                {
//                    int curri = (i+j)%20;
//                    unsigned int tj = patch4[curri];

//                    if (tj < min_t)
//                        min_t = tj;
//                }

//                bool did_break = false;
//                for (int j = streak_size; j < 20; j++)
//                {

//                    int curri = (i+j)%20;
//                    unsigned int tj = patch4[curri];

//                    if (tj >= min_t)
//                    {
//                        did_break = true;
//                        break;
//                    }
//                }

//                if (!did_break)
//                {
//                    found_streak = true;
//                    break;
//                }
//            }
//            if (found_streak)
//            {
//                break;
//            }
//        }
//    }

//    return found_streak;

//}

