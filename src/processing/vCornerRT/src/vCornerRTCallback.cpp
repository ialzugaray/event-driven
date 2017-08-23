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

#include "vCornerRTCallback.h"

using namespace ev;

vCornerCallback::vCornerCallback(int height, int width, int sobelsize, int windowRad, double temporalsize,
                                 double sigma, int qlen, double thresh)
{
    this->height = height;
    this->width = width;
    this->windowRad = windowRad;
    this->temporalsize = temporalsize / ev::vtsHelper::tsscaler;

    //ensure that sobel size is an odd number
    if(!(sobelsize % 2))
    {
        std::cout << "Warning: sobelsize should be odd" << std::endl;
        sobelsize--;
        std::cout << "sobelsize = " << sobelsize << " will be used" << std::endl;
    }

    this->qlen = qlen;
    this->thresh = thresh;

    this->cpudelay = 0.005;
    this->prevstamp = 0;
    this->t1 = this->t2 = 0;// = yarp::os::Time::now();

    int gaussiansize = 2*windowRad + 2 - sobelsize;
    convolution.configure(sobelsize, gaussiansize);
    convolution.setSobelFilters();
    convolution.setGaussianFilter(sigma);

    std::cout << "Using a " << sobelsize << "x" << sobelsize << " filter ";
    std::cout << "and a " << 2*windowRad + 1 << "x" << 2*windowRad + 1 << " spatial window" << std::endl;

    //create surface representations
    std::cout << "Creating surfaces..." << std::endl;
//    surfaceOfR.initialise(height, width);
//    surfaceOnR.initialise(height, width);
//    surfaceOfL.initialise(height, width);
//    surfaceOnL.initialise(height, width);
    surfaceleft = new temporalSurface(width, height, this->temporalsize);
    surfaceright = new temporalSurface(width, height, this->temporalsize);

}
/**********************************************************/
bool vCornerCallback::open(const std::string moduleName, bool strictness)
{
    this->strictness = strictness;
    if(strictness) {
        std::cout << "Setting " << moduleName << " to strict" << std::endl;
        this->setStrict();
    }
    this->useCallback();

    std::string inPortName = "/" + moduleName + "/vBottle:i";
    bool check1 = BufferedPort<ev::vBottle>::open(inPortName);

    if(strictness) outPort.setStrict();
    std::string outPortName = "/" + moduleName + "/vBottle:o";
    bool check2 = outPort.open(outPortName);

    std::string debugPortName = "/" + moduleName + "/score:o";
    bool check3 = debugPort.open(debugPortName);

    outfile.open("/home/vvasco/dev/egomotion/affine/testing data/corners.txt");

    return check1 && check2 && check3;

}

/**********************************************************/
void vCornerCallback::close()
{
    //close ports
    debugPort.close();
    outPort.close();
    yarp::os::BufferedPort<ev::vBottle>::close();

    outfile.close();

    delete surfaceleft;
    delete surfaceright;

}

/**********************************************************/
void vCornerCallback::interrupt()
{
    //pass on the interrupt call to everything needed
    debugPort.interrupt();
    outPort.interrupt();
    yarp::os::BufferedPort<ev::vBottle>::interrupt();

}

/**********************************************************/
void vCornerCallback::onRead(ev::vBottle &bot)
{
    yarp::os::Stamp st;
    ev::vBottle fillerbottle;
    bool isc = false;

    /*get the event queue in the vBottle bot*/
    ev::vQueue q = bot.get<AE>();
    for(ev::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {
        auto ae = is_event<AE>(*qi);
        ev::temporalSurface *cSurf;
        if(ae->getChannel() == 0)
            cSurf = surfaceleft;
        else
            cSurf = surfaceright;
        cSurf->fastAddEvent(*qi);

        //            vQueue subsurf;
        ////            cSurf->getSurfaceN(subsurf, 0, qlen, windowRad, ae->x, ae->y);
        //            cSurf->getSurf(subsurf, windowRad);
        //            isc = detectcorner(subsurf, ae->x, ae->y);

        //            this->getEnvelope(st);
        //            outfile << ae->channel << " " << unwrapper(ae->stamp) << " " << ae->polarity << " "
        //                    << ae->x << " " << ae->y << " " << std::setprecision(15) << st.getTime() << " ";

        isc = detectcornerfast(cSurf, ae, circle3, circle4);

        //if it's a corner, add it to the output bottle
        if(isc) {
            auto ce = make_event<LabelledAE>(ae);
            ce->ID = 1;
            fillerbottle.addEvent(ce);

//            outfile << ce->ID << std::endl;
        }
//        else
//            outfile << 0 << std::endl;

        if(debugPort.getOutputCount()) {
            yarp::os::Bottle &scorebottleout = debugPort.prepare();
            scorebottleout.clear();
            scorebottleout.addDouble(cpudelay);
            debugPort.write();
        }

    }

    if( (yarp::os::Time::now() - t2) > 0.001 && fillerbottle.size() ) {
        yarp::os::Stamp st;
        this->getEnvelope(st);
        outPort.setEnvelope(st);
        ev::vBottle &eventsout = outPort.prepare();
        eventsout.clear();
        eventsout = fillerbottle;
        outPort.write(strictness);
        fillerbottle.clear();
        t2 = yarp::os::Time::now();
    }

}

/**********************************************************/
bool vCornerCallback::detectcorner(const vQueue subsurf, int x, int y)
{

    //set the final response to be centred on the curren event
    convolution.setResponseCenter(x, y);

    //update filter response
    for(unsigned int i = 0; i < subsurf.size(); i++)
    {
        //events are in the surface
        auto vi = is_event<AE>(subsurf[i]);
        convolution.applysobel(vi);

    }
    convolution.applygaussian();

    double score = convolution.getScore();

    //reset responses
    convolution.reset();

//    std::cout << score << std::endl;

    //if score > thresh tag ae as ce
    return score > thresh;

}

/**********************************************************/
bool vCornerCallback::detectcornerfast(ev::temporalSurface *cSurf, ev::event<ev::AddressEvent> ae,
                                       int (&circle3_)[16][2], int (&circle4_)[20][2])
{
    bool found_streak = false;
    vQueue evonSurf;

    for (int i = 0; i < 16; i++)
    {
        for (int streak_size = 3; streak_size <= 6; streak_size++)
        {
            // check that streak event is larger than neighbor
            int xi = ae->x + circle3_[i][0];
            int yi = ae->y + circle3_[i][1];
            cSurf->getSurf(evonSurf, xi, yi, 0);
            double tsi = evonSurf[0]->stamp;

            int xe = ae->x + circle3_[(i-1+16)%16][0];
            int ye = ae->y + circle3_[(i-1+16)%16][1];

            cSurf->getSurf(evonSurf, xe, ye, 0);
            double tse = evonSurf[0]->stamp;

            if (tsi < tse)
                continue;

            // check that streak event is larger than neighbor
            int xsi = ae->x + circle3_[(i+streak_size-1)%16][0];
            int ysi = ae->y + circle3_[(i+streak_size-1)%16][1];
            cSurf->getSurf(evonSurf, xsi, ysi, 0);
            double tssi = evonSurf[0]->stamp;

            int xse = ae->x + circle3_[(i+streak_size)%16][0];
            int yse = ae->y + circle3_[(i+streak_size)%16][1];
            cSurf->getSurf(evonSurf, xse, yse, 0);
            double tsse = evonSurf[0]->stamp;

            if (tssi < tsse)
                continue;

            double min_t = tsi;
            for (int j = 1; j < streak_size; j++)
            {
                int xj = ae->x + circle3_[(i+j)%16][0];
                int yj = ae->y + circle3_[(i+j)%16][1];
                cSurf->getSurf(evonSurf, xj, yj, 0);

                const double tj = evonSurf[0]->stamp;
                if (tj < min_t)
                    min_t = tj;
            }

            bool did_break = false;
            for (int j = streak_size; j < 16; j++)
            {
                int xj = ae->x + circle3_[(i+j)%16][0];
                int yj = ae->y + circle3_[(i+j)%16][1];
                cSurf->getSurf(evonSurf, xj, yj, 0);

                const double tj = evonSurf[0]->stamp;

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

    if (found_streak)
    {
        found_streak = false;
        for (int i=0; i<20; i++)
        {
            for (int streak_size = 4; streak_size<=8; streak_size++)
            {
                // check that streak event is larger than neighbor
                int xi = ae->x + circle4_[i][0];
                int yi = ae->y + circle4_[i][1];
                cSurf->getSurf(evonSurf, xi, yi, 0);
                double tsi = evonSurf[0]->stamp;

                int xe = ae->x + circle4_[(i-1+20)%20][0];
                int ye = ae->y + circle4_[(i-1+20)%20][1];

                cSurf->getSurf(evonSurf, xe, ye, 0);
                double tse = evonSurf[0]->stamp;

                if (tsi < tse)
                    continue;

                // check that streak event is larger than neighbor
                int xsi = ae->x + circle4_[(i+streak_size-1)%20][0];
                int ysi = ae->y + circle4_[(i+streak_size-1)%20][1];
                cSurf->getSurf(evonSurf, xsi, ysi, 0);
                double tssi = evonSurf[0]->stamp;

                int xse = ae->x + circle4_[(i+streak_size)%20][0];
                int yse = ae->y + circle4_[(i+streak_size)%20][1];
                cSurf->getSurf(evonSurf, xse, yse, 0);
                double tsse = evonSurf[0]->stamp;

                if (tssi < tsse)
                    continue;

                double min_t = evonSurf[0]->stamp;
                for (int j=1; j<streak_size; j++)
                {
                    int xj = ae->x + circle4_[(i+j)%20][0];
                    int yj = ae->y + circle4_[(i+j)%20][1];
                    cSurf->getSurf(evonSurf, xj, yj, 0);

                    const double tj = evonSurf[0]->stamp;
                    if (tj < min_t)
                        min_t = tj;
                }

                bool did_break = false;
                for (int j=streak_size; j<20; j++)
                {
                    int xj = ae->x + circle4_[(i+j)%20][0];
                    int yj = ae->y + circle4_[(i+j)%20][1];
                    cSurf->getSurf(evonSurf, xj, yj, 0);

                    const double tj =  evonSurf[0]->stamp;
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

//empty line to make gcc happy

