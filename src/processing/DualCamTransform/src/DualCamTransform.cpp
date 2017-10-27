/*
 * Copyright (C) 2010 iCub Facility
 * Authors: Massimiliano Iacono
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

#include <DualCamTransform.h>


using namespace ev;
using namespace yarp::math;

int main(int argc, char * argv[])
{
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()) {
        yError() << "Could not find yarp network";
        return 1;
    }
    
    yarp::os::ResourceFinder rf;
    rf.setVerbose( true );
    rf.setDefaultContext( "cameraCalibration" );
    rf.setDefaultConfigFile( "DualCamTransform.ini" );
    rf.configure( argc, argv );
    
    DualCamTransformModule dualCamTransformModule;
    return dualCamTransformModule.runModule(rf);
}

bool DualCamTransformModule::updateModule() {
    
    //if calibrate parameter is set the update will perform a calibration step maxIter times
    if (calibrateLeft) {
        
        if ( leftImageCollector.isImageReady() && vLeftImageCollector.isImageReady() ) {
            yarp::sig::ImageOf<yarp::sig::PixelBgr> leftImg = leftImageCollector.getImage();
            yarp::sig::ImageOf<yarp::sig::PixelBgr> vLeftImg = vLeftImageCollector.getImage();
            
            if (performCalibStep( leftImg, vLeftImg, leftH )) {
                nIter++;
                yInfo() << nIter << " of " << maxIter << " images collected";
            }
            //When max number of iteration is reached calibration is finalized
            if (nIter >= maxIter){
                finalizeCalibration( leftH, "TRANSFORM_LEFT");
                calibrateLeft = false;
            }
        }
        return true;
    }
    
    if (calibrateRight) {
        
        if ( rightImageCollector.isImageReady() && vRightImageCollector.isImageReady() ) {
            yarp::sig::ImageOf<yarp::sig::PixelBgr> rightImg = rightImageCollector.getImage();
            yarp::sig::ImageOf<yarp::sig::PixelBgr> vRightImg = vRightImageCollector.getImage();
            
            if (performCalibStep( rightImg, vRightImg, rightH )) {
                nIter++;
                yInfo() << nIter << " of " << maxIter << " images collected";
            }
            //When max number of iteration is reached calibration is finalized
            if (nIter >= maxIter){
                finalizeCalibration( rightH, "TRANSFORM_RIGHT");
                calibrateRight = false;
            }
        }
        return true;
    }
    
    //After calibration step start reading events and compute the canvas size
    if (!calibrateRight && !calibrateLeft && !eventCollector.isPortReading()) {
        eventCollector.clearQueues();
        eventCollector.startReading();
        if ( cropToImage ) {
            int imgWidth, imgHeight;
            leftImageCollector.getImageSize(imgWidth, imgHeight);
            
            leftCanvasWidth = imgWidth;
            rightCanvasWidth = imgWidth;
            leftCanvasHeight = imgHeight;
            rightCanvasHeight = imgHeight;
            leftXOffset = 0;
            leftYOffset = 0;
            rightXOffset = 0;
            rightYOffset = 0;
        } else {
            getCanvasSize( leftH, leftCanvasWidth, leftCanvasHeight, leftXOffset, leftYOffset );
            getCanvasSize( rightH, rightCanvasWidth, rightCanvasHeight, rightXOffset, rightYOffset );
        }
    }
    
    
    
    //If image is ready transform and draw events on it
    if (leftImageCollector.isImageReady()){
        yarp::sig::ImageOf<yarp::sig::PixelBgr> &leftCanvas = leftImagePortOut.prepare();
        leftCanvas.resize(leftCanvasWidth,leftCanvasHeight);
        leftCanvas.zero();
        yarp::sig::ImageOf<yarp::sig::PixelBgr > leftImg = leftImageCollector.getImage();
        for ( int x = 0; x < leftImg.width(); ++x ) {
            for ( int y = 0; y < leftImg.height(); ++y ) {
                leftCanvas(x + leftXOffset, y + leftYOffset) = leftImg(x,y);
            }
        }
        ev::vQueue vLeftQueue = eventCollector.getEventsFromChannel(0);
        transform( leftCanvas, vLeftQueue, leftH, leftXOffset, leftYOffset );
        leftImagePortOut.write();
    }
    
    if (rightImageCollector.isImageReady()){

//        yarp::os::Bottle *boxIn = boxPortIn.read();
        yarp::os::Bottle boxIn = boxCollector.getBox();
        int xtl = 0, ytl = 0, xtr = 0, ytr = 0, xbl = 0, ybl = 0, xbr = 0, ybr = 0;
        if(!boxIn.isNull()) {
            xtl = boxIn.get(2).asInt();
            ytl = boxIn.get(3).asInt();

            xbr = boxIn.get(4).asInt();
            ybr = boxIn.get(5).asInt();

            xtr = boxIn.get(6).asInt();
            ytr = boxIn.get(7).asInt();

            xbl = boxIn.get(8).asInt();
            ybl = boxIn.get(9).asInt();
        }

        yarp::sig::ImageOf<yarp::sig::PixelBgr> &rightCanvas = rightImagePortOut.prepare();
        rightCanvas.resize(rightCanvasWidth,rightCanvasHeight);
        rightCanvas.zero();
        yarp::sig::ImageOf<yarp::sig::PixelBgr > rightImg = rightImageCollector.getImage();
        for ( int x = 0; x < rightImg.width(); ++x ) {
            for ( int y = 0; y < rightImg.height(); ++y ) {
                rightCanvas(x + rightXOffset, y + rightYOffset) = rightImg(x,y);
            }
        }
        ev::vQueue vRightQueue = eventCollector.getEventsFromChannel(1);
        transform( rightCanvas, vRightQueue, rightH, rightXOffset, rightYOffset );

        cv::Point topleft, bottomright;
        getMappedBox(rightH, xtl, ytl, xbr, ybr, xtr, ytr, xbl, ybl, topleft, bottomright, rightXOffset, rightYOffset);

        cv::Mat imOut((IplImage*)rightCanvas.getIplImage(),false);
        cv::rectangle(imOut, topleft, bottomright, CV_RGB(0, 255, 255) , 2);
        rightImagePortOut.write();
    }
    
    return true;
    
    
}

void DualCamTransformModule::transform( yarp::sig::ImageOf<yarp::sig::PixelBgr> &img, const vQueue &vQueue
                                        , const yarp::sig::Matrix &homography, int xOffset, int yOffset ) {

    int xhack = -50;
    int yhack = 50;
    ev::vBottle &outBottle = vPortOut.prepare();
    outBottle.clear();
    for ( auto &it : vQueue ) {
        
        auto v = is_event<AE >( it );
        
        double x = v->x;
        double y = v->y;
        yarp::sig::Vector evCoord( 3 );
        
        //Converting to homogeneous coordinates
        evCoord[0] = x;
        evCoord[1] = y;
        evCoord[2] = 1;
        
        //Applying trasformation
        evCoord *= homography;
        
        //Converting back from homogenous coordinates
        x = (evCoord[0] / evCoord[2]) + xOffset + 1 + xhack;
        y = (evCoord[1] / evCoord[2]) + yOffset + 1 + yhack;
        
        //Drawing event on img
        bool inBound = x >= 0 && x < img.width() && y >= 0 && y < img.height();
        if (inBound){
            img( x, y ) = yarp::sig::PixelBgr( 255, 255, 255 );
        }
        
        if (y <= 255) {
            outBottle.addEvent( v );
            v->x = x;
            v->y = y;
        }
    }
    if (outBottle.size() > 0) {
//        vPortOut.setEnvelope(vQueue.back()->stamp);
        vPortOut.write();
    }
}

void DualCamTransformModule::getMappedBox(const yarp::sig::Matrix &homography,
                                          int xtl, int ytl, int xbr, int ybr, int xtr, int ytr, int xbl, int ybl,
                                          cv::Point &topleft, cv::Point &bottomright,
                                          int xOffset, int yOffset) const {
    int xhack = -50;
    int yhack = 50;

    //Transform the coordinates of top-left and bottom-right corners of the box
    yarp::sig::Vector topLCorn( 3 );
    topLCorn[0] = xtl;
    topLCorn[1] = ytl;
    topLCorn[2] = 1;

    topLCorn *= homography;
    xtl = (topLCorn[0] / topLCorn[2]) + xOffset + 1 + xhack;
    ytl = (topLCorn[1] / topLCorn[2]) + yOffset + 1 + yhack;

    yarp::sig::Vector botRCorn( 3 );
    botRCorn[0] = xbr;
    botRCorn[1] = ybr;
    botRCorn[2] = 1;

    botRCorn *= homography;
    xbr = (botRCorn[0] / botRCorn[2]) + xOffset + 1 + xhack;
    ybr = (botRCorn[1] / botRCorn[2]) + yOffset + 1 + yhack;

    yarp::sig::Vector topRCorn( 3 );
    topRCorn[0] = xtr;
    topRCorn[1] = ytr;
    topRCorn[2] = 1;

    topRCorn *= homography;
    xtr = (topRCorn[0] / topRCorn[2]) + xOffset + 1 + xhack;
    ytr = (topRCorn[1] / topRCorn[2]) + yOffset + 1 + yhack;

    yarp::sig::Vector botLCorn( 3 );
    botLCorn[0] = xbl;
    botLCorn[1] = ybl;
    botLCorn[2] = 1;

    botLCorn *= homography;
    xbl = (botLCorn[0] / botLCorn[2]) + xOffset + 1 + xhack;
    ybl = (botLCorn[1] / botLCorn[2]) + yOffset + 1 + yhack;

    topleft = cv::Point(xtl, ytl);
    bottomright = cv::Point(xbr, ybr);

}

void DualCamTransformModule::finalizeCalibration( yarp::sig::Matrix &homography, std::string groupName) {
    //TODO Make this method more flexible and general
    homography /= nIter;
    homography = homography.transposed();
    cv::destroyAllWindows();
    cvWaitKey(1);
    yInfo() << "Camera calibration is over after " << nIter << " steps";
    nIter = 0;
    yInfo() << "Saving calibration results to " << confFileName;
    std::fstream confFile;
    confFile.open( confFileName.c_str());
    std::string line;
    bool groupFound = false;
    if (confFile.is_open()){
        while (std::getline(confFile, line)){
            if (line.find("[" + groupName +"]") != std::string::npos) {
                groupFound = true;
                break;
            }
        }
        if (!groupFound) {
            confFile.close();
            confFile.open(confFileName, std::ios::app);
            if (confFile.is_open())
                confFile << "\n[" << groupName << "]" << std::endl;
        }
        confFile << "\nhomography ( ";
        for (int r = 0; r < homography.rows(); ++r) {
            for (int c  = 0; c  < homography.cols(); ++c ) {
                confFile << homography(r, c ) << " ";
            }
        }
        confFile << ") \n\n";
        confFile.close();
    } else {
        yError() << "Cannot open config file, results not saved";
    }
    
    
}

void DualCamTransformModule::getCanvasSize( const yarp::sig::Matrix &homography, int &canvasWidth, int &canvasHeight, int &xOffset
                                    , int &yOffset ) const {
    //Transform the coordinates of each corner of the sensor
    yarp::sig::Vector botRCorn( 3 );
    botRCorn[0] = width;
    botRCorn[1] = height;
    botRCorn[2] = 1;
    
    botRCorn *= homography;
    botRCorn[0] /= botRCorn[2];
    botRCorn[1] /= botRCorn[2];
    
    yarp::sig::Vector topLCorn( 3 );
    topLCorn[0] = 0;
    topLCorn[1] = 0;
    topLCorn[2] = 1;
    
    topLCorn *= homography;
    topLCorn[0] /= topLCorn[2];
    topLCorn[1] /= topLCorn[2];
    
    yarp::sig::Vector topRCorn( 3 );
    topRCorn[0] = width;
    topRCorn[1] = 0;
    topRCorn[2] = 1;
    
    topRCorn *= homography;
    topRCorn[0] /= topRCorn[2];
    topRCorn[1] /= topRCorn[2];
    
    yarp::sig::Vector botLCorn( 3 );
    botLCorn[0] = 0;
    botLCorn[1] = height;
    botLCorn[2] = 1;
    
    botLCorn *= homography;
    botLCorn[0] /= botLCorn[2];
    botLCorn[1] /= botLCorn[2];
    
    //Getting the min and max coordinates of transformed corners
    int minX = std::min (topLCorn[0], botLCorn[0]);
    int minY = std::min (topLCorn[1], topRCorn[1]);
    int maxX = std::max (topRCorn[0], botRCorn[0]);
    int maxY = std::max (botLCorn[1], botRCorn[1]);
    
    //Horizontal and vertical offset of events wrt frames
    xOffset = - minX;
    yOffset = - minY;
    
    //Canvas size must be as big as to contain all transformed events
    canvasWidth =   maxX - minX + 1;
    canvasHeight =   maxY - minY + 1;
}

bool DualCamTransformModule::performCalibStep( yarp::sig::ImageOf<yarp::sig::PixelBgr> &frame
                                       , yarp::sig::ImageOf<yarp::sig::PixelBgr> &vImg, yarp::sig::Matrix &homography ) const {
    auto *frameIplImg = (IplImage *) frame.getIplImage();
    cv::Mat frameMat = cv::cvarrToMat( frameIplImg );
    imshow( "img", frameMat );
    
    auto *vIplImg = (IplImage *) vImg.getIplImage();
    cv::Mat vMat = cv::cvarrToMat( vIplImg );
    imshow( "vImg", vMat );
    cv::waitKey( 1 );
    cv::Size boardSize( 4, 11 );
    std::vector<cv::Point2f> frameCenters; //Vector for storing centers of circle grid on frame image
    std::vector<cv::Point2f> vCenters; //Vector for storing centers of circle grid on event image
    
    bool frameFound = findCirclesGrid( frameMat, boardSize, frameCenters,
                                       cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING );
    bool eventFound = findCirclesGrid( vMat, boardSize, vCenters,
                                       cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING );
    
    if ( frameFound && eventFound ) {
        cvCvtColor( frameIplImg, frameIplImg, CV_RGB2BGR );
        cvCvtColor( vIplImg, vIplImg, CV_RGB2BGR );
        
        cv::Mat frameCentersMat( frameCenters );
        cv::Mat vCentersMat( vCenters );
        drawChessboardCorners( frameMat, boardSize, frameCentersMat, frameFound );
        drawChessboardCorners( vMat, boardSize, vCentersMat, eventFound );
        cv::Mat h = findHomography( vCenters, frameCenters, CV_RANSAC );
        imshow( "img", frameMat );
        imshow( "vImg", vMat );
        cv::waitKey( 1000 );
        for ( int r = 0; r < homography.rows(); ++r) {
            for ( int c  = 0; c  < homography.cols(); ++c ) {
                homography( r, c ) += h.at<double>( r, c );
            }
        }
        return true;
    } else {
        return false;
    }
}

bool DualCamTransformModule::configure( yarp::os::ResourceFinder &rf ) {
    std::string moduleName = rf.check("name",yarp::os::Value("/DualCamTransform")).asString();
    setName(moduleName.c_str());
    cropToImage = rf.check("cropToImage",yarp::os::Value(false)).asBool();
    
    calibrateLeft = rf.check("calibrateLeft", yarp::os::Value(false)).asBool();
    calibrateRight = rf.check("calibrateRight", yarp::os::Value(false)).asBool();
    
    height = rf.check("height", yarp::os::Value(240)).asInt();
    width = rf.check("width", yarp::os::Value(304)).asInt();
    
    this -> confFileName = rf.getHomeContextPath().c_str();
    confFileName += "/DualCamTransform.ini";
    
    leftH.resize( 3, 3 );
    rightH.resize( 3, 3 );
    leftH.eye();
    rightH.eye();
    
    //If no calibration required, read homography from config file
    if (!calibrateLeft) {
        calibrateLeft = !readConfigFile( rf, "TRANSFORM_LEFT", leftH ); //If no config found, calibration necessary
    }
    
    //If no calibration required, read homography from config file
    if (!calibrateRight) {
        calibrateRight = !readConfigFile( rf, "TRANSFORM_RIGHT", rightH ); //If no config found, calibration necessary
    }
    
    //Initialize calibration
    if (calibrateLeft) {
        vLeftImageCollector.open( getName( "/left/vImg:i" ) );
        maxIter = rf.check( "maxIter", yarp::os::Value( 20 ) ).asInt();
        nIter = 0;
        vLeftImageCollector.start();
    }
    
    if (calibrateRight) {
        vRightImageCollector.open( getName( "/right/vImg:i" ) );
        maxIter = rf.check( "maxIter", yarp::os::Value( 20 ) ).asInt();
        nIter = 0;
        vRightImageCollector.start();
    }
    
    leftImageCollector.open(getName("/left/img:i"));
    rightImageCollector.open(getName("/right/img:i"));
    eventCollector.open(getName("/vBottle:i"));
    boxCollector.open(getName("/box:i"));
    leftImagePortOut.open(getName("/left/img:o"));
    rightImagePortOut.open(getName("/right/img:o"));
    vPortOut.open(getName("/vBottle:o"));
    eventCollector.start();
    leftImageCollector.start();
    rightImageCollector.start();
    boxCollector.start();
    return true;
}

bool DualCamTransformModule::readConfigFile( const yarp::os::ResourceFinder &rf, std::string groupName
                                     , yarp::sig::Matrix &homography ) const {
    yarp::os::Bottle &conf = rf.findGroup( groupName );
    
    //If config file not found, calibration necessary
    if ( conf.isNull() ) {
        yInfo() << "Could not find transform config in group " << groupName << ". Calibration is necessary.";
        return false;
    }
    
    yarp::os::Bottle *list = conf.find( "homography" ).asList();
    
    if ( list->size() != 9 ) {
        yError() << "Config file in " << groupName << "corrupted. Calibration is neccessary";
        return false;
    }
    
    for ( int r = 0; r < homography.rows(); ++r ) {
        for ( int c = 0; c < homography.cols(); ++c ) {
            homography( r, c ) = list->get( r * homography.rows() + c ).asDouble();
        }
    }
    
    return true;
}

bool DualCamTransformModule::interruptModule() {
    leftImagePortOut.interrupt();
    leftImageCollector.interrupt();
    vLeftImageCollector.interrupt();
    eventCollector.interrupt();
    boxCollector.interrupt();
    return true;
}

bool DualCamTransformModule::close() {
    leftImagePortOut.close();
    leftImageCollector.close();
    vLeftImageCollector.close();
    eventCollector.close();
    boxCollector.close();
    return true;
}

double DualCamTransformModule::getPeriod() {
    return 0;
}

/**************************ImagePort*********************************/

yarp::sig::ImageOf<yarp::sig::PixelBgr> ImagePort::getImage(){
    yarp::sig::ImageOf<yarp::sig::PixelBgr> outImg;
    mutex.lock();
    outImg = image;
    imageReady = false;
    mutex.unlock();
    return outImg;
};

void ImagePort::onRead( yarp::sig::ImageOf<yarp::sig::PixelBgr> &inImg ){
    mutex.lock();
    image = inImg;
    imageReady = true;
    mutex.unlock();
}

/**************************BoxPort*********************************/

yarp::os::Bottle BoxPort::getBox(){
    yarp::os::Bottle outBox;
    mutex.lock();
    outBox = box;
    box.clear();
    mutex.unlock();
    return outBox;
}

void BoxPort::onRead( yarp::os::Bottle &inBox ){
    mutex.lock();
    box = inBox;
    mutex.unlock();
}

/***********************EventPort***********************/


void EventPort::onRead(ev::vBottle &bot) {
    if (!isReading)
        return;
    //get new events
    ev::vQueue newQueue = bot.get<ev::AE>();
    if(newQueue.empty()){
        return;
    }

    mutex.wait();
    //append new events to queue
    
    for ( auto &it : newQueue ) {
        auto v = ev::is_event<ev::AE >( it );
        if (v->channel)
            vRightQueue.push_back(v);
        else
            vLeftQueue.push_back(v);
    }
    mutex.post();
}

ev::vQueue EventPort::getEventsFromChannel( int channel ) {
    
    ev::vQueue outQueue;
    mutex.wait();
    if (channel){
        outQueue = vRightQueue;
        vRightQueue.clear();
    } else {
        outQueue = vLeftQueue;
        vLeftQueue.clear();
    }
    mutex.post();
    return outQueue;
}

void EventPort::clearQueues() {
    mutex.wait();
    vLeftQueue.clear();
    vRightQueue.clear();
    mutex.post();
}

/**************************ImageCollector***************/

void ImageCollector::getImageSize( int &width, int &height ) {
    while (!imagePort.isImageReady()){
        yarp::os::Time::delay(.5);
    }
    yarp::sig::ImageOf<yarp::sig::PixelBgr> img = imagePort.getImage();
    width = img.width();
    height = img.height();
}
