/*
 * Copyright (C) 2018 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff
 * email:  vadim.tikhanoff@iit.it
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

#ifndef QUERYTHREAD_H_
#define QUERYTHREAD_H_

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/LockGuard.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/PortReport.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/LogStream.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>

#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>

#include <highgui.h>
#include <opencv2/imgproc.hpp>
#include <cv.h>

#include <stdio.h>
#include <string>
#include <deque>
#include <algorithm>
#include <vector>
#include <iostream>
#include <fstream>
#include <list>


class QueryThread: public yarp::os::PeriodicThread
{

private:

    yarp::os::ResourceFinder                    &rf;
    yarp::os::Mutex                             mutex;
    bool                                        verbose;
    
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> port_in_img;
    yarp::os::BufferedPort<yarp::os::Bottle>    port_in_scores;
    
    yarp::os::Port                              port_out_crop;
    
    int                                         skip_frames;
    int                                         frame_counter;
    std::string                                 displayed_class;
    
    yarp::os::Bottle                            blob_person;
    int                                         personIndex;
    bool                                        allowedTrain;
    std::list<yarp::os::Bottle>                 scores_buffer;

public:

    QueryThread(yarp::os::ResourceFinder &_rf) : yarp::os::PeriodicThread(0.005), rf(_rf) { }
    
    bool set_skip_frames(int skip_frames);
    
    bool set_person(yarp::os::Bottle &person, int personIndex, bool allowedTrain);
    
    yarp::os::Bottle classify(yarp::os::Bottle &persons);
    
    bool clear_hist();

    virtual bool threadInit();

    virtual void run();

    virtual void interrupt();

    virtual bool releaseThread();
};


#endif /* QUERYTHREAD_H_ */
