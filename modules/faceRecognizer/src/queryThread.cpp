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

#include <queryThread.h>
#include <yarp/cv/Cv.h>

/********************************************************/
bool QueryThread::threadInit()
{
    std::lock_guard<std::mutex> lg(mtx);
    verbose = rf.check("verbose");
    std::string name = rf.find("name").asString().c_str();
    skip_frames = rf.check("skip_frames",yarp::os::Value(5)).asInt32();
    
    personIndex = -1;
    allowedTrain = false;
    
    frame_counter = 0;
    displayed_class="?";

    img_cnt = 0;
    
    //input
    port_in_scores.open(("/"+name+"/scores:i").c_str());
    port_out_crop.open(("/"+name+"/crop:o").c_str());
    
    return true;
}

/********************************************************/
void QueryThread::setImage(const yarp::sig::ImageOf<yarp::sig::PixelRgb> &img,
                           const yarp::os::Stamp &stamp)
{
    std::lock_guard<std::mutex> lg(img_mtx);
    img_buffer=std::make_pair(img,stamp);
    img_cnt++;
}

/********************************************************/
bool QueryThread::getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb> &img,
                           yarp::os::Stamp &stamp)
{
    std::lock_guard<std::mutex> lg(img_mtx);
    if (img_cnt>0)
    {
        img=img_buffer.first;
        stamp=img_buffer.second;
        img_cnt=0;
        return true;
    }
    else
        return false;
}

/********************************************************/
void QueryThread::run()
{
    if (personIndex>-1)
    {
        std::lock_guard<std::mutex> lg(mtx);
        yarp::sig::ImageOf<yarp::sig::PixelRgb> img;
        yarp::os::Stamp stamp;
        if (!getImage(img,stamp))
        {
            return;
        }
        cv::Mat img_mat = yarp::cv::toCvMat(img);
    
        int tlx  = -1;
        int tly  = -1;
        int brx  = -1;
        int bry  = -1;
        bool cropValid = false;
        
        if (blob_person.size()>0 && allowedTrain)
        {
            yInfo()<<"TRAININ Person blob" << blob_person.toString().c_str() << "with index " << personIndex;
            yarp::os::Bottle *window = blob_person.get(personIndex).asList();
            tlx = window->get(0).asInt32();
            tly = window->get(1).asInt32();
            brx = window->get(2).asInt32();
            bry = window->get(3).asInt32();
            
            if (tlx<5)
                tlx = 5;
            if (tlx>img_mat.cols-10)
                tlx = img_mat.cols-10;
            if (tly<5)
                tly = 5;
            if (tly>img_mat.rows-10)
                tly = img_mat.rows-10;
            
            if (brx<10)
                brx = 10;
            if (brx>img_mat.cols-5)
                brx = img_mat.cols-5;
            if (bry<10)
                bry = 10;
            if (bry>img_mat.rows-5)
                bry = img_mat.rows-5;
            
            cropValid = true;
        }
        
        if (cropValid)
        {
            cv::Rect img_ROI = cv::Rect(cv::Point( tlx, tly ), cv::Point( brx, bry ));
            cv::Mat img_crop_mat;
            img_mat(img_ROI).copyTo(img_crop_mat);

            if (frame_counter<skip_frames)
            {
                frame_counter++;
            }
            else
            {
                port_out_crop.setEnvelope(stamp);
                auto img_crop = yarp::cv::fromCvMat<yarp::sig::PixelRgb>(img_crop_mat);
                port_out_crop.write(img_crop);
                frame_counter = 0;
            }
        }
    }
}

/********************************************************/
yarp::os::Bottle QueryThread::classify(yarp::os::Bottle &persons)
{
    std::lock_guard<std::mutex> lg(mtx);
    yarp::os::Bottle reply;

    // as classify() gets called within the same thread that calls
    // setImage(), we are here safe to copy the image straight away
    auto img=img_buffer.first;  // copying is strictly required
    cv::Mat img_mat = yarp::cv::toCvMat(img);
    
    yInfo() << "Starting classification";
    
    for (int b=0; b<persons.size(); b++)
    {
        int tlx  = -1;
        int tly  = -1;
        int brx  = -1;
        int bry  = -1;
        
        yarp::os::Bottle *window = persons.get(b).asList();
        tlx = window->get(0).asInt32();
        tly = window->get(1).asInt32();
        brx = window->get(2).asInt32();
        bry = window->get(3).asInt32();
        
        if (tlx<5)
            tlx = 5;
        if (tlx>img_mat.cols-10)
            tlx = img_mat.cols-10;
        if (tly<5)
            tly = 5;
        if (tly>img_mat.rows-10)
            tly = img_mat.rows-10;
        
        if (brx<10)
            brx = 10;
        if (brx>img_mat.cols-5)
            brx = img_mat.cols-5;
        if (bry<10)
            bry = 10;
        if (bry>img_mat.rows-5)
            bry = img_mat.rows-5;
        
        cv::Rect img_ROI = cv::Rect(cv::Point( tlx, tly ), cv::Point( brx, bry ));
        cv::Mat img_crop_mat;
        img_mat(img_ROI).copyTo(img_crop_mat);
        auto img_crop = yarp::cv::fromCvMat<yarp::sig::PixelRgb>(img_crop_mat);
        port_out_crop.write(img_crop);
        
        yarp::os::Bottle &Obj_score=reply.addList();
        
        yarp::os::Bottle *class_scores = port_in_scores.read(true);
        
        if (class_scores == NULL)
            yError() << "error with scores";
        else
        {
            for (int i =0; i<class_scores->size(); i++)
            {
                
                yarp::os::Bottle *obj=class_scores->get(i).asList();
               
                yarp::os::Bottle &current_score=Obj_score.addList();
                current_score.addString(obj->get(0).asString().c_str());
                double normalizedVal=((obj->get(1).asFloat64())+1.0)/2.0;
            
                current_score.addFloat64(normalizedVal);
            }
        }
    }
    return reply;
}

/********************************************************/
bool QueryThread::set_skip_frames(int skip_frames)
{
    std::lock_guard<std::mutex> lg(mtx);
    if (skip_frames>=0)
    {
        this->skip_frames = skip_frames;
        return true;
    }
    else
        return false;
}

/********************************************************/
bool QueryThread::set_person(yarp::os::Bottle &person, int personIndex, bool allowTrain)
{
    std::lock_guard<std::mutex> lg(mtx);
    if (person.size()>0)
    {
        this->allowedTrain = allowTrain;
        this->personIndex = personIndex;
        this->blob_person = person;
        return true;
    }
    else
        return false;
}

/********************************************************/
bool QueryThread::clear_hist()
{
    std::lock_guard<std::mutex> lg(mtx);
    scores_buffer.clear();
    return true;
}


/********************************************************/
void QueryThread::interrupt()
{
    port_in_scores.interrupt();
    port_out_crop.interrupt();
}

/********************************************************/
bool QueryThread::releaseThread()
{
    port_in_scores.close();
    port_out_crop.close();
    return true;
}
