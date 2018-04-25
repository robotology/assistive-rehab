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

#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/LockGuard.h>
#include <yarp/sig/Image.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <sstream>
#include <iostream>
#include <ctime>
#include <cstring>

#include "recognition_IDL.h"

/********************************************************/
class Module : public yarp::os::RFModule, public recognition_IDL
{
    yarp::os::ResourceFinder    *rf;
    yarp::os::RpcServer         rpcPort;
    yarp::os::RpcClient         rpcClassifier;
    yarp::os::Port              imgClassifier;

    yarp::os::BufferedPort<yarp::os::Bottle>        blobsPort;
    yarp::os::BufferedPort<yarp::os::Bottle >       targetInPort;
    yarp::os::BufferedPort<yarp::os::Bottle >       targetOutPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > imageInPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > imageOutPort;

    yarp::os::Mutex     mutex;

    double              lastBlobsArrivalTime;
    double              blobs_detection_timeout;
    yarp::os::Bottle    receivedBlobs;

    yarp::sig::ImageOf<yarp::sig::PixelBgr> img;

    bool                closing;
    bool                interrupting;
    std::string         label;
    bool                allowedTrain;
    double              trainingTime;
    double              confidenceThreshold;

    double              time_spent;
    clock_t             begin, end;

    bool                isLiftArm;
    bool                gotTime;

public:

    /********************************************************/
    bool attach(yarp::os::RpcServer &source) override
    {
        return this->yarp().attachAsServer(source);
    }

    /**********************************************************/
    bool train(const std::string &name) override
    {
        allowedTrain = true;
        gotTime = false;
        label = name;
        return true;
    }

    /**********************************************************/
    bool forget(const std::string &label) override
    {
        yarp::os::Bottle cmd, reply;
        cmd.clear();
        reply.clear();

        if (label=="all")
        {
            cmd.addVocab(yarp::os::Vocab::encode("forget"));
            cmd.addString("all");
            yInfo() << "Sending clearing request:" << cmd.toString();
            rpcClassifier.write(cmd,reply);
            yInfo() << "Received reply:" << reply.toString();
        }
        else
        {
            cmd.addVocab(yarp::os::Vocab::encode("forget"));
            cmd.addString(label.c_str());
            yInfo() << "Sending clearing request" << cmd.toString();
            rpcClassifier.write(cmd,reply);
            yInfo() << "Received reply:" << reply.toString();
        }
        return true;
    }

    /**********************************************************/
    bool setConfidenceThreshold(const double thresh) override
    {
        yarp::os::LockGuard lg(mutex);
        confidenceThreshold = thresh;
        return true;
    }

    /**********************************************************/
    double getConfidenceThreshold() override
    {
        return confidenceThreshold;
    }

    /**********************************************************/
    bool interactionMode(const std::string &mode) override
    {
        bool success = true;

        if (mode.compare("liftArm") == 0)
            isLiftArm = true;
        else if (mode.compare("closeFace") == 0)
            isLiftArm = false;
        else
            success = false;
        
        return success;
    }

    /********************************************************/
    bool configure(yarp::os::ResourceFinder &rf) override
    {
        this->rf=&rf;
        std::string moduleName = rf.check("name", yarp::os::Value("recognition-manager"), "module name (string)").asString();
        setName(moduleName.c_str());

        blobs_detection_timeout  = rf.check("blobs_detection_timeout",yarp::os::Value(0.2)).asDouble();

        trainingTime = rf.check("training_time",yarp::os::Value(10)).asDouble();

        rpcPort.open(("/"+getName("/rpc")).c_str());

        blobsPort.open(("/"+getName("/blobs:i")).c_str());
        rpcClassifier.open(("/"+getName("/classify:rpc")).c_str());
        imgClassifier.open(("/"+getName("/imgClassifier:o")).c_str());
        imageInPort.open(("/"+getName("/image:i")).c_str());
        imageOutPort.open(("/"+getName("/image:o")).c_str());
        targetOutPort.open(("/"+getName("/target:o")).c_str());
        targetInPort.open(("/"+getName("/target:i")).c_str());

        closing = false;
        interrupting = false;

        allowedTrain = false;

        confidenceThreshold = 0.70;
        time_spent = 0.0;
        isLiftArm = true;
        gotTime = false;

        attach(rpcPort);

        return true;
    }

    /********************************************************/
    yarp::os::Bottle getSkeletons()
    {
        yarp::os::LockGuard lg(mutex);
        yarp::os::Bottle receivedSkeletons;
        if (yarp::os::Bottle *skeletons=targetInPort.read())
        {
            receivedSkeletons=*skeletons;
        }
        return receivedSkeletons;
    }

    /********************************************************/
    yarp::os::Bottle getBlobs()
    {
        yarp::os::LockGuard lg(mutex);
        if (yarp::os::Bottle *pBlobs=blobsPort.read(false))
        {
            lastBlobsArrivalTime=yarp::os::Time::now();

            receivedBlobs=*pBlobs;
            if (receivedBlobs.size()==1)
            {
                if (receivedBlobs.get(0).asVocab()==yarp::os::Vocab::encode("empty"))
                    receivedBlobs.clear();
            }
        }
        else if (yarp::os::Time::now()-lastBlobsArrivalTime>blobs_detection_timeout)
            receivedBlobs.clear();

        return receivedBlobs;
    }

    /**********************************************************/
    void getImage()
    {
        yarp::os::LockGuard lg(mutex);
        if (yarp::sig::ImageOf<yarp::sig::PixelBgr> *tmp=imageInPort.read())
        {
            img=*tmp;
        }
    }

    /********************************************************/
    yarp::os::Bottle classify(const yarp::os::Bottle &blobs)
    {
        yarp::os::LockGuard lg(mutex);
        yarp::os::Bottle cmd,reply;

        imgClassifier.write(img);

        cmd.addVocab(yarp::os::Vocab::encode("classify"));
        yarp::os::Bottle &options=cmd.addList();

        for (int i=0; i<blobs.size(); i++)
        {
            std::ostringstream tag;
            tag<<"blob_"<<i;
            yarp::os::Bottle &item=options.addList();
            item.addString(tag.str().c_str());
            item.addList()=*blobs.get(i).asList();
        }
        yInfo() << "Sending classification request:" << cmd.toString();
        rpcClassifier.write(cmd,reply);
        yInfo() << "Received reply:" << reply.toString();

        return reply;
    }

    /**********************************************************/
    int findClosestBlob(const yarp::os::Bottle &blobs)
    {
        double area = 0.0;
        int largest = 0;
        for (int i=0;i<blobs.size(); i++)
        {
            int width = 0;
            int height = 0;
            yarp::os::Bottle *item=blobs.get(i).asList();
            width = item->get(2).asInt() - item->get(0).asInt();
            height = item->get(3).asInt() - item->get(1).asInt();
            double tmparea = width * height;
            if (tmparea > area)
            {
                area = tmparea;
                largest = i;
            }
        }
        return largest;
    }

    /**********************************************************/
    int findArmLift(const yarp::os::Bottle &blobs,
                    const yarp::os::Bottle &skeletons)
    {
        int index = -1;
        
        if (skeletons.size() > 0)
        {
            int skeletonSize = skeletons.get(0).asList()->size();
            int internalElements = 0;
            
            std::vector<cv::Point> relbow;
            std::vector<cv::Point> rwrist;
            std::vector<cv::Point> lelbow;
            std::vector<cv::Point> lwrist;
            std::vector<cv::Point> neck;

            cv::Point point;

            if (skeletonSize>0)
                internalElements = skeletons.get(0).asList()->get(0).asList()->size();


            for (int i = 0; i < skeletonSize; i++)
            {
                if (yarp::os::Bottle *propField = skeletons.get(0).asList()->get(i).asList())
                {
                    for (int ii = 0; ii < internalElements; ii++)
                    {
                        if (yarp::os::Bottle *propFieldPos = propField->get(ii).asList())
                        {
                            if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"RElbow") == 0)
                            {
                                point.x = (int)propFieldPos->get(1).asDouble();
                                point.y = (int)propFieldPos->get(2).asDouble();
                                relbow.push_back(point);
                            }
                            if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"RWrist") == 0)
                            {
                                point.x = (int)propFieldPos->get(1).asDouble();
                                point.y = (int)propFieldPos->get(2).asDouble();
                                rwrist.push_back(point);
                            }
                            if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"LElbow") == 0)
                            {
                                point.x = (int)propFieldPos->get(1).asDouble();
                                point.y = (int)propFieldPos->get(2).asDouble();
                                lelbow.push_back(point);
                            }
                            if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"LWrist") == 0)
                            {
                                point.x = (int)propFieldPos->get(1).asDouble();
                                point.y = (int)propFieldPos->get(2).asDouble();
                                lwrist.push_back(point);
                            }
                            if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"Neck") == 0)
                            {
                                point.x = (int)propFieldPos->get(1).asDouble();
                                point.y = (int)propFieldPos->get(2).asDouble();
                                neck.push_back(point);
                            }
                        }
                    }
                }
            }
            
            int getIndex = -1;
     
            if (neck.size() > 0)
            {
                for (int i = 0; i < skeletonSize; i++)
                {
                    if ( relbow[i].y - rwrist[i].y > 20 && relbow[i].y > 0 && rwrist[i].y > 0 )
                    {
                        yInfo() << "Skeleton" << i << "right-hand raised";
                        getIndex = i;
                    }
                    else if ( lelbow[i].y - lwrist[i].y > 20 && lelbow[i].y > 0 && lwrist[i].y > 0 )
                    {
                        yInfo() << "Skeleton" << i << "left-hand raised";
                        getIndex = i;
                    }
                    else
                        yError() << "Skeleton" << i << "check";
                }

                if (getIndex > -1)
                { 
                    for (int i=0;i<blobs.size(); i++)
                    {
                        yarp::os::Bottle *item=blobs.get(i).asList();
                        
                        int cog = item->get(2).asInt() - ( (item->get(2).asInt() -item->get(0).asInt()) / 2);             
                        
                        if ( abs(cog - neck[getIndex].x) < 20)
                            index = i;
                    }
                }
            }
        }
        
        return index;
    }

    /**********************************************************/
    void sendTrain(const std::string &object, const yarp::os::Bottle &blobs,
                   const int i)
    {
        yarp::os::LockGuard lg(mutex);
        imgClassifier.write(img);
        yarp::os::Bottle cmd,reply;
        cmd.addVocab(yarp::os::Vocab::encode("train"));
        yarp::os::Bottle &options=cmd.addList().addList();
        options.addString(object.c_str());
        options.add(blobs.get(i));
        yInfo() << "Sending training request:" << cmd.toString();
        rpcClassifier.write(cmd,reply);
        yInfo() << "Received reply:" << reply.toString();
    }

    /**********************************************************/
    yarp::os::Bottle sendOutputImage(const yarp::os::Bottle &blobs,
                                     const int i, const yarp::os::Bottle &labels)
    {
        yarp::os::Bottle winners;
        yarp::sig::ImageOf<yarp::sig::PixelBgr> imgOut = img;
        cv::Mat imgMat=cv::cvarrToMat(img.getIplImage());

        cv::Scalar highlight(14,198,8);

        if (allowedTrain)
        {
            highlight[0] = 57;
            highlight[1] = 28;
            highlight[2] = 203;
        }

        cv::Scalar lowlight(203,127, 28);

        winners.clear();

        for (int j=0; j<blobs.size(); j++)
        {
            cv::Point tl,br,txtLoc;
            yarp::os::Bottle *item=blobs.get(j).asList();
            tl.x=(int)item->get(0).asDouble();
            tl.y=(int)item->get(1).asDouble();
            br.x=(int)item->get(2).asDouble();
            br.y=(int)item->get(3).asDouble();
            txtLoc.x=tl.x;
            txtLoc.y=tl.y-5;
            std::ostringstream tag;

            double confidence = 0.0;
            int largest = 0;
 
            int classSize = 0;

            if (labels.size() > 0)       
                classSize = labels.get(j).asList()->size();

            int elementSize = 0;

            if (classSize>0)
                elementSize = labels.get(j).asList()->get(0).asList()->size();

            for (int y=0; y<classSize; y++)
            {
                if (labels.get(j).asList()->get(y).asList()->get(1).asDouble() > confidence)
                {
                    confidence = labels.get(j).asList()->get(y).asList()->get(1).asDouble();
                    largest = y;
                }
            }
            if (classSize>0)
            {
                yarp::os::Bottle &tmp = winners.addList();
                if (labels.get(j).asList()->get(largest).asList()->get(1).asDouble() > confidenceThreshold)
                {
                    tmp.addString(labels.get(j).asList()->get(largest).asList()->get(0).asString());
                    tmp.addDouble(labels.get(j).asList()->get(largest).asList()->get(1).asDouble());
                    tag<<labels.get(j).asList()->get(largest).asList()->get(0).asString();
                }
                else
                {
                    tmp.addString("?");
                    tmp.addDouble(0.0);
                    tag<<"-";
                }

                
                cv::putText(imgMat,tag.str().c_str(),txtLoc,  cv::FONT_HERSHEY_SIMPLEX ,
                            0.5,(j==i)?highlight:lowlight,2);
            }

            cv::rectangle(imgMat,tl,br,(j==i)?highlight:lowlight,2);
        }

        imageOutPort.prepare()=img;
        imageOutPort.write();

        return winners;
    }

    /**********************************************************/
    void sendTargetData(const yarp::os::Bottle &labels,
                        const yarp::os::Bottle &skeletons)
    {

        if (skeletons.size() > 0)
        {

            yarp::os::Bottle &target  = targetOutPort.prepare();

            target = skeletons;

            yarp::os::Bottle toSend;
            toSend.clear();
            yarp::os::Bottle &addStructure = toSend.addList();


            if (target.get(0).asList()->size() == labels.size())
            {
                yarp::os::Bottle &addSkeletons = addStructure.addList();

                for (int i=0; i<target.get(0).asList()->size(); i++)
                {
                    yarp::os::Bottle *skeleton = target.get(0).asList()->get(i).asList();
                    yarp::os::Bottle &options=skeleton->addList();

                    options.addString("Name");

                    options.addString(labels.get(i).asList()->get(0).asString());
                    options.addDouble(labels.get(i).asList()->get(1).asDouble());
                    addSkeletons=*skeleton;
                }
            }
            else
            {
                yarp::os::Bottle &addSkeletons = addStructure.addList();
                for (int i=0; i<target.get(0).asList()->size(); i++)
                {
                    yarp::os::Bottle *skeleton = target.get(0).asList()->get(i).asList();
                    yarp::os::Bottle &options=skeleton->addList();

                    options.addString("Name");
                    options.addString("?");
                    options.addDouble(0.0);
                    addSkeletons=*skeleton;
                }
            }
            //target = toSend;
            targetOutPort.write();
        }
    }

    /**********************************************************/
    bool interruptModule() override
    {
        interrupting=true;
        targetInPort.interrupt();
        blobsPort.interrupt();
        imageInPort.interrupt();
        return true;
    }

    /**********************************************************/
    bool close() override
    {
        blobsPort.close();
        rpcClassifier.close();
        imgClassifier.close();
        imageInPort.close();
        imageOutPort.close();
        targetInPort.close();
        targetOutPort.close();

        return true;
    }

    /**********************************************************/
    bool quit() override
    {
        closing = true;
        return true;
    }

    /********************************************************/
    double getPeriod() override
    {
        // in synch with incoming data
        return 0.0;
    }

    /********************************************************/
    bool updateModule() override
    {
        getImage();
        yarp::os::Bottle blobs = getBlobs();
        yarp::os::Bottle skeletons = getSkeletons();

        if (interrupting)
            return false;

        int person = -1;

        if (isLiftArm)
            person = findArmLift(blobs, skeletons);
        else
            person = findClosestBlob(blobs);

        if (allowedTrain && person > -1)
        {
            if (!gotTime)
            {
                begin = clock();
                gotTime = true;
            }

            sendTrain(label, blobs, person);
            time_spent = (double)( clock() - begin) / CLOCKS_PER_SEC;
            yInfo() << "time spent " << time_spent*100;

            if (time_spent*100 > 10)
            {
                allowedTrain = false;
                time_spent = 0.0;
            }
        }

        yarp::os::Bottle scores;
        yarp::os::Bottle labels;

        scores.clear();
        labels.clear();

        if (blobs.size()!=0)
            scores = classify(blobs);

        if (scores.size()!=0 && scores.get(0).asList()->size() > 0)
        {
            for (int i=0; i<blobs.size(); i++)
            {
                yarp::os::Bottle &labelblobs = labels.addList();
                if (scores.get(i).asList()->size() > 0 && scores.get(i).asList()->get(1).asList()->size() > 0)
                {
                    int elements = scores.get(i).asList()->get(1).asList()->size();
                    for (int x=0; x<elements; x++)
                    {
                        std::string tmp = scores.get(i).asList()->get(1).asList()->get(x).asList()->get(0).asString();
                        double conf = scores.get(i).asList()->get(1).asList()->get(x).asList()->get(1).asDouble();
                        yarp::os::Bottle &item=labelblobs.addList();
                        item.addString(tmp);
                        item.addDouble(conf);
                    }
                }
                else
                    yError() << "no classes avaiblable";
            }
        }
        else
        {
            for (int i=0; i<blobs.size(); i++)
            {   
                yarp::os::Bottle &fillScores = scores.addList();
                yarp::os::Bottle &labelblobs = labels.addList();
            }

            if (blobs.size() == 0)
            {
                yarp::os::Bottle &fillScores = scores.addList();
                yarp::os::Bottle &labelblobs = labels.addList();
            }
        }

        yarp::os::Bottle winners = sendOutputImage(blobs, person, labels);
        sendTargetData(winners, skeletons);

        return !closing;
    }
};

/********************************************************/
int main(int argc, char *argv[])
{
    yarp::os::Network::init();

    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError() << "YARP server not available!";
        return 1;
    }

    Module module;
    yarp::os::ResourceFinder rf;

    rf.setVerbose();
    rf.configure(argc,argv);

    return module.runModule(rf);
}
