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

#include <yarp/os/Vocab.h>
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
#include <yarp/cv/Cv.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <sstream>
#include <iostream>
#include <cstring>
#include <utility>

#include <queryThread.h>
#include "recognition_IDL.h"

#define                 ACK                 yarp::os::createVocab('a','c','k')
#define                 NACK                yarp::os::createVocab('n','a','c','k')

/********************************************************/
class Module : public yarp::os::RFModule, public recognition_IDL
{
    yarp::os::ResourceFinder    *rf;
    yarp::os::RpcServer         rpcPort;
    yarp::os::Port              imgClassifier;

    yarp::os::BufferedPort<yarp::os::Bottle>        blobsPort;
    yarp::os::BufferedPort<yarp::os::Bottle >       targetInPort;
    yarp::os::BufferedPort<yarp::os::Bottle >       targetOutPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imageInPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imageOutPort;

    yarp::os::RpcClient          port_rpc_classifier;

    yarp::os::Mutex     mutex;

    double              lastBlobsArrivalTime;
    double              blobs_detection_timeout;
    yarp::os::Bottle    receivedBlobs;

    yarp::sig::ImageOf<yarp::sig::PixelRgb> img;

    bool                closing;
    bool                interrupting;
    std::string         label;
    bool                allowedTrain;
    double              trainingTime;
    double              confidenceThreshold;

    double              t0;

    bool                isLiftArm;
    bool                gotTime;
    bool                sentTrain;
    int                 skip_frames;
    int                 frame_counter;

    bool                recognition_started;

    QueryThread         *thr_query;

public:

    /********************************************************/
    bool attach(yarp::os::RpcServer &source) override
    {
        return this->yarp().attachAsServer(source);
    }

    /**********************************************************/
    bool train(const std::string &name) override
    {
        yarp::os::LockGuard lg(mutex);
        allowedTrain = true;
        gotTime = false;
        label = name;
        sentTrain = false;
        recognition_started = false;
        return true;
    }

    /**********************************************************/
    bool forget(const std::string &label) override
    {
        yarp::os::LockGuard lg(mutex);
        yarp::os::Bottle cmd, reply;
        cmd.clear();
        reply.clear();

        bool done = false, class_known = false, empty_classes = false;
        for (int i=0; !done && i<10; i++)
        {
            yarp::os::Bottle cmd_classifier,reply_classifier;
            cmd_classifier.addString("objList");
            port_rpc_classifier.write(cmd_classifier,reply_classifier);

            yDebug() << reply_classifier.toString().c_str();

            if (reply_classifier.size()>0 && reply_classifier.get(0).asVocab()==ACK)
            {
                done = true;
                yarp::os::Bottle *class_list = reply_classifier.get(1).asList();
                if (class_list->size()==0)
                {
                    empty_classes = true;
                    yInfo()<<"There are no known classes.";
                }
                else
                {
                    for (int idx_class=0; idx_class<class_list->size(); idx_class++)
                    {
                        if (label==class_list->get(idx_class).asString().c_str())
                        {
                            class_known = true;
                            break;
                        }
                    }
                }
            }
        }

        if (empty_classes==false)
        {
            if (class_known || label=="all")
            {
                if (!send_doublecmd2rpc_classifier("forget", label.c_str(), 10))
                {
                    yInfo() <<"Classifier busy for forgetting one object!";
                }

                recognition_started = false;

            }
            else
            {
                yInfo() << "Class is unknown.";
            }
        }
        recognition_started = false;

        yDebug() << "done forgetting" <<label;
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
        std::string moduleName = rf.check("name", yarp::os::Value("faceRecognizer"), "module name (string)").asString();
        setName(moduleName.c_str());

        blobs_detection_timeout  = rf.check("blobs_detection_timeout",yarp::os::Value(0.2)).asDouble();

        trainingTime = rf.check("training_time",yarp::os::Value(10.0)).asDouble();

        skip_frames = rf.check("skip_frames",yarp::os::Value(5)).asInt();

        rpcPort.open(("/"+getName("/rpc")).c_str());

        blobsPort.open(("/"+getName("/blobs:i")).c_str());
        imgClassifier.open(("/"+getName("/imgClassifier:o")).c_str());
        imageInPort.open(("/"+getName("/image:i")).c_str());
        imageOutPort.open(("/"+getName("/image:o")).c_str());
        targetOutPort.open(("/"+getName("/target:o")).c_str());
        targetInPort.open(("/"+getName("/target:i")).c_str());

        port_rpc_classifier.open(("/"+getName("/classifier:io")).c_str());

        frame_counter = 0;

        closing = false;
        interrupting = false;

        allowedTrain = false;

        confidenceThreshold = 0.70;
        isLiftArm = true;
        gotTime = false;
        sentTrain = false;
        recognition_started = false;

        thr_query = new QueryThread(rf);
        thr_query->start();

        bool ok = thr_query->set_skip_frames(skip_frames);

        if (ok)
            yDebug() << "Correctly set number of Frames";
        else
            yError() << "Erros setting number of Frames";

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
        if (yarp::sig::ImageOf<yarp::sig::PixelRgb> *tmp=imageInPort.read())
        {
            img=*tmp;
        }
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
            size_t skeletonSize = skeletons.get(0).asList()->size();
            size_t internalElements = 0;

            std::vector<cv::Point> relbow;
            std::vector<cv::Point> rwrist;
            std::vector<cv::Point> lelbow;
            std::vector<cv::Point> lwrist;
            std::vector<cv::Point> neck;

            cv::Point point;

            if (skeletonSize>0)
                internalElements = skeletons.get(0).asList()->get(0).asList()->size();

            for (size_t i = 0; i < skeletonSize; i++)
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
                        //yInfo() << "Skeleton" << i << "right-hand raised";
                        getIndex = i;
                    }
                    else if( lelbow[i].y - lwrist[i].y > 20 && lelbow[i].y > 0 && lwrist[i].y > 0 )
                    {
                        //yInfo() << "Skeleton" << i << "left-hand raised";
                        getIndex = i;
                    }
                    //else
                        //yInfo() << "Skeleton" << i << "check";
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
    yarp::os::Bottle sendOutputImage(const yarp::os::Bottle &blobs,
                                     const int i, const yarp::os::Bottle &labels)
    {
        yarp::os::Bottle winners;
        yarp::sig::ImageOf<yarp::sig::PixelRgb> imgOut = img;
        cv::Mat imgMat=yarp::cv::toCvMat(img);

        cv::Scalar highlight(14,198,8);

        if (allowedTrain)
        {
            highlight[0] = 204;
            highlight[1] = 0;
            highlight[2] = 0;
        }

        cv::Scalar lowlight(0, 102, 204);

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

            size_t classSize = 0;

            if (labels.size() > 0)
                classSize = labels.get(j).asList()->size();

            size_t elementSize = 0;

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
                cv::putText(imgMat,tag.str(),txtLoc,cv::FONT_HERSHEY_SIMPLEX,0.5,(j==i)?highlight:lowlight,2);
            }
            cv::rectangle(imgMat,tl,br,(j==i)?highlight:lowlight,2);
        }

        imageOutPort.prepare()=img;
        imageOutPort.writeStrict();

        return winners;
    }

    /**********************************************************/
    void sendTargetData(const yarp::os::Bottle &blobs,
                        const yarp::os::Bottle &labels,
                        const yarp::os::Bottle &skeletons)
    {
        //yInfo() << "*************************************************************";
        if (skeletons.size() > 0)
        {
            yarp::os::Bottle &target  = targetOutPort.prepare();

            target = skeletons;
            //yInfo() << "******************************** SIZE TARGET " << target.get(0).asList()->size();

            yarp::os::Bottle toSend;
            toSend.clear();
            yarp::os::Bottle &addStructure = toSend.addList();

            size_t skeletonSize = target.get(0).asList()->size();
            size_t internalElements = 0;

            std::vector<cv::Point> neck;
            std::vector<cv::Point> lear;
            std::vector<cv::Point> rear;

            cv::Point point;

            if (skeletonSize>0)
                internalElements = target.get(0).asList()->get(0).asList()->size();

            for (int i = 0; i < skeletonSize; i++)
            {
                if (yarp::os::Bottle *propField = skeletons.get(0).asList()->get(i).asList())
                {
                    //yInfo() << propField->toString();
                    for (int ii = 0; ii < internalElements; ii++)
                    {
                        if (yarp::os::Bottle *propFieldPos = propField->get(ii).asList())
                        {
                            if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"Neck") == 0)
                            {
                                point.x = (int)propFieldPos->get(1).asDouble();
                                point.y = (int)propFieldPos->get(2).asDouble();
                                neck.push_back(point);
                            }
                            if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"LEar") == 0)
                            {
                                point.x = (int)propFieldPos->get(1).asDouble();
                                point.y = (int)propFieldPos->get(2).asDouble();
                                lear.push_back(point);
                            }
                            if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"REar") == 0)
                            {
                                point.x = (int)propFieldPos->get(1).asDouble();
                                point.y = (int)propFieldPos->get(2).asDouble();
                                rear.push_back(point);
                            }
                        }
                    }
                }
            }

            std::vector<std::pair <int,int> > elements;

            //consider only if we get blobs and valid necks
            if (blobs.size() > 1 )
            {
                for (int i=0; i<neck.size(); i++)
                {
                    if (neck[i].x >0)
                    {
                        for (int j=0; j<blobs.size(); j++)
                        {
                            yarp::os::Bottle *item=blobs.get(j).asList();
                            int cog = item->get(2).asInt() - ( (item->get(2).asInt() -item->get(0).asInt()) / 2);
                            if ( abs(cog - neck[i].x) < 50 || abs(cog - lear[i].x) < 50 || abs(cog - rear[i].x) < 50)
                            {
                                //yInfo() << "adding " << i << j;
                                elements.push_back(std::make_pair(i, j));
                            }
                        }
                    }
                    else
                    {
                       elements.push_back(std::make_pair(i, -1));
                    }
                }
            }
            else
            {
                if (skeletonSize > 0 && blobs.size() > 0)
                    elements.push_back(std::make_pair(0, 0));
            }

            yarp::os::Bottle &addSkeletons = addStructure.addList();

            for (int i=0; i<target.get(0).asList()->size(); i++)
            {
                //yInfo() << "IN addSkeletons";
                yarp::os::Bottle *skeleton = target.get(0).asList()->get(i).asList();
                yarp::os::Bottle &options=skeleton->addList();

                options.addString("Name");

                if (blobs.size() > 0 )
                {
                    if (elements[i].second != -1 && labels.size() > 0 && i<=elements.size()-1 )
                    {
                        //yInfo() << "Adding name for skeleton " << i << "with blob " << elements[i].second;
                        options.addString(labels.get(elements[i].second).asList()->get(0).asString());
                        options.addDouble(labels.get(elements[i].second).asList()->get(1).asDouble());
                    }
                    else
                    {
                        //yInfo() << "Adding empty name";
                        options.addString("?");
                        options.addDouble(0.0);
                    }
                }
                else
                {
                    //yInfo() << "Adding empty name AS NO BLOB FOUND";
                    options.addString("?");
                    options.addDouble(0.0);
                }
                addSkeletons=*skeleton;
            }

            targetOutPort.writeStrict();
        }
    }

    /**********************************************************/
    bool interruptModule() override
    {
        interrupting=true;
        thr_query->interrupt();
        targetInPort.interrupt();
        blobsPort.interrupt();
        imageInPort.interrupt();
        port_rpc_classifier.interrupt();
        return true;
    }

    /**********************************************************/
    bool close() override
    {
        thr_query->stop();
        delete thr_query;

        blobsPort.close();
        imgClassifier.close();
        imageInPort.close();
        imageOutPort.close();
        targetInPort.close();
        targetOutPort.close();
        port_rpc_classifier.close();

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
    bool send_cmd2rpc_classifier(std::string cmdstring, int Ntrials)
    {
        bool done = false;
        for (int i=0; !done && i<Ntrials; i++)
        {
            yarp::os::Bottle cmd_classifier,reply_classifier;
            cmd_classifier.addString(cmdstring.c_str());
            port_rpc_classifier.write(cmd_classifier,reply_classifier);

            yDebug() << "reply for send_cmd2rpc_classifier " << reply_classifier.toString().c_str();
            if (reply_classifier.size()>0 && (reply_classifier.get(0).asVocab()==ACK || reply_classifier.get(0).asString() =="ok"))
                done = true;
        }

        return done;
    }

    /********************************************************/
    bool send_doublecmd2rpc_classifier(std::string cmdstring1, std::string cmdstring2, int Ntrials)
    {
        bool done = false;
        for (int i=0; !done && i<Ntrials; i++)
        {
            yarp::os::Bottle cmd_classifier,reply_classifier;
            cmd_classifier.addString(cmdstring1.c_str());
            cmd_classifier.addString(cmdstring2.c_str());
            port_rpc_classifier.write(cmd_classifier,reply_classifier);

            yDebug() << "reply for send_doublecmd2rpc_classifier " << reply_classifier.toString().c_str();
            if (reply_classifier.size()>0 && (reply_classifier.get(0).asVocab()==ACK || reply_classifier.get(0).asString() =="ok"))
                done = true;
        }

        return done;
    }

    /********************************************************/
    bool start_train(std::string class_name)
    {
        if (!send_doublecmd2rpc_classifier("save", class_name.c_str(), 10))
        {
            yDebug() << "Classifier busy for saving!";
            return false;
        }

        return true;
    }

    /********************************************************/
    bool stop_train(std::string class_name)
    {
        if (!send_cmd2rpc_classifier("stop", 10))
        {
            yDebug() << "Classifier busy for stopping to save scores: please make it stops somehow!";
            return false;
        }
        return true;
    }

    /********************************************************/
    bool updateModule() override
    {
        if (port_rpc_classifier.getOutputCount()==0)
        {
            yError() << "Please connect a classifier to start!";
            yarp::os::Time::delay(1.0);
        }
        else
        {
            getImage();
            yarp::os::Bottle blobs = getBlobs();
            yarp::os::Bottle skeletons = getSkeletons();
            yarp::os::Bottle labels;

            if (interrupting)
                return false;

            int person = -1;

            if (isLiftArm)
                person = findArmLift(blobs, skeletons);
            else
                person = findClosestBlob(blobs);

            thr_query->set_person(blobs, person, allowedTrain);

            if (allowedTrain && person > -1)
            {
                if (!gotTime)
                {
                    t0 = yarp::os::Time::now();
                    gotTime = true;
                }

                if (!sentTrain)
                {
                    yDebug() << "STARRTING SAVING label" << label.c_str();
                    start_train(label);
                    sentTrain = true;
                }

                if (gotTime)
                {
                    if (yarp::os::Time::now() - t0 > trainingTime)
                    {
                        yDebug() << "STOPPING SAVING label" << label.c_str();
                        allowedTrain = false;
                        stop_train(label);

                        yDebug() << "TRAINING FEATURES with Label";
                        if (!send_cmd2rpc_classifier("train", 10))
                        {
                            yDebug() << "Classifier busy for training!";
                        }
                        gotTime = false;
                        recognition_started = false;
                    }
                }
            }

            int classes = 0;
            yarp::os::Bottle cmdObjClass,objClassList;
            cmdObjClass.addString("objList");
            port_rpc_classifier.write(cmdObjClass,objClassList);
            if (objClassList.get(0).asString()=="ack")
                if (yarp::os::Bottle *objList=objClassList.get(1).asList())
                    classes = (int)objList->size();

            //yInfo()<< "number of classes " << classes;

            if (!allowedTrain && recognition_started==false && classes>0)
            {
                yDebug() << "STARTING RECOGNITION " << label.c_str();
                yarp::os::Bottle cmd_classifier,reply_classifier;
                cmd_classifier.addString("recognize");
                port_rpc_classifier.write(cmd_classifier,reply_classifier);

                if (reply_classifier.get(0).asString()=="nack")
                    yError() << "Do not have any classes";

                yDebug() << "STARTING RECOGNITION reply_classifier " << reply_classifier.toString().c_str();

                if (reply_classifier.size()>0)
                {
                    recognition_started = true;

                    if (reply_classifier.get(0).asVocab()!=ACK)
                    {
                        thr_query->clear_hist();
                    }

                } else
                {
                    std::cout << "Cannot get response to recognize command." << std::endl;
                    thr_query->clear_hist();
                }
            }
            yarp::os::LockGuard lg(mutex);
            if (recognition_started)
            {
                labels.clear();
                labels = thr_query->classify(blobs);
            }

            bool isConsistent = true;

            //check for consistency
            if (classes>0)
            {
                for (int i =0; i<labels.size(); i++)
                {
                    //yInfo() << "check for consistency " << labels.get(i).asList()->size();
                    if (labels.get(i).asList()->size() == 0)
                        isConsistent=false;
                }
            }
            if (isConsistent)
            {
                if (allowedTrain)
                    sendOutputImage(blobs, person, labels);
                else
                {
                    yarp::os::Bottle winners = sendOutputImage(blobs, person, labels);
                    sendTargetData(blobs, winners, skeletons);
                }
            }
        }
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
    rf.setDefaultContext("faceRecognizer");
    rf.setDefaultConfigFile("config.ini");
    rf.setDefault("name","faceRecognizer");
    rf.configure(argc,argv);

    return module.runModule(rf);
}
