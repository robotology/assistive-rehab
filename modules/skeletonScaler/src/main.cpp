/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file main.cpp
 * @authors: Valentina Vasco <valentina.vasco@iit.it>
 */

#include "string"
#include "cmath"

#include "yarp/sig/Vector.h"
#include "yarp/os/all.h"

#include "AssistiveRehab/skeleton.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace assistive_rehab;

class Scaler : public RFModule
{
//    BufferedPort<Bottle> playerPort;
    BufferedPort<Bottle> retrieverPort;
    RpcClient cmdPort;

    string file;
    string context;
    double opacity;
    int nsessions;
    double twarp;

    /****************************************************************/
    bool configure(ResourceFinder &rf)
    {
//        playerPort.open("/skeletonScaler/player:i");
        retrieverPort.open("/skeletonScaler/retriever:i");
        cmdPort.open("/skeletonScaler/cmd");

        if(!Network::connect(cmdPort.getName(),"/skeletonPlayer/cmd:rpc", "tcp"))
        {
            yError() << "Cannot connect to /skeletonPlayer/cmd:rpc";
            return false;
        }

        file=rf.check("file",Value("abduction.log")).asString();
        context=rf.check("context",Value("motionAnalyzer")).asString();

        nsessions=rf.check("nsessions",Value(0)).asInt();
        twarp=rf.check("twarp",Value(0.5)).asDouble();

        if(!loadData(file,context))
        {
            yError() << "Unable to load data";
            return false;
        }

        if(!start(nsessions,twarp))
        {
            yError() << "Unable to start";
            return false;
        }

        int idx=file.find(".");
        setTag(file.substr(0,idx));

        opacity=0.1;
        setOpacity(opacity);

        return true;
    }

    /****************************************************************/
    double getPeriod()
    {
        return 10.0; //0.01;
    }

    /****************************************************************/
    bool updateModule()
    {
//        //get skeleton from skeletonPlayer
//        Bottle *inputPlayer = playerPort.read();
//        SkeletonWaist playedSkel;
//        for (int i=0; i<inputPlayer->size(); i++)
//        {
//            if (Bottle *b=inputPlayer->get(i).asList())
//            {
//                if (b->check("tag"))
//                {
//                    Property prop(b->toString().c_str());
//                    string tag=prop.find("tag").asString();
//                    if (!tag.empty())
//                    {
//                        if (prop.check("tag"))
//                        {
//                            Skeleton* sk1 = skeleton_factory(prop);
//                            playedSkel.update_fromstd(sk1->toProperty());
////                            playedSkel.print();
//                            delete sk1;
//                        }
//                    }
//                }
//            }
//        }

        //get skeleton from skeletonRetriever
        SkeletonWaist retrievedSkel;
        Bottle *inputRetriever = retrieverPort.read();
        for (int i=0; i<inputRetriever->size(); i++)
        {
            if (Bottle *b=inputRetriever->get(i).asList())
            {
                if (b->check("tag"))
                {
                    Property prop(b->toString().c_str());
                    string tag=prop.find("tag").asString();
                    if (!tag.empty())
                    {
                        if (prop.check("tag"))
                        {
                            Skeleton* sk2 = skeleton_factory(prop);
                            retrievedSkel.update_fromstd(sk2->toProperty());
//                            retrievedSkel.print();

                            delete sk2;
                        }
                    }
                }
            }
        }

        Vector xyz=retrievedSkel[KeyPointTag::shoulder_center]->getPoint();
        Vector rot(3,0.0);
        double theta=0.0;
        if(file.compare("flexion")>0)
        {
            rot[1]=1.0;
            theta=M_PI/2;
        }
        if(!moveSkeleton(xyz,rot,theta))
            yWarning() << "Unable to move";

        double maxpath;
        getMaxPath(maxpath);

        double scale = retrievedSkel.getMaxPath()/maxpath;
//        yInfo() << retrievedSkel.getMaxPath() << maxpath << retrievedSkel.getMaxPath()/maxpath;
        if(!setScale(scale))
        {
            yError() << "Unable to scale";
            return false;
        }

        return true;
    }

    /****************************************************************/
    bool moveSkeleton(const Vector& xyz,const Vector& rot,const double& theta)
    {
        Bottle cmd,rep;
        cmd.addString("move");
        cmd.addDouble(xyz[0]);
        cmd.addDouble(xyz[1]);
        cmd.addDouble(xyz[2]);
        cmd.addDouble(rot[0]);
        cmd.addDouble(rot[1]);
        cmd.addDouble(rot[2]);
        cmd.addDouble(theta);
        yInfo() << cmd.toString();
        if(cmdPort.write(cmd,rep))
        {
            if(rep.get(0).asVocab()==Vocab::encode("ok"))
                return true;
        }

        return false;
    }

    /****************************************************************/
    void getMaxPath(double& maxpath)
    {
        Bottle cmd,rep;
        cmd.addString("get_maxpath");
//        yInfo() << cmd.toString();
        if(cmdPort.write(cmd,rep))
            maxpath=rep.get(0).asDouble();
    }

    /****************************************************************/
    bool setScale(const double& scale)
    {
        Bottle cmd,rep;
        cmd.addString("scale");
        cmd.addDouble(scale);
        yInfo() << cmd.toString();
        if(cmdPort.write(cmd,rep))
        {
            if(rep.get(0).asVocab()==Vocab::encode("ok"))
                return true;
        }

        return false;
    }

    /****************************************************************/
    bool setOpacity(const double& opacity_)
    {
        Bottle cmd,rep;
        cmd.addString("set_opacity");
        cmd.addDouble(opacity_);
        yInfo() << cmd.toString();
        if(cmdPort.write(cmd,rep))
        {
            if(rep.get(0).asVocab()==Vocab::encode("ok"))
                return true;
        }

        return false;
    }

    /****************************************************************/
    bool loadData(const string& file_, const string& context_)
    {
        Bottle cmd,rep;
        cmd.addString("load");
        cmd.addString(file_);
        cmd.addString(context_);
        yInfo() << cmd.toString();
        if(cmdPort.write(cmd,rep))
        {
            if(rep.get(0).asVocab()==Vocab::encode("ok"))
                return true;
        }

        return false;
    }

    /****************************************************************/
    bool start(const int& nsessions_, const double& twarp_)
    {
        Bottle cmd,rep;
        cmd.addString("start");
        cmd.addInt(nsessions_);
        cmd.addDouble(twarp_);
        yInfo() << cmd.toString();
        if(cmdPort.write(cmd,rep))
        {
            if(rep.get(0).asVocab()==Vocab::encode("ok"))
                return true;
        }

        return false;
    }

    /****************************************************************/
    void stop()
    {
        Bottle cmd,rep;
        cmd.addString("stop");
        yInfo() << cmd.toString();
        cmdPort.write(cmd,rep);
    }

    /****************************************************************/
    void setTag(const string& tag)
    {
        Bottle cmd,rep;
        cmd.addString("set_tag");
        cmd.addString(tag);
        yInfo() << cmd.toString();
        cmdPort.write(cmd,rep);
    }

    /****************************************************************/
    bool interruptModule()
    {
       stop();
//       playerPort.interrupt();
       retrieverPort.interrupt();
       cmdPort.interrupt();

       return true;
    }

    /****************************************************************/
    bool close()
    {
//        playerPort.close();
        retrieverPort.close();
        cmdPort.close();

        return true;
    }

};

/****************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"Unable to find Yarp server!";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    Scaler scaler;
    return scaler.runModule(rf);
}

