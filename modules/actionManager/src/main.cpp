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

#include <stdio.h>
#include <fstream>
#include <yarp/os/all.h>
#include <yarp/sig/Matrix.h>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <tensorflow/core/public/session.h>
#include <tensorflow/core/protobuf/meta_graph.pb.h>
#include <tensorflow/cc/client/client_session.h>
#include <tensorflow/cc/ops/standard_ops.h>
#include <tensorflow/cc/ops/array_ops.h>
#include "AssistiveRehab/skeleton.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace tensorflow;
using namespace assistive_rehab;

/****************************************************************/
class Recognizer : public RFModule
{
    RpcClient opcPort;

    Status status;
    Tensor input;
    Session* session;
    SessionOptions sess_opts;
    SkeletonWaist skeletonIn;
    int step,nsteps;
    yarp::sig::Matrix input_matrix;
    vector<float> min,max;
    unordered_map<int,string> class_map;

    /****************************************************************/
    bool updateInput(const SkeletonWaist &skel_, const int i)
    {       
        input.tensor<float,3>()(0,i,0) = (skel_[KeyPointTag::head]->getPoint()[0]-min[0])/(max[0]-min[0]);
        input.tensor<float,3>()(0,i,1) = (skel_[KeyPointTag::head]->getPoint()[1]-min[1])/(max[1]-min[1]);
        input.tensor<float,3>()(0,i,2) = (skel_[KeyPointTag::shoulder_center]->getPoint()[0]-min[2])/(max[2]-min[2]);
        input.tensor<float,3>()(0,i,3) = (skel_[KeyPointTag::shoulder_center]->getPoint()[1]-min[3])/(max[3]-min[3]);
        input.tensor<float,3>()(0,i,4) = (skel_[KeyPointTag::shoulder_right]->getPoint()[0]-min[4])/(max[4]-min[4]);
        input.tensor<float,3>()(0,i,5) = (skel_[KeyPointTag::shoulder_right]->getPoint()[1]-min[5])/(max[5]-min[5]);
        input.tensor<float,3>()(0,i,6) = (skel_[KeyPointTag::elbow_right]->getPoint()[0]-min[6])/(max[6]-min[6]);
        input.tensor<float,3>()(0,i,7) = (skel_[KeyPointTag::elbow_right]->getPoint()[1]-min[7])/(max[7]-min[7]);
        input.tensor<float,3>()(0,i,8) = (skel_[KeyPointTag::hand_right]->getPoint()[0]-min[8])/(max[8]-min[8]);
        input.tensor<float,3>()(0,i,9) = (skel_[KeyPointTag::hand_right]->getPoint()[1]-min[9])/(max[9]-min[9]);
        input.tensor<float,3>()(0,i,10) = (skel_[KeyPointTag::shoulder_left]->getPoint()[0]-min[10])/(max[10]-min[10]);
        input.tensor<float,3>()(0,i,11) = (skel_[KeyPointTag::shoulder_left]->getPoint()[1]-min[11])/(max[11]-min[11]);
        input.tensor<float,3>()(0,i,12) = (skel_[KeyPointTag::elbow_left]->getPoint()[0]-min[12])/(max[12]-min[12]);
        input.tensor<float,3>()(0,i,13) = (skel_[KeyPointTag::elbow_left]->getPoint()[1]-min[13])/(max[13]-min[13]);
        input.tensor<float,3>()(0,i,14) = (skel_[KeyPointTag::hand_left]->getPoint()[0]-min[14])/(max[14]-min[14]);
        input.tensor<float,3>()(0,i,15) = (skel_[KeyPointTag::hand_left]->getPoint()[1]-min[15])/(max[15]-min[15]);
        input.tensor<float,3>()(0,i,16) = (skel_[KeyPointTag::hip_right]->getPoint()[0]-min[16])/(max[16]-min[16]);
        input.tensor<float,3>()(0,i,17) = (skel_[KeyPointTag::hip_right]->getPoint()[1]-min[17])/(max[17]-min[17]);
        input.tensor<float,3>()(0,i,18) = (skel_[KeyPointTag::knee_right]->getPoint()[0]-min[18])/(max[18]-min[18]);
        input.tensor<float,3>()(0,i,19) = (skel_[KeyPointTag::knee_right]->getPoint()[1]-min[19])/(max[19]-min[19]);
        input.tensor<float,3>()(0,i,20) = (skel_[KeyPointTag::ankle_right]->getPoint()[0]-min[20])/(max[20]-min[20]);
        input.tensor<float,3>()(0,i,21) = (skel_[KeyPointTag::ankle_right]->getPoint()[1]-min[21])/(max[21]-min[21]);
        input.tensor<float,3>()(0,i,22) = (skel_[KeyPointTag::hip_left]->getPoint()[0]-min[22])/(max[22]-min[22]);
        input.tensor<float,3>()(0,i,23) = (skel_[KeyPointTag::hip_left]->getPoint()[1]-min[23])/(max[23]-min[23]);
        input.tensor<float,3>()(0,i,24) = (skel_[KeyPointTag::knee_left]->getPoint()[0]-min[24])/(max[24]-min[24]);
        input.tensor<float,3>()(0,i,25) = (skel_[KeyPointTag::knee_left]->getPoint()[1]-min[25])/(max[25]-min[25]);
        input.tensor<float,3>()(0,i,26) = (skel_[KeyPointTag::ankle_left]->getPoint()[0]-min[26])/(max[26]-min[26]);
        input.tensor<float,3>()(0,i,27) = (skel_[KeyPointTag::ankle_left]->getPoint()[1]-min[27])/(max[27]-min[27]);
        input.tensor<float,3>()(0,i,28) = (skel_[KeyPointTag::head]->getPoint()[0]-min[28])/(max[28]-min[28]);
        input.tensor<float,3>()(0,i,29) = (skel_[KeyPointTag::head]->getPoint()[1]-min[29])/(max[29]-min[29]);
        input.tensor<float,3>()(0,i,30) = (skel_[KeyPointTag::head]->getPoint()[0]-min[30])/(max[30]-min[30]);
        input.tensor<float,3>()(0,i,31) = (skel_[KeyPointTag::head]->getPoint()[1]-min[31])/(max[31]-min[31]);
        input.tensor<float,3>()(0,i,32) = (skel_[KeyPointTag::head]->getPoint()[0]-min[32])/(max[32]-min[32]);
        input.tensor<float,3>()(0,i,33) = (skel_[KeyPointTag::head]->getPoint()[1]-min[33])/(max[33]-min[33]);
        input.tensor<float,3>()(0,i,34) = (skel_[KeyPointTag::head]->getPoint()[1]-min[34])/(max[34]-min[34]);
        input.tensor<float,3>()(0,i,35) = (skel_[KeyPointTag::head]->getPoint()[1]-min[35])/(max[35]-min[35]);

        return true;
    }

    /********************************************************/
    void getSkeleton()
    {
        //ask for the property id
        Bottle cmd,reply;
        cmd.addVocab(Vocab::encode("ask"));
        Bottle &content = cmd.addList().addList();
        content.addString("skeleton");

        opcPort.write(cmd,reply);
        if(reply.size() > 1)
        {
            if(reply.get(0).asVocab() == Vocab::encode("ack"))
            {
                if(Bottle *idField = reply.get(1).asList())
                {
                    if(Bottle *idValues = idField->get(1).asList())
                    {
//                        if(!skel_tag.empty())
//                        {
                            for(int i=0; i<idValues->size(); i++)
                            {
                                int id = idValues->get(i).asInt();

                                //given the id, get the value of the property
                                cmd.clear();
                                cmd.addVocab(Vocab::encode("get"));
                                Bottle &content = cmd.addList().addList();
                                Bottle replyProp;
                                content.addString("id");
                                content.addInt(id);

                                opcPort.write(cmd,replyProp);
                                if(replyProp.get(0).asVocab() == Vocab::encode("ack"))
                                {
                                    if(Bottle *propField = replyProp.get(1).asList())
                                    {
                                        Property prop(propField->toString().c_str());
                                        string tag=prop.find("tag").asString();
                                        if(!tag.empty())
                                        {
//                                            if(prop.check("tag") && tag==skel_tag)
//                                            {
                                                Skeleton* skeleton = skeleton_factory(prop);
                                                skeletonIn.update(skeleton->toProperty());
                                                delete skeleton;
//                                            }
                                        }
                                    }
                                }
                            }
//                        }
                    }
                }
            }
        }
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf) override
    {
        string moduleName = rf.check("name", Value("actionManager")).asString();
        setName(moduleName.c_str());

        nsteps = rf.check("nsteps",Value(50)).asInt(); //must be the one defined when training the model

        opcPort.open(("/" + getName() + "/opc").c_str());

        Bottle &bGroup=rf.findGroup("general");
        if (bGroup.isNull())
        {
            yError()<<"Unable to find group \"general\"";
            return false;
        }
        if (!bGroup.check("num-sections"))
        {
            yError()<<"Unable to find key \"num-sections\"";
            return false;
        }
        int num_sections=bGroup.find("num-sections").asInt();
        for (int i=0; i<num_sections; i++)
        {
            ostringstream section;
            section<<"section-"<<i;
            Bottle &bSection=rf.findGroup(section.str());
            if (bSection.isNull())
            {
                string msg="Unable to find section";
                msg+="\""+section.str()+"\"";
                yError()<<msg;
                return false;
            }
            if (!bSection.check("label") || !bSection.check("value"))
            {
                yError()<<"Unable to find key \"key\" and/or \"value\"";
                return false;
            }
            int value=bSection.find("value").asInt();
            string label=bSection.find("label").asString();
            class_map[value]=label;
        }

        ifstream file( "/home/vvasco/dev/action-recognition/lstm/fdg-data/2d/train_test-1/scale_values.txt" );
        string line = "";
        while (getline(file, line))
        {
            vector<string> vec;
            boost::algorithm::split(vec, line, boost::is_any_of(","));
            min.push_back(stof((vec[0]).c_str(),0));
            max.push_back(stof((vec[1]).c_str(),0));
        }

        // Set up input paths
        const string pathToGraph = "/home/vvasco/dev/action-recognition/lstm/fdg-model/2d/model-chevalier/train_test-1/model-4/model.meta";
        const string checkpointPath = "/home/vvasco/dev/action-recognition/lstm/fdg-model/2d/model-chevalier/train_test-1/model-4/model";

        sess_opts.config.mutable_gpu_options()->set_allow_growth(true); //to limit GPU usage
        session = NewSession(sess_opts);
        if (session == nullptr)
        {
            throw runtime_error("Could not create Tensorflow session");
        }

        // Read in the protobuf graph we exported
        MetaGraphDef graph_def;
        status = ReadBinaryProto(Env::Default(), pathToGraph, &graph_def);
        if (!status.ok())
        {
            throw runtime_error("Error reading graph definition from " + pathToGraph + ": " + status.ToString());
        }

        // Add the graph to the session
        status = session->Create(graph_def.graph_def());
        if (!status.ok())
        {
            throw runtime_error("Error creating graph: " + status.ToString());
        }

        // Read weights from the saved checkpoint
        Tensor checkpointPathTensor(DT_STRING, TensorShape());
        checkpointPathTensor.scalar<std::string>()() = checkpointPath;
        status = session->Run(
                {{ graph_def.saver_def().filename_tensor_name(), checkpointPathTensor },},
                {},
                {graph_def.saver_def().restore_op_name()},
                nullptr);
        if (!status.ok())
        {
            throw runtime_error("Error loading checkpoint from " + checkpointPath + ": " + status.ToString());
        }

        input = Tensor(DT_FLOAT, TensorShape({1,nsteps,36}));
        step = 0;

        return true;
    }

    /****************************************************************/
    double getPeriod() override
    {
        return 0.1;
    }

    /****************************************************************/
    bool updateModule() override
    {
        if(opcPort.getOutputCount() > 0)
        {
            getSkeleton();
            updateInput(skeletonIn,step);
            if(step < nsteps)
            {
                yInfo() << "Filling batch.." << step;
                step++;
            }
            else
            {
                //run the inference
//                cout << (input).tensor<float, (3)>() << endl;
                cout << "input" << input.DebugString() << endl;
                string input_layer = "x:0";
                string output_layer = "ArgMax:0";
                vector<Tensor> outputTensors;
                status = session->Run({{input_layer, input}}, {output_layer}, {}, &outputTensors);
                cout << "output " << outputTensors[0].DebugString() << endl;

                auto out = outputTensors[0].scalar<int64>();
                cout << "prediction " << out << endl;

                step = 0;
            }
        }

        return true;
    }

    /****************************************************************/
    bool interruptModule() override
    {
        opcPort.interrupt();
        return true;
    }

    /****************************************************************/
    bool close() override
    {
        session->Close();
        delete session;

        opcPort.close();
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
    rf.setDefaultContext("actionManager");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);

    Recognizer recognizer;
    return recognizer.runModule(rf);
}


