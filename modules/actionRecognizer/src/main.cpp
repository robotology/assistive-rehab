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
#include <math.h>
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
#include "src/actionRecognizer_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace tensorflow;
using namespace assistive_rehab;

/****************************************************************/
class Recognizer : public RFModule, public actionRecognizer_IDL
{
    string moduleName;
    unordered_map<string,int> keypoint2int;
    unordered_map<int,string> class_map;
    int nframes,nsteps,nclasses,nfeatures;
    vector<float> min,max;
    string skel_tag;
    SkeletonWaist skeletonIn;

    bool predict;
    int idx_frame,idx_step;
    string pathToGraph,checkpointPath;
    Status status;
    Tensor input;
    Session* session;
    SessionOptions sess_opts;
    vector<string> predictions;
    vector<float> outscores;

    RpcServer analyzerPort;
    BufferedPort<Bottle> outPort;
    RpcClient opcPort;

    Mutex mutex;
    bool starting;
    string action_to_perform;

public:
    /********************************************************/
    bool configure(ResourceFinder &rf)
    {
        string moduleName = rf.check("name", Value("actionRecognizer")).asString();
        setName(moduleName.c_str());
        predict = rf.check("predict",Value(true)).asBool();
        Bottle &bGroup=rf.findGroup("general");
        if (bGroup.isNull())
        {
            yError()<<"Unable to find group \"general\"";
            return false;
        }
        if (!bGroup.check("num-classes") || !bGroup.check("num-features"))
        {
            yError()<<"Unable to find key \"num-classes\" or \"num-features\"";
            return false;
        }
        nclasses=bGroup.find("num-classes").asInt();
        nfeatures=bGroup.find("num-features").asInt();
        nsteps=bGroup.find("num-steps").asInt();
        for (int i=0; i<nclasses; i++)
        {
            ostringstream class_i;
            class_i<<"class-"<<i;
            Bottle &bClass=rf.findGroup(class_i.str());
            if (bClass.isNull())
            {
                string msg="Unable to find section";
                msg+="\""+class_i.str()+"\"";
                yError()<<msg;
                return false;
            }
            if (!bClass.check("label") || !bClass.check("value"))
            {
                yError()<<"Unable to find key \"label\" and/or \"value\"";
                return false;
            }
            int value=bClass.find("value").asInt();
            string label=bClass.find("label").asString();
            class_map[value]=label;
        }

        ifstream file(rf.findFileByName("scale_values.txt"));
        string line = "";
        while(getline(file, line))
        {
            vector<string> vec;
            boost::algorithm::split(vec, line, boost::is_any_of(","));
            min.push_back(stof((vec[0]).c_str(),0));
            max.push_back(stof((vec[1]).c_str(),0));
        }
        keypoint2int[KeyPointTag::head] = 0;
        keypoint2int[KeyPointTag::hand_left] = 2;
        keypoint2int[KeyPointTag::elbow_left] = 4;
        keypoint2int[KeyPointTag::shoulder_left] = 6;
        keypoint2int[KeyPointTag::hand_right] = 8;
        keypoint2int[KeyPointTag::elbow_right] = 10;
        keypoint2int[KeyPointTag::shoulder_right] = 12;
        keypoint2int[KeyPointTag::ankle_left] = 14;
        keypoint2int[KeyPointTag::knee_left] = 16;
        keypoint2int[KeyPointTag::hip_left] = 18;
        keypoint2int[KeyPointTag::ankle_right] = 20;
        keypoint2int[KeyPointTag::knee_right] = 22;
        keypoint2int[KeyPointTag::hip_right] = 24;
        keypoint2int[KeyPointTag::hip_center] = 26;
        keypoint2int[KeyPointTag::shoulder_center] = 28;

        skel_tag = " ";
        starting = false;
        idx_step=0;

        // Set up input paths
        pathToGraph = rf.findFileByName("model.meta");
        checkpointPath = pathToGraph.substr(0, pathToGraph.find_last_of("/\\")) + "/model";
        if(predict)
        {
            yInfo() << "Loading model from:" << pathToGraph;
            yInfo() << "Loading scaling values from: " << rf.findFileByName("scale_values.txt");

            sess_opts.config.mutable_gpu_options()->set_allow_growth(true); //to limit GPU usage
            session = NewSession(sess_opts);
            if (session == nullptr)
            {
                throw runtime_error("Could not create Tensorflow session");
                return false;
            }

            // Read in the protobuf graph we exported
            MetaGraphDef graph_def;
            status = ReadBinaryProto(Env::Default(), pathToGraph, &graph_def);
            if (!status.ok())
            {
                throw runtime_error("Error reading graph definition from " + pathToGraph + ": " + status.ToString());
                return false;
            }

            // Add the graph to the session
            status = session->Create(graph_def.graph_def());
            if (!status.ok())
            {
                throw runtime_error("Error creating graph: " + status.ToString());
                return false;
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
                return false;
            }
        }

        analyzerPort.open("/" + moduleName + "/rpc");
        opcPort.open("/" + moduleName + "/opc");
        outPort.open("/" + moduleName + "/target:o");
        attach(analyzerPort);
        return true;
    }

    /**********************************************************/
    bool close() override
    {
        if(predict)
        {
            session->Close();
            delete session;
        }
        analyzerPort.close();
        opcPort.close();
        outPort.close();
        return true;
    }

    /********************************************************/
    double getPeriod() override
    {
        return 0.1;
    }

    /********************************************************/
    bool attach(RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

    /****************************************************************/
    bool run(const int32_t nframes_) override
    {
        LockGuard lg(mutex);
        nframes = (int)nframes_;
        input = Tensor(DT_FLOAT, TensorShape({1,nframes,36}));
        idx_frame = 0;
        for(size_t i=0; i<nframes; i++)
        {
            for(size_t j=0; j<nfeatures; j++)
            {
                input.tensor<float,3>()(0,i,j) = 0.0;
            }
        }

        starting = true;
        return starting;
    }

    /****************************************************************/
    bool tags(const string &skel_tag_) override
    {
        LockGuard lg(mutex);
        skel_tag = skel_tag_;
        return true;
    }

    /****************************************************************/
    bool load(const string &exercise) override
    {
        LockGuard lg(mutex);
        action_to_perform = exercise;
        yInfo() << "Exercise to perform" << action_to_perform;
        return true;
    }

    /****************************************************************/
    bool stop() override
    {
        LockGuard lg(mutex);
        starting = false;
        skel_tag = " ";
        idx_step = 0;
        return !starting;
    }

    /****************************************************************/
    bool updateInput(const string & tag, const int i, const float &x, const float &y)
   {
        int j = keypoint2int[tag];
        input.tensor<float,3>()(0,i,j) = (x-min[j])/(max[j]-min[j]);
        input.tensor<float,3>()(0,i,j+1) = (y-min[j+1])/(max[j+1]-min[j+1]);
        return true;
    }

    /********************************************************/
    void getSkeleton()
    {
        //ask for the property id
        Bottle cmd, reply;
        cmd.addVocab(Vocab::encode("ask"));
        Bottle &content = cmd.addList().addList();
        content.addString("skeleton");
        opcPort.write(cmd, reply);
        if(reply.size() > 1)
        {
            if(reply.get(0).asVocab() == Vocab::encode("ack"))
            {
                if(Bottle *idField = reply.get(1).asList())
                {
                    if(Bottle *idValues = idField->get(1).asList())
                    {
                        if(!skel_tag.empty())
                        {
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
                                opcPort.write(cmd, replyProp);
                                if(replyProp.get(0).asVocab() == Vocab::encode("ack"))
                                {
                                    if(Bottle *propField = replyProp.get(1).asList())
                                    {
                                        Property prop(propField->toString().c_str());
                                        string tag=prop.find("tag").asString();
                                        if(!tag.empty())
                                        {
                                            if(prop.check("tag") && tag==skel_tag)
                                            {
                                                Skeleton* skeleton = skeleton_factory(prop);
                                                skeletonIn.update(skeleton->toProperty());
                                                delete skeleton;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    /****************************************************************/
    bool updateModule() override
    {
        if(opcPort.getOutputCount() > 0 && starting)
        {
            for(size_t i=0; i<skeletonIn.getNumKeyPoints(); i++)
            {
                string tagjoint=skeletonIn[i]->getTag();
                float x=skeletonIn[i]->getPoint()[0];
                float y=skeletonIn[i]->getPoint()[1];
                updateInput(tagjoint,idx_frame,x,y);
            }

            if(idx_frame < nframes)
            {
                if(idx_frame%10==0)
                    yInfo() << "Acquiring frame" << idx_frame;
                idx_frame++;
            }
            else
            {
                string predicted_action;
                float outscore;
                if(predict)
                {
                    //run the inference
                    //                cout << (input).tensor<float, (3)>() << endl;
                    cout << "input: " << input.DebugString() << endl;
                    string input_layer = "x:0";
                    vector<string> output_layers = {"add_1:0","ArgMax:0"};
                    vector<Tensor> outputTensors;
                    status = session->Run({{input_layer, input}}, {output_layers}, {}, &outputTensors);
                    cout << "add_1: " << outputTensors[0].DebugString() << endl;
                    cout << "ArgMax: " << outputTensors[1].DebugString() << endl;

                    auto score = outputTensors[0].tensor<float,2>();
                    auto out = outputTensors[1].scalar<int64>();
                    int pred = out(0);
                    cout << "prediction: " << pred << " " << class_map[pred] << endl;
                    cout << "scores: ";
                    float n=0.0;
                    vector<float> scores(nclasses,0.0);
                    for(size_t k=0; k<nclasses; k++)
                        n+=exp(score(k));
                    for(size_t k=0; k<nclasses; k++)
                    {
                        scores[k]=exp(score(k))/n;
                        cout << scores[k] << " ";
                    }
                    cout << endl;
                    predicted_action=class_map[pred];
                    outscore=scores[pred];
                    predictions.push_back(predicted_action);
                    outscores.push_back(outscore);
                }
                else
                {
                    predicted_action=action_to_perform;
                    outscore=1.0;
                    predictions.push_back(predicted_action);
                    outscores.push_back(outscore);
                }

                idx_frame=0;
                idx_step++;

                if(idx_step==nsteps)
                {
                    //we consider the most voted action
                    int max=count(predictions.begin(),predictions.end(),predictions.at(0));
                    string voted_action=predictions[0];
                    float voted_score=outscores[0];
                    for(size_t i=1; i<predictions.size(); i++)
                    {
                        int temp=count(predictions.begin(),predictions.end(),predictions.at(i));
                        if(temp>max)
                        {
                            max=temp;
                            voted_action=predictions[i];
                            voted_score=outscores[i];
                        }
                    }
                    yInfo() << "The most voted action is" << voted_action << "voted" << max << "times";

                    Bottle &outBottle = outPort.prepare();
                    outBottle.clear();
                    outBottle.addString(action_to_perform);
                    outBottle.addString(voted_action);
                    outBottle.addDouble(voted_score);
                    outPort.write();

                    idx_step=0;
                    predictions.clear();
                    outscores.clear();
                }
            }
        }

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
    rf.setDefaultContext("actionRecognizer");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);

    Recognizer recognizer;
    return recognizer.runModule(rf);
}


