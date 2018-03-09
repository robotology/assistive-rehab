/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
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

#include <vector>

#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <Extractor.h>

#include "motionAnalyzer_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


/********************************************************/
class Manager : public RFModule,
                public motionAnalyzer_IDL
{

    RpcClient opcPort;
    RpcServer rpcPort;
    BufferedPort<Bottle> scopePort;

    ResourceFinder *rf;

    struct listEntry
    {
        vector<string> tag;
//        int id_joint;
        int min;
        int max;
    };
    map<string, listEntry> motion_list;

    /********************************************************/
    bool load()
    {
        ResourceFinder rf;
        rf.setVerbose();
        rf.setDefaultContext(this->rf->getContext().c_str());
        rf.setDefaultConfigFile(this->rf->find("configuration-file").asString().c_str());
        rf.configure(0, NULL);

        Bottle &bGeneral = rf.findGroup("GENERAL");

        if(!bGeneral.isNull())
        {
            int nmovements = bGeneral.find("number_movements").asInt();
            string side = bGeneral.find("side").asString();

            if(Bottle *motion_tag = bGeneral.find("motion_tag").asList())
            {
                if(Bottle *n_motion_tag = bGeneral.find("number_motion").asList())
                {
                    for(int i=0; i<motion_tag->size(); i++)
                    {
                        string curr_tag = motion_tag->get(i).asString();
                        int motion_number = n_motion_tag->get(i).asInt();

                        for(int j=0; j<motion_number; j++)
                        {
                            Bottle &bMotion = rf.findGroup(curr_tag+"_"+to_string(j));
                            if(!bMotion.isNull())
                            {
                                if(Bottle *bJoint = bMotion.find("tag").asList())
                                {
                                    motion_list[curr_tag+"_"+to_string(j)].tag.resize(bJoint->size());
                                    for(int k=0; k<bJoint->size(); k++)
                                        motion_list[curr_tag+"_"+to_string(j)].tag[k] = bJoint->get(k).asString()+side;
//                                    motion_list[curr_tag+"_"+to_string(j)].id_joint = bJoint->get(1).asInt();
                                    motion_list[curr_tag+"_"+to_string(j)].min = bMotion.check("min",Value(-1)).asInt();
                                    motion_list[curr_tag+"_"+to_string(j)].max = bMotion.check("max",Value(-1)).asInt();
                                }
                            }
                        }
                    }
                }
            }
        }
        else
        {
            yError() << "Error in loading parameters. Stopping module!";
            return false;
        }

        print_list();

        return true;
    }

    /********************************************************/
    void print_list()
    {
        yInfo() << "Loaded motion list:\n";
        for(map<string, listEntry>::iterator it = motion_list.begin(); it!= motion_list.end(); it++)
        {
            listEntry &entry = it->second;
            yInfo() << "[" << it->first << "]";
            for(int i=0; i<entry.tag.size(); i++)
                yInfo() << "Tag = " << entry.tag[i];
//            yInfo() << "id joint = " << entry.id_joint;
            yInfo() << "Min = " << entry.min;
            yInfo() << "Max = " << entry.max << "\n";
        }
    }

    /********************************************************/
    bool getJoint(vector<string> &joints, listEntry &entry)
    {
        joints.resize(entry.tag.size());
        for(int i=0; i<entry.tag.size(); i++)
        {
            joints[i] = entry.tag[i];
        }

        return true;
    }

    /********************************************************/
    bool getSkeleton(vector<string> joint_names, Matrix &joint_3d_position)
    {

        //            yInfo() << "Get skeleton value for" << joint_name;

        //ask for the property id
        Bottle cmd, reply;
        cmd.addVocab(Vocab::encode("ask"));
        Bottle &content = cmd.addList().addList();
        content.addString("body");

        //            yInfo() << "Query opc: " << cmd.toString();
        opcPort.write(cmd, reply);
        //            yInfo() << "Reply from opc:" << reply.toString();

        if(reply.size() > 1)
        {
            if(reply.get(0).asVocab() == Vocab::encode("ack"))
            {
                if(Bottle *idField = reply.get(1).asList())
                {
                    if(Bottle *idValues = idField->get(1).asList())
                    {
                        int id = idValues->get(9).asInt();
//                        yInfo() << id;

                        //given the id, get the value of the property
                        cmd.clear();
                        cmd.addVocab(Vocab::encode("get"));
                        Bottle &content = cmd.addList().addList();
                        content.addString("id");
                        content.addInt(id);
                        Bottle replyProp;

                        //                            yInfo() << "Command sent to the port: " << cmd.toString();
                        opcPort.write(cmd, replyProp);
                        //                            yInfo() << "Reply from opc:" << replyProp.toString();

                        if(replyProp.get(0).asVocab() == Vocab::encode("ack"))
                        {
                            if(Bottle *propField = replyProp.get(1).asList())
                            {
                                if(Bottle *propSubField = propField->find("body").asList())
                                {
                                    //                                       yInfo() << propSubField->get(0).asString();
                                    //                                        yInfo() << joint_name+"Right";
                                    for(int i=0; i<joint_names.size(); i++)
                                    {
                                        if(Bottle *joint_3d = propSubField->find(joint_names[i]).asList())
                                        {
                                            joint_3d_position.resize(3, joint_names.size());
                                            joint_3d_position(0, i) = joint_3d->get(0).asDouble();
                                            joint_3d_position(1, i) = joint_3d->get(1).asDouble();
                                            joint_3d_position(2, i) = joint_3d->get(2).asDouble();
                                            yInfo() << joint_names[i] << " " << joint_3d_position(0, i) << " "
                                                    << joint_3d_position(1, i) << " " << joint_3d_position(2, i);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        cout << "\n";
    }

    /********************************************************/
    bool attach(RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

public:
    /********************************************************/
    bool configure(ResourceFinder &rf)
    {
        this->rf = &rf;
        string moduleName = rf.check("name", Value("motionAnalyzer")).asString();
        setName(moduleName.c_str());

        string robot = rf.check("robot", Value("icub")).asString();

        opcPort.open(("/" + getName() + "/opc").c_str());
        scopePort.open(("/" + getName() + "/scope").c_str());
        rpcPort.open(("/" + getName() + "/cmd").c_str());
        attach(rpcPort);

        if(!load())
            return false;

        return true;
    }

    /********************************************************/
    bool close()
    {
        opcPort.close();
        scopePort.close();
        rpcPort.close();

        return true;
    }

    /********************************************************/
    double getPeriod()
    {
        return 1.0;
    }

    /********************************************************/
    bool updateModule()
    {
        //if we query the database
        if(opcPort.getOutputCount() > 0)
        {
            Extractor extractor;

            vector<string> joint_names;
            for (map<string, listEntry>::iterator it = motion_list.begin(); it!= motion_list.end(); it++)
            {
                listEntry &entry = it->second;
                getJoint(joint_names, entry);

                Matrix j_pos;
                getSkeleton(joint_names, j_pos);

                //extract metric
                extractor.configure(j_pos);
                double rom = extractor.computeRom();

                //write it on the output
                Bottle &scopebottleout = scopePort.prepare();
                scopebottleout.clear();
                scopebottleout.addDouble(rom);
                scopePort.write();
            }
        }

        return true;
    }

};

int main(int argc, char *argv[])
{
    Network yarp;
    if(!yarp.checkNetwork())
    {
        yError() << "YARP server not available";
        return -1;
    }

    Manager manager;
    ResourceFinder rf;

    rf.setVerbose();
    rf.setDefaultContext("motionAnalyzer");
//    rf.setDefaultConfigFile("motion-list.ini");
    rf.setDefault("configuration-file", "motion-list.ini");

    rf.configure(argc, argv);


    return manager.runModule(rf);

    return 0;
}
