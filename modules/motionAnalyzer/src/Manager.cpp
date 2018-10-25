/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file Manager.cpp
 * @authors: Valentina Vasco <valentina.vasco@iit.it>
 */

#include <iostream>
#include <vector>
#include <list>

#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include "Manager.h"
#include "Metric.h"
#include "Processor.h"

#include "src/motionAnalyzer_IDL.h"

#include "matio.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace assistive_rehab;

void Manager::init()
{
    numKeypoints = 15;

    elbowLeft.resize(3);
    elbowRight.resize(3);
    handLeft.resize(3);
    handRight.resize(3);
    head.resize(3);
    shoulderCenter.resize(3);
    shoulderLeft.resize(3);
    shoulderRight.resize(3);
    hipLeft.resize(3);
    hipRight.resize(3);
    kneeLeft.resize(3);
    kneeRight.resize(3);
    ankleLeft.resize(3);
    ankleRight.resize(3);

    cameraposinit.resize(3);
    focalpointinit.resize(3);
}

bool Manager::loadInitialConf()
{
    ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultContext(this->rf->getContext().c_str());
    rf.setDefaultConfigFile(this->rf->find("from").asString().c_str());
    rf.configure(0, NULL);

    Bottle &bGeneral = rf.findGroup("GENERAL");

    if(!bGeneral.isNull())
    {
        if(Bottle *bCamerapos_init = bGeneral.find("cameraposinit").asList())
        {
            cameraposinit[0] = bCamerapos_init->get(0).asDouble();
            cameraposinit[1] = bCamerapos_init->get(1).asDouble();
            cameraposinit[2] = bCamerapos_init->get(2).asDouble();
        }
        else
            yError() << "Could not load initial camera pos";

        if(Bottle *bFocalpoint_init = bGeneral.find("focalpointinit").asList())
        {
            focalpointinit[0] = bFocalpoint_init->get(0).asDouble();
            focalpointinit[1] = bFocalpoint_init->get(1).asDouble();
            focalpointinit[2] = bFocalpoint_init->get(2).asDouble();
        }
        else
            yError() << "Could not load initial focal point";

        dtw_thresh = bGeneral.find("dtw_thresh").asDouble();
        mean_thresh = bGeneral.find("mean_thresh").asDouble();
        sx_thresh = bGeneral.find("sx_thresh").asDouble();
        sy_thresh = bGeneral.find("sy_thresh").asDouble();
        sz_thresh = bGeneral.find("sz_thresh").asDouble();
        f_static = bGeneral.find("f_static").asInt();
        range_freq = bGeneral.find("range_freq").asInt();
    }

    return true;
}

bool Manager::loadMotionList()
{
    LockGuard lg(mutex);

    ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultContext(this->rf->getContext().c_str());
    rf.setDefaultConfigFile(this->rf->find("from").asString().c_str());
    rf.configure(0, NULL);

    Bottle &bGeneral = rf.findGroup("GENERAL");

    if(!bGeneral.isNull())
    {
        if(Bottle *metric_tag = bGeneral.find("metric_tag").asList())
        {
            if(Bottle *n_metric_tag = bGeneral.find("number_metrics").asList())
            {
                for(int i=0; i<metric_tag->size(); i++)
                {
                    string curr_tag = metric_tag->get(i).asString();
                    int motion_number = n_metric_tag->get(i).asInt();

                    for(int j=0; j<motion_number; j++)
                    {
                        Bottle &bMotion = rf.findGroup(curr_tag+"_"+to_string(j));
                        if(!bMotion.isNull())
                        {
                            string motion_type = bMotion.find("motion_type").asString();
                            string tag_joint = bMotion.find("tag_joint").asString();
                            double min = bMotion.find("min").asDouble();
                            double max = bMotion.find("max").asDouble();
                            double duration = bMotion.find("duration").asDouble();
                            Vector camerapos;
                            camerapos.resize(3);
                            if(Bottle *bCamerapos = bMotion.find("camerapos").asList())
                            {
                                camerapos[0] = bCamerapos->get(0).asDouble();
                                camerapos[1] = bCamerapos->get(1).asDouble();
                                camerapos[2] = bCamerapos->get(2).asDouble();
                            }
                            else
                                camerapos = cameraposinit;

                            Vector focalpoint;
                            focalpoint.resize(3);
                            if(Bottle *bFocalpoint = bMotion.find("focalpoint").asList())
                            {
                                focalpoint[0] = bFocalpoint->get(0).asDouble();
                                focalpoint[1] = bFocalpoint->get(1).asDouble();
                                focalpoint[2] = bFocalpoint->get(2).asDouble();
                            }
                            else
                                focalpoint = focalpointinit;

                            Vector ref_dir;
                            ref_dir.resize(3);
                            if(Bottle *bRefdir = bMotion.find("ref_dir").asList())
                            {
                                ref_dir[0] = bRefdir->get(0).asDouble();
                                ref_dir[1] = bRefdir->get(1).asDouble();
                                ref_dir[2] = bRefdir->get(2).asDouble();
                            }
                            else
                                yError() << "Could not find reference direction";

                            string tag_plane = bMotion.find("tag_plane").asString();

                            relaxed_joints.clear();
                            relaxed_dtw_thresh.clear();
                            relaxed_mean_thresh.clear();
                            relaxed_sx_thresh.clear();
                            relaxed_sy_thresh.clear();
                            relaxed_sz_thresh.clear();
                            relaxed_f_static.clear();
                            relaxed_range_freq.clear();

                            relaxed_dtw_thresh.push_back(dtw_thresh);
                            relaxed_mean_thresh.push_back(mean_thresh);
                            relaxed_sx_thresh.push_back(sx_thresh);
                            relaxed_sy_thresh.push_back(sy_thresh);
                            relaxed_sz_thresh.push_back(sz_thresh);
                            relaxed_f_static.push_back(f_static);
                            relaxed_range_freq.push_back(range_freq);

                            if(Bottle *bRelaxedJoints = bMotion.find("relaxed_list").asList())
                            {
                                for(size_t i=0; i<bRelaxedJoints->size(); i++)
                                    relaxed_joints.push_back(bRelaxedJoints->get(i).asString());

                                if(Bottle *bDtwThresh = bMotion.find("dtw_thresh").asList())
                                {
                                    for(size_t i=0; i<bDtwThresh->size(); i++)
                                        relaxed_dtw_thresh.push_back(bDtwThresh->get(i).asDouble());
                                }

                                if(Bottle *bMeanThresh = bMotion.find("mean_thresh").asList())
                                {
                                    for(size_t i=0; i<bMeanThresh->size(); i++)
                                        relaxed_mean_thresh.push_back(bMeanThresh->get(i).asDouble());
                                }
                                
                                if(Bottle *bSxThresh = bMotion.find("sx_thresh").asList())
                                {
                                    for(size_t i=0; i<bSxThresh->size(); i++)
                                        relaxed_sx_thresh.push_back(bSxThresh->get(i).asDouble());
                                }
                                
                                if(Bottle *bSyThresh = bMotion.find("sy_thresh").asList())
                                {
                                    for(size_t i=0; i<bSyThresh->size(); i++)
                                        relaxed_sy_thresh.push_back(bSyThresh->get(i).asDouble());
                                }
                                
                                if(Bottle *bSzThresh = bMotion.find("sz_thresh").asList())
                                {
                                    for(size_t i=0; i<bSzThresh->size(); i++)
                                        relaxed_sz_thresh.push_back(bSzThresh->get(i).asDouble());
                                }

                                if(Bottle *bFstatic = bMotion.find("f_static").asList())
                                {
                                    for(size_t i=0; i<bFstatic->size(); i++)
                                        relaxed_f_static.push_back(bFstatic->get(i).asInt());
                                }

                                if(Bottle *bRangeFreq = bMotion.find("range_freq").asList())
                                {
                                    for(size_t i=0; i<bRangeFreq->size(); i++)
                                        relaxed_range_freq.push_back(bRangeFreq->get(i).asInt());
                                }
                            }

                            if(curr_tag == Rom_Processor::metric_tag)
                            {
                                metric_repertoire = new Rom();
                            }
                            else if(curr_tag == EndPoint_Processor::metric_tag)
                            {
                                Vector target;
                                target.resize(3);
                                if(Bottle *bTarget = bMotion.find("target").asList())
                                {
                                    target[0] = bTarget->get(0).asDouble();
                                    target[1] = bTarget->get(1).asDouble();
                                    target[2] = bTarget->get(2).asDouble();
                                }
                                else
                                    yError() << "Could not find target";

                                metric_repertoire = new EndPoint();
                                metric_repertoire->setTarget(target);
                            }

                            metric_repertoire->initialize(curr_tag, motion_type, tag_joint, ref_dir, tag_plane,
                                                          min, max, duration, camerapos, focalpoint,
                                                          relaxed_joints,relaxed_dtw_thresh,relaxed_mean_thresh,
                                                          relaxed_sx_thresh,relaxed_sy_thresh, relaxed_sz_thresh,
                                                          relaxed_f_static,relaxed_range_freq);

                            //add the current metric to the repertoire
                            motion_repertoire.insert(pair<string, Metric*>(curr_tag+"_"+to_string(j), metric_repertoire));
                            motion_repertoire[curr_tag+"_"+to_string(j)]->print();
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

    return true;
}

/********************************************************/
bool Manager::loadMetric(const string &metric_tag)
{
    LockGuard lg(mutex);

    if(motion_repertoire.count(metric_tag))
    {
        metric = motion_repertoire.at(metric_tag);
        yInfo() << "Metric to analyze";
        metric->print();

        processor = createProcessor(metric_tag, metric);

        //send commands to skeletonScaler
        Bottle cmd,reply;
        cmd.addVocab(Vocab::encode("load"));
        string f = metric->getMotionType();
        cmd.addString(f);
        actionPort.write(cmd,reply);
        if(reply.get(0).asVocab()!=Vocab::encode("ok"))
        {
            yError() << "actionRecognizer could not motion type" << f;
            return false;
        }
        cmd.clear();
        reply.clear();
        cmd.addVocab(Vocab::encode("load"));
        f = f + ".log";
        cmd.addString(f);
        string context = rf->getContext();
        cmd.addString(context);
        scalerPort.write(cmd,reply);
        if(reply.get(0).asVocab()!=Vocab::encode("ok"))
        {
            yError() << "skeletonScaler could not load" << f << "file";
            return false;
        }

        cmd.clear();
        reply.clear();
        cmd.addVocab(Vocab::encode("rot"));
        Vector cp = metric->getCameraPos();
        Vector fp = metric->getFocalPoint();
        cmd.addList().read(cp);
        cmd.addList().read(fp);
        scalerPort.write(cmd,reply);
        if(reply.get(0).asVocab()!=Vocab::encode("ok"))
        {
            yWarning() << "skeletonScaler could not rotate properly the camera";
        }

        cmd.clear();
        reply.clear();
        cmd.addVocab(Vocab::encode("tagt"));
        string tag_template = metric->getMotionType();
        cmd.addString(tag_template);
        dtwPort.write(cmd,reply);
        if(reply.get(0).asVocab()!=Vocab::encode("ok"))
        {
            yError() << "alignmentManager could not load the skeleton tag";
            return false;
        }

        if(metric!=NULL)
            return true;

        return false;
    }
    else
    {
        yWarning() << "The metric does not exist in the repertoire";
        return false;
    }
}
/********************************************************/
string Manager::getMotionType()
{
    LockGuard lg(mutex);

    if(metric!=NULL)
        return metric->getMotionType();
    else
        return "";

}

/********************************************************/
vector<string> Manager::listMetrics()
{
    LockGuard lg(mutex);

    vector<string> reply;
    for (map<string,Metric*>::iterator it=motion_repertoire.begin(); it!=motion_repertoire.end(); it++)
    {
        reply.push_back(it->first);
    }

    return reply;
}

/********************************************************/
vector<string> Manager::listRelaxedJoints()
{
    LockGuard lg(mutex);

    vector<string> reply=metric->getRelaxedJoints();
    return reply;
}

/********************************************************/
bool Manager::selectSkel(const string &skel_tag)
{
    LockGuard lg(mutex);

    this->skel_tag = skel_tag;
    yInfo() << "Analyzing skeleton " << this->skel_tag.c_str();

    //send tag to skeletonScaler
    Bottle cmd, reply;
    cmd.addVocab(Vocab::encode("tag"));
    cmd.addString(this->skel_tag);
    yInfo() << cmd.toString();

    scalerPort.write(cmd, reply);
    dtwPort.write(cmd, reply);

#ifndef NDEBUG
    yInfo() << "Debugging";
    {
        double t1 = Time::now();
        while( (Time::now()-t1) < 15.0 )
        {
            //do nothing
            yInfo() << "Waiting to start";
        }
        return start();
    }
#endif

    return true;
}

/********************************************************/
bool Manager::start()
{
    LockGuard lg(mutex);
                
    yInfo() << "Start!";

    Bottle cmd,reply;
    cmd.addVocab(Vocab::encode("run"));
    scalerPort.write(cmd,reply);
    if(reply.get(0).asVocab()!=Vocab::encode("ok"))
    {
		yError() << "Could not run skeletonScaler";
		return false;		
	}
	
	Time::delay(1.0);
	
    bool out=false;
    while(out==false)
    {    
        getSkeleton();

        //we do not start if we haven't selected a skeleton tag
        if(skel_tag.empty() || !updated)
        {
            yWarning() << "Please select a proper skeleton tag";
            return false;
        }

        out=skeletonIn.update_planes();
        Time::yield();
    }    

    processor->setInitialConf(skeletonIn);

    //start alignmentManager
    reply.clear();
    cmd.addDouble(metric->getDuration());
    cmd.addList().read(metric->getDtwThresh());
    cmd.addList().read(metric->getMeanThresh());
    cmd.addList().read(metric->getSxThresh());
    cmd.addList().read(metric->getSyThresh());
    cmd.addList().read(metric->getSzThresh());
    cmd.addList().read(metric->getFstatic());
    cmd.addList().read(metric->getRangeFreq());
    dtwPort.write(cmd,reply);
    actionPort.write(cmd,reply);
    if(reply.get(0).asVocab()==Vocab::encode("ok"))
    {
        tstart_session = Time::now()-tstart;
        starting = true;
        return true;
    }
    return false;
}

/********************************************************/
bool Manager::stop()
{
    LockGuard lg(mutex);

    //stop skeletonScaler
    Bottle cmd, reply;
    cmd.addVocab(Vocab::encode("stop"));
    scalerPort.write(cmd,reply);
    dtwPort.write(cmd,reply);
    actionPort.write(cmd,reply);
    if(reply.get(0).asVocab()==Vocab::encode("ok") && starting)
    {
        cmd.clear();
        reply.clear();
        cmd.addVocab(Vocab::encode("rot"));
        cmd.addList().read(cameraposinit);
        cmd.addList().read(focalpointinit);
        yInfo() << cmd.toString();
        scalerPort.write(cmd, reply);

//        yInfo() << "stopping";
        starting = false;
//        metric = NULL;
        skel_tag = "";
        tend_session = Time::now()-tstart;

        // Use MATIO to write the results in a .mat file
        string filename_report = out_folder + "/user-" + skeletonIn.getTag() + "-" + metric->getMotionType() + "-" + to_string(nsession) + ".mat";
        mat_t *matfp = Mat_CreateVer(filename_report.c_str(),NULL,MAT_FT_MAT5);
        if (matfp == NULL)
            yError() << "Error creating MAT file";

        yInfo() << "Writing to file";
        if(writeKeypointsToFile(matfp))
        {
            time_samples.clear();
            all_keypoints.clear();
            all_planes.clear();
            ideal_samples.clear();
        }
        else
            yError() << "Could not save to file";

        yInfo() << "Keypoints saved to file" << filename_report.c_str();
        Mat_Close(matfp);

        nsession++;

        return true;
    }
    yError() << "Could not stop... maybe not started?";
    return false;
}

/********************************************************/
void Manager::getSkeleton()
{
    //ask for the property id
    Bottle cmd, reply;
    cmd.addVocab(Vocab::encode("ask"));
    Bottle &content = cmd.addList().addList();
    content.addString("skeleton");

//    yInfo() << "Query opc: " << cmd.toString();
    opcPort.write(cmd, reply);
//    yInfo() << "Reply from opc:" << reply.toString();

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
                        updated=false;
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

//                            yInfo() << "Command sent to the port: " << cmd.toString();
                            opcPort.write(cmd, replyProp);
//                            yInfo() << "Reply from opc:" << replyProp.toString();

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
                                            if(skeleton->update_planes())
                                                all_keypoints.push_back(skeletonIn.get_unordered());
                                            updated=true;
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

/********************************************************/
bool Manager::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}

/********************************************************/
bool Manager::configure(ResourceFinder &rf)
{
    this->rf = &rf;
    string moduleName = rf.check("name", Value("motionAnalyzer")).asString();
    setName(moduleName.c_str());

    string robot = rf.check("robot", Value("icub")).asString();

    opcPort.open(("/" + getName() + "/opc").c_str());
    scopePort.open(("/" + getName() + "/scope").c_str());
    scalerPort.open(("/" + getName() + "/scaler:cmd").c_str());
    dtwPort.open(("/" + getName() + "/dtw:cmd").c_str());
    actionPort.open(("/" + getName() + "/action:cmd").c_str());
    rpcPort.open(("/" + getName() + "/cmd").c_str());
    attach(rpcPort);

    metric=NULL;

    init();
    loadInitialConf();
    if(!loadMotionList())
        return false;

    out_folder = rf.getHomeContextPath();

    tstart = Time::now();

    nsession = 0;
    finishedSession = false;
    starting = false;

    skel_tag="";

    return true;
}

/********************************************************/
bool Manager::interruptModule()
{
    opcPort.interrupt();
    scopePort.interrupt();
    scalerPort.interrupt();
    dtwPort.interrupt();
    actionPort.interrupt();
    rpcPort.interrupt();
    yInfo() << "Interrupted module";

    return true;
}

/********************************************************/
bool Manager::close()
{
    delete metric_repertoire;
    delete metric;
    delete processor;

    yInfo() << "Freed memory";

    opcPort.close();
    scopePort.close();
    scalerPort.close();
    dtwPort.close();
    actionPort.close();
    rpcPort.close();

    yInfo() << "Closed ports";

    return true;
}

/********************************************************/
double Manager::getPeriod()
{
    return 0.01;
}

/********************************************************/
bool Manager::updateModule()
{
    LockGuard lg(mutex);

    //if we query the database
    if(opcPort.getOutputCount() > 0 && starting)
    {
        //if no metric has been defined we do not analyze motion
        if(metric != NULL)
        {
            //get skeleton and normalize
            getSkeleton();

            if(updated)
            {
                //update time array
                time_samples.push_back(Time::now()-tstart);

                processor->update(skeletonIn);
                result = processor->computeMetric();
                all_planes.push_back(processor->getPlaneNormal());
                ideal_samples.push_back(processor->getIdeal());

                //write on output port
                Bottle &scopebottleout = scopePort.prepare();
                scopebottleout.clear();
                scopebottleout.addDouble(result);
                scopebottleout.addDouble(processor->getIdeal());
                scopePort.write();

            }
        }
        else
            yInfo() << "Please specify metric";
    }

    return true;
}

/********************************************************/
bool Manager::writeStructToMat(const string& name, const vector< vector< pair<string,Vector> > >& keypoints_skel, mat_t *matfp)
{
    const char *fields[numKeypoints];
    for(int i=0; i<numKeypoints; i++)
    {
//        if(keypoints_skel[0][i].first.c_str() != KeyPointTag::hip_center)
//        cout << i << " " << keypoints_skel[0][i].first.c_str() << endl;
        fields[i]=keypoints_skel[0][i].first.c_str();
    }

    matvar_t *field;

    size_t dim_struct[2] = {1,1};

    size_t nSamples = keypoints_skel.size()-1;

    matvar_t *matvar = Mat_VarCreateStruct(name.c_str(),2,dim_struct,fields,numKeypoints);
    if(matvar != NULL)
    {
        vector<double> field_vector;
        field_vector.resize(3*(nSamples));

        size_t dims_field[2] = {nSamples,3};

    //    print(keypoints_skel);

        for(int i=0; i<numKeypoints; i++)
        {
            for(int j=0; j<nSamples; j++)
            {
                field_vector[j] = keypoints_skel[j][i].second[0];
                field_vector[j+nSamples] = keypoints_skel[j][i].second[1];
                field_vector[j+2*nSamples] = keypoints_skel[j][i].second[2];
            }

            field = Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_field,field_vector.data(),MAT_F_GLOBAL);
            Mat_VarSetStructFieldByName(matvar, fields[i], 0, field);
        }

        Mat_VarWrite(matfp,matvar,MAT_COMPRESSION_NONE);
//        Mat_VarWriteAppend(matfp,matvar,MAT_COMPRESSION_NONE,1);
        Mat_VarFree(matvar);
    }
    else
        return false;

    return true;

}

/********************************************************/
bool Manager::writeStructToMat(const string& name, const Metric& metric, mat_t *matfp)
{
    int numFields=8;
    const char *fields[numFields] = {"motion_type", "ref_joint", "ref_direction", "ref_plane", "max", "min", "tstart", "tend"};
    matvar_t *field;

    size_t dim_struct[2] = {1,1};
    matvar_t *matvar = Mat_VarCreateStruct(name.c_str(),2,dim_struct,fields,numFields);

    if(matvar != NULL)
    {
        string motion_met = metric.getMotionType();
        char *motion_c = new char[motion_met.length() + 1];
        strcpy(motion_c, motion_met.c_str());
        size_t dims_field_motion[2] = {1,motion_met.size()};
        field = Mat_VarCreate(NULL,MAT_C_CHAR,MAT_T_UTF8,2,dims_field_motion,motion_c,0);
        Mat_VarSetStructFieldByName(matvar, fields[0], 0, field);
        delete [] motion_c;

        string joint_met = metric.getTagJoint();
        char *joint_c = new char[joint_met.length() + 1];
        strcpy(joint_c, joint_met.c_str());
        size_t dims_field_joint[2] = {1,joint_met.size()};

        field = Mat_VarCreate(NULL,MAT_C_CHAR,MAT_T_UTF8,2,dims_field_joint,joint_c,0);
        Mat_VarSetStructFieldByName(matvar, fields[1], 0, field);
        delete [] joint_c;

        size_t dims_field_dir[2] = {1,3};
        Vector ref_met = metric.getRefDir();
        field = Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_field_dir,ref_met.data(),MAT_F_DONT_COPY_DATA);
        Mat_VarSetStructFieldByName(matvar, fields[2], 0, field);

        size_t nPlanes = all_planes.size();
        vector<double> field_vector;
        field_vector.resize(3*nPlanes);
        size_t dims_field_plane[2] = {nPlanes,3};
        for(size_t i=0; i<nPlanes; i++)
        {
            field_vector[i] = all_planes[i][0];
            field_vector[i+nPlanes] = all_planes[i][1];
            field_vector[i+2*nPlanes] = all_planes[i][2];
        }
        field = Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_field_plane,field_vector.data(),MAT_F_GLOBAL);
        Mat_VarSetStructFieldByName(matvar, fields[3], 0, field);

//        field = Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_field_plane,all_planes.data(),MAT_F_GLOBAL);
//        Mat_VarSetStructFieldByName(matvar, fields[3], 0, field);

//        size_t dims_field_plane[2] = {1,3};
//        Vector plane_met = plane_normal;
//        field = Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_field_plane,plane_met.data(),MAT_F_DONT_COPY_DATA);
//        Mat_VarSetStructFieldByName(matvar, fields[3], 0, field);

        size_t dims_field_ideal[2] = {ideal_samples.size(),1};
        field = Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_field_ideal,ideal_samples.data(),MAT_F_DONT_COPY_DATA);
        Mat_VarSetStructFieldByName(matvar, fields[4], 0, field);

//        size_t dims_field_max[2] = {1,1};
//        double max_val = metric.getMax();
//        field = Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_field_max,&max_val,MAT_F_DONT_COPY_DATA);
//        Mat_VarSetStructFieldByName(matvar, fields[4], 0, field);

        size_t dims_field_min[2] = {1,1};
        double min_val = metric.getMin();
        field = Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_field_min,&min_val,MAT_F_DONT_COPY_DATA);
        Mat_VarSetStructFieldByName(matvar, fields[5], 0, field);

        size_t dims_field_tstart[2] = {1,1};
        cout << "started at " << tstart_session;
        field = Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_field_tstart,&tstart_session,MAT_F_DONT_COPY_DATA);
        Mat_VarSetStructFieldByName(matvar, fields[6], 0, field);

        cout << " ended at " << tend_session << endl;
        size_t dims_field_tend[2] = {1,1};
        field = Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_field_tend,&tend_session,MAT_F_DONT_COPY_DATA);
        Mat_VarSetStructFieldByName(matvar, fields[7], 0, field);

        Mat_VarWrite(matfp,matvar,MAT_COMPRESSION_NONE);
//        Mat_VarWriteAppend(matfp, matvar, MAT_COMPRESSION_NONE,1);
        Mat_VarFree(matvar);
    }
    else
        return false;


    return true;

}

/********************************************************/
void Manager::print(const vector< vector< pair<string,Vector> > >& keypoints_skel)
{
    for(int i=0; i<keypoints_skel[0].size(); i++)
    {
        for(int j=0; j<keypoints_skel.size(); j++)
        {
            cout << keypoints_skel[j][i].first << " " << keypoints_skel[j][i].second[0] << " "
                 << keypoints_skel[j][i].second[1] << " " << keypoints_skel[j][i].second[2] << endl;
        }
    }
    cout << endl;
}

/********************************************************/
bool Manager::writeKeypointsToFile(mat_t *matfp)
{
    // Use MATIO to write the results in a .mat file

    //Save time samples
    size_t nSamples = time_samples.size();
    size_t dims[2] = {nSamples,1};
    matvar_t *matvar = Mat_VarCreate("Time_samples", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, time_samples.data(), 0);
    if(matvar != NULL)
    {
        Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_NONE);
//        Mat_VarWriteAppend(matfp, matvar, MAT_COMPRESSION_NONE,1);
        Mat_VarFree(matvar);
    }
    else
    {
        yError() << "Could not save time samples.. the file will not be saved";
        return false;
    }

//    if(all_keypoints.size() != 0)
//        print(all_keypoints);

    //Save keypoint
    if(!writeStructToMat("Keypoints", all_keypoints, matfp))
    {
        yError() << "Could not save keypoints.. the file will not be saved";
        return false;
    }

//    //Save kind of metric to process
//    for(int i=0; i<metrics.size(); i++)
//    {
//        //         cout << metrics[i]->getName().c_str() << endl;
//        if(!writeStructToMat(metrics[i]->getName().c_str(), *metrics[i]))
//        {
//            yError() << "Could not save metrics.. the file will not be saved";
//            return false;
//        }
//    }

    //Save kind of metric to process
    if(!writeStructToMat(metric->getName().c_str(), *metric, matfp))
    {
        yError() << "Could not save metrics.. the file will not be saved";
        return false;
    }

    return true;
}
