/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file Processor.cpp
 * @authors: Valentina Vasco <valentina.vasco@iit.it>
 */

#include "Processor.h"
#include <iomanip>

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::os;
using namespace iCub::ctrl;
using namespace assistive_rehab;

/****************************************/
/*             PROCESSOR                */
/****************************************/
Processor *createProcessor(const string& metric_type, const Metric* metric_)
{
    if(metric_type.find(MetricType::rom)!=std::string::npos)
    {
        yInfo() << "Creating processor for" << MetricType::rom << "\n";
        return new Rom_Processor(metric_);
    }
    else if(metric_type.find(MetricType::step)!=std::string::npos)
    {
        yInfo() << "Creating processor for" << MetricType::step << "\n";
        return new Step_Processor(metric_);
    }
    else if(metric_type.find(MetricType::end_point)!=std::string::npos)
    {
        yInfo() << "Creating processor for" << MetricType::end_point << "\n";
        return new EndPoint_Processor(metric_);
    }
    else
    {
        yError() << "10";
        return 0;
    }
}

/********************************************************/
Processor::Processor()
{
    plane_normal.resize(3);
    curr_frame.resize(4,4);
    curr_frame=eye(4,4);
}

void print(const Matrix& m)
{
    for(int i=0;i<m.rows();i++)
    {
        for(int j=0;j<m.cols();j++)
        {
            cout << m[i][j] << " ";
        }
        cout << "\n";
    }
}

/********************************************************/
void Processor::setInitialConf(SkeletonStd &skeleton_, Matrix &T)
{
    first_skeleton.update(skeleton_.toProperty());
    first_skeleton.normalize();

    if(!first_skeleton.update_planes())
        yError() << "Not all planes are updated";

    Vector coronal = first_skeleton.getCoronal();
    Vector sagittal = first_skeleton.getSagittal();
    Vector transverse = first_skeleton.getTransverse();
    Vector p = first_skeleton[KeyPointTag::shoulder_center]->getPoint();
    Matrix T1(4,4);
    T1.setSubcol(coronal,0,0);
    T1.setSubcol(sagittal,0,1);
    T1.setSubcol(transverse,0,2);
    T1.setSubcol(p,0,3);
    T1(3,3)=1.0;
    curr_frame = SE3inv(T1);
    T = curr_frame;

    t0=Time::now();
}

/********************************************************/
void Processor::setStartingTime(const double &tnow)
{
    t0=tnow;
}

/****************************************************************/
void Processor::update(SkeletonStd &curr_skeleton_)
{
    curr_skeleton.update(curr_skeleton_.toProperty());
}

/****************************************************************/
Vector Processor::projectOnPlane(const Vector &v,const Vector &plane)
{
    double dist=dot(v,plane);
    return v-dist*plane;
}

void Processor::updateCurrentFrame(SkeletonStd &skeleton_)
{
    if(!skeleton_.update_planes())
        yError() << "Not all planes are updated";

    Vector coronal = skeleton_.getCoronal();
    Vector sagittal = skeleton_.getSagittal();
    Vector transverse = skeleton_.getTransverse();
    Vector p = skeleton_[KeyPointTag::shoulder_center]->getPoint();
    Matrix T1(4,4);
    T1.setSubcol(coronal,0,0);
    T1.setSubcol(sagittal,0,1);
    T1.setSubcol(transverse,0,2);
    T1.setSubcol(p,0,3);
    T1(3,3)=1.0;
    curr_frame = SE3inv(T1);

    skeleton_.print(std::cout);

    yDebug() << "DIR\n" << T1.toString();
    yDebug() << "INV\n" << curr_frame.toString();

}

/****************************************************************/
Vector Processor::toCurrFrame(const string &tag)
{
    Vector k=curr_skeleton[tag]->getPoint();
    k.push_back(1.0);
    return (curr_frame*k).subVector(0,2);
}

/****************************************/
/*            ROM PROCESSOR             */
/****************************************/
Rom_Processor::Rom_Processor()
{

}

/********************************************************/
Rom_Processor::Rom_Processor(const Metric* rom_)
{
    rom=(Rom*)rom_;
    range=0.0;
    prev_range=0.0;
}

/********************************************************/
void Rom_Processor::estimate()
{
    Vector v1(3,0.0);
    Vector ref_dir(3,0.0);

    //get reference keypoint from skeleton
    curr_skeleton.normalize();
    string tag_joint=rom->getTagJoint();
    if(curr_skeleton[tag_joint]->isUpdated() && curr_skeleton[tag_joint]->getChild(0)->isUpdated())
    {
        double theta;
        if(curr_skeleton[tag_joint]->getNumChild())
        {
            if(rom->getTagPlane()=="coronal")
            {
                plane_normal=curr_skeleton.getCoronal();
            }
            else if(rom->getTagPlane()=="sagittal")
            {
                plane_normal=curr_skeleton.getSagittal();
            }
            else if(rom->getTagPlane()=="transverse")
            {
                plane_normal=curr_skeleton.getTransverse();
            }
            Vector k1=toCurrFrame(tag_joint);
            Vector k2=toCurrFrame(curr_skeleton[tag_joint]->getChild(0)->getTag());
            v1=k2-k1;
            v1=projectOnPlane(v1,plane_normal);
            double n1=norm(v1);
            if(n1>0.0)
                v1/=n1;
            if(!rom->getRefJoint().empty())
            {
                Vector k_dir=toCurrFrame(rom->getRefJoint());
                ref_dir=k1-k_dir;
            }
            else
            {
                ref_dir=rom->getRefDir();
            }
            double n2=norm(ref_dir);
            double dot_p=dot(v1,ref_dir);
            theta=acos(dot_p/n2);
            range=theta*(180/M_PI);
            prev_range=range;
        }
        else
        {
            yError() << "The keypoint does not have a child ";
            range=0.0;
        }
    }
    else
    {
        range=prev_range;
    }
}

/********************************************************/
yarp::os::Property Rom_Processor::getResult()
{
    Property result;
    vector<string> props=rom->getProperties();
    result.put(props[0],range);
    return result;
}

/********************************************************/
void Rom_Processor::reset()
{
    range=0.0;
    prev_range=0.0;
}

/****************************************/
/*            STEP PROCESSOR            */
/****************************************/
Step_Processor::Step_Processor()
{

}

/********************************************************/
Step_Processor::Step_Processor(const Metric* step_)
{
    step=(Step*)step_;
    filter_dist=new MedianFilter(step->getFilterWindow(), yarp::sig::Vector(1, 0.0));
    filter_width=new MedianFilter(step->getFilterWindow(), yarp::sig::Vector(1, 0.0));
    step_thresh=step->getThresh();
    step_window=step->getStepWindow();
    time_window=step->getTimeWindow();

    steplen=0.0;
    stepwidth=0.0;
    cadence=0.0;
    speed=0.0;
    numsteps=0;
    steplen_raw=0.0;
    stepwidth_raw=0.0;
    
    prev_steplen=0.0;
    prev_stepwidth=0.0;
    prev_cadence=0.0;
    prev_speed=0.0;
    prev_steplen_raw=0.0;
    prev_stepwidth_raw=0.0;
}

/********************************************************/
void Step_Processor::estimate()
{
    if(curr_skeleton[KeyPointTag::ankle_left]->isUpdated() &&
            curr_skeleton[KeyPointTag::ankle_right]->isUpdated())
    {
        updateCurrentFrame(curr_skeleton);
        Vector k1=toCurrFrame(KeyPointTag::ankle_left);
        Vector k2=toCurrFrame(KeyPointTag::ankle_right);

        Vector v=k1-k2;        

        Vector sag_plane=curr_skeleton.getSagittal();
        Vector v_steplen=projectOnPlane(v,sag_plane);
        double d1=norm(v_steplen);

        Vector cor_plane=curr_skeleton.getCoronal();
        Vector v_stepwidth=projectOnPlane(v,cor_plane);
        double d2=norm(v_stepwidth);

        yDebug() << "vlen: " << v_steplen.toString() << "vwidth" << v_stepwidth.toString();

        // double d1=abs(k1[1]-k2[1]);        
        // double d2=abs(k1[0]-k2[0]);
        
        estimateSpatialParams(d1,d2);
        cadence=estimateCadence();
        speed=estimateSpeed();

        prev_steplen=steplen;
        prev_stepwidth=stepwidth;
        prev_cadence=cadence;
        prev_speed=speed;
        prev_steplen_raw=steplen_raw;
        prev_stepwidth_raw=stepwidth_raw;
    }
    else
    {
        steplen=prev_steplen;
        stepwidth=prev_stepwidth;
        cadence=prev_cadence;
        speed=prev_speed;
        steplen_raw=prev_steplen_raw;
        stepwidth_raw=prev_stepwidth_raw;
    }
//    yInfo()<<"Step parameters:"<<Time::now()-t0<<steplen<<stepwidth<<cadence<<speed<<numsteps;
}

/********************************************************/
Property Step_Processor::getResult()
{
    Property result;
    vector<string> props=step->getProperties();
    result.put(props[0],steplen);
    result.put(props[1],stepwidth);
    result.put(props[2],cadence);
    result.put(props[3],speed);
    result.put(props[4],numsteps);
    result.put(props[5],steplen_raw);
    result.put(props[6],stepwidth_raw);
    return result;
}

/********************************************************/
void Step_Processor::estimateSpatialParams(const double &dist,const double &width)
{
    Vector u1({dist}),u2({width});
    Vector output1=filter_dist->filt(u1);
    Vector output2=filter_width->filt(u2);
    feetdist.push_back(output1[0]);
    feetwidth.push_back(output2[0]);
    tdist.push_back(Time::now());

    pair<deque<double>,deque<int>> step_params;
    double tlast=0.0;
    step_params=findPeaks(feetdist,step_thresh);
    deque<double> stepvec=step_params.first;
    deque<int> strikes=step_params.second;
    if (strikes.size()>0)
    {
        tlast=tdist[strikes.back()];
    }

    steplen_raw = output1[0];
    stepwidth_raw = output2[0];
    steplen=0.0;
    stepwidth=0.0;
    if ((Time::now()-tlast)<=time_window
         //steplen_raw > step_thresh
         )
    {

        for(int i=0;i<stepvec.size();i++)
        {
            steplen+=stepvec[i];
        }
        if (stepvec.size()>0)
        {
            steplen/=stepvec.size();
        }
        int laststrike=stepvec.size()>numsteps ? strikes.back() : 0;
       //steplen = stepvec.back();
        stepwidth=feetwidth[laststrike];
        numsteps=(int)strikes.size();
    }
}

/********************************************************/
double Step_Processor::estimateCadence()
{
    return numsteps/(Time::now()-t0);
}

/********************************************************/
double Step_Processor::estimateSpeed()
{
    return steplen*cadence;
}

/********************************************************/
pair<deque<double>,deque<int>> Step_Processor::findPeaks(const Vector &d, const double &minv)
{
    deque<double> val;
    deque<int> idx;
    for(int i=0; i<d.size(); i++)
    {
        if (d[i]<=minv)
        {
            continue;
        }
        if(i==0 && d[i]>=d[i+1])
        {
            val.push_back(d[i]);
            idx.push_back(i);
        }
        else if(i==d.size() && d[i]>=d[i-1])
        {
            val.push_back(d[i]);
            idx.push_back(i);
        }
	        else if (i >2 && (i<d.size()-2))
        {
			bool isMaximum = true;
			for (int j=0;j<3;j++){
				if (d[i] < d[i+j] || d[i] < d[i-j])
				{
					isMaximum = false;
				}
			}
			if (isMaximum) {
				val.push_back(d[i]);
				idx.push_back(i);
			}
        }
    }
    if (val.size()>step_window)
    {
        val.pop_front();
    }
    pair<deque<double>,deque<int>> step_params;
    step_params=make_pair(val,idx);
    return step_params;
}

/********************************************************/
void Step_Processor::reset()
{
    steplen=0.0;
    stepwidth=0.0;
    cadence=0.0;
    speed=0.0;
    numsteps=0;
    steplen_raw=0.0;
    stepwidth_raw=0.0;


    prev_steplen=0.0;
    prev_stepwidth=0.0;
    prev_cadence=0.0;
    prev_speed=0.0;
    prev_steplen_raw=0.0;
    prev_stepwidth_raw=0.0;

}

/********************************************************/
Step_Processor::~Step_Processor()
{
    delete filter_dist;
    delete filter_width;
}

/****************************************/
/*        END POINT PROCESSOR           */
/****************************************/
EndPoint_Processor::EndPoint_Processor()
{

}

/********************************************************/
EndPoint_Processor::EndPoint_Processor(const Metric *ep_)
{
    ep=(EndPoint*)ep_;
    linEst=new AWLinEstimator(16,0.5);
    jerkEst=new JerkEstimator(16,0.1);
    ideal_traj=0.0;
    prev_est_traj=0.0;
    prev_ideal_traj=0.0;
    prev_vel=0.0;
    prev_smoothness=0.0;
}

/********************************************************/
EndPoint_Processor::~EndPoint_Processor()
{
    delete linEst;
    delete jerkEst;
}

/********************************************************/
void EndPoint_Processor::estimate()
{
    if(ep->getTagPlane() == "coronal")
        plane_normal[0]=1.0;
    else if(ep->getTagPlane() == "sagittal")
        plane_normal[1]=1.0;
    else if(ep->getTagPlane() == "transverse")
        plane_normal[2]=1.0;

    curr_skeleton.normalize();
    string tag_joint = ep->getTagJoint();
    if(curr_skeleton[tag_joint]->isUpdated())
    {
        //we compute ideal and current trajectory wrt left/right shoulder
        Vector ref = first_skeleton[tag_joint]->getParent(0)->getParent(0)->getPoint();
        ref.push_back(1.0);
        Vector v = curr_skeleton[tag_joint]->getPoint();
        v.push_back(1.0);
        Vector transformed_v = curr_frame*v;
        Vector transformed_ref = curr_frame*ref;
        Vector dv = transformed_v.subVector(0,2)-transformed_ref.subVector(0,2);
        est_traj = getTrajectory(dv);

        yDebug() << first_skeleton[tag_joint]->getParent(0)->getParent(0)->getTag() << ref.toString();
        yDebug() << tag_joint << v.toString();
        cout << endl;

        Vector t=ep->getTarget();
        t.push_back(1.0);
        Vector transformed_t = curr_frame*t;
        Vector dt = transformed_t.subVector(0,2)-transformed_ref.subVector(0,2);
        ideal_traj = getTrajectory(dt);

        //we compute velocity and smoothness of the end-point
        AWPolyElement el(transformed_v,Time::now());
        vel=norm(linEst->estimate(el));
        smoothness = norm(jerkEst->estimate(el));

        prev_est_traj=est_traj;
        prev_ideal_traj=ideal_traj;
        prev_vel=vel;
        prev_smoothness=smoothness;
    }
    else
    {
        est_traj=prev_est_traj;
        ideal_traj=prev_ideal_traj;
        vel=prev_vel;
        smoothness=prev_smoothness;
    }

}

/********************************************************/
Property EndPoint_Processor::getResult()
{
    Property result;
    vector<string> props=ep->getProperties();
    result.put(props[0],est_traj);
    result.put(props[1],vel);
    result.put(props[2],smoothness);
    return result;
}

/********************************************************/
double EndPoint_Processor::getTrajectory(const Vector &v)
{
    double dist=dot(v,plane_normal);
    Vector proj=v-dist;
    double traj=norm(proj);
    return traj;
}

/********************************************************/
void EndPoint_Processor::reset()
{
    ideal_traj=0.0;
    est_traj=0.0;
    vel=0.0;
    smoothness=0.0;

    prev_est_traj=0.0;
    prev_ideal_traj=0.0;
    prev_vel=0.0;
    prev_smoothness=0.0;
}
