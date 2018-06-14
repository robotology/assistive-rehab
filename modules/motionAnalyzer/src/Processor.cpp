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

#include "algorithm"
#include "Processor.h"

using namespace std;
using namespace yarp::math;
using namespace assistive_rehab;

const string Rom_Processor::motion_type = "ROM";

Processor* createProcessor(const string& motion_tag, const Metric* metric_)
{
    if(motion_tag.compare(Rom_Processor::motion_type) >= 0)
    {
        yInfo() << "Creating processor for" << Rom_Processor::motion_type << "\n";
        return new Rom_Processor(metric_);
    }
    else
        return 0;
}

/********************************************************/
Processor::Processor()
{
    deviation = 0.0;
    invT.resize(4,4);
    invT.zero();

//    devvector.resize(100);
//    for(int i=0;i<100;i++)
//        devvector[i].resize(14);
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

void Processor::setInitialConf(SkeletonWaist* skeleton_init_, const map<string, pair<string, double> > &keypoints2conf_,
                               SkeletonWaist &skeleton)
{
    skeleton_init = skeleton_init_;
    keypoints2conf = keypoints2conf_;

    if(!skeleton.update_planes())
        yError() << "Not all planes are updated";

    coronal = skeleton.getCoronal();
    sagittal = skeleton.getSagittal();
    transverse = skeleton.getTransverse();
    Vector p = skeleton[KeyPointTag::shoulder_center]->getPoint();
    Matrix T1(4,4);
    T1.setSubcol(coronal,0,0);
    T1.setSubcol(sagittal,0,1);
    T1.setSubcol(transverse,0,2);
    T1.setSubcol(p,0,3);
    T1(3,3)=1.0;

    inv_reference_system = SE3inv(T1);

 //   skeleton_init.print();
}

bool Processor::isStatic(const KeyPoint& keypoint)
{
    if(keypoints2conf[keypoint.getTag()].first == "static")
        return true;
    else
        return false;
}

/****************************************************************/
void Processor::update(SkeletonWaist &curr_skeleton_, SkeletonWaist &template_skeleton_)
{
    curr_skeleton.update(curr_skeleton_.toProperty());
    curr_skeleton.normalize();

    template_skeleton.update(template_skeleton_.toProperty());
    template_skeleton.normalize();
}

void Processor::checkDeviation()
{
    vector< pair <string,Vector> > row;
    for(unsigned int i=0; i<curr_skeleton.getNumKeyPoints(); i++)
    {
        if(curr_skeleton[i]->isUpdated() && curr_skeleton[i]->getTag() != KeyPointTag::hip_center)
        {
            Vector k1=curr_skeleton[i]->getPoint();
            k1.push_back(1.0);
            Vector transformed_kp=(inv_reference_system*k1).subVector(0,2);

            Vector k2=template_skeleton[i]->getPoint();
            k2.push_back(1.0);
            Vector transformed_kp_template=(inv_reference_system*k2).subVector(0,2);

            Vector dev=transformed_kp-transformed_kp_template;
            row.push_back(make_pair(curr_skeleton[i]->getTag(),dev));
        }
        else
        {
            Vector v(3,0.0);
            row.push_back(make_pair(curr_skeleton[i]->getTag(),v));
        }
    }
    devvector.push_back(row);

//    for(int i=0; i<devvector.size(); i++)
//    {
//        for(int j=0; j<devvector[i].size(); j++)
//            cout << i << " " << j << " " << devvector[i][j].first
//                 << " " << devvector[i][j].second.toString(3,3) << "\n";
//        cout << "\n";
//    }

    if(devvector.size()>100)
        devvector.erase(devvector.begin());
}

vector< pair<string,vector<string>> > Processor::getFeedback()
{
    checkDeviation();

    vector< pair<string,vector<string>> > feedback;
    for(int j=0; j<15; j++)
    {
        Vector avg_dev(3,0.0);
        for(int i=0; i<devvector.size(); i++)
        {
            avg_dev[0]+=devvector[i][j].second[0];
            avg_dev[1]+=devvector[i][j].second[1];
            avg_dev[2]+=devvector[i][j].second[2];
        }
        avg_dev/=devvector.size();
        string tag=devvector[0][j].first;
//        yInfo() << tag << norm(avg_dev);

        if(norm(avg_dev) > 3.0) // keypoints2conf[tag].second)
        {
            vector<double> err;
            double errx = dot(avg_dev,coronal);
            double erry = dot(avg_dev,sagittal);
            double errz = dot(avg_dev,transverse);
            err.push_back(fabs(errx));
            err.push_back(fabs(erry));
            err.push_back(fabs(errz));
            sort(err.begin(),err.end());

            vector<string> direction(3,"");
            if(err[0] > 2.0)
            {
                if(errx > 0.0)
                    direction[0] = "forward";
                else
                    direction[0] = "backward";
            }
            if(err[1] > 2.0)
            {
                if(erry > 0.0)
                    direction[1] = "right";
                else
                    direction[1] = "left";
            }
            if(err[2] > 2.0)
            {
                if(errz > 0.0)
                    direction[2] = "up";
                else
                    direction[2] = "down";
            }
            yInfo() << tag << errx << erry << errz;

            feedback.push_back(make_pair(tag,direction));
        }
    }

    return feedback;
}

bool Processor::isDeviatingFromIntialPose()
{
    bool isDeviating = false;
    deviation = 0.0;

    for(unsigned int i=0; i<curr_skeleton.getNumKeyPoints(); i++)
    {
        if(curr_skeleton[i]->isUpdated() && curr_skeleton[i]->getTag() != KeyPointTag::hip_center)
        {
            if(isStatic(*curr_skeleton[i]))
            {
                deviation+= isDeviatingFromIntialPose(*curr_skeleton[i], *(*skeleton_init)[i]);
                isDeviating = true;
//                yWarning() << curr_skeleton[i]->getTag() << "deviating from initial pose";
            }
        }
    }
//    cout << "\n";

    return isDeviating;
}

double Processor::isDeviatingFromIntialPose(const KeyPoint& keypoint, const KeyPoint& keypoint_init)
{
    Vector k1=keypoint.getPoint();
    k1.push_back(1.0);
    Vector transformed_kp = (inv_reference_system*k1).subVector(0,2);
    double dev = norm(transformed_kp-keypoint_init.getPoint());

/*    yInfo() << keypoint.getTag()
            << dev
            << transformed_kp.toString(3,3)
            << keypoint_init.getPoint().toString(3,3); */

    if(dev > keypoints2conf[keypoint.getTag()].second)
        return dev;
    else
        return 0.0;
}

/********************************************************/
Rom_Processor::Rom_Processor()
{

}

Rom_Processor::Rom_Processor(const Metric *rom_)
{
    rom = (Rom*)rom_;
    prev_result = 0.0;
    prev_score = 0.0;
}

//void Rom_Processor::setInitialConf(const SkeletonWaist &skeleton_init_, const map<string, pair<string, double> > &keypoints2conf_)
//{
//    skeleton_init.update(skeleton_init_.get_unordered());
//    keypoints2conf = keypoints2conf_;
//}

double Rom_Processor::computeMetric(Vector &v1, Vector &plane_normal_, Vector &ref_dir, double &score_exercise)
{
    double result;
    v1.resize(3);
    plane_normal_.resize(3);
    ref_dir.resize(3);

    //get reference keypoint from skeleton
    string tag_joint = rom->getTagJoint();

    if(curr_skeleton[tag_joint]->isUpdated() && curr_skeleton[tag_joint]->getChild(0)->isUpdated())
    {
        Vector kp_ref = curr_skeleton[tag_joint]->getPoint();

        double theta;
        if(curr_skeleton[tag_joint]->getNumChild())
        {
            Vector kp_child = curr_skeleton[tag_joint]->getChild(0)->getPoint();

            int component_to_check;
            if(rom->getTagPlane() == "coronal")
            {
                Vector cor(3,0.0);
                cor[0]=1.0;
                plane_normal_ = cor;
                component_to_check = 0;
            }
            else if(rom->getTagPlane() == "sagittal")
            {
                Vector sag(3,0.0);
                sag[1]=1.0;
                plane_normal_ = sag;
                component_to_check = 1;
            }
            else if(rom->getTagPlane() == "transverse")
            {
                Vector trans(3,0.0);
                trans[2]=1.0;
                plane_normal_ = trans;
                component_to_check = 2;
            }

            ref_dir = rom->getRefDir();

            Vector k1=kp_ref;
            k1.push_back(1.0);
            Vector k2=kp_child;
            k2.push_back(1.0);
            Vector transformed_kp_ref = inv_reference_system*k1;
            Vector transformed_kp_child = inv_reference_system*k2;

            v1 = transformed_kp_child.subVector(0,2)-transformed_kp_ref.subVector(0,2);

            score_exercise = 0.7;
            //if(abs(v1[component_to_check])>rom->getRangePlane())
            //{
            //    yInfo() << "out of the plane band" << v1[component_to_check];
            //    score_exercise = 0.4;
            //}

            double dist = dot(v1,plane_normal_);
            v1 = v1-dist*plane_normal_;
            double n1 = norm(v1);
            if(n1 > 0.0)
                v1 /= n1;

            double n2 = norm(ref_dir);
            double dot_p = dot(v1,ref_dir);

            theta = acos(dot_p/n2); 
            result = theta * (180/M_PI);           
            prev_result = result;
            prev_score = score_exercise;    
        }
        else
        {
            yError() << "The keypoint does not have a child ";
            result = 0.0;
            score_exercise = 0.0;
            v1.zero();
            plane_normal_.zero();
            ref_dir.zero();
        }
    }
    else
    { 
        result = prev_result;
        score_exercise = prev_score;
        v1.zero();
        plane_normal_.zero();
        ref_dir.zero();
    }

    return result;

}
