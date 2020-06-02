/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file Exercise.cpp
 * @authors: Valentina Vasco <valentina.vasco@iit.it>
 */

#include "Exercise.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

namespace ExerciseType
{
const string rehabilitation="rehabilitation";
const string test="test";
}

namespace ExerciseTag
{
const std::string abduction_left="abduction_left";
const std::string internal_rotation_left="internal_rotation_left";
const std::string external_rotation_left="external_rotation_left";
const std::string reaching_left="reaching_left";
const std::string tug="tug";
}

/************************/
/*       EXERCISE       */
/************************/
Exercise::Exercise()
{

}

Exercise::Exercise(const string &name_, const string &type_)
{
    name = name_;
    type = type_;
}

void Exercise::addMetric(const Metric* m)
{
    metrics.push_back(m);
}

const Metric* Exercise::getCurrMetric(const std::string &metric_tag) const
{
    const Metric* m;
    for(int i=0;i<metrics.size();i++)
    {
        string tag=metrics[i]->getParams().find("name").asString();
        if(tag==metric_tag)
        {
            m=metrics[i];
            break;
        }
    }
    return m;
}

vector<string> Exercise::listMetrics() const
{
    vector<string> m(metrics.size());
    for(int i=0;i<metrics.size();i++)
    {
        m[i]=metrics[i]->getParams().find("name").asString();
    }

    return m;
}

void Exercise::print(ostream &os)
{
    os<<"name= "<<name<<endl;
    os<<"type= "<<type<<endl;
    for(int i=0; i<metrics.size(); i++)
    {
        os<<"metric= "<<endl;
        metrics[i]->print();
    }
}

Exercise::~Exercise()
{
    for(int i=0; i<metrics.size(); i++)
    {
        delete metrics[i];
    }
}

/*****************************************/
/*      Range of Motion Exercises        */
/*****************************************/
RangeOfMotion::RangeOfMotion(const string &name_)
{
    name=name_;
    type=ExerciseType::rehabilitation;
}

void RangeOfMotion::setFeedbackParams(const Property &p)
{
    feedparams=p;
    duration=feedparams.find("duration").asInt();
    twarp=feedparams.find("twarp").asDouble();
    Bottle t=feedparams.findGroup("thresh");
    Bottle jnt=t.findGroup("joint");
    Bottle sx=t.findGroup("sx");
    Bottle sy=t.findGroup("sy");
    Bottle sz=t.findGroup("sz");
    Bottle freq=t.findGroup("freq");
    Bottle psd=t.findGroup("psd");

    for(int i=0;i<jnt.size()-1;i++)
    {
        Bottle *t1=jnt.get(i+1).asList();
        joint_list.push_back(t1->get(1).asString());

        Bottle *t2=sx.get(i+1).asList();
        sx_thresh.push_back(t2->get(1).asDouble());

        Bottle *t3=sy.get(i+1).asList();
        sy_thresh.push_back(t3->get(1).asDouble());

        Bottle *t4=sz.get(i+1).asList();
        sz_thresh.push_back(t4->get(1).asDouble());

        Bottle *t6=freq.get(i+1).asList();
        range_freq.push_back(t6->get(1).asDouble());

        Bottle *t7=psd.get(i+1).asList();
        psd_thresh.push_back(t7->get(1).asDouble());
    }
}

Matrix RangeOfMotion::getFeedbackThresholds()
{
    feedbackMat.resize(5,joint_list.size());
    for(size_t i=0; i<joint_list.size(); i++)
    {
        feedbackMat[0][i]=sx_thresh[i];
        feedbackMat[1][i]=sy_thresh[i];
        feedbackMat[2][i]=sz_thresh[i];
        feedbackMat[3][i]=range_freq[i];
        feedbackMat[4][i]=psd_thresh[i];
    }
    return feedbackMat;
}

///************************/
///*      ABDUCTION       */
///************************/
//AbductionLeft::AbductionLeft()
//{
//    name=ExerciseTag::abduction_left;
//    type=ExerciseType::rehabilitation;
//}

//void AbductionLeft::setFeedbackParams(const Property &p)
//{
//    feedparams=p;
//    duration=feedparams.find("duration").asInt();
//    twarp=feedparams.find("twarp").asDouble();
//    Bottle t=feedparams.findGroup("thresh");
//    Bottle jnt=t.findGroup("joint");
//    Bottle sx=t.findGroup("sx");
//    Bottle sy=t.findGroup("sy");
//    Bottle sz=t.findGroup("sz");
//    Bottle freq=t.findGroup("freq");
//    Bottle psd=t.findGroup("psd");

//    for(int i=0;i<jnt.size()-1;i++)
//    {
//        Bottle *t1=jnt.get(i+1).asList();
//        joint_list.push_back(t1->get(1).asString());

//        Bottle *t2=sx.get(i+1).asList();
//        sx_thresh.push_back(t2->get(1).asDouble());

//        Bottle *t3=sy.get(i+1).asList();
//        sy_thresh.push_back(t3->get(1).asDouble());

//        Bottle *t4=sz.get(i+1).asList();
//        sz_thresh.push_back(t4->get(1).asDouble());

//        Bottle *t6=freq.get(i+1).asList();
//        range_freq.push_back(t6->get(1).asDouble());

//        Bottle *t7=psd.get(i+1).asList();
//        psd_thresh.push_back(t7->get(1).asDouble());
//    }
//}

//Matrix AbductionLeft::getFeedbackThresholds()
//{
//    feedbackMat.resize(5,joint_list.size());
//    for(size_t i=0; i<joint_list.size(); i++)
//    {
//        feedbackMat[0][i]=sx_thresh[i];
//        feedbackMat[1][i]=sy_thresh[i];
//        feedbackMat[2][i]=sz_thresh[i];
//        feedbackMat[3][i]=range_freq[i];
//        feedbackMat[4][i]=psd_thresh[i];
//    }
//    return feedbackMat;
//}

///************************/
///*     INTERNAL ROT     */
///************************/
//InternalRotationLeft::InternalRotationLeft()
//{
//    name=ExerciseTag::internal_rotation_left;
//    type=ExerciseType::rehabilitation;
//}

//void InternalRotationLeft::setFeedbackParams(const Property &p)
//{
//    feedparams=p;
//    duration=feedparams.find("duration").asInt();
//    twarp=feedparams.find("twarp").asDouble();
//    Bottle t=feedparams.findGroup("thresh");
//    Bottle jnt=t.findGroup("joint");
//    Bottle sx=t.findGroup("sx");
//    Bottle sy=t.findGroup("sy");
//    Bottle sz=t.findGroup("sz");
//    Bottle freq=t.findGroup("freq");
//    Bottle psd=t.findGroup("psd");

//    for(int i=0;i<jnt.size()-1;i++)
//    {
//        Bottle *t1=jnt.get(i+1).asList();
//        joint_list.push_back(t1->get(1).asString());

//        Bottle *t2=sx.get(i+1).asList();
//        sx_thresh.push_back(t2->get(1).asDouble());

//        Bottle *t3=sy.get(i+1).asList();
//        sy_thresh.push_back(t3->get(1).asDouble());

//        Bottle *t4=sz.get(i+1).asList();
//        sz_thresh.push_back(t4->get(1).asDouble());

//        Bottle *t6=freq.get(i+1).asList();
//        range_freq.push_back(t6->get(1).asDouble());

//        Bottle *t7=psd.get(i+1).asList();
//        psd_thresh.push_back(t7->get(1).asDouble());
//    }
//}

//Matrix InternalRotationLeft::getFeedbackThresholds()
//{
//    feedbackMat.resize(5,joint_list.size());
//    for(size_t i=0; i<joint_list.size(); i++)
//    {
//        feedbackMat[0][i]=sx_thresh[i];
//        feedbackMat[1][i]=sy_thresh[i];
//        feedbackMat[2][i]=sz_thresh[i];
//        feedbackMat[3][i]=range_freq[i];
//        feedbackMat[4][i]=psd_thresh[i];
//    }
//    return feedbackMat;
//}

///************************/
///*     EXTERNAL ROT     */
///************************/
//ExternalRotationLeft::ExternalRotationLeft()
//{
//    name=ExerciseTag::external_rotation_left;
//    type=ExerciseType::rehabilitation;
//}

//void ExternalRotationLeft::setFeedbackParams(const Property &p)
//{
//    feedparams=p;
//    duration=feedparams.find("duration").asInt();
//    twarp=feedparams.find("twarp").asDouble();
//    Bottle t=feedparams.findGroup("thresh");
//    Bottle jnt=t.findGroup("joint");
//    Bottle sx=t.findGroup("sx");
//    Bottle sy=t.findGroup("sy");
//    Bottle sz=t.findGroup("sz");
//    Bottle freq=t.findGroup("freq");
//    Bottle psd=t.findGroup("psd");

//    for(int i=0;i<jnt.size()-1;i++)
//    {
//        Bottle *t1=jnt.get(i+1).asList();
//        joint_list.push_back(t1->get(1).asString());

//        Bottle *t2=sx.get(i+1).asList();
//        sx_thresh.push_back(t2->get(1).asDouble());

//        Bottle *t3=sy.get(i+1).asList();
//        sy_thresh.push_back(t3->get(1).asDouble());

//        Bottle *t4=sz.get(i+1).asList();
//        sz_thresh.push_back(t4->get(1).asDouble());

//        Bottle *t6=freq.get(i+1).asList();
//        range_freq.push_back(t6->get(1).asDouble());

//        Bottle *t7=psd.get(i+1).asList();
//        psd_thresh.push_back(t7->get(1).asDouble());
//    }
//}

//Matrix ExternalRotationLeft::getFeedbackThresholds()
//{
//    feedbackMat.resize(5,joint_list.size());
//    for(size_t i=0; i<joint_list.size(); i++)
//    {
//        feedbackMat[0][i]=sx_thresh[i];
//        feedbackMat[1][i]=sy_thresh[i];
//        feedbackMat[2][i]=sz_thresh[i];
//        feedbackMat[3][i]=range_freq[i];
//        feedbackMat[4][i]=psd_thresh[i];
//    }
//    return feedbackMat;
//}

/************************/
/*      REACHING        */
/************************/
ReachingLeft::ReachingLeft()
{
    name=ExerciseTag::reaching_left;
    type=ExerciseType::rehabilitation;
}

Matrix ReachingLeft::getFeedbackThresholds()
{
    feedbackMat.resize(3,joint_list.size());
    for(size_t i=0; i<joint_list.size(); i++)
    {
        feedbackMat[0][i]=radius[i];
        feedbackMat[1][i]=zscore_thresh[i];
        feedbackMat[2][i]=inliers_thresh[i];
    }
    return feedbackMat;
}

void ReachingLeft::setFeedbackParams(const Property &p)
{
    feedparams=p;
    duration=feedparams.find("duration").asInt();
    twarp=feedparams.find("twarp").asDouble();
    Bottle t=feedparams.findGroup("thresh");
    Bottle jnt=t.findGroup("joint");
    Bottle rad=t.findGroup("radius");
    Bottle score=t.findGroup("zscore_thresh");
    Bottle inl=t.findGroup("inliers_thresh");
    for(int i=0;i<jnt.size()-1;i++)
    {
        Bottle *t1=jnt.get(i+1).asList();
        joint_list.push_back(t1->get(1).asString());

        Bottle *t2=rad.get(i+1).asList();
        radius.push_back(t2->get(1).asDouble());

        Bottle *t3=score.get(i+1).asList();
        zscore_thresh.push_back(t3->get(1).asDouble());

        Bottle *t4=inl.get(i+1).asList();
        inliers_thresh.push_back(t4->get(1).asDouble());
    }
}

/************************/
/*         TUG          */
/************************/
Tug::Tug(const double &finishline_thresh, const double &standing_thresh, const double &distance, const double &time_high, const double &time_medium)
{
    name=ExerciseTag::tug;
    type=ExerciseType::test;
    this->finishline_thresh=finishline_thresh;
    this->standing_thresh=standing_thresh;
    this->distance=distance;
    this->time_high=time_high;
    this->time_medium=time_medium;
}

yarp::os::Property Tug::publish()
{
    Property p;
//    Property &p_group=p.addGroup("exercise");
    p.put("name",name);
    p.put("distance",distance);
    p.put("finish-line-thresh",finishline_thresh);
    p.put("standing-thresh",standing_thresh);
    p.put("time-high",time_high);
    p.put("time-medium",time_medium);
    return p;
}

