/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file skeleton.cpp
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#include <algorithm>
#include <iostream>
#include <yarp/math/Math.h>
#include "AssistiveRehab/skeleton.h"

using namespace std;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::math;
using namespace assistive_rehab;


namespace assistive_rehab
{

namespace KeyPointTag
{
const string shoulder_center="shoulderCenter";
const string head="head";
const string shoulder_left="shoulderLeft";
const string elbow_left="elbowLeft";
const string hand_left="handLeft";
const string shoulder_right="shoulderRight";
const string elbow_right="elbowRight";
const string hand_right="handRight";
const string hip_center="hipCenter";
const string hip_left="hipLeft";
const string knee_left="kneeLeft";
const string ankle_left="ankleLeft";
const string hip_right="hipRight";
const string knee_right="kneeRight";
const string ankle_right="ankleRight";
}

}

KeyPoint::KeyPoint() : updated(false), tag(""), point(3,0.0)
{
}

KeyPoint::KeyPoint(const string &tag_, const Vector &point_, const bool updated_) :
   updated(updated_), tag(tag_), point(point_)
{
}

bool KeyPoint::setPoint(const Vector &point)
{
    unsigned int len=(unsigned int)std::min(this->point.length(),point.length());
    if (len>0)
    {
        this->point.setSubvector(0,point.subVector(0,len-1));
        updated=true;
        return true;
    }
    else
        return false;
}

const KeyPoint *KeyPoint::getParent(const unsigned int i) const
{
    return ((i>=0) && (i<parent.size()))?parent[i]:nullptr;
}

const KeyPoint *KeyPoint::getChild(const unsigned int i) const
{
    return ((i>=0) && (i<child.size()))?child[i]:nullptr;
}

Skeleton::Skeleton()
{
    tag="";
    T=eye(4,4);
}

Skeleton::~Skeleton()
{
    for (auto &k:keypoints)
        delete k;
}

Property Skeleton::helper_toproperty(KeyPoint* k)
{
    Property prop;
    if (k!=nullptr)
    {
        Bottle position;
        position.addList().read(k->point);

        prop.put("tag",k->getTag());
        prop.put("status",k->isUpdated()?"updated":"stale");
        prop.put("position",position.get(0));
        
        if (k->child.size()>0)
        {
            Bottle child;
            Bottle &child_=child.addList();
            for (auto &c:k->child)
            {
                Property p=helper_toproperty(c);
                Bottle b; b.addList().read(p);
                child_.append(b);
            }
            prop.put("child",child.get(0));
        }
    }

    return prop;
}

void Skeleton::helper_fromproperty(Bottle *prop, KeyPoint *parent)
{
    if (prop!=nullptr)
    {
        for (int i=0; i<prop->size(); i++)
        {
            Bottle *b=prop->get(i).asList();
            string tag=b->check("tag",Value("")).asString();
            bool updated=(b->check("status",Value("stale")).asString()=="updated");

            Vector point(3,0.0);
            if (Bottle *p=b->find("position").asList())
            {
                point.resize(p->size());
                for (int j=0; j<p->size(); j++)
                    point[j]=p->get(j).asDouble();
            }

            KeyPoint *k=new KeyPoint(tag,point,updated);
            tag2key[tag]=k;
            keypoints.push_back(k);
            key2id[k]=(unsigned int)keypoints.size()-1;
            if (parent!=nullptr)
            {
                k->parent.push_back(parent);
                parent->child.push_back(k);
            }

            helper_fromproperty(b->find("child").asList(),k);
        }
    }
}

void Skeleton::helper_normalize(KeyPoint* k, const vector<Vector> &helperpoints)
{
    if (k!=nullptr)
    {
        for (auto &c:k->child)
        {
            Vector dir=helperpoints[key2id[c]]-helperpoints[key2id[k]];
            double n=norm(dir);
            if (n>0.0)
                dir/=norm(dir);
            c->point=k->point+dir;
            helper_normalize(c,helperpoints);
        }
    }
}

bool Skeleton::setTransformation(const Matrix &T)
{
    if ((T.rows()>=4) || (T.cols()>=4))
    {
        this->T=T;
        return true;
    }
    else
        return false;
}

Property Skeleton::toProperty()
{
    Property prop;
    prop.put("tag",tag);

    Bottle transformation;
    transformation.addList().read(T);
    prop.put("transformation",transformation.get(0));

    Bottle skeleton;
    Bottle &skeleton_=skeleton.addList();
    if (keypoints.size()>0)
    {
        Property p=helper_toproperty(keypoints[0]);
        skeleton_.addList().read(p);
    }
    prop.put("skeleton",skeleton.get(0));

    return prop;
}

void Skeleton::fromProperty(const Property &prop)
{
    for (auto &k:keypoints)
        delete k;

    keypoints.clear();
    tag2key.clear();
    key2id.clear();

    tag=prop.check("tag",Value("")).asString();
    if (prop.check("transformation"))
    {
        if (Bottle *b=prop.find("transformation").asList())
            b->write(T);
    }
    else
        T=eye(3,3);
    helper_fromproperty(prop.find("skeleton").asList(),nullptr);
}

const KeyPoint *Skeleton::operator [](const string &tag) const
{
    auto it=tag2key.find(tag);
    return (it!=tag2key.end())?it->second:nullptr;
}

const KeyPoint *Skeleton::operator [](const unsigned int i) const
{
    return (i<keypoints.size())?keypoints[i]:nullptr;
}

vector<Vector> Skeleton::get_ordered() const
{
    vector<Vector> ordered;
    for (auto &k:keypoints)
        ordered.push_back(k->getPoint());
    return ordered;
}

vector<pair<string, Vector>> Skeleton::get_unordered() const
{
    vector<pair<string, Vector>> unordered;
    for (auto &it:tag2key)
        unordered.push_back(make_pair(it.first,it.second->getPoint()));
    return unordered;
}

void Skeleton::normalize()
{
    if (keypoints.size()>0)
    {
        vector<Vector> helperpoints;
        for (auto &k:keypoints)
            helperpoints.push_back(k->getPoint());
        helper_normalize(keypoints[0],helperpoints);
    }
}

void Skeleton::print() const
{
    cout<<"tag = \""<<tag<<"\""<<endl;
    cout<<"transformation"<<endl<<T.toString(3,3)<<endl;
    for (auto &k:keypoints)
    {
        cout<<"keypoint[\""<<k->getTag()<<"\"] = ("
            <<k->getPoint().toString(3,3)<<"); status="
            <<(k->isUpdated()?"updated":"stale")
            <<"; parent={";
            for (auto &p:k->parent) cout<<"\""<<p->getTag()<<"\" ";
            cout<<"}; child={";
            for (auto &c:k->child) cout<<"\""<<c->getTag()<<"\" ";
            cout<<"}"<<endl;
    }
}

SkeletonStd::SkeletonStd()
{
    tag2key[KeyPointTag::shoulder_center]=new KeyPoint(KeyPointTag::shoulder_center);
    tag2key[KeyPointTag::head]=new KeyPoint(KeyPointTag::head);
    tag2key[KeyPointTag::shoulder_left]=new KeyPoint(KeyPointTag::shoulder_left);
    tag2key[KeyPointTag::elbow_left]=new KeyPoint(KeyPointTag::elbow_left);
    tag2key[KeyPointTag::hand_left]=new KeyPoint(KeyPointTag::hand_left);
    tag2key[KeyPointTag::shoulder_right]=new KeyPoint(KeyPointTag::shoulder_right);
    tag2key[KeyPointTag::elbow_right]=new KeyPoint(KeyPointTag::elbow_right);
    tag2key[KeyPointTag::hand_right]=new KeyPoint(KeyPointTag::hand_right);
    tag2key[KeyPointTag::hip_left]=new KeyPoint(KeyPointTag::hip_left);
    tag2key[KeyPointTag::knee_left]=new KeyPoint(KeyPointTag::knee_left);
    tag2key[KeyPointTag::ankle_left]=new KeyPoint(KeyPointTag::ankle_left);
    tag2key[KeyPointTag::hip_right]=new KeyPoint(KeyPointTag::hip_right);
    tag2key[KeyPointTag::knee_right]=new KeyPoint(KeyPointTag::knee_right);
    tag2key[KeyPointTag::ankle_right]=new KeyPoint(KeyPointTag::ankle_right);

    keypoints.push_back(tag2key[KeyPointTag::shoulder_center]);
    keypoints.push_back(tag2key[KeyPointTag::head]);
    keypoints.push_back(tag2key[KeyPointTag::shoulder_left]);
    keypoints.push_back(tag2key[KeyPointTag::elbow_left]);
    keypoints.push_back(tag2key[KeyPointTag::hand_left]);
    keypoints.push_back(tag2key[KeyPointTag::shoulder_right]);
    keypoints.push_back(tag2key[KeyPointTag::elbow_right]);
    keypoints.push_back(tag2key[KeyPointTag::hand_right]);
    keypoints.push_back(tag2key[KeyPointTag::hip_left]);
    keypoints.push_back(tag2key[KeyPointTag::knee_left]);
    keypoints.push_back(tag2key[KeyPointTag::ankle_left]);
    keypoints.push_back(tag2key[KeyPointTag::hip_right]);
    keypoints.push_back(tag2key[KeyPointTag::knee_right]);
    keypoints.push_back(tag2key[KeyPointTag::ankle_right]);

    unsigned int id=0;
    for (auto &k:keypoints)
        key2id[k]=id++;

    // shoulderCenter
    tag2key[KeyPointTag::shoulder_center]->child.push_back(tag2key[KeyPointTag::head]);
    tag2key[KeyPointTag::shoulder_center]->child.push_back(tag2key[KeyPointTag::shoulder_left]);
    tag2key[KeyPointTag::shoulder_center]->child.push_back(tag2key[KeyPointTag::shoulder_right]);
    tag2key[KeyPointTag::shoulder_center]->child.push_back(tag2key[KeyPointTag::hip_left]);
    tag2key[KeyPointTag::shoulder_center]->child.push_back(tag2key[KeyPointTag::hip_right]);

    // head
    tag2key[KeyPointTag::head]->parent.push_back(tag2key[KeyPointTag::shoulder_center]);

    // shoulderLeft
    tag2key[KeyPointTag::shoulder_left]->parent.push_back(tag2key[KeyPointTag::shoulder_center]);
    tag2key[KeyPointTag::shoulder_left]->child.push_back(tag2key[KeyPointTag::elbow_left]);

    // elbowLeft
    tag2key[KeyPointTag::elbow_left]->parent.push_back(tag2key[KeyPointTag::shoulder_left]);
    tag2key[KeyPointTag::elbow_left]->child.push_back(tag2key[KeyPointTag::hand_left]);

    // handLeft
    tag2key[KeyPointTag::hand_left]->parent.push_back(tag2key[KeyPointTag::elbow_left]);

    // shoulderRight
    tag2key[KeyPointTag::shoulder_right]->parent.push_back(tag2key[KeyPointTag::shoulder_center]);
    tag2key[KeyPointTag::shoulder_right]->child.push_back(tag2key[KeyPointTag::elbow_right]);

    // elbowRight
    tag2key[KeyPointTag::elbow_right]->parent.push_back(tag2key[KeyPointTag::shoulder_right]);
    tag2key[KeyPointTag::elbow_right]->child.push_back(tag2key[KeyPointTag::hand_right]);

    // handRight
    tag2key[KeyPointTag::hand_right]->parent.push_back(tag2key[KeyPointTag::elbow_right]);

    // hipLeft
    tag2key[KeyPointTag::hip_left]->parent.push_back(tag2key[KeyPointTag::shoulder_center]);
    tag2key[KeyPointTag::hip_left]->child.push_back(tag2key[KeyPointTag::knee_left]);

    // kneeLeft
    tag2key[KeyPointTag::knee_left]->parent.push_back(tag2key[KeyPointTag::hip_left]);
    tag2key[KeyPointTag::knee_left]->child.push_back(tag2key[KeyPointTag::ankle_left]);

    // ankleLeft
    tag2key[KeyPointTag::ankle_left]->parent.push_back(tag2key[KeyPointTag::knee_left]);

    // hipRight
    tag2key[KeyPointTag::hip_right]->parent.push_back(tag2key[KeyPointTag::shoulder_center]);
    tag2key[KeyPointTag::hip_right]->child.push_back(tag2key[KeyPointTag::knee_right]);

    // kneeRight
    tag2key[KeyPointTag::knee_right]->parent.push_back(tag2key[KeyPointTag::hip_right]);
    tag2key[KeyPointTag::knee_right]->child.push_back(tag2key[KeyPointTag::ankle_right]);

    // ankleRight
    tag2key[KeyPointTag::ankle_right]->parent.push_back(tag2key[KeyPointTag::knee_right]);
}

void SkeletonStd::update(const vector<Vector> &ordered)
{
    Vector p(4,1);
    unsigned int i=0;
    for (auto &k:keypoints)
    {
        k->stale();
        if (i<ordered.size())
        {
            auto &v=ordered[i];
            p[0]=v[0];
            p[1]=v[1];
            p[2]=v[2];
            k->setPoint((T*p).subVector(0,2));
        }
        i++;
    }
}

void SkeletonStd::update(const vector<pair<string, Vector>> &unordered)
{
    for (auto &k:keypoints)
        k->stale();

    Vector p(4,1);
    for (auto &it1:unordered)
    {
        auto it2=tag2key.find(get<0>(it1));
        if (it2!=tag2key.end())
        {
            auto &v=get<1>(it1);
            p[0]=v[0];
            p[1]=v[1];
            p[2]=v[2];
            it2->second->setPoint((T*p).subVector(0,2));
        }
    }
}

SkeletonWaist::SkeletonWaist() : SkeletonStd()
{
    waist_pos=7;
    tag2key[KeyPointTag::hip_center]=new KeyPoint(KeyPointTag::hip_center);
    keypoints.insert(keypoints.begin()+waist_pos+1,tag2key[KeyPointTag::hip_center]);

    key2id.clear();
    unsigned int id=0;
    for (auto &k:keypoints)
        key2id[k]=id++;

    // shoulderCenter
    tag2key[KeyPointTag::shoulder_center]->child.pop_back();
    tag2key[KeyPointTag::shoulder_center]->child.pop_back();
    tag2key[KeyPointTag::shoulder_center]->child.push_back(tag2key[KeyPointTag::hip_center]);

    // hipCenter
    tag2key[KeyPointTag::hip_center]->parent.push_back(tag2key[KeyPointTag::shoulder_center]);
    tag2key[KeyPointTag::hip_center]->child.push_back(tag2key[KeyPointTag::hip_left]);
    tag2key[KeyPointTag::hip_center]->child.push_back(tag2key[KeyPointTag::hip_right]);

    // hipLeft
    tag2key[KeyPointTag::hip_left]->parent[0]=tag2key[KeyPointTag::hip_center];

    // hipRight
    tag2key[KeyPointTag::hip_right]->parent[0]=tag2key[KeyPointTag::hip_center];
}

void SkeletonWaist::update_fromstd(const vector<Vector> &ordered)
{
    Vector p(4,1);
    unsigned int i=0;
    for (auto &k:keypoints)
    {
        k->stale();
        if (i<ordered.size())
        {
            unsigned int pos=(i<waist_pos)?i:i+1;
            auto &v=ordered[pos];
            p[0]=v[0];
            p[1]=v[1];
            p[2]=v[2];
            k->setPoint((T*p).subVector(0,2));
        }
        i++;
    }

    if (tag2key[KeyPointTag::hip_left]->isUpdated() || tag2key[KeyPointTag::hip_right]->isUpdated())
        tag2key[KeyPointTag::hip_center]->setPoint(0.5*(tag2key[KeyPointTag::hip_left]->getPoint()+
                                                        tag2key[KeyPointTag::hip_right]->getPoint()));
}

void SkeletonWaist::update_fromstd(const vector<pair<string, Vector>> &unordered)
{
    update(unordered);
    if (tag2key[KeyPointTag::hip_left]->isUpdated() || tag2key[KeyPointTag::hip_right]->isUpdated())
        tag2key[KeyPointTag::hip_center]->setPoint(0.5*(tag2key[KeyPointTag::hip_left]->getPoint()+
                                                        tag2key[KeyPointTag::hip_right]->getPoint()));
}

