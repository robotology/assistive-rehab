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
const string foot_left="footLeft";
const string hip_right="hipRight";
const string knee_right="kneeRight";
const string ankle_right="ankleRight";
const string foot_right="footRight";
}

namespace SkeletonType
{
const string Skeleton="assistive_rehab::Skeleton";
const string SkeletonStd="assistive_rehab::SkeletonStd";
}

}

KeyPoint::KeyPoint() : updated(false), tag(""),
                       point(3,numeric_limits<double>::quiet_NaN()),
                       pixel(2,numeric_limits<double>::quiet_NaN())
{
}

KeyPoint::KeyPoint(const string &tag_, const Vector &point_,
                   const Vector &pixel_, const bool updated_) :
   updated(updated_), tag(tag_), point(point_), pixel(pixel_)
{
}

bool KeyPoint::setPoint(const Vector &point)
{
    unsigned int len=(unsigned int)std::min(this->point.length(),point.length());
    if (len>0)
    {
        this->point.setSubvector(0,point.subVector(0,len-1));
        this->pixel=numeric_limits<double>::quiet_NaN();
        updated=true;
        return true;
    }
    else
        return false;
}

bool KeyPoint::setPoint(const Vector &point, const Vector &pixel)
{
    unsigned int len1=(unsigned int)std::min(this->point.length(),point.length());
    unsigned int len2=(unsigned int)std::min(this->pixel.length(),pixel.length());
    if ((len1>0) && (len2>0))
    {
        this->point.setSubvector(0,point.subVector(0,len1-1));
        this->pixel.setSubvector(0,pixel.subVector(0,len2-1));
        updated=true;
        return true;
    }
    else
        return false;
}

const KeyPoint* KeyPoint::getParent(const unsigned int i) const
{
    return ((i>=0) && (i<parent.size()))?parent[i]:nullptr;
}

const KeyPoint* KeyPoint::getChild(const unsigned int i) const
{
    return ((i>=0) && (i<child.size()))?child[i]:nullptr;
}

Skeleton::Skeleton()
{
    type=SkeletonType::Skeleton;
    tag="";
    T=eye(4,4);
    coronal=sagittal=transverse=zeros(3);
}

Skeleton::~Skeleton()
{
    for (auto &k:keypoints)
        delete k;
}

Property Skeleton::helper_toproperty(KeyPoint *k) const
{
    Property prop;
    if (k!=nullptr)
    {
        Bottle position; position.addList().read(k->point);
        Bottle pixel; pixel.addList().read(k->pixel);

        prop.put("tag",k->getTag());
        prop.put("status",k->isUpdated()?"updated":"stale");
        prop.put("position",position.get(0));
        prop.put("pixel",pixel.get(0));
        
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

            Vector point(3,numeric_limits<double>::quiet_NaN());
            if (Bottle *p=b->find("position").asList())
                p->write(point);

            Vector pixel(2,numeric_limits<double>::quiet_NaN());
            if (Bottle *p=b->find("pixel").asList())
                p->write(pixel);

            KeyPoint *k=new KeyPoint(tag,point,pixel,updated);
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

void Skeleton::helper_updatefromproperty(Bottle *prop)
{
    if (prop!=nullptr)
    {
        for (int i=0; i<prop->size(); i++)
        {
            Bottle *b=prop->get(i).asList();
            if (b->check("tag"))
            {
                auto it=tag2key.find(b->find("tag").asString());
                if (it!=tag2key.end())
                {
                    auto &k=it->second;
                    if (b->check("status"))
                        k->updated=(b->find("status").asString()=="updated");

                    if (Bottle *p=b->find("position").asList())
                        p->write(k->point);

                    if (Bottle *p=b->find("pixel").asList())
                        p->write(k->pixel);

                    helper_updatefromproperty(b->find("child").asList());
                }
            }
        }
    }
}

void Skeleton::helper_normalize(KeyPoint* k, const vector<Vector> &helperpoints,
                                const double n)
{
    if (k!=nullptr)
    {
        if (k->isUpdated())
        {
            for (auto &c:k->child)
            {
                if (c->isUpdated())
                {
                    Vector dir=helperpoints[key2id[c]]-helperpoints[key2id[k]];
                    double d=norm(dir);
                    if (d>0.0)
                    {
                        dir*=n/d;
                    }
                    c->point=k->point+dir;
                    helper_normalize(c,helperpoints,n);
                }
            }
        }
    }
}

void Skeleton::helper_scale(KeyPoint* k, const vector<Vector> &helperpoints,
                            const double s)
{
    if (k!=nullptr)
    {
        if (k->isUpdated())
        {
            for (auto &c:k->child)
            {
                if (c->isUpdated())
                {
                    Vector dir=s*(helperpoints[key2id[c]]-helperpoints[key2id[k]]);
                    c->point=k->point+dir;
                    helper_scale(c,helperpoints,s);
                }
            }
        }
    }
}

double Skeleton::helper_getmaxpath(KeyPoint* k, vector<bool> &visited) const
{
    Vector paths(1,0.0);
    if (k!=nullptr)
    {
        if (k->isUpdated())
        {
            auto id=key2id.find(k)->second;
            visited[id]=true;

            for (auto &p:k->parent)
            {
                if (p->isUpdated())
                {
                    auto id=key2id.find(p)->second;
                    if (!visited[id])
                    {
                        paths.push_back(norm(k->getPoint()-p->getPoint())+
                                        helper_getmaxpath(p,visited));
                    }
                }
            }
            for (auto &c:k->child)
            {
                if (c->isUpdated())
                {
                    auto id=key2id.find(c)->second;
                    if (!visited[id])
                    {
                        paths.push_back(norm(k->getPoint()-c->getPoint())+
                                        helper_getmaxpath(c,visited));
                    }
                }
            }
        }
    }
    return findMax(paths);
}

bool Skeleton::setTransformation(const Matrix &T)
{
    if ((T.rows()>=4) || (T.cols()>=4))
    {
        this->T=T.submatrix(0,3,0,3);
        return true;
    }
    else
        return false;
}

bool Skeleton::setCoronal(const Vector &coronal)
{
    if (coronal.length()>=3)
    {
        this->coronal=coronal.subVector(0,2);
        return true;
    }
    else
        return false;
}

bool Skeleton::setSagittal(const Vector &sagittal)
{
    if (sagittal.length()>=3)
    {
        this->sagittal=sagittal.subVector(0,2);
        return true;
    }
    else
        return false;
}

bool Skeleton::setTransverse(const Vector &transverse)
{
    if (transverse.length()>=3)
    {
        this->transverse=transverse.subVector(0,2);
        return true;
    }
    else
        return false;
}

Vector Skeleton::getCoronal() const
{
    return (coronal);
}

Vector Skeleton::getSagittal() const
{
    return (sagittal);
}

Vector Skeleton::getTransverse() const
{
    return (transverse);
}

double Skeleton::getMaxPath() const
{
    Vector paths(1,0.0);
    for (auto &k:keypoints)
    {
        vector<bool> visited(getNumKeyPoints(),false);
        paths.push_back(helper_getmaxpath(k,visited));
    }
    return findMax(paths);
}

Property Skeleton::toProperty()
{
    Property prop;
    prop.put("type",type);
    prop.put("tag",tag);

    Bottle transformation;
    transformation.addList().read(T);
    prop.put("transformation",transformation.get(0));

    Bottle plane;
    plane.addList().read(coronal);
    prop.put("coronal",plane.get(0));

    plane.clear();
    plane.addList().read(sagittal);
    prop.put("sagittal",plane.get(0));

    plane.clear();
    plane.addList().read(transverse);
    prop.put("transverse",plane.get(0));

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
        T=eye(4,4);

    coronal=sagittal=transverse=zeros(3);
    if (prop.check("coronal"))
        if (Bottle *b=prop.find("coronal").asList())
            b->write(coronal);
    if (prop.check("sagittal"))
        if (Bottle *b=prop.find("sagittal").asList())
            b->write(sagittal);
    if (prop.check("transverse"))
        if (Bottle *b=prop.find("transverse").asList())
            b->write(transverse);
    helper_fromproperty(prop.find("skeleton").asList(),nullptr);
}

int Skeleton::getNumFromKey(const string &tag) const
{
    auto it1=tag2key.find(tag);
    if (it1!=tag2key.end())
    {
        auto it2=key2id.find(it1->second);
        if (it2!=key2id.end())
            return (int)it2->second;
    }
    return -1;
}

const KeyPoint *Skeleton::operator[](const string &tag) const
{
    auto it=tag2key.find(tag);
    return (it!=tag2key.end())?it->second:nullptr;
}

const KeyPoint *Skeleton::operator[](const unsigned int i) const
{
    return (i<keypoints.size())?keypoints[i]:nullptr;
}

void Skeleton::update()
{
    Vector p(4,1.0);
    for (auto &k:keypoints)
    {
        if (k->isUpdated())
        {
            auto v=k->getPoint();
            auto px=k->getPixel();
            p[0]=v[0];
            p[1]=v[1];
            p[2]=v[2];
            k->setPoint((T*p).subVector(0,2),px);
        }
    }

    update_planes();
}

void Skeleton::update(const vector<Vector> &ordered)
{
    Vector p(4,1.0);
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

    update_planes();
}

void Skeleton::update(const vector<pair<string,Vector>> &unordered)
{
    for (auto &k:keypoints)
        k->stale();

    Vector p(4,1.0);
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

    update_planes();
}

void Skeleton::update_withpixels(const vector<pair<Vector,Vector>> &ordered)
{
    Vector p(4,1.0);
    unsigned int i=0;
    for (auto &k:keypoints)
    {
        k->stale();
        if (i<ordered.size())
        {
            auto &v=ordered[i];
            p[0]=v.first[0];
            p[1]=v.first[1];
            p[2]=v.first[2];
            k->setPoint((T*p).subVector(0,2),v.second);
        }
        i++;
    }

    update_planes();
}

void Skeleton::update_withpixels(const vector<pair<string,pair<Vector,Vector>>> &unordered)
{
    for (auto &k:keypoints)
        k->stale();

    Vector p(4,1.0);
    for (auto &it1:unordered)
    {
        auto it2=tag2key.find(get<0>(it1));
        if (it2!=tag2key.end())
        {
            auto &v=get<1>(it1);
            p[0]=v.first[0];
            p[1]=v.first[1];
            p[2]=v.first[2];
            it2->second->setPoint((T*p).subVector(0,2),v.second);
        }
    }

    update_planes();
}

void Skeleton::update(const Property &prop)
{
    if (prop.check("type"))
        if (prop.find("type").asString()!=type)
            return;
    if (prop.check("tag"))
        tag=prop.find("tag").asString();
    if (prop.check("transformation"))
        if (Bottle *b=prop.find("transformation").asList())
            b->write(T);
    if (prop.check("coronal"))
        if (Bottle *b=prop.find("coronal").asList())
            b->write(coronal);
    if (prop.check("sagittal"))
        if (Bottle *b=prop.find("sagittal").asList())
            b->write(sagittal);
    if (prop.check("transverse"))
        if (Bottle *b=prop.find("transverse").asList())
            b->write(transverse);
    helper_updatefromproperty(prop.find("skeleton").asList());
}

vector<Vector> Skeleton::get_ordered() const
{
    vector<Vector> ordered;
    for (auto &k:keypoints)
        ordered.push_back(k->getPoint());
    return ordered;
}

vector<pair<string,Vector>> Skeleton::get_unordered() const
{
    vector<pair<string,Vector>> unordered;
    for (auto &it:tag2key)
        unordered.push_back(make_pair(it.first,it.second->getPoint()));
    return unordered;
}

vector<pair<Vector,Vector>> Skeleton::get_ordered_withpixels() const
{
    vector<pair<Vector,Vector>> ordered;
    for (auto &k:keypoints)
        ordered.push_back(make_pair(k->getPoint(),k->getPixel()));
    return ordered;
}

vector<pair<string,pair<Vector,Vector>>> Skeleton::get_unordered_withpixels() const
{
    vector<pair<string,pair<Vector,Vector>>> unordered;
    for (auto &it:tag2key)
        unordered.push_back(make_pair(it.first,make_pair(it.second->getPoint(),it.second->getPixel())));
    return unordered;
}

void Skeleton::normalize(const double n)
{
    if (keypoints.size()>0)
    {
        vector<Vector> helperpoints;
        for (auto &k:keypoints)
            helperpoints.push_back(k->getPoint());
        helper_normalize(keypoints[0],helperpoints,n);
    }
}

void Skeleton::scale(const double s)
{
    if (keypoints.size()>0)
    {
        vector<Vector> helperpoints;
        for (auto &k:keypoints)
            helperpoints.push_back(k->getPoint());
        helper_scale(keypoints[0],helperpoints,s);
    }
}

void Skeleton::print(ostream &os) const
{
    os<<"tag = \""<<tag<<"\""<<endl;
    os<<"transformation ="<<endl<<T.toString(3,3)<<endl;
    os<<"coronal = ("<<coronal.toString(3,3)<<")"<<endl;
    os<<"sagittal = ("<<sagittal.toString(3,3)<<")"<<endl;
    os<<"transverse = ("<<transverse.toString(3,3)<<")"<<endl;
    for (auto &k:keypoints)
    {
        os<<"keypoint[\""<<k->getTag()<<"\"] = ("
          <<k->getPoint().toString(3,3)<<"); pixel=("
          <<k->getPixel().toString(1,1)<<"); status="
          <<(k->isUpdated()?"updated":"stale")
          <<"; parent={";
          for (auto &p:k->parent) os<<"\""<<p->getTag()<<"\" ";
          os<<"}; child={";
          for (auto &c:k->child) os<<"\""<<c->getTag()<<"\" ";
          os<<"}"<<endl;
    }
}

SkeletonStd::SkeletonStd()
{
    type=SkeletonType::SkeletonStd;

    tag2key[KeyPointTag::shoulder_center]=new KeyPoint(KeyPointTag::shoulder_center);
    tag2key[KeyPointTag::head]=new KeyPoint(KeyPointTag::head);
    tag2key[KeyPointTag::shoulder_left]=new KeyPoint(KeyPointTag::shoulder_left);
    tag2key[KeyPointTag::elbow_left]=new KeyPoint(KeyPointTag::elbow_left);
    tag2key[KeyPointTag::hand_left]=new KeyPoint(KeyPointTag::hand_left);
    tag2key[KeyPointTag::shoulder_right]=new KeyPoint(KeyPointTag::shoulder_right);
    tag2key[KeyPointTag::elbow_right]=new KeyPoint(KeyPointTag::elbow_right);
    tag2key[KeyPointTag::hand_right]=new KeyPoint(KeyPointTag::hand_right);
    tag2key[KeyPointTag::hip_center]=new KeyPoint(KeyPointTag::hip_center);
    tag2key[KeyPointTag::hip_left]=new KeyPoint(KeyPointTag::hip_left);
    tag2key[KeyPointTag::knee_left]=new KeyPoint(KeyPointTag::knee_left);
    tag2key[KeyPointTag::ankle_left]=new KeyPoint(KeyPointTag::ankle_left);
    tag2key[KeyPointTag::foot_left]=new KeyPoint(KeyPointTag::foot_left);
    tag2key[KeyPointTag::hip_right]=new KeyPoint(KeyPointTag::hip_right);
    tag2key[KeyPointTag::knee_right]=new KeyPoint(KeyPointTag::knee_right);
    tag2key[KeyPointTag::ankle_right]=new KeyPoint(KeyPointTag::ankle_right);
    tag2key[KeyPointTag::foot_right]=new KeyPoint(KeyPointTag::foot_right);

    keypoints.push_back(tag2key[KeyPointTag::shoulder_center]);
    keypoints.push_back(tag2key[KeyPointTag::head]);
    keypoints.push_back(tag2key[KeyPointTag::shoulder_left]);
    keypoints.push_back(tag2key[KeyPointTag::elbow_left]);
    keypoints.push_back(tag2key[KeyPointTag::hand_left]);
    keypoints.push_back(tag2key[KeyPointTag::shoulder_right]);
    keypoints.push_back(tag2key[KeyPointTag::elbow_right]);
    keypoints.push_back(tag2key[KeyPointTag::hand_right]);
    keypoints.push_back(tag2key[KeyPointTag::hip_center]);
    keypoints.push_back(tag2key[KeyPointTag::hip_left]);
    keypoints.push_back(tag2key[KeyPointTag::knee_left]);
    keypoints.push_back(tag2key[KeyPointTag::ankle_left]);
    keypoints.push_back(tag2key[KeyPointTag::foot_left]);
    keypoints.push_back(tag2key[KeyPointTag::hip_right]);
    keypoints.push_back(tag2key[KeyPointTag::knee_right]);
    keypoints.push_back(tag2key[KeyPointTag::ankle_right]);
    keypoints.push_back(tag2key[KeyPointTag::foot_right]);

    unsigned int id=0;
    for (auto &k:keypoints)
        key2id[k]=id++;

    // shoulderCenter
    tag2key[KeyPointTag::shoulder_center]->child.push_back(tag2key[KeyPointTag::head]);
    tag2key[KeyPointTag::shoulder_center]->child.push_back(tag2key[KeyPointTag::shoulder_left]);
    tag2key[KeyPointTag::shoulder_center]->child.push_back(tag2key[KeyPointTag::shoulder_right]);
    tag2key[KeyPointTag::shoulder_center]->child.push_back(tag2key[KeyPointTag::hip_center]);

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

    // hipCenter
    tag2key[KeyPointTag::hip_center]->parent.push_back(tag2key[KeyPointTag::shoulder_center]);
    tag2key[KeyPointTag::hip_center]->child.push_back(tag2key[KeyPointTag::hip_left]);
    tag2key[KeyPointTag::hip_center]->child.push_back(tag2key[KeyPointTag::hip_right]);

    // hipLeft
    tag2key[KeyPointTag::hip_left]->parent.push_back(tag2key[KeyPointTag::hip_center]);
    tag2key[KeyPointTag::hip_left]->child.push_back(tag2key[KeyPointTag::knee_left]);

    // kneeLeft
    tag2key[KeyPointTag::knee_left]->parent.push_back(tag2key[KeyPointTag::hip_left]);
    tag2key[KeyPointTag::knee_left]->child.push_back(tag2key[KeyPointTag::ankle_left]);

    // ankleLeft
    tag2key[KeyPointTag::ankle_left]->parent.push_back(tag2key[KeyPointTag::knee_left]);
    tag2key[KeyPointTag::ankle_left]->child.push_back(tag2key[KeyPointTag::foot_left]);

    // footLeft
    tag2key[KeyPointTag::foot_left]->parent.push_back(tag2key[KeyPointTag::ankle_left]);

    // hipRight
    tag2key[KeyPointTag::hip_right]->parent.push_back(tag2key[KeyPointTag::hip_center]);
    tag2key[KeyPointTag::hip_right]->child.push_back(tag2key[KeyPointTag::knee_right]);

    // kneeRight
    tag2key[KeyPointTag::knee_right]->parent.push_back(tag2key[KeyPointTag::hip_right]);
    tag2key[KeyPointTag::knee_right]->child.push_back(tag2key[KeyPointTag::ankle_right]);

    // ankleRight
    tag2key[KeyPointTag::ankle_right]->parent.push_back(tag2key[KeyPointTag::knee_right]);
    tag2key[KeyPointTag::ankle_right]->child.push_back(tag2key[KeyPointTag::foot_right]);

    // footRight
    tag2key[KeyPointTag::foot_right]->parent.push_back(tag2key[KeyPointTag::ankle_right]);
}

bool SkeletonStd::update_planes()
{
    int cnt=0;
    if (tag2key[KeyPointTag::shoulder_left]->isUpdated() &&
        tag2key[KeyPointTag::shoulder_right]->isUpdated())
    {
        sagittal=tag2key[KeyPointTag::shoulder_left]->getPoint()-
                 tag2key[KeyPointTag::shoulder_right]->getPoint();
        double n=norm(sagittal);
        if (n>0.0)
            sagittal/=n;
        cnt++;
    }

    if (tag2key[KeyPointTag::shoulder_center]->isUpdated() &&
        tag2key[KeyPointTag::hip_center]->isUpdated())
    {
        transverse=tag2key[KeyPointTag::shoulder_center]->getPoint()-
                   tag2key[KeyPointTag::hip_center]->getPoint();
        double n=norm(transverse);
        if (n>0.0)
            transverse/=n;
        cnt++;
    }

    coronal=cross(sagittal,transverse);
    return (cnt>=2);
}

Skeleton *assistive_rehab::skeleton_factory(const Property &prop)
{
    Skeleton *skeleton=nullptr;
    if (prop.check("type"))
    {
        string type=prop.find("type").asString();
        if (type==SkeletonType::SkeletonStd)
            skeleton=new SkeletonStd;

        if (skeleton!=nullptr)
            skeleton->fromProperty(prop);
    }
    return skeleton;
}

