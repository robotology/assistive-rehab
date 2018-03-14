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
#include "skeleton.h"

using namespace std;
using namespace yarp::sig;
using namespace yarp::os;
using namespace assist_rehab;


namespace assist_rehab
{

namespace KeyPointNum
{
const unsigned int shoulder_center=0;
const unsigned int head=1;
const unsigned int shoulder_left=2;
const unsigned int elbow_left=3;
const unsigned int hand_left=4;
const unsigned int shoulder_right=5;
const unsigned int elbow_right=6;
const unsigned int hand_right=7;
const unsigned int hip_left=8;
const unsigned int knee_left=9;
const unsigned int ankle_left=10;
const unsigned int hip_right=11;
const unsigned int knee_right=12;
const unsigned int ankle_right=13;
}

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
const string hip_left="hipLeft";
const string knee_left="kneeLeft";
const string ankle_left="ankleLeft";
const string hip_right="hipRight";
const string knee_right="kneeRight";
const string ankle_right="ankleRight";
}

}

KeyPoint::KeyPoint() : updated(false), id(-1), tag(""), point(3,0.0)
{
}

KeyPoint::KeyPoint(const int id_, const string &tag_, const Vector &point_, const bool updated_) :
   updated(updated_), id(id_), tag(tag_), point(point_)
{
}

KeyPoint::~KeyPoint()
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

Skeleton::~Skeleton()
{
    for (auto &it:keypoints)
        delete it;
}

const KeyPoint *Skeleton::operator[](const unsigned int id) const
{
    auto it=id2key.find(id);
    return (it!=id2key.end())?it->second:nullptr;
}

const KeyPoint *Skeleton::operator[](const string &tag) const
{
    auto it=tag2key.find(tag);
    return (it!=tag2key.end())?it->second:nullptr;
}

void Skeleton::print() const
{
    for (auto &it:keypoints)
    {
        cout<<"keypoint["<<it->getId()<<"]; \""
           <<it->getTag()<<"\": ("
           <<it->getPoint().toString(3,3)<<") "
           <<(it->isUpdated()?"updated":"stale")<<endl;
    }
}

SkeletonStd::SkeletonStd()
{
    id2tag[KeyPointNum::shoulder_center]=KeyPointTag::shoulder_center;
    id2tag[KeyPointNum::head]=KeyPointTag::head;
    id2tag[KeyPointNum::shoulder_left]=KeyPointTag::shoulder_left;
    id2tag[KeyPointNum::elbow_left]=KeyPointTag::elbow_left;
    id2tag[KeyPointNum::hand_left]=KeyPointTag::hand_left;
    id2tag[KeyPointNum::shoulder_right]=KeyPointTag::shoulder_right;
    id2tag[KeyPointNum::elbow_right]=KeyPointTag::elbow_right;
    id2tag[KeyPointNum::hand_right]=KeyPointTag::hand_right;
    id2tag[KeyPointNum::hip_left]=KeyPointTag::hip_left;
    id2tag[KeyPointNum::knee_left]=KeyPointTag::knee_left;
    id2tag[KeyPointNum::ankle_left]=KeyPointTag::ankle_left;
    id2tag[KeyPointNum::hip_right]=KeyPointTag::hip_right;
    id2tag[KeyPointNum::knee_right]=KeyPointTag::knee_right;
    id2tag[KeyPointNum::ankle_right]=KeyPointTag::ankle_right;

    for (auto &it:id2tag)
    {
        KeyPoint *keypoint=new KeyPoint(it.first,it.second);
        id2key[it.first]=keypoint;
        tag2key[it.second]=keypoint;
        keypoints.push_back(keypoint);
    }

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
    int i=0;
    for (auto &it:keypoints)
    {
        it->stale();
        if (i<ordered.size())
            it->setPoint(ordered[i]);
        i++;
    }
}

void SkeletonStd::update(const vector<pair<string, Vector>> &unordered)
{
    for (auto &it:keypoints)
        it->stale();

    for (auto &it1:unordered)
    {
        auto it2=tag2key.find(get<0>(it1));
        if (it2!=tag2key.end())
            it2->second->setPoint(get<1>(it1));
    }
}

void SkeletonStd::update(const vector<pair<unsigned int, Vector>> &unordered)
{
    for (auto &it:keypoints)
        it->stale();

    for (auto &it1:unordered)
    {
        auto it2=id2key.find(get<0>(it1));
        if (it2!=id2key.end())
            it2->second->setPoint(get<1>(it1));
    }
}
