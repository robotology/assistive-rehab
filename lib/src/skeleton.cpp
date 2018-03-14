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
#include "AssistiveRehab/skeleton.h"

using namespace std;
using namespace yarp::sig;
using namespace yarp::os;
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
    for (auto &k:keypoints)
        delete k;
}

const KeyPoint *Skeleton::operator[](const string &tag) const
{
    auto &it=tag2key.find(tag);
    return (it!=tag2key.end())?it->second:nullptr;
}

const KeyPoint *Skeleton::operator [](const unsigned int i) const
{
    return (i<keypoints.size())?keypoints[i]:nullptr;
}

void Skeleton::print() const
{
    for (auto &k:keypoints)
    {
        cout<<"keypoint[\""<<k->getTag()<<"\"]; ("
           <<k->getPoint().toString(3,3)<<") "
           <<(k->isUpdated()?"updated":"stale")<<endl;
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
    for (auto &k:keypoints)
    {
        k->stale();
        if (i<ordered.size())
            k->setPoint(ordered[i]);
        i++;
    }
}

void SkeletonStd::update(const vector<pair<string, Vector>> &unordered)
{
    for (auto &k:keypoints)
        k->stale();

    for (auto &it1:unordered)
    {
        auto &it2=tag2key.find(get<0>(it1));
        if (it2!=tag2key.end())
            it2->second->setPoint(get<1>(it1));
    }
}

SkeletonWaist::SkeletonWaist() : SkeletonStd()
{
    tag2key[KeyPointTag::hip_center]=new KeyPoint(KeyPointTag::hip_center);
    keypoints.insert(keypoints.begin()+8,tag2key[KeyPointTag::hip_center]);

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
