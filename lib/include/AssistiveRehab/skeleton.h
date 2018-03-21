/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file skeleton.h
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#ifndef ASSISTIVE_REHAB_SKELETON_H
#define ASSISTIVE_REHAB_SKELETON_H

#include <string>
#include <vector>
#include <unordered_map>
#include <utility>
#include <yarp/os/Bottle.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>

namespace assistive_rehab
{

namespace KeyPointTag
{
extern const std::string shoulder_center;
extern const std::string head;
extern const std::string shoulder_left;
extern const std::string elbow_left;
extern const std::string hand_left;
extern const std::string shoulder_right;
extern const std::string elbow_right;
extern const std::string hand_right;
extern const std::string hip_center;
extern const std::string hip_left;
extern const std::string knee_left;
extern const std::string ankle_left;
extern const std::string hip_right;
extern const std::string knee_right;
extern const std::string ankle_right;
}

class Skeleton;
class SkeletonStd;
class SkeletonWaist;

class KeyPoint
{
    friend class Skeleton;
    friend class SkeletonStd;
    friend class SkeletonWaist;

    bool updated;
    std::string tag;
    yarp::sig::Vector point;
    std::vector<KeyPoint*> parent;
    std::vector<KeyPoint*> child;

    void stale() { updated=false; }

public:
    KeyPoint();
    KeyPoint(const std::string &tag_, const yarp::sig::Vector &point_=yarp::sig::Vector(3,0.0),
             const bool updated_=false);
    virtual ~KeyPoint() { }

    bool isUpdated() const { return updated; }
    std::string getTag() const { return tag; }
    const yarp::sig::Vector &getPoint() const { return point; }
    bool setPoint(const yarp::sig::Vector &point);

    unsigned int getNumParent() const { return (unsigned int)parent.size(); }
    const KeyPoint *getParent(const unsigned int i) const;

    unsigned int getNumChild() const { return (unsigned int)child.size(); }
    const KeyPoint *getChild(const unsigned int i) const;
};

class Skeleton
{
protected:
    std::string tag;
    std::vector<KeyPoint*> keypoints;
    std::unordered_map<std::string, KeyPoint*> tag2key;
    std::unordered_map<KeyPoint*, unsigned int> key2id;

    yarp::os::Property helper_toproperty(KeyPoint* k);
    void helper_fromproperty(yarp::os::Bottle *prop, KeyPoint *parent);
    void helper_normalize(KeyPoint* k, const std::vector<yarp::sig::Vector> &helperpoints);

public:
    Skeleton();
    virtual ~Skeleton();

    void setTag(const std::string &tag) { this->tag=tag; }
    std::string getTag() const { return tag; }

    virtual yarp::os::Property toProperty();
    virtual void fromProperty(const yarp::os::Property &prop);

    unsigned int getNumKeyPoints() const { return (unsigned int)keypoints.size(); }
    const KeyPoint*operator [](const std::string &tag) const;
    const KeyPoint*operator [](const unsigned int i) const;

    virtual void update(const std::vector<yarp::sig::Vector> &ordered) = 0;
    virtual void update(const std::vector<std::pair<std::string, yarp::sig::Vector>> &unordered) = 0;

    virtual std::vector<yarp::sig::Vector> get_ordered() const;
    virtual std::vector<std::pair<std::string, yarp::sig::Vector>> get_unordered() const;

    void normalize();
    void print() const;
};

class SkeletonStd : public Skeleton
{
public:
    SkeletonStd();

    void update(const std::vector<yarp::sig::Vector> &ordered)override;
    void update(const std::vector<std::pair<std::string, yarp::sig::Vector>> &unordered)override;
};

class SkeletonWaist : public SkeletonStd
{
protected:
    unsigned int waist_pos;

public:
    SkeletonWaist();

    void update_fromstd(const std::vector<yarp::sig::Vector> &ordered);
    void update_fromstd(const std::vector<std::pair<std::string, yarp::sig::Vector>> &unordered);
};

}

#endif
