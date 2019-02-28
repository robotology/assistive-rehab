/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * \defgroup skeleton skeleton
 *
 * Classes for skeleton.
 *
 * \section intro_sec Description
 *
 * The class Skeleton can be used to deal with skeletons, defined as series of keypoints linked together
 * with a predefined structure.
 * The class Keypoint can be used to deal with single keypoints of the skeleton.
 *
 * A skeleton can be defined as:
 * - a standard skeleton
 * - a waist skeleton: which introduces a virtual link identified by the tag hip_center.
 *
 * A skeleton can be defined from:
 * - an ordered list of keypoints, according to the following order:
 *   - 0: shoulder_center
 *   - 1: head
 *   - 2: shoulder_left
 *   - 3: elbow_left
 *   - 4: hand_left
 *   - 5: shoulder_right
 *   - 6: elbow_right
 *   - 7: hand_right
 *   - 8: hip_left
 *   - 9: knee_left
 *   - 10: ankle_left
 *   - 11: hip_right
 *   - 12: knee_right
 *   - 13: ankle_right
 *   A waist skeleton has hip_center at index 8 and the following keypoints are shifted one index ahead.
 * - an unordered list of keypoints, where each keypoint is identified by its tag;
 * - its properties.
 *
 * \author Ugo Pattacini <ugo.pattacini@iit.it>
 *
 */

#ifndef ASSISTIVE_REHAB_SKELETON_H
#define ASSISTIVE_REHAB_SKELETON_H

#include <string>
#include <vector>
#include <unordered_map>
#include <utility>
#include <ostream>
#include <iostream>
#include <yarp/os/Bottle.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

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

namespace SkeletonType
{
extern const std::string Skeleton;
extern const std::string SkeletonStd;
extern const std::string SkeletonWaist;
}

class Skeleton;
class SkeletonStd;
class SkeletonWaist;

/**
* Basic class for single keypoint of a skeleton.
*/
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
    /**
    * Default constructor.
    */
    KeyPoint();

    /**
    * Overloaded constructor.
    * @param tag_ string containing the keypoint's tag.
    * @param point_ vector containing the keypoint camera 
    *               coordinates x,y,z.
    * @param updated_ true if the keypoint has been updated.
    */
    KeyPoint(const std::string &tag_, const yarp::sig::Vector &point_=yarp::sig::Vector(3,0.0),
             const bool updated_=false);

    /**
    * Deleted copy constructor.
    */
    KeyPoint(const KeyPoint&) = delete;

    /**
    * Deleted copy operator.
    */
    KeyPoint& operator=(const KeyPoint&) = delete;

    /**
    * Virtual destructor.
    */
    virtual ~KeyPoint() { }

    /**
    * Return true if the keypoint has been updated.
    * @return true if the keypoint has been updated.
    */
    bool isUpdated() const { return updated; }

    /**
    * Return a reference to the tag of the keypoint.
    * @return string containing the keypoint's tag.
    */
    const std::string& getTag() const { return tag; }

    /**
    * Return a reference to the vector containing the keypoint's 
    * x,y,z camera coordinates. 
    * @return reference to the vector containing the keypoint's 
    *         x,y,z camera coordinates.
    */
    const yarp::sig::Vector &getPoint() const { return point; }

    /**
    * Set keypoint's x,y,z camera coordinates to a desired value.
    * @param point vector containing the desider point.
    * @return true/false on success/failure.
    */
    bool setPoint(const yarp::sig::Vector &point);

    /**
    * Retrieve the number of parents of the keypoint.
    * @return number of parents of the keypoint.
    */
    unsigned int getNumParent() const { return (unsigned int)parent.size(); }

    /**
    * Get a pointer to the parent of the keypoint specified by index.
    * @param i int containing the index of the desired keypoint.
    * @return pointer to the keypoint's parent.
    */
    const KeyPoint* getParent(const unsigned int i) const;

    /**
    * Retrieve the number of children of the keypoint.
    * @return number of children of the keypoint.
    */
    unsigned int getNumChild() const { return (unsigned int)child.size(); }

    /**
    * Get a pointer to the child of the keypoint specified by index.
    * @param i int containing the index of the desired keypoint.
    * @return pointer to the keypoint's child.
    */
    const KeyPoint* getChild(const unsigned int i) const;
};

/**
* \ingroup skeleton
*
* Abstract class for skeleton.
*/
class Skeleton
{
protected:
    std::string type; /**< skeleton's type ("assistive_rehab::SkeletonStd" or "assistive_rehab::SkeletonWaist") */
    std::string tag; /**< skeleton's tag */
    std::vector<KeyPoint*> keypoints; /**< vector of pointer to KeyPoint */
    std::unordered_map<std::string,KeyPoint*> tag2key; /**< map associating a tag to a pointer to a KeyPoint */
    std::unordered_map<KeyPoint*,unsigned int> key2id; /**< map associating a pointer to a KeyPoint to an index */

    yarp::sig::Matrix T; /**< transformation matrix */
    yarp::sig::Vector coronal; /**< vector containing the normal to the coronal plane */
    yarp::sig::Vector sagittal; /**< vector containing the normal to the sagittal plane */
    yarp::sig::Vector transverse; /**< vector containing the normal to the transverse plane */

    yarp::os::Property helper_toproperty(KeyPoint *k) const;
    void helper_fromproperty(yarp::os::Bottle *prop, KeyPoint *parent);
    void helper_updatefromproperty(yarp::os::Bottle *prop);
    void helper_normalize(KeyPoint* k, const std::vector<yarp::sig::Vector> &helperpoints, const double n);
    void helper_scale(KeyPoint* k, const std::vector<yarp::sig::Vector> &helperpoints, const double s);
    double helper_getmaxpath(KeyPoint* k, std::vector<bool> &visited) const;

public:
    /**
    * Default constructor.
    */
    Skeleton();

    /**
    * Deleted copy constructor.
    */
    Skeleton(const Skeleton&) = delete;

    /**
    * Deleted copy operator.
    */
    Skeleton& operator=(const Skeleton&) = delete;

    /**
    * Virtual destructor.
    */
    virtual ~Skeleton();

    /**
    * Return a reference to the type of the skeleton.
    * @return reference to the skeleton's type.
    */
    const std::string& getType() const { return type; }

    /**
    * Set the tag of the skeleton.
    * @param tag string containing the desired tag.
    */
    void setTag(const std::string &tag) { this->tag=tag; }

    /**
    * Return a reference to the tag of the skeleton.
    * @return reference to a string containing the skeleton's tag.
    */
    const std::string& getTag() const { return tag; }

    /**
    * Set a new transformation matrix of the skeleton.
    * @param T matrix containing the desired transformation.
    */
    bool setTransformation(const yarp::sig::Matrix &T);

    /**
    * Retrieve the transformation matrix of the skeleton.
    * @return reference to the skeleton's transformation matrix.
    * @note if the transformation matrix is the identity matrix, keypoints are defined with respect to the camera.
    */
    const yarp::sig::Matrix& getTransformation() const { return T; }

    /**
    * Set a new coronal plane to the skeleton.
    * @param coronal vector containing the x,y,z coordinates of the normal to the desired plane.
    */
    bool setCoronal(const yarp::sig::Vector &coronal);

    /**
    * Set a new sagittal plane to the skeleton.
    * @param sagittal vector containing the x,y,z coordinates of the normal to the desired plane.
    */
    bool setSagittal(const yarp::sig::Vector &sagittal);

    /**
    * Set a new transverse plane to the skeleton.
    * @param transverse vector containing the x,y,z coordinates of the normal to the desired plane.
    */
    bool setTransverse(const yarp::sig::Vector &transverse);

    /**
    * Retrieve the coronal plane of the skeleton.
    * @return vector containing x,y,z coordinates of the normal to skeleton's coronal plane.
    */
    yarp::sig::Vector getCoronal() const;

    /**
    * Retrieve the sagittal plane of the skeleton.
    * @return vector containing the x,y,z coordinates of the normal skeleton's sagittal plane.
    */
    yarp::sig::Vector getSagittal() const;

    /**
    * Retrieve the transverse plane of the skeleton.
    * @return vector containing the x,y,z coordinates of the normal skeleton's sagittal plane.
    */
    yarp::sig::Vector getTransverse() const;

    /**
    * Retrieve the skeleton's maximum path.
    * @return skeleton's maximum path.
    */
    double getMaxPath() const;

    /**
    * Export the skeleton structure as a property.
    * @return a Property object containing the properties of a skeleton.
    *
    * Available properties are:
    * - type: string containing skeleton's type ("assistive_rehab::SkeletonStd" or "assistive_rehab::SkeletonWaist").
    * - tag: string containing skeleton's tag.
    * - transformation: 4 x 4 skeleton's roto-translation matrix.
    * - coronal: vector containing skeleton's coronal plane.
    * - sagittal: vector containing skeleton's sagittal plane.
    * - transverse: vector containing skeleton's transverse plane.
    * - skeleton: list containing keypoints with the following subproperties:
    *     - tag: string containing keypoint's tag.
    *     - status: string containing keypoint's status (updated or stale).
    *     - position: vector containing keypoint's coordinates x,y,z.
    *     - child: list containing keypoint's child, specified as position, status, tag.
    */
    virtual yarp::os::Property toProperty();

    /**
    * Import the skeleton structure from its properties.
    * @param prop Property object containing the properties of a skeleton.
    *
    * Available properties are:
    * - type: string containing skeleton's type ("assistive_rehab::SkeletonStd" or "assistive_rehab::SkeletonWaist").
    * - tag: string containing skeleton's tag.
    * - transformation: 4 x 4 skeleton's roto-translation matrix.
    * - coronal: vector containing skeleton's coronal plane.
    * - sagittal: vector containing skeleton's sagittal plane.
    * - transverse: vector containing skeleton's transverse plane.
    * - skeleton: list containing keypoints with the following subproperties:
    *     - tag: string containing keypoint's tag.
    *     - status: string containing keypoint's status (updated or stale).
    *     - position: vector containing keypoint's coordinates x,y,z.
    *     - child: list containing keypoint's child, specified as position, status, tag.
    */
    virtual void fromProperty(const yarp::os::Property &prop);

    /**
    * Retrieve the number of keypoints of the skeleton.
    * @return skeleton's number of keypoints.
    */
    unsigned int getNumKeyPoints() const { return (unsigned int)keypoints.size(); }

    /**
    * Retrieve the index of the keypoint from its tag.
    * @param tag string containing the keypoint's tag.
    * @return skeleton's number of keypoints.
    */
    int getNumFromKey(const std::string &tag) const;

    /**
    * Keypoint access. Returns a pointer to the keypoint specified by its tag.
    * @param tag string containing the keypoint's tag.
    * @return a (const) pointer to the keypoint.
    */
    const KeyPoint* operator[](const std::string &tag) const;

    /**
    * Keypoint access. Returns a pointer to the keypoint specified by its index.
    * @param i int containing the keypoint's index.
    * @return a (const) pointer to the keypoint.
    */
    const KeyPoint* operator[](const unsigned int i) const;

    /**
    * Update skeleton.
    */
    virtual void update();

    /**
    * Update skeleton from ordered list.
    * @param ordered vector containing the ordered list of keypoints.
    * The single keypoint is specified as vector containing the x,y,z coordinates.
    */
    virtual void update(const std::vector<yarp::sig::Vector> &ordered);

    /**
    * Update skeleton from unordered list.
    * @param unordered vector containing an unordered list of keypoints.
    * The single keypoint is specified as pair which associates a string, containing the keypoint's tag,
    * and a vector, containing the x,y,z coordinates.
    */
    virtual void update(const std::vector<std::pair<std::string,yarp::sig::Vector>> &unordered);

    /**
    * Update skeleton from properties.
    * @param prop a Property object containing skeleton information.
    *
    * Available properties are:
    * - type: string containing skeleton's type ("assistive_rehab::SkeletonStd" or "assistive_rehab::SkeletonWaist").
    * - tag: string containing skeleton's tag.
    * - transformation: 4 x 4 skeleton's roto-translation matrix.
    * - coronal: vector containing skeleton's coronal plane.
    * - sagittal: vector containing skeleton's sagittal plane.
    * - transverse: vector containing skeleton's transverse plane.
    * - skeleton: list containing keypoints with the following subproperties:
    *     - tag: string containing keypoint's tag.
    *     - status: string containing keypoint's status (updated or stale).
    *     - position: vector containing keypoint's coordinates x,y,z.
    *     - child: list containing keypoint's child, specified as position, status, tag.
    */
    virtual void update(const yarp::os::Property &prop);

    /**
    * Update skeleton planes.
    * @return true/false on success/failure (failure occurs if not all planes are updated).
    */
    virtual bool update_planes() = 0;

    /**
    * Retrieve the ordered list of keypoints.
    * @return vector containing the ordered list of keypoints, each 
    *         specified as vector containing the x,y,z camera
    *         coordinates.
    */
    virtual std::vector<yarp::sig::Vector> get_ordered() const;

    /**
    * Retrieve the unordered list of keypoints.
    * @return vector containing an unordered list of keypoints,
    * each specified as pair which associates a string, containing the keypoint's tag,
    * and a vector, containing the x,y,z camera coordinates.
    */
    virtual std::vector<std::pair<std::string,yarp::sig::Vector>> get_unordered() const;

    /**
    * Normalize skeleton in order to have unitary/desired distance among keypoints.
    * @param n double containing the normalization factor.
    */
    void normalize(const double n=1.0);

    /**
    * Rescale skeleton.
    * @param s double containing the scaling factor.
    */
    void scale(const double s);

    /**
    * Print skeleton information.
    */
    void print(std::ostream &os=std::cout) const;
};

/**
* \ingroup skeleton
*
* Basic class for skeleton standard.
*/
class SkeletonStd : public Skeleton
{
public:
    /**
    * Default constructor.
    */
    SkeletonStd();

    /**
    * Update skeleton planes.
    * @return true/false on success/failure (failure occurs if not all planes are updated).
    */
    bool update_planes() override;
};

/**
* \ingroup skeleton
*
* Basic class for skeleton waist.
*/
class SkeletonWaist : public SkeletonStd
{
protected:
    unsigned int waist_pos;
    
public:
    /**
    * Default constructor.
    */
    SkeletonWaist();

    /**
    * Update skeleton waist from standard from ordered list.
    * @param ordered vector containing the ordered list of keypoints.
    * The single keypoint is specified as vector containing the 
    * x,y,z camera coordinates. 
    */
    virtual void update_fromstd(const std::vector<yarp::sig::Vector> &ordered);

    /**
    * Update skeleton waist from standard from unordered list.
    * @param unordered vector containing an unordered list of keypoints.
    * The single keypoint is specified as pair which associates a string, containing the keypoint's tag,
    * and a vector, containing the x,y,z camera coordinates.
    */
    virtual void update_fromstd(const std::vector<std::pair<std::string, yarp::sig::Vector>> &unordered);

    /**
    * Update skeleton waist from standard from yarp properties.
    * @param prop a Property object containing skeleton information.
    */
    virtual void update_fromstd(const yarp::os::Property &prop);

    /**
    * Update skeleton planes.
    * @return true/false on success/failure (failure occurs if not all planes are updated).
    */
    bool update_planes() override;
};

/**
* \ingroup skeleton
*
* Populate skeleton from a Property object.
* @param prop reference to a Property object.
* @return a pointer to a Skeleton object.
*
* Available properties are:
* - type: string containing skeleton's type ("assistive_rehab::SkeletonStd" or "assistive_rehab::SkeletonWaist").
* - tag: string containing skeleton's tag.
* - transformation: 4 x 4 skeleton's roto-translation matrix.
* - coronal: vector containing skeleton's coronal plane.
* - sagittal: vector containing skeleton's sagittal plane.
* - transverse: vector containing skeleton's transverse plane.
* - skeleton: list containing keypoints with the following subproperties:
*     - tag: string containing keypoint's tag.
*     - status: string containing keypoint's status (updated or stale).
*     - position: vector containing keypoint's camera
*       coordinates x,y,z.
*     - child: list containing keypoint's child, specified as
*       position, status, tag.
*/
Skeleton *skeleton_factory(const yarp::os::Property &prop);

}

#endif
