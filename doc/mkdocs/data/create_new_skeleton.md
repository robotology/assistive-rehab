# How to manage a skeleton object

This tutorial will explain you the main functionalities to manage a skeleton object for:

- defining a skeleton;
- retrieving a skeleton;
- accessing single keypoints of a skeleton;
- additional functionalities.

## Defining a skeleton

A skeleton can be defined as:

- [unordered list of keypoints](#from-an-unordered-list-of-keypoints): the list defines the keypoints as pairs of points associated to the keypoint tag;
- [ordered list of keypoints](#from-an-ordered-list-of-keypoints): the list defines the keypoints according to the following order:
    - 0: `shoulder_center`
    - 1: `head`
    - 2: `shoulder_left`
    - 3: `elbow_left`
    - 4: `hand_left`
    - 5: `shoulder_right`
    - 6: `elbow_right`
    - 7: `hand_right`
    - 8: `hip_left`
    - 9: `knee_left`
    - 10: `ankle_left`
    - 11: `hip_right`
    - 12: `knee_right`
    - 13: `ankle_right`

    !!! note
        The mentioned order is valid for a `skeletonStd` object. For a `SkeletonWaist` object, `hip_center` is inserted at index 8 and the following keypoints are shifted one index ahead.

- [property-like structure](#from-a-property-like-structure): the available properties are the following:

    - type: string containing skeleton's type ("assistive_rehab::SkeletonStd" or "assistive_rehab::SkeletonWaist").
    - tag: string containing skeleton's tag.
    - transformation: 4 x 4 skeleton's roto-translation matrix.
    - coronal: vector containing skeleton's coronal plane.
    - sagittal: vector containing skeleton's sagittal plane.
    - transverse: vector containing skeleton's transverse plane.
    - skeleton: list containing keypoints with the following subproperties:
        - tag: string containing keypoint's tag.
        - status: string containing keypoint's status (updated or stale).
        - position: vector containing keypoint's camera coordinates x,y,z.
        - child: list containing keypoint's child, specified as position, status, tag.

    An example is the following:

    ```
    (coronal (0.0 0.0 1.0)) (sagittal (1.0 0.0 0.0)) (skeleton (((child (((position (0.0 1.0 0.0)) (status updated) (tag head)) ((child (((child (((position (3.0 0.0 3.67394039744206e-16)) (status updated) (tag handLeft)))) (position (2.0 0.0 2.44929359829471e-16)) (status updated) (tag elbowLeft)))) (position (1.0 0.0 1.22464679914735e-16)) (status updated) (tag shoulderLeft)) ((child (((child (((position (-3.0 0.0 -3.67394039744206e-16)) (status updated) (tag handRight)))) (position (-2.0 0.0 -2.44929359829471e-16)) (status updated) (tag elbowRight)))) (position (-1.0 0.0 -1.22464679914735e-16)) (status updated) (tag shoulderRight)) ((child (((child (((child (((position (0.552786404500042 -1.10557280900008 6.76968100883151e-17)) (status stale) (tag ankleLeft)))) (position (1.0 -2.0 1.22464679914735e-16)) (status updated) (tag kneeLeft)))) (position (1.0 -1.0 1.22464679914735e-16)) (status updated) (tag hipLeft)) ((child (((child (((position (-0.552786404500042 -1.10557280900008 -6.76968100883151e-17)) (status stale) (tag ankleRight)))) (position (-1.0 -2.0 -1.22464679914735e-16)) (status updated) (tag kneeRight)))) (position (-1.0 -1.0 -1.22464679914735e-16)) (status updated) (tag hipRight)))) (position (0.0 -1.0 0.0)) (status updated) (tag hipCenter)))) (position (0.0 0.0 0.0)) (status updated) (tag shoulderCenter)))) (tag test) (transformation (4 4 (-1.0 0.0 1.22464679914735e-16 0.0 0.0 1.0 0.0 0.0 -1.22464679914735e-16 0.0 -1.0 0.0 0.0 0.0 0.0 1.0))) (transverse (0.0 1.0 0.0)) (type "class assistive_rehab::SkeletonWaist")
    ```

### From an unordered list of keypoints

The following code snippet creates a `SkeletonWaist` object from an unordered list of keypoints:

```cpp

#include <cstdlib>
#include <cmath>
#include <yarp/sig/Vector.h>
#include "AssistiveRehab/skeleton.h"

int main()
{
    assistive_rehab::SkeletonWaist skeleton;
    std::vector<std::pair<std::string,yarp::sig::Vector>> unordered;
    {
        yarp::sig::Vector p(3,0.0); p[0]=0.0; p[1]=0.0; p[2]=0.0;
        unordered.push_back(make_pair(assistive_rehab::KeyPointTag::shoulder_center,p));
    }
    {
        yarp::sig::Vector p(3,0.0); p[0]=0.0; p[1]=-0.1; p[2]=0.0;
        unordered.push_back(make_pair(assistive_rehab::KeyPointTag::head,p));
    }
    {
        yarp::sig::Vector p(3,0.0); p[0]=0.1; p[1]=0.0; p[2]=0.0;
        unordered.push_back(make_pair(assistive_rehab::KeyPointTag::shoulder_left,p));
    }
    {
        yarp::sig::Vector p(3,0.0); p[0]=0.2; p[1]=0.0; p[2]=0.0;
        unordered.push_back(make_pair(assistive_rehab::KeyPointTag::elbow_left,p));
    }
    {
        yarp::sig::Vector p(3,0.0); p[0]=0.3; p[1]=0.0; p[2]=0.0;
        unordered.push_back(make_pair(assistive_rehab::KeyPointTag::hand_left,p));
    }
    {
        yarp::sig::Vector p(3,0.0); p[0]=-0.1; p[1]=0.0; p[2]=0.0;
        unordered.push_back(make_pair(assistive_rehab::KeyPointTag::shoulder_right,p));
    }
    {
        yarp::sig::Vector p(3,0.0); p[0]=-0.2; p[1]=0.0; p[2]=0.0;
        unordered.push_back(make_pair(assistive_rehab::KeyPointTag::elbow_right,p));
    }
    {
        yarp::sig::Vector p(3,0.0); p[0]=-0.3; p[1]=0.0; p[2]=0.0;
        unordered.push_back(make_pair(assistive_rehab::KeyPointTag::hand_right,p));
    }
    {
        yarp::sig::Vector p(3,0.0); p[0]=0.1; p[1]=0.1; p[2]=0.0;
        unordered.push_back(make_pair(assistive_rehab::KeyPointTag::hip_left,p));
    }
    {
        yarp::sig::Vector p(3,0.0); p[0]=0.1; p[1]=0.2; p[2]=0.0;
        unordered.push_back(make_pair(assistive_rehab::KeyPointTag::knee_left,p));
    }
    {
        yarp::sig::Vector p(3,0.0); p[0]=-0.1; p[1]=0.1; p[2]=0.0;
        unordered.push_back(make_pair(assistive_rehab::KeyPointTag::hip_right,p));
    }
    {
        yarp::sig::Vector p(3,0.0); p[0]=-0.1; p[1]=0.2; p[2]=0.0;
        unordered.push_back(make_pair(assistive_rehab::KeyPointTag::knee_right,p));
    }

    skeleton.setTag("unordered");
    skeleton.update_fromstd(unordered);
    skeleton.print();

    return EXIT_SUCCESS;
}

```

The result is:

```
tag = "unordered"
transformation =
 1.000  0.000  0.000  0.000
 0.000  1.000  0.000  0.000
 0.000  0.000  1.000  0.000
 0.000  0.000  0.000  1.000
coronal = ( 0.000  0.000 -1.000)
sagittal = ( 1.000  0.000  0.000)
transverse = ( 0.000 -1.000  0.000)
keypoint["shoulderCenter"] = ( 0.000  0.000  0.000); status=updated; parent={}; child={"head" "shoulderLeft" "shoulderRight" "hipCenter" }
keypoint["head"] = ( 0.000 -0.100  0.000); status=updated; parent={"shoulderCenter" }; child={}
keypoint["shoulderLeft"] = ( 0.100  0.000  0.000); status=updated; parent={"shoulderCenter" }; child={"elbowLeft" }
keypoint["elbowLeft"] = ( 0.200  0.000  0.000); status=updated; parent={"shoulderLeft" }; child={"handLeft" }
keypoint["handLeft"] = ( 0.300  0.000  0.000); status=updated; parent={"elbowLeft" }; child={}
keypoint["shoulderRight"] = (-0.100  0.000  0.000); status=updated; parent={"shoulderCenter" }; child={"elbowRight" }
keypoint["elbowRight"] = (-0.200  0.000  0.000); status=updated; parent={"shoulderRight" }; child={"handRight" }
keypoint["handRight"] = (-0.300  0.000  0.000); status=updated; parent={"elbowRight" }; child={}
keypoint["hipCenter"] = ( 0.000  0.100  0.000); status=updated; parent={"shoulderCenter" }; child={"hipLeft" "hipRight" }
keypoint["hipLeft"] = ( 0.100  0.100  0.000); status=updated; parent={"hipCenter" }; child={"kneeLeft" }
keypoint["kneeLeft"] = ( 0.100  0.200  0.000); status=updated; parent={"hipLeft" }; child={"ankleLeft" }
keypoint["ankleLeft"] = ( 0.000  0.000  0.000); status=stale; parent={"kneeLeft" }; child={}
keypoint["hipRight"] = (-0.100  0.100  0.000); status=updated; parent={"hipCenter" }; child={"kneeRight" }
keypoint["kneeRight"] = (-0.100  0.200  0.000); status=updated; parent={"hipRight" }; child={"ankleRight" }
keypoint["ankleRight"] = ( 0.000  0.000  0.000); status=stale; parent={"kneeRight" }; child={}


```

### From an ordered list of keypoints

The following code snippet creates a `SkeletonWaist` object from an ordered list of keypoints:

```cpp

#include <cstdlib>
#include <cmath>
#include <yarp/sig/Vector.h>
#include "AssistiveRehab/skeleton.h"

int main()
{
    assistive_rehab::SkeletonWaist skeleton;
    std::vector<yarp::sig::Vector> ordered;
    {
        yarp::sig::Vector p(3,0.0); p[0]=0.0; p[1]=0.0; p[2]=0.0;
        ordered.push_back(p);
    }
    {
        yarp::sig::Vector p(3,0.0); p[0]=0.0; p[1]=-0.1; p[2]=0.0;
        ordered.push_back(p);
    }
    {
        yarp::sig::Vector p(3,0.0); p[0]=0.1; p[1]=0.0; p[2]=0.0;
        ordered.push_back(p);
    }
    {
        yarp::sig::Vector p(3,0.0); p[0]=0.2; p[1]=0.0; p[2]=0.0;
        ordered.push_back(p);
    }
    {
        yarp::sig::Vector p(3,0.0); p[0]=0.3; p[1]=0.0; p[2]=0.0;
        ordered.push_back(p);
    }
    {
        yarp::sig::Vector p(3,0.0); p[0]=-0.1; p[1]=0.0; p[2]=0.0;
        ordered.push_back(p);
    }
    {
        yarp::sig::Vector p(3,0.0); p[0]=-0.2; p[1]=0.0; p[2]=0.0;
        ordered.push_back(p);
    }
    {
        yarp::sig::Vector p(3,0.0); p[0]=-0.3; p[1]=0.0; p[2]=0.0;
        ordered.push_back(p);
    }
    {
        yarp::sig::Vector p(3,0.0); p[0]=0.1; p[1]=0.1; p[2]=0.0;
        ordered.push_back(p);
    }
    {
        yarp::sig::Vector p(3,0.0); p[0]=0.1; p[1]=0.2; p[2]=0.0;
        ordered.push_back(p);
    }
    {
        yarp::sig::Vector p(3,0.0); p[0]=-0.1; p[1]=0.1; p[2]=0.0;
        ordered.push_back(p);
    }
    {
        yarp::sig::Vector p(3,0.0); p[0]=-0.1; p[1]=0.2; p[2]=0.0;
        ordered.push_back(p);
    }

    skeleton.setTag("ordered");
    skeleton.update_fromstd(ordered);
    skeleton.print();

    return EXIT_SUCCESS;
}

```

The result is:

```
tag = "ordered"
transformation =
 1.000  0.000  0.000  0.000
 0.000  1.000  0.000  0.000
 0.000  0.000  1.000  0.000
 0.000  0.000  0.000  1.000
coronal = ( 0.000  0.000 -1.000)
sagittal = ( 1.000  0.000  0.000)
transverse = ( 0.000 -1.000  0.000)
keypoint["shoulderCenter"] = ( 0.000  0.000  0.000); status=updated; parent={}; child={"head" "shoulderLeft" "shoulderRight" "hipCenter" }
keypoint["head"] = ( 0.000 -0.100  0.000); status=updated; parent={"shoulderCenter" }; child={}
keypoint["shoulderLeft"] = ( 0.100  0.000  0.000); status=updated; parent={"shoulderCenter" }; child={"elbowLeft" }
keypoint["elbowLeft"] = ( 0.200  0.000  0.000); status=updated; parent={"shoulderLeft" }; child={"handLeft" }
keypoint["handLeft"] = ( 0.300  0.000  0.000); status=updated; parent={"elbowLeft" }; child={}
keypoint["shoulderRight"] = (-0.100  0.000  0.000); status=updated; parent={"shoulderCenter" }; child={"elbowRight" }
keypoint["elbowRight"] = (-0.200  0.000  0.000); status=updated; parent={"shoulderRight" }; child={"handRight" }
keypoint["handRight"] = ( 0.000  0.000  0.000); status=stale; parent={"elbowRight" }; child={}
keypoint["hipCenter"] = ( 0.000  0.150  0.000); status=updated; parent={"shoulderCenter" }; child={"hipLeft" "hipRight" }
keypoint["hipLeft"] = ( 0.100  0.100  0.000); status=updated; parent={"hipCenter" }; child={"kneeLeft" }
keypoint["kneeLeft"] = ( 0.100  0.200  0.000); status=updated; parent={"hipLeft" }; child={"ankleLeft" }
keypoint["ankleLeft"] = (-0.100  0.100  0.000); status=updated; parent={"kneeLeft" }; child={}
keypoint["hipRight"] = (-0.100  0.200  0.000); status=updated; parent={"hipCenter" }; child={"kneeRight" }
keypoint["kneeRight"] = ( 0.000  0.000  0.000); status=stale; parent={"hipRight" }; child={"ankleRight" }
keypoint["ankleRight"] = ( 0.000  0.000  0.000); status=stale; parent={"kneeRight" }; child={}
```

### From a property-like structure

Assuming that the object `skeleton1` has been previously defined using one of the instructions above, a new `Skeleton` object can be defined as following:

```cpp

    yarp::os::Property prop=skeleton1.toProperty();
    assistive_rehab::Skeleton* skeleton2;
    skeleton2=assistive_rehab::skeleton_factory(prop);
    skeleton2->setTag("properties");
    skeleton2->print();
    delete skeleton2;

```

The result is:

```
tag = "properties"
transformation =
 1.000  0.000  0.000  0.000
 0.000  1.000  0.000  0.000
 0.000  0.000  1.000  0.000
 0.000  0.000  0.000  1.000
coronal = ( 0.000  0.000 -1.000)
sagittal = ( 1.000  0.000  0.000)
transverse = ( 0.000 -1.000  0.000)
keypoint["shoulderCenter"] = ( 0.000  0.000  0.000); status=updated; parent={}; child={"head" "shoulderLeft" "shoulderRight" "hipCenter" }
keypoint["head"] = ( 0.000 -0.100  0.000); status=updated; parent={"shoulderCenter" }; child={}
keypoint["shoulderLeft"] = ( 0.100  0.000  0.000); status=updated; parent={"shoulderCenter" }; child={"elbowLeft" }
keypoint["elbowLeft"] = ( 0.200  0.000  0.000); status=updated; parent={"shoulderLeft" }; child={"handLeft" }
keypoint["handLeft"] = ( 0.300  0.000  0.000); status=updated; parent={"elbowLeft" }; child={}
keypoint["shoulderRight"] = (-0.100  0.000  0.000); status=updated; parent={"shoulderCenter" }; child={"elbowRight" }
keypoint["elbowRight"] = (-0.200  0.000  0.000); status=updated; parent={"shoulderRight" }; child={"handRight" }
keypoint["handRight"] = (-0.300  0.000  0.000); status=updated; parent={"elbowRight" }; child={}
keypoint["hipCenter"] = ( 0.000  0.100  0.000); status=updated; parent={"shoulderCenter" }; child={"hipLeft" "hipRight" }
keypoint["hipLeft"] = ( 0.100  0.100  0.000); status=updated; parent={"hipCenter" }; child={"kneeLeft" }
keypoint["kneeLeft"] = ( 0.100  0.200  0.000); status=updated; parent={"hipLeft" }; child={"ankleLeft" }
keypoint["ankleLeft"] = ( 0.000  0.000  0.000); status=stale; parent={"kneeLeft" }; child={}
keypoint["hipRight"] = (-0.100  0.100  0.000); status=updated; parent={"hipCenter" }; child={"kneeRight" }
keypoint["kneeRight"] = (-0.100  0.200  0.000); status=updated; parent={"hipRight" }; child={"ankleRight" }
keypoint["ankleRight"] = ( 0.000  0.000  0.000); status=stale; parent={"kneeRight" }; child={}

```

## Retrieving a skeleton

A skeleton can be retrieved as a property-like structure from a yarp port:

```cpp

    assistive_rehab::SkeletonWaist skeleton;
    if(yarp::os::Bottle* b=opcPort.read(false))
    {
        yarp::os::Property prop;
        prop.fromString(b->get(0).asList()->toString());
        skeleton.update_fromstd(assistive_rehab::skeleton_factory(prop)->toProperty());
    }

```

or from the OPC as following:

```cpp

    assistive_rehab::SkeletonWaist skeleton;
    yarp::os::Bottle cmd,reply;
    cmd.addVocab(yarp::os::Vocab::encode("ask"));
    yarp::os::Bottle &content=cmd.addList().addList();
    content.addString("skeleton");
    opcPort.write(cmd,reply);
    if(reply.size()>1)
    {
        if(reply.get(0).asVocab()==Vocab::encode("ack"))
        {
            if(yarp::os::Bottle *idField=reply.get(1).asList())
            {
                if(yarp::os::Bottle *idValues=idField->get(1).asList())
                {
                    int id=idValues->get(0).asInt();
                    cmd.clear();
                    cmd.addVocab(Vocab::encode("get"));
                    yarp::os::Bottle &content=cmd.addList().addList();
                    yarp::os::Bottle replyProp;
                    content.addString("id");
                    content.addInt(id);
                    opcPort.write(cmd,replyProp);
                    if(replyProp.get(0).asVocab() == yarp::os::Vocab::encode("ack"))
                    {
                        if(yarp::os::Bottle *propField=replyProp.get(1).asList())
                        {
                            yarp::os::Property prop(propField->toString().c_str());
                            skeleton.update_fromstd(assistive_rehab::skeleton_factory(prop)->toProperty());
                        }
                    }
                }
            }
        }
    }

```

## Accessing single keypoints of a skeleton

If you want to access to a keypoint of the skeleton, you can use the `operator[]` of the class `Skeleton`, by passing as parameter the keypoint's tag.
For example, the following snippet allows you the get the `shoulder_center` coordinates:

```cpp

   yarp::sig::Vector sc=skeleton[assistive_rehab::KeyPointTag::shoulder_center]->getPoint();

```

## Additional functionalities

### Normalization

Some applications might require a normalized skeleton, to avoid having different results for different human physiques.
The normalization provided in the library makes the length of the observed human segments always equal to 1 and can be applied as following:

```cpp

   skeleton.normalize();

```

### Keypoints' reference system

Keypoints in the skeleton are defined with respect to the camera. To change the reference system, given a transformation matrix _T_, you can use the method `setTransformation`, as following:

```cpp

   skeleton.setTransformation(T);

```

!!! tip
    If you want to use the skeleton as reference system, you can create the transformation matrix _T_ from the skeleton's planes:
    ```cpp

       yarp::sig::Vector coronal=skeleton.getCoronal();
       yarp::sig::Vector sagittal=skeleton.getSagittal();
       yarp::sig::Vector transverse=skeleton.getTransverse();
       yarp::sig::Vector p=skeleton[assistive_rehab::KeyPointTag::shoulder_center]->getPoint();
       yarp::sig::Matrix T1(4,4);
       T1.setSubcol(coronal,0,0);
       T1.setSubcol(sagittal,0,1);
       T1.setSubcol(transverse,0,2);
       T1.setSubcol(p,0,3);
       T1(3,3)=1.0;
       T=yarp::math::SE3inv(T1);
       skeleton.setTransformation(T);

    ```
