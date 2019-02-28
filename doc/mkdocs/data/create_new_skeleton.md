# How to manage a skeleton object

This tutorial will explain you the main functionalities to manage a skeleton object for:

- defining a skeleton;
- retrieving a skeleton;
- accessing single keypoints of a skeleton;
- dealing with pixels;
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
        - pixel: vector containing keypoint's image coordinates u,v.
        - child: list containing keypoint's child, specified as position, status, tag.

    An example is the following:

    ```
    (coronal (0.0 0.0 0.0)) (sagittal (-0.577350269189626 -0.577350269189626 -0.577350269189626)) (skeleton (((child (((pixel (10.0 10.0)) (position (0.1 0.1 0.1)) (status updated) (tag head)) ((child (((child (((pixel (40.0 40.0)) (position (0.4 0.4 0.4)) (status updated) (tag handLeft)))) (pixel (30.0 30.0)) (position (0.3 0.3 0.3)) (status updated) (tag elbowLeft)))) (pixel (20.0 20.0)) (position (0.2 0.2 0.2)) (status updated) (tag shoulderLeft)) ((child (((child (((pixel (70.0 70.0)) (position (0.7 0.7 0.7)) (status updated) (tag handRight)))) (pixel (60.0 60.0)) (position (0.6 0.6 0.6)) (status updated) (tag elbowRight)))) (pixel (50.0 50.0)) (position (0.5 0.5 0.5)) (status updated) (tag shoulderRight)) ((child (((child (((pixel (100.0 100.0)) (position (1.0 1.0 1.0)) (status updated) (tag ankleLeft)))) (pixel (90.0 90.0)) (position (0.9 0.9 0.9)) (status updated) (tag kneeLeft)))) (pixel (80.0 80.0)) (position (0.8 0.8 0.8)) (status updated) (tag hipLeft)) ((child (((child (((pixel (130.0 130.0)) (position (1.3 1.3 1.3)) (status updated) (tag ankleRight)))) (pixel (120.0 120.0)) (position (1.2 1.2 1.2)) (status updated) (tag kneeRight)))) (pixel (110.0 110.0)) (position (1.1 1.1 1.1)) (status updated) (tag hipRight)))) (pixel (0.0 0.0)) (position (0.0 0.0 0.0)) (status updated) (tag shoulderCenter)))) (tag test) (transformation (4 4 (1.0 0.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 0.0 1.0))) (transverse (-0.577350269189626 -0.577350269189626 -0.577350269189626)) (type "assistive_rehab::SkeletonStd")
    ```

### From an unordered list of keypoints

The following code snippet creates a `SkeletonWaist` object from an unordered list of keypoints:

```cpp

#include <yarp/sig/Vector.h>
#include "AssistiveRehab/skeleton.h"

int main()
{
    assistive_rehab::SkeletonWaist skeleton;
    std::vector<std::pair<std::string,yarp::sig::Vector>> unordered;
    {
        yarp::sig::Vector p(3); p[0]=0.0; p[1]=0.0; p[2]=0.0;
        unordered.push_back(std::make_pair(assistive_rehab::KeyPointTag::shoulder_center,p));
    }
    {
        yarp::sig::Vector p(3); p[0]=0.0; p[1]=-0.1; p[2]=0.0;
        unordered.push_back(std::make_pair(assistive_rehab::KeyPointTag::head,p));
    }
    {
        yarp::sig::Vector p(3); p[0]=0.1; p[1]=0.0; p[2]=0.0;
        unordered.push_back(std::make_pair(assistive_rehab::KeyPointTag::shoulder_left,p));
    }
    {
        yarp::sig::Vector p(3); p[0]=0.2; p[1]=0.0; p[2]=0.0;
        unordered.push_back(std::make_pair(assistive_rehab::KeyPointTag::elbow_left,p));
    }
    {
        yarp::sig::Vector p(3); p[0]=0.3; p[1]=0.0; p[2]=0.0;
        unordered.push_back(std::make_pair(assistive_rehab::KeyPointTag::hand_left,p));
    }
    {
        yarp::sig::Vector p(3); p[0]=-0.1; p[1]=0.0; p[2]=0.0;
        unordered.push_back(std::make_pair(assistive_rehab::KeyPointTag::shoulder_right,p));
    }
    {
        yarp::sig::Vector p(3); p[0]=-0.2; p[1]=0.0; p[2]=0.0;
        unordered.push_back(std::make_pair(assistive_rehab::KeyPointTag::elbow_right,p));
    }
    {
        yarp::sig::Vector p(3); p[0]=-0.3; p[1]=0.0; p[2]=0.0;
        unordered.push_back(std::make_pair(assistive_rehab::KeyPointTag::hand_right,p));
    }
    {
        yarp::sig::Vector p(3); p[0]=0.1; p[1]=0.1; p[2]=0.0;
        unordered.push_back(std::make_pair(assistive_rehab::KeyPointTag::hip_left,p));
    }
    {
        yarp::sig::Vector p(3); p[0]=0.1; p[1]=0.2; p[2]=0.0;
        unordered.push_back(std::make_pair(assistive_rehab::KeyPointTag::knee_left,p));
    }
    {
        yarp::sig::Vector p(3); p[0]=-0.1; p[1]=0.1; p[2]=0.0;
        unordered.push_back(std::make_pair(assistive_rehab::KeyPointTag::hip_right,p));
    }
    {
        yarp::sig::Vector p(3); p[0]=-0.1; p[1]=0.2; p[2]=0.0;
        unordered.push_back(std::make_pair(assistive_rehab::KeyPointTag::knee_right,p));
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
-1.000  0.000  0.000  0.000
 0.000  1.000  0.000  0.000
-0.000  0.000 -1.000  0.000
 0.000  0.000  0.000  1.000
coronal = (-0.000  0.000  1.000)
sagittal = (-1.000  0.000 -0.000)
transverse = ( 0.000 -1.000  0.000)
keypoint["shoulderCenter"] = ( 0.000  0.000  0.000); pixel=( nan  nan); status=updated; parent={}; child={"head" "shoulderLeft" "shoulderRight" "hipCenter" }
keypoint["head"] = ( 0.000 -0.100  0.000); pixel=( nan  nan); status=updated; parent={"shoulderCenter" }; child={}
keypoint["shoulderLeft"] = (-0.100  0.000 -0.000); pixel=( nan  nan); status=updated; parent={"shoulderCenter" }; child={"elbowLeft" }
keypoint["elbowLeft"] = (-0.200  0.000 -0.000); pixel=( nan  nan); status=updated; parent={"shoulderLeft" }; child={"handLeft" }
keypoint["handLeft"] = (-0.300  0.000 -0.000); pixel=( nan  nan); status=updated; parent={"elbowLeft" }; child={}
keypoint["shoulderRight"] = ( 0.100  0.000  0.000); pixel=( nan  nan); status=updated; parent={"shoulderCenter" }; child={"elbowRight" }
keypoint["elbowRight"] = ( 0.200  0.000  0.000); pixel=( nan  nan); status=updated; parent={"shoulderRight" }; child={"handRight" }
keypoint["handRight"] = ( 0.300  0.000  0.000); pixel=( nan  nan); status=updated; parent={"elbowRight" }; child={}
keypoint["hipCenter"] = ( 0.000  0.100  0.000); pixel=( nan  nan); status=updated; parent={"shoulderCenter" }; child={"hipLeft" "hipRight" }
keypoint["hipLeft"] = (-0.100  0.100 -0.000); pixel=( nan  nan); status=updated; parent={"hipCenter" }; child={"kneeLeft" }
keypoint["kneeLeft"] = (-0.100  0.200 -0.000); pixel=( nan  nan); status=updated; parent={"hipLeft" }; child={"ankleLeft" }
keypoint["ankleLeft"] = ( nan  nan  nan); pixel=( nan  nan); status=stale; parent={"kneeLeft" }; child={}
keypoint["hipRight"] = ( 0.100  0.100  0.000); pixel=( nan  nan); status=updated; parent={"hipCenter" }; child={"kneeRight" }
keypoint["kneeRight"] = ( 0.100  0.200  0.000); pixel=( nan  nan); status=updated; parent={"hipRight" }; child={"ankleRight" }
keypoint["ankleRight"] = ( nan  nan  nan); pixel=( nan  nan); status=stale; parent={"kneeRight" }; child={}
```

!!! note
    Don't get worried about `nan` in the `pixel` fields: it's correct since we didn't set them up yet. Uninitialized points and/or pixels take `nan` values.

### From an ordered list of keypoints

The following code snippet creates a `SkeletonWaist` object from an ordered list of keypoints:

```cpp

#include <yarp/sig/Vector.h>
#include "AssistiveRehab/skeleton.h"

int main()
{
    assistive_rehab::SkeletonWaist skeleton;
    std::vector<yarp::sig::Vector> ordered;
    {
        yarp::sig::Vector p(3); p[0]=0.0; p[1]=0.0; p[2]=0.0;
        ordered.push_back(p);
    }
    {
        yarp::sig::Vector p(3); p[0]=0.0; p[1]=-0.1; p[2]=0.0;
        ordered.push_back(p);
    }
    {
        yarp::sig::Vector p(3); p[0]=0.1; p[1]=0.0; p[2]=0.0;
        ordered.push_back(p);
    }
    {
        yarp::sig::Vector p(3); p[0]=0.2; p[1]=0.0; p[2]=0.0;
        ordered.push_back(p);
    }
    {
        yarp::sig::Vector p(3); p[0]=0.3; p[1]=0.0; p[2]=0.0;
        ordered.push_back(p);
    }
    {
        yarp::sig::Vector p(3); p[0]=-0.1; p[1]=0.0; p[2]=0.0;
        ordered.push_back(p);
    }
    {
        yarp::sig::Vector p(3); p[0]=-0.2; p[1]=0.0; p[2]=0.0;
        ordered.push_back(p);
    }
    {
        yarp::sig::Vector p(3); p[0]=-0.3; p[1]=0.0; p[2]=0.0;
        ordered.push_back(p);
    }
    {
        yarp::sig::Vector p(3); p[0]=0.1; p[1]=0.1; p[2]=0.0;
        ordered.push_back(p);
    }
    {
        yarp::sig::Vector p(3); p[0]=0.1; p[1]=0.2; p[2]=0.0;
        ordered.push_back(p);
    }
    {
        yarp::sig::Vector p(3); p[0]=-0.1; p[1]=0.1; p[2]=0.0;
        ordered.push_back(p);
    }
    {
        yarp::sig::Vector p(3); p[0]=-0.1; p[1]=0.2; p[2]=0.0;
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
keypoint["shoulderCenter"] = ( 0.000  0.000  0.000); pixel=( nan  nan); status=updated; parent={}; child={"head" "shoulderLeft" "shoulderRight" "hipCenter" }
keypoint["head"] = ( 0.000 -0.100  0.000); pixel=( nan  nan); status=updated; parent={"shoulderCenter" }; child={}
keypoint["shoulderLeft"] = ( 0.100  0.000  0.000); pixel=( nan  nan); status=updated; parent={"shoulderCenter" }; child={"elbowLeft" }
keypoint["elbowLeft"] = ( 0.200  0.000  0.000); pixel=( nan  nan); status=updated; parent={"shoulderLeft" }; child={"handLeft" }
keypoint["handLeft"] = ( 0.300  0.000  0.000); pixel=( nan  nan); status=updated; parent={"elbowLeft" }; child={}
keypoint["shoulderRight"] = (-0.100  0.000  0.000); pixel=( nan  nan); status=updated; parent={"shoulderCenter" }; child={"elbowRight" }
keypoint["elbowRight"] = (-0.200  0.000  0.000); pixel=( nan  nan); status=updated; parent={"shoulderRight" }; child={"handRight" }
keypoint["handRight"] = ( 0.000  0.000  0.000); pixel=( nan  nan); status=stale; parent={"elbowRight" }; child={}
keypoint["hipCenter"] = ( 0.000  0.150  0.000); pixel=( nan  nan); status=updated; parent={"shoulderCenter" }; child={"hipLeft" "hipRight" }
keypoint["hipLeft"] = ( 0.100  0.100  0.000); pixel=( nan  nan); status=updated; parent={"hipCenter" }; child={"kneeLeft" }
keypoint["kneeLeft"] = ( 0.100  0.200  0.000); pixel=( nan  nan); status=updated; parent={"hipLeft" }; child={"ankleLeft" }
keypoint["ankleLeft"] = (-0.100  0.100  0.000); pixel=( nan  nan); status=updated; parent={"kneeLeft" }; child={}
keypoint["hipRight"] = (-0.100  0.200  0.000); pixel=( nan  nan); status=updated; parent={"hipCenter" }; child={"kneeRight" }
keypoint["kneeRight"] = ( 0.000  0.000  0.000); pixel=( nan  nan); status=stale; parent={"hipRight" }; child={"ankleRight" }
keypoint["ankleRight"] = ( 0.000  0.000  0.000); pixel=( nan  nan); status=stale; parent={"kneeRight" }; child={}
```

### From a property-like structure

Assuming that the object `skeleton1` has been previously defined using one of the instructions above, a new `Skeleton` object can be defined as following:

```cpp

    yarp::os::Property prop=skeleton1.toProperty();
    std::unique_ptr<assistive_rehab::Skeleton> skeleton2(assistive_rehab::skeleton_factory(prop));
    skeleton2->setTag("properties");
    skeleton2->print();

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
keypoint["shoulderCenter"] = ( 0.000  0.000  0.000); pixel=( nan  nan); status=updated; parent={}; child={"head" "shoulderLeft" "shoulderRight" "hipCenter" }
keypoint["head"] = ( 0.000 -0.100  0.000); pixel=( nan  nan); status=updated; parent={"shoulderCenter" }; child={}
keypoint["shoulderLeft"] = ( 0.100  0.000  0.000); pixel=( nan  nan); status=updated; parent={"shoulderCenter" }; child={"elbowLeft" }
keypoint["elbowLeft"] = ( 0.200  0.000  0.000); pixel=( nan  nan); status=updated; parent={"shoulderLeft" }; child={"handLeft" }
keypoint["handLeft"] = ( 0.300  0.000  0.000); pixel=( nan  nan); status=updated; parent={"elbowLeft" }; child={}
keypoint["shoulderRight"] = (-0.100  0.000  0.000); pixel=( nan  nan); status=updated; parent={"shoulderCenter" }; child={"elbowRight" }
keypoint["elbowRight"] = (-0.200  0.000  0.000); pixel=( nan  nan); status=updated; parent={"shoulderRight" }; child={"handRight" }
keypoint["handRight"] = (-0.300  0.000  0.000); pixel=( nan  nan); status=updated; parent={"elbowRight" }; child={}
keypoint["hipCenter"] = ( 0.000  0.100  0.000); pixel=( nan  nan); status=updated; parent={"shoulderCenter" }; child={"hipLeft" "hipRight" }
keypoint["hipLeft"] = ( 0.100  0.100  0.000); pixel=( nan  nan); status=updated; parent={"hipCenter" }; child={"kneeLeft" }
keypoint["kneeLeft"] = ( 0.100  0.200  0.000); pixel=( nan  nan); status=updated; parent={"hipLeft" }; child={"ankleLeft" }
keypoint["ankleLeft"] = ( 0.000  0.000  0.000); pixel=( nan  nan); status=stale; parent={"kneeLeft" }; child={}
keypoint["hipRight"] = (-0.100  0.100  0.000); pixel=( nan  nan); status=updated; parent={"hipCenter" }; child={"kneeRight" }
keypoint["kneeRight"] = (-0.100  0.200  0.000); pixel=( nan  nan); status=updated; parent={"hipRight" }; child={"ankleRight" }
keypoint["ankleRight"] = ( 0.000  0.000  0.000); pixel=( nan  nan); status=stale; parent={"kneeRight" }; child={}

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
For example, the following snippet allows you the get the `shoulder_center` 3D coordinates:

```cpp

   yarp::sig::Vector sc=skeleton[assistive_rehab::KeyPointTag::shoulder_center]->getPoint();

```

## Dealing with pixels

Occasionally, it might be worth storing also the pixels alongside the points, which are used by the algorithm to reconstruct the 3D skeleton.
This is particularly useful when the skeleton is employed to enable gaze tracking, for example. In this context, the 3D information of the keypoints
needs normally to be transformed from the camera frame to the root frame of the robot. Instead, having the pixels would ease this process. 

The updating methods described above do have their pixel-wise counterparts:

```cpp

    std::vector<std::pair<yarp::sig::Vector,yarp::sig::Vector>> ordered_withpixels;
    yarp::sig::Vector p(3,0.1); yarp::sig::Vector px(2,10.0);
    ordered_withpixels.push_back(std::make_pair(p,px));
    skeleton.update_withpixels(ordered_withpixels);

    std::vector<std::pair<std::string<std::pair<yarp::sig::Vector,yarp::sig::Vector>>> unordered_withpixels;
    yarp::sig::Vector p(3,0.1); yarp::sig::Vector px(2,10.0);
    unordered_withpixels.push_back(std::make_pair(assistive_rehab::KeyPointTag::shoulder_center,std::make_pair(p,px)));
    skeleton.update_withpixels(unordered_withpixels);

    yarp::sig::Vector pixel=skeleton[assistive_rehab::KeyPointTag::shoulder_center].getPixel();
    
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
