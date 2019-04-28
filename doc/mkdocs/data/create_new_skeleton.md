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
    - 8: `hip_center`
    - 9: `hip_left`
    - 10: `knee_left`
    - 11: `ankle_left`
    - 12: `foot_left`
    - 13: `hip_right`
    - 14: `knee_right`
    - 15: `ankle_right`
    - 16: `foot_right`

- [property-like structure](#from-a-property-like-structure): the available properties are the following:

    - type: string containing skeleton's type ("assistive_rehab::SkeletonStd").
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
    (coronal (-0.0171187190241832 0.168776145353523 -0.984581522687332)) (sagittal (0.999852451902468 0.0040979367701588 -0.0166817666585492)) (skeleton (((child (((pixel (143.0 18.0)) (position (-0.0958624515848492 -0.655793450456223 2.01140785217285)) (status updated) (tag head)) ((child (((child (((pixel (182.0 106.0)) (position (0.132475296984726 -0.0970469850595488 2.24838447570801)) (status updated) (tag handLeft)))) (pixel (179.0 77.0)) (position (0.126047049721491 -0.301864959631551 2.20449256896973)) (status updated) (tag elbowLeft)))) (pixel (173.0 40.0)) (position (0.0829495294433831 -0.549497736338085 2.17046546936035)) (status updated) (tag shoulderLeft)) ((child (((child (((pixel (117.0 102.0)) (position (-0.318757589652556 -0.129758383247854 2.30834770202637)) (status updated) (tag handRight)))) (pixel (124.0 75.0)) (position (-0.265819393982889 -0.319631634579399 2.22591972351074)) (status updated) (tag elbowRight)))) (pixel (132.0 42.0)) (position (-0.194277581569445 -0.550633963158095 2.17509078979492)) (status updated) (tag shoulderRight)) ((child (((child (((child (((child (((pixel (153.0 200.0)) (position (-0.0305329581785242 0.615750619982796 2.40194511413574)) (status updated) (tag footLeft)))) (pixel (156.0 187.0)) (position (-0.0161527965668162 0.538963623868779 2.52594947814941)) (status updated) (tag ankleLeft)))) (pixel (158.0 148.0)) (position (-0.00361483866030924 0.218990793550177 2.42797660827637)) (status updated) (tag kneeLeft)))) (pixel (164.0 105.0)) (position (0.0249104292284522 -0.102991968370345 2.2304573059082)) (status updated) (tag hipLeft)) ((child (((child (((child (((pixel (138.0 194.0)) (position (-0.170998545149134 0.592533010093213 2.49751472473145)) (status updated) (tag footRight)))) (pixel (144.0 182.0)) (position (-0.130142124436922 0.509498389750101 2.55434036254883)) (status updated) (tag ankleRight)))) (pixel (142.0 147.0)) (position (-0.133305882322194 0.209480672220591 2.39202308654785)) (status updated) (tag kneeRight)))) (pixel (138.0 103.0)) (position (-0.158599348153779 -0.109193487775799 2.21346664428711)) (status updated) (tag hipRight)))) (pixel (151.0 103.0)) (position (-0.0738603465810386 -0.102184160595443 2.20890045166016)) (status updated) (tag hipCenter)))) (pixel (153.0 41.0)) (position (-0.054339762871519 -0.540136057902244 2.13348770141602)) (status updated) (tag shoulderCenter)))) (tag "#7") (transformation (4 4 (1.0 0.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 0.0 1.0))) (transverse (0.0438836580749896 -0.984546958261919 -0.169533216601232)) (type "assistive_rehab::SkeletonStd")
    ```

### From an unordered list of keypoints

The following code snippet creates a `SkeletonStd` object from an unordered list of keypoints:

```cpp

#include <yarp/sig/Vector.h>
#include "AssistiveRehab/skeleton.h"

int main()
{
    assistive_rehab::SkeletonStd skeleton;
    std::vector<std::pair<std::string,yarp::sig::Vector>> unordered;
    {
        yarp::sig::Vector p(3); p[0]=0.0; p[1]=0.0; p[2]=0.0;
        unordered.push_back(std::make_pair(KeyPointTag::shoulder_center,p));
    }
    {
        yarp::sig::Vector p(3); p[0]=0.0; p[1]=-0.1; p[2]=0.0;
        unordered.push_back(std::make_pair(KeyPointTag::head,p));
    }
    {
        yarp::sig::Vector p(3); p[0]=0.1; p[1]=0.0; p[2]=0.0;
        unordered.push_back(std::make_pair(KeyPointTag::shoulder_left,p));
    }
    {
        yarp::sig::Vector p(3); p[0]=0.2; p[1]=0.0; p[2]=0.0;
        unordered.push_back(std::make_pair(KeyPointTag::elbow_left,p));
    }
    {
        yarp::sig::Vector p(3); p[0]=0.3; p[1]=0.0; p[2]=0.0;
        unordered.push_back(std::make_pair(KeyPointTag::hand_left,p));
    }
    {
        yarp::sig::Vector p(3); p[0]=-0.1; p[1]=0.0; p[2]=0.0;
        unordered.push_back(std::make_pair(KeyPointTag::shoulder_right,p));
    }
    {
        yarp::sig::Vector p(3); p[0]=-0.2; p[1]=0.0; p[2]=0.0;
        unordered.push_back(std::make_pair(KeyPointTag::elbow_right,p));
    }
    {
        yarp::sig::Vector p(3); p[0]=-0.3; p[1]=0.0; p[2]=0.0;
        unordered.push_back(std::make_pair(KeyPointTag::hand_right,p));
    }
    {
        yarp::sig::Vector p(3); p[0]=0.0; p[1]=0.1; p[2]=0.0;
        unordered.push_back(std::make_pair(KeyPointTag::hip_center,p));
    }
    {
        yarp::sig::Vector p(3); p[0]=0.1; p[1]=0.1; p[2]=0.0;
        unordered.push_back(std::make_pair(KeyPointTag::hip_left,p));
    }
    {
        yarp::sig::Vector p(3); p[0]=0.1; p[1]=0.2; p[2]=0.0;
        unordered.push_back(std::make_pair(KeyPointTag::knee_left,p));
    }
    {
        yarp::sig::Vector p(3); p[0]=-0.1; p[1]=0.1; p[2]=0.0;
        unordered.push_back(std::make_pair(KeyPointTag::hip_right,p));
    }
    {
        yarp::sig::Vector p(3); p[0]=-0.1; p[1]=0.2; p[2]=0.0;
        unordered.push_back(std::make_pair(KeyPointTag::knee_right,p));
    }

    skeleton.setTag("unordered");
    skeleton.update(unordered);
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
keypoint["ankleLeft"] = ( nan  nan  nan); pixel=( nan  nan); status=stale; parent={"kneeLeft" }; child={"footLeft" }
keypoint["footLeft"] = ( nan  nan  nan); pixel=( nan  nan); status=stale; parent={"ankleLeft" }; child={}
keypoint["hipRight"] = ( 0.100  0.100  0.000); pixel=( nan  nan); status=updated; parent={"hipCenter" }; child={"kneeRight" }
keypoint["kneeRight"] = ( 0.100  0.200  0.000); pixel=( nan  nan); status=updated; parent={"hipRight" }; child={"ankleRight" }
keypoint["ankleRight"] = ( nan  nan  nan); pixel=( nan  nan); status=stale; parent={"kneeRight" }; child={"footRight" }
keypoint["footRight"] = ( nan  nan  nan); pixel=( nan  nan); status=stale; parent={"ankleRight" }; child={}
```

!!! note
    Don't get worried about `nan` in the `pixel` fields: it's correct since we didn't set them up yet. Uninitialized points and/or pixels take `nan` values.

### From an ordered list of keypoints

The following code snippet creates a `SkeletonStd` object from an ordered list of keypoints:

```cpp

#include <yarp/sig/Vector.h>
#include "AssistiveRehab/skeleton.h"

int main()
{
    assistive_rehab::SkeletonStd skeleton;
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
    skeleton.update(ordered);
    skeleton.print();

    return EXIT_SUCCESS;
}

```

The result is:

```
tag = "ordered"
transformation =
-1.000  0.000  0.000  0.000
 0.000  1.000  0.000  0.000
-0.000  0.000 -1.000  0.000
 0.000  0.000  0.000  1.000
coronal = (-0.000  0.000  0.707)
sagittal = (-1.000  0.000 -0.000)
transverse = ( 0.707 -0.707  0.000)
keypoint["shoulderCenter"] = ( 0.000  0.000  0.000); pixel=( nan  nan); status=updated; parent={}; child={"head" "shoulderLeft" "shoulderRight" "hipCenter" }
keypoint["head"] = ( 0.000 -0.100  0.000); pixel=( nan  nan); status=updated; parent={"shoulderCenter" }; child={}
keypoint["shoulderLeft"] = (-0.100  0.000 -0.000); pixel=( nan  nan); status=updated; parent={"shoulderCenter" }; child={"elbowLeft" }
keypoint["elbowLeft"] = (-0.200  0.000 -0.000); pixel=( nan  nan); status=updated; parent={"shoulderLeft" }; child={"handLeft" }
keypoint["handLeft"] = (-0.300  0.000 -0.000); pixel=( nan  nan); status=updated; parent={"elbowLeft" }; child={}
keypoint["shoulderRight"] = ( 0.100  0.000  0.000); pixel=( nan  nan); status=updated; parent={"shoulderCenter" }; child={"elbowRight" }
keypoint["elbowRight"] = ( 0.200  0.000  0.000); pixel=( nan  nan); status=updated; parent={"shoulderRight" }; child={"handRight" }
keypoint["handRight"] = ( 0.300  0.000  0.000); pixel=( nan  nan); status=updated; parent={"elbowRight" }; child={}
keypoint["hipCenter"] = (-0.100  0.100 -0.000); pixel=( nan  nan); status=updated; parent={"shoulderCenter" }; child={"hipLeft" "hipRight" }
keypoint["hipLeft"] = (-0.100  0.200 -0.000); pixel=( nan  nan); status=updated; parent={"hipCenter" }; child={"kneeLeft" }
keypoint["kneeLeft"] = ( 0.100  0.100  0.000); pixel=( nan  nan); status=updated; parent={"hipLeft" }; child={"ankleLeft" }
keypoint["ankleLeft"] = ( 0.100  0.200  0.000); pixel=( nan  nan); status=updated; parent={"kneeLeft" }; child={"footLeft" }
keypoint["footLeft"] = ( nan  nan  nan); pixel=( nan  nan); status=stale; parent={"ankleLeft" }; child={}
keypoint["hipRight"] = ( nan  nan  nan); pixel=( nan  nan); status=stale; parent={"hipCenter" }; child={"kneeRight" }
keypoint["kneeRight"] = ( nan  nan  nan); pixel=( nan  nan); status=stale; parent={"hipRight" }; child={"ankleRight" }
keypoint["ankleRight"] = ( nan  nan  nan); pixel=( nan  nan); status=stale; parent={"kneeRight" }; child={"footRight" }
keypoint["footRight"] = ( nan  nan  nan); pixel=( nan  nan); status=stale; parent={"ankleRight" }; child={}
```

### From a property-like structure

Assuming that the object `skeleton1` has been previously defined using one of the instructions above, a new `Skeleton` object can be defined as following:

```cpp

    yarp::os::Property prop=skeleton1.toProperty();
    std::unique_ptr<assistive_rehab::Skeleton> skeleton2(assistive_rehab::skeleton_factory(prop));
    skeleton2->setTag("properties");
    skeleton2->print();

```

## Retrieving a skeleton

A skeleton can be retrieved as a property-like structure from a yarp port:

```cpp

    assistive_rehab::SkeletonStd skeleton;
    if(yarp::os::Bottle* b=opcPort.read(false))
    {
        yarp::os::Property prop;
        prop.fromString(b->get(0).asList()->toString());
        skeleton.update(assistive_rehab::skeleton_factory(prop)->toProperty());
    }

```

or from the OPC as following:

```cpp

    assistive_rehab::SkeletonStd skeleton;
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
                            skeleton.update(assistive_rehab::skeleton_factory(prop)->toProperty());
                        }
                    }
                }
            }
        }
    }

```

## Accessing single keypoints of a skeleton

If you want to access to a keypoint of the skeleton, you can use the `operator[]` of the class `Skeleton`, by passing as parameter the keypoint's tag.
For example, the following snippet allows you to get the `shoulder_center` 3D coordinates:

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
