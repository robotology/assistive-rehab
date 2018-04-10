/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file test-skeleton.cpp
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#include <cstdlib>
#include <memory>
#include <cmath>
#include <utility>
#include <iostream>
#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include "AssistiveRehab/skeleton.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace assistive_rehab;

void print_hierarchy(const KeyPoint *k)
{
    cout<<"keypoint[\""<<k->getTag()<<"\"] = ("
        <<k->getPoint().toString(3,3)<<"); status="
        <<(k->isUpdated()?"updated":"stale")<<endl;
    for (unsigned int i=0; i<k->getNumChild(); i++)
        print_hierarchy(k->getChild(i));
}

int main()
{
    cout<<"### Defining the Skeleton"<<endl;
    SkeletonWaist skeleton1;
    skeleton1.setTag("test");

    Vector rot(4,0.0); rot[1]=1.0; rot[3]=M_PI;
    skeleton1.setTransformation(axis2dcm(rot));

    cout<<"### Printing the Skeleton's structure (internal method)"<<endl;
    skeleton1.print();
    cout<<endl;

    cout<<"### Printing the Skeleton's structure (using recursive access)"<<endl;
    print_hierarchy(skeleton1[0]);
    cout<<endl;

    vector<pair<string,Vector>> unordered;
    {
        Vector p(3,0.0); p[0]=0.0; p[1]=0.0; p[2]=0.0;
        unordered.push_back(make_pair(KeyPointTag::shoulder_center,p));
    }
    {
        Vector p(3,0.0); p[0]=0.0; p[1]=-0.1; p[2]=0.0;
        unordered.push_back(make_pair(KeyPointTag::head,p));
    }
    {
        Vector p(3,0.0); p[0]=0.1; p[1]=0.0; p[2]=0.0;
        unordered.push_back(make_pair(KeyPointTag::shoulder_left,p));
    }
    {
        Vector p(3,0.0); p[0]=0.2; p[1]=0.0; p[2]=0.0;
        unordered.push_back(make_pair(KeyPointTag::elbow_left,p));
    }
    {
        Vector p(3,0.0); p[0]=0.3; p[1]=0.0; p[2]=0.0;
        unordered.push_back(make_pair(KeyPointTag::hand_left,p));
    }
    {
        Vector p(3,0.0); p[0]=-0.1; p[1]=0.0; p[2]=0.0;
        unordered.push_back(make_pair(KeyPointTag::shoulder_right,p));
    }
    {
        Vector p(3,0.0); p[0]=-0.2; p[1]=0.0; p[2]=0.0;
        unordered.push_back(make_pair(KeyPointTag::elbow_right,p));
    }
    {
        Vector p(3,0.0); p[0]=-0.3; p[1]=0.0; p[2]=0.0;
        unordered.push_back(make_pair(KeyPointTag::hand_right,p));
    }
    {
        Vector p(3,0.0); p[0]=0.1; p[1]=0.1; p[2]=0.0;
        unordered.push_back(make_pair(KeyPointTag::hip_left,p));
    }
    {
        Vector p(3,0.0); p[0]=0.1; p[1]=0.2; p[2]=0.0;
        unordered.push_back(make_pair(KeyPointTag::knee_left,p));
    }
    {
        Vector p(3,0.0); p[0]=-0.1; p[1]=0.1; p[2]=0.0;
        unordered.push_back(make_pair(KeyPointTag::hip_right,p));
    }
    {
        Vector p(3,0.0); p[0]=-0.1; p[1]=0.2; p[2]=0.0;
        unordered.push_back(make_pair(KeyPointTag::knee_right,p));
    }

    cout<<"### Updating Skeleton's structure"<<endl;
    skeleton1.update_fromstd(unordered);
    skeleton1.print();
    cout<<endl;

    cout<<"### Normalizing Skeleton's structure"<<endl;
    skeleton1.normalize();
    skeleton1.print();
    cout<<endl;

    cout<<"### Exporting Skeleton's structure"<<endl;
    Property prop=skeleton1.toProperty();
    cout<<prop.toString()<<endl;
    cout<<endl;

    cout<<"### Importing Skeleton's structure"<<endl;
    unique_ptr<Skeleton> skeleton2(factory(prop));
    cout<<"type = \""<<skeleton2->getType()<<"\""<<endl;
    skeleton2->print();
    cout<<endl;

    return EXIT_SUCCESS;
}
