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
#include <utility>
#include <iostream>
#include <yarp/sig/Vector.h>
#include "AssistiveRehab/skeleton.h"

using namespace std;
using namespace yarp::sig;
using namespace assistive_rehab;

void print_hierarchy(const KeyPoint *k)
{
    cout<<"keypoint[\""<<k->getTag()<<"\"]; ("
        <<k->getPoint().toString(3,3)<<") "
        <<(k->isUpdated()?"updated":"stale")<<endl;
    for (unsigned int i=0; i<k->getNumChild(); i++)
        print_hierarchy(k->getChild(i));
}

int main()
{
    cout<<"### Defining the Skeleton"<<endl;
    SkeletonWaist skeleton;

    cout<<"### Printing the Skeleton's structure (internal method)"<<endl;
    skeleton.print();
    cout<<endl;

    cout<<"### Printing the Skeleton's structure (using recursive access)"<<endl;
    print_hierarchy(skeleton[0]);
    cout<<endl;

    vector<pair<string, Vector>> unordered;
    {
        Vector p(3,0.0); p[0]=0.0; p[1]=0.0; p[2]=0.0;
        unordered.push_back(make_pair(KeyPointTag::shoulder_center,p));
    }
    {
        Vector p(3,0.0); p[0]=0.0; p[1]=0.1; p[2]=0.0;
        unordered.push_back(make_pair(KeyPointTag::head,p));
    }
    {
        Vector p(3,0.0); p[0]=-0.1; p[1]=0.0; p[2]=0.0;
        unordered.push_back(make_pair(KeyPointTag::shoulder_left,p));
    }
    {
        Vector p(3,0.0); p[0]=-0.2; p[1]=0.0; p[2]=0.0;
        unordered.push_back(make_pair(KeyPointTag::elbow_left,p));
    }
    {
        Vector p(3,0.0); p[0]=-0.3; p[1]=0.0; p[2]=0.0;
        unordered.push_back(make_pair(KeyPointTag::hand_left,p));
    }
    {
        Vector p(3,0.0); p[0]=0.1; p[1]=0.0; p[2]=0.0;
        unordered.push_back(make_pair(KeyPointTag::shoulder_right,p));
    }
    {
        Vector p(3,0.0); p[0]=0.2; p[1]=0.0; p[2]=0.0;
        unordered.push_back(make_pair(KeyPointTag::elbow_right,p));
    }
    {
        Vector p(3,0.0); p[0]=0.3; p[1]=0.0; p[2]=0.0;
        unordered.push_back(make_pair(KeyPointTag::hand_right,p));
    }
    {
        Vector p(3,0.0); p[0]=-0.1; p[1]=-0.1; p[2]=0.0;
        unordered.push_back(make_pair(KeyPointTag::hip_left,p));
    }
    {
        Vector p(3,0.0); p[0]=-0.1; p[1]=-0.2; p[2]=0.0;
        unordered.push_back(make_pair(KeyPointTag::knee_left,p));
    }
    {
        Vector p(3,0.0); p[0]=0.1; p[1]=-0.1; p[2]=0.0;
        unordered.push_back(make_pair(KeyPointTag::hip_right,p));
    }
    {
        Vector p(3,0.0); p[0]=0.1; p[1]=-0.2; p[2]=0.0;
        unordered.push_back(make_pair(KeyPointTag::knee_right,p));
    }

    cout<<"### Updating Skeleton's structure"<<endl;
    skeleton.update_fromstd(unordered);
    skeleton.print();
    cout<<endl;

    cout<<"### Normalizing Skeleton's structure"<<endl;
    skeleton.normalize();
    skeleton.print();
    cout<<endl;

    return EXIT_SUCCESS;
}
