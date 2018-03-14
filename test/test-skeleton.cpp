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
    cout<<"keypoint["<<k->getId()<<"]; \""
        <<k->getTag()<<"\": ("
        <<k->getPoint().toString(3,3)<<") "
        <<(k->isUpdated()?"updated":"stale")<<endl;
    for (unsigned int i=0; i<k->getNumChild(); i++)
        print_hierarchy(k->getChild(i));
}

int main()
{
    cout<<"### Defining the Skeleton"<<endl;
    SkeletonStd skeleton;

    cout<<"### Printing the Skeleton's structure (internal method)"<<endl;
    skeleton.print();
    cout<<endl;

    cout<<"### Printing the Skeleton's structure (using recursive access)"<<endl;
    print_hierarchy(skeleton[0]);
    cout<<endl;

    Vector p1(3,1.0), p2(3,2.0);
    vector<pair<string, Vector>> unordered;
    unordered.push_back(make_pair(KeyPointTag::elbow_right,p1));
    unordered.push_back(make_pair(KeyPointTag::elbow_left,p2));

    cout<<"### Updating Skeleton's structure"<<endl;
    skeleton.update(unordered);
    skeleton.print();

    return EXIT_SUCCESS;
}
