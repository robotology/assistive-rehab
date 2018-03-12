#ifndef __METRIC_H__
#define __METRIC_H__

#include <iostream>

#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>

using namespace std;
using namespace yarp::sig;

class Metric
{

public:
//    Metric();
//    virtual ~Metric();
    virtual void print() {;}
};

class Rom : public Metric
{

public:

    string tag_joint;
    int id_joint;
    int n_movements;
    double min;
    double max;
    double az;
    double el;

    Vector elbowLeft;
    Vector elbowRight;
    Vector handLeft;
    Vector handRight;
    Vector head;
    Vector shoulderCenter;
    Vector shoulderLeft;
    Vector shoulderRight;
    Vector hipLeft;
    Vector hipRight;
    Vector kneeLeft;
    Vector kneeRight;

    Rom() {;}
    Rom(const Rom &rom);
    Rom(const string &tag_joint_, int &id_joint_, int &n_movements_,
        double &min_, double &max_, double &az_, double &el_,
        Vector &elbowLeft_, Vector &elbowRight_, Vector &handLeft_, Vector &handRight_,
        Vector &head_, Vector &shoulderCenter_, Vector &shoulderLeft_, Vector &shoulderRight_,
        Vector &hipLeft_, Vector &hipRight_, Vector &kneeLeft_, Vector &kneeRight_);
    void print();

};

#endif
