#include "Metric.h"

Rom::Rom(const Rom &rom)
{
    tag_joint = rom.tag_joint;
    id_joint = rom.id_joint;
    n_movements = rom.n_movements;
    min = rom.min;
    max = rom.max;
}

Rom::Rom(const string &tag_joint_, int &id_joint_, int &n_movements_,
         double &min_, double &max_, Vector &elbowLeft_, Vector &elbowRight_,
         Vector &handLeft_, Vector &handRight_, Vector &head_, Vector &shoulderCenter_,
         Vector &shoulderLeft_, Vector &shoulderRight_, Vector &hipLeft_, Vector &hipRight_,
         Vector &kneeLeft_, Vector &kneeRight_)
{
    tag_joint = tag_joint_;
    id_joint = id_joint_;
    n_movements = n_movements_;
    min = min_;
    max = max_;
    elbowLeft = elbowLeft_;
    elbowRight = elbowRight_;
    handLeft = handLeft_;
    handRight = handRight_;
    head = head_;
    shoulderCenter = shoulderCenter_;
    shoulderLeft = shoulderLeft_;
    shoulderRight = shoulderRight_;
    hipLeft = hipLeft_;
    hipRight = hipRight_;
    kneeLeft = kneeLeft_;
    kneeRight = kneeRight_;
}

void Rom::update(Vector &elbowLeft_, Vector &elbowRight_, Vector &handLeft_,
                 Vector &handRight_, Vector &head_, Vector &shoulderCenter_,
                 Vector &shoulderLeft_, Vector &shoulderRight_, Vector &hipLeft_,
                 Vector &hipRight_, Vector &kneeLeft_, Vector &kneeRight_)
{
    elbowLeft = elbowLeft_;
    elbowRight = elbowRight_;
    handLeft = handLeft_;
    handRight = handRight_;
    head = head_;
    shoulderCenter = shoulderCenter_;
    shoulderLeft = shoulderLeft_;
    shoulderRight = shoulderRight_;
    hipLeft = hipLeft_;
    hipRight = hipRight_;
    kneeLeft = kneeLeft_;
    kneeRight = kneeRight_;
}

void Rom::print()
{
    yInfo() << "Tag joint = " << tag_joint;
    yInfo() << "id joint = " << id_joint;
    yInfo() << "Number of movements = " << n_movements;
    yInfo() << "Min = " << min;
    yInfo() << "Max = " << max;
    yInfo() << "elbowLeft = (" << elbowLeft[0] << elbowLeft[1] << elbowLeft[2] << ")";
    if(elbowLeft[3] == 0)
        yInfo() << "stationary";
    else
        yInfo() << "mobile";
    yInfo() << "elbowRight = (" << elbowRight[0] << elbowRight[1] << elbowRight[2] << ")";
    if(elbowRight[3] == 0)
        yInfo() << "stationary" << "\n";
    else
        yInfo() << "mobile" << "\n";
}
