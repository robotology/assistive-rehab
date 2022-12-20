#include <iostream>
#include <mutex>
#include <condition_variable>

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcClient.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>
#include <yarp/cv/Cv.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IRGBDSensor.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/filters.h>

#include "detector.h"

using namespace yarp::os;
using namespace yarp::math;

/****************************************************************/
int Detector::opcCheck(const std::string &line_tag)
{
    int id=-1;
    if (m_opcPort.getOutputCount())
    {
        Bottle cmd,rep;
        cmd.addVocab32("ask");

        Bottle &bLine=cmd.addList();
        Bottle &b1=bLine.addList();
        b1.addString(line_tag);
        if (m_opcPort.write(cmd,rep))
        {
            if (rep.get(0).asVocab32()==Vocab32::encode("ack"))
            {
                if (Bottle *idB=rep.get(1).asList())
                {
                    if (Bottle *idList=idB->get(1).asList())
                    {
                        if (idList->size()>0)
                        {
                            id=idList->get(0).asInt32();
                            opcGetLine(id,line_tag);
                        }
                    }
                }
            }
        }
    }
    return id;
}

/****************************************************************/
yarp::os::Property Detector::opcGetLine(const int id, const std::string &line_tag)
{
    Property prop;
    if (m_opcPort.getOutputCount())
    {
        Bottle cmd,rep;
        cmd.addVocab32("get");

        Bottle &l=cmd.addList();
        l.addString("id");
        l.addInt32(id);
        if (m_opcPort.write(cmd,rep))
        {
            if (rep.get(0).asVocab32()==Vocab32::encode("ack"))
            {
                Property lineProp(rep.get(1).toString().c_str());
                if (Bottle *b=lineProp.find(line_tag).asList())
                {
                    prop.fromString(b->toString().c_str());
                }
            }
        }
    }
    return prop;
}

/****************************************************************/
bool Detector::opcAdd(const std::string &line_tag)
{
    int opc_id=opcCheck(line_tag);
    int i= m_line2idx[line_tag];
    return opcAdd(m_lines_pose_world[i], m_lines_size[i],opc_id,line_tag);
}

/****************************************************************/
bool Detector::opcAdd(const yarp::sig::Vector &pose_world, const yarp::sig::Vector &line_size,
            const int opc_id, const std::string &line_tag)
{
    if (m_opcPort.getOutputCount()>0)
    {
        Bottle cmd,rep;
        cmd.addVocab32("add");

        Bottle bPoseWorld;
        Property poseProp;
        bPoseWorld.addList().read(pose_world);
        poseProp.put("pose_world",bPoseWorld.get(0));

        Bottle bSize;
        bSize.addList().read(line_size);
        poseProp.put("size",bSize.get(0));

        Property prop;
        Bottle line;
        line.addList().read(poseProp);
        prop.put(line_tag,line.get(0));
        if(opc_id<0)
        {
            cmd.addList().read(prop);
            if (m_opcPort.write(cmd,rep))
            {
                if (rep.get(0).asVocab32()==Vocab32::encode("ack"))
                {
                    int id=rep.get(1).asList()->get(1).asInt32();
                    return opcSet(prop,id);
                }
            }
        }
        else
        {
            return opcSet(prop,opc_id);
        }
    }

    return false;
}

/****************************************************************/
bool Detector::opcSet(const yarp::os::Property &prop, const int id_line)
{
    if (m_opcPort.getOutputCount()>0)
    {
        Bottle cmd,rep;
        cmd.addVocab32("set");
        Bottle &pl=cmd.addList();
        pl.read(prop);
        Bottle id;
        Bottle &id_pl=id.addList();
        id_pl.addString("id");
        id_pl.addInt32(id_line);
        pl.append(id);
        if (m_opcPort.write(cmd,rep))
        {
            return (rep.get(0).asVocab32()==Vocab32::encode("ack"));
        }
    }

    return false;
}

/****************************************************************/
bool Detector::opcDel(const int id)
{
    if (m_opcPort.getOutputCount()>0)
    {
        Bottle cmd,rep;
        cmd.addVocab32("del");
        Bottle &pl=cmd.addList().addList();
        pl.addString("id");
        pl.addInt32(id);
        if (m_opcPort.write(cmd,rep))
        {
            if(rep.get(0).asVocab32()==Vocab32::encode("ack"))
            {
                yInfo()<<"Line with id"<<id<<"removed from opc";
                return true;
            }
        }
    }

    return false;
}
