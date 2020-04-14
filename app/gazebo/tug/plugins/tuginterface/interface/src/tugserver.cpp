// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Valentina Vasco
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 *
 */

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <include/utils.h>
#include <include/tugserver.h>

using namespace std;
using namespace gazebo;

/****************************************************************/
TugServer::TugServer() : world(0), actor(0), numwaypoints(0), speed(0.0)
{

}

/****************************************************************/
TugServer::~TugServer()
{

}

/****************************************************************/
bool TugServer::start()
{
    actor->Play();
    yInfo()<<"Playing script";
    return true;
}

/****************************************************************/
bool TugServer::stop()
{
    actor->Stop();
    yInfo()<<"Stop script";
    return true;
}

/****************************************************************/
bool TugServer::pause(const double time)
{
    yInfo()<<"Pausing for"<<time<<"seconds";
    double t0=yarp::os::Time::now();
    world->SetPaused(true);
    while(true)
    {
        if ((yarp::os::Time::now()-t0)>=time)
        {
            world->SetPaused(false);
            break;
        }
    }
    return true;
}


/****************************************************************/
bool TugServer::setSpeed(const double speed)
{
    this->speed=speed;
    wp_map.clear();
    wp_map=createMap(targets,speed);
    wp_map=generateWaypoints(numwaypoints,speed,wp_map);

    sdf::ElementPtr world_sdf=world->SDF();
    sdf::ElementPtr actor_sdf=world_sdf->GetElement("actor");
    updateScript(actor_sdf,wp_map);
    actor->UpdateParameters(actor_sdf);

    yInfo()<<"Setting speed to"<<speed;
    return true;
}

/****************************************************************/
double TugServer::getSpeed()
{
    return this->speed;
}

/****************************************************************/
void TugServer::init(const double &speed, const int numwaypoints,
                     const yarp::sig::Matrix &targets)
{
    this->speed=speed;
    this->numwaypoints=numwaypoints;
    this->targets=targets;
}






