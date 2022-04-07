/*
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBO_YARP_CONTACT_PLUGIN_HH
#define GAZEBO_YARP_CONTACT_PLUGIN_HH

#include <string>
#include <vector>


#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Vector.h>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/ConfHelpers.hh>

#include <yarp/sig/Vector.h>

namespace gazebo
{

    class contactPlugin : public SensorPlugin
    {
    public:
        contactPlugin();
        virtual ~contactPlugin();

        /**
         * Loads robot model, reads configuration, 
         * opens network wrapper device and opens device driver
         */
        void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

        /**
         * Callback for the WorldUpdateBegin Gazebo event.
         */
        //void OnUpdate(const gazebo::common::UpdateInfo&);

    private:
        sensors::ContactSensorPtr parentSensor;
        event::ConnectionPtr updateConnection;
        std::string robotName;
        std::string partName;

        //yarp::os::Property config;
        yarp::os::BufferedPort<yarp::os::Bottle>* output_port;

        virtual void OnUpdate();

        bool openPorts();

        bool closePorts();

    };
}

#endif