/******************************************************************************
* Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
*
* This program is free software; you can redistribute it and/or modify it under
* the terms of the GNU General Public License as published by the Free Software
* Foundation; either version 2 of the License, or (at your option) any later
* version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
* FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
* details.
*
* You should have received a copy of the GNU General Public License along with
* this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.                                                                     *
 ******************************************************************************/
/**
 * @authors: Alexandre Antunes <alexandre.gomespereira@iit.it>
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
