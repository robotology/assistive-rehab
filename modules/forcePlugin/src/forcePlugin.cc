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

#include "forcePlugin.hh"


using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(forcePlugin)

forcePlugin::forcePlugin() : SensorPlugin()
{
}

forcePlugin::~forcePlugin()
{
    closePorts();
    yarp::os::Network::fini();
}

void forcePlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
    // Get the parent sensor.
    this->parentSensor = std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(_sensor);

    yarp::os::Network::init();
    if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout))
    {
        yError() << "forcePlugin::Load error: yarp network does not seem to be available, is the yarp server running?";
        return;
    }

    // Make sure parent sensor is valid
    if (!this->parentSensor)
    {
        yError() << "forcePlugin: requires a force sensor.";
        return;
    }

    yInfo() << "name is: " << _sensor->Name();
    partName = _sensor->Name();


    // Connect to the sensor update event
    this->updateConnection = this->parentSensor->ConnectUpdated(std::bind(&forcePlugin::OnUpdate, this));

    // Make sure the parent sensor is active
    this->parentSensor->SetActive(true);

    // Try to open the YARP ports
    if (!this->openPorts()) {
        yError() << "unable to open the port";
        return;
    }
    yInfo() << "ports opened!";
}

bool forcePlugin::openPorts()
{
    yInfo() << "trying to open the port";
    output_port = new yarp::os::BufferedPort<yarp::os::Bottle>();
    std::string portName = "/" + partName + "/force:o"; 
    bool ok = output_port->open(portName);
    yInfo() << "status was" << ok;
    if (ok)
    {
        yInfo() << "Port /force:o opened successfully";
        return true;
    }
    yInfo() << "port not opened...";
    return false;
}

bool forcePlugin::closePorts()
{
    output_port->close();
    return true;
}

void forcePlugin::OnUpdate()
{
    yarp::os::Bottle msg;

    // get all the forces
    //msgs::forces forces;
    std::string collision1_str, collision2_str;
    msg.addString("Force on XYZ axis:");
    ignition::math::Vector3d force = this->parentSensor->Force();
    msg.addFloat64(force[0]);
    msg.addFloat64(force[1]);
    msg.addFloat64(force[2]);
    yInfo() << "Force: "
            << force[0]
            << force[1]
            << force[2];
    output_port->prepare() = msg;
    output_port->write();
    return;
}
