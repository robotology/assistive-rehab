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

#include "contactPlugin.hh"


using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(contactPlugin)

contactPlugin::contactPlugin() : SensorPlugin()
{
}

contactPlugin::~contactPlugin()
{
    closePorts();
    yarp::os::Network::fini();
}

void contactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
    // Get the parent sensor.
    this->parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

    yarp::os::Network::init();
    if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout))
    {
        yError() << "contactPlugin::Load error: yarp network does not seem to be available, is the yarp server running?";
        return;
    }

    // Make sure parent sensor is valid
    if (!this->parentSensor)
    {
        yError() << "contactPlugin: requires a contact sensor.";
        return;
    }

    yInfo() << "name is: " << _sensor->Name();
    partName = _sensor->Name();


    // Connect to the sensor update event
    this->updateConnection = this->parentSensor->ConnectUpdated(std::bind(&contactPlugin::OnUpdate, this));

    // Make sure the parent sensor is active
    this->parentSensor->SetActive(true);

    // Try to open the YARP ports
    if (!this->openPorts()) {
        yError() << "unable to open the port";
        return;
    }
    yInfo() << "ports opened!";
}

bool contactPlugin::openPorts()
{
    yInfo() << "trying to open the port";
    output_port = new yarp::os::BufferedPort<yarp::os::Bottle>();
    std::string portName = "/" + partName + "/contacts:o+"; 
    bool ok = output_port->open(portName);
    yInfo() << "status was" << ok;
    if (ok)
    {
        yInfo() << "Port /collision:o opened successfully";
        return true;
    }
    yInfo() << "port not opened...";
    return false;
}

bool contactPlugin::closePorts()
{
    output_port->close();
    return true;
}

void contactPlugin::OnUpdate()
{
    yarp::os::Bottle msg;

    // get all the contacts
    msgs::Contacts contacts;
    bool got_collision = false;
    std::string collision1_str, collision2_str;
    contacts = this->parentSensor->Contacts();
    for (unsigned int i = 0; i < contacts.contact_size(); ++i)
    {
        got_collision = true;
        yarp::os::Bottle contact_bottle;
        //yInfo() << "Collision between[" << contacts.contact(i).collision1() << "] and [" << contacts.contact(i).collision2() << "]\n";
        collision1_str = "" + contacts.contact(i).collision1();
        //msg.addString(collision1_str);
        contact_bottle.addString(collision1_str);
        collision2_str = "" + contacts.contact(i).collision2();
        //msg.addString(collision2_str);
        contact_bottle.addString(collision2_str);
        //for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
        for (unsigned int j = 0; j < contacts.contact(i).wrench_size(); ++j)
        {
            yInfo() << j << " wrench between [" <<                              
                contacts.contact(i).wrench(j).body_1_name() << "] and [" <<         
                contacts.contact(i).wrench(j).body_2_name() << "]\n";   
            yarp::os::Bottle contact_pair_bottle;
            /*yInfo() << j << " Position:"
                    << contacts.contact(i).position(j).x() << " "
                    << contacts.contact(i).position(j).y() << " "
                    << contacts.contact(i).position(j).z() << "\n";

            yInfo() << j << " Normal:"
                    << contacts.contact(i).normal(j).x() << " "
                    << contacts.contact(i).normal(j).y() << " "
                    << contacts.contact(i).normal(j).z() << "\n";
            yInfo() << "  Depth:" << contacts.contact(i).depth(j) << "\n";

            contact_pair_bottle.addFloat64(contacts.contact(i).position(j).x());
            contact_pair_bottle.addFloat64(contacts.contact(i).position(j).y());
            contact_pair_bottle.addFloat64(contacts.contact(i).position(j).z());
            contact_pair_bottle.addFloat64(contacts.contact(i).normal(j).x());
            contact_pair_bottle.addFloat64(contacts.contact(i).normal(j).y());
            contact_pair_bottle.addFloat64(contacts.contact(i).normal(j).z());
            contact_pair_bottle.addFloat64(contacts.contact(i).depth(j)); 
            */
            if (contacts.contact(i).wrench(j).body_2_wrench().force().x() != 0.0)
            {
                yInfo() << j << " Forces:"
                        << contacts.contact(i).wrench(j).body_2_wrench().force().x() << " "
                        << contacts.contact(i).wrench(j).body_2_wrench().force().y() << " "
                        << contacts.contact(i).wrench(j).body_2_wrench().force().z() << "\n";
            }

            contact_pair_bottle.addFloat64(contacts.contact(i).wrench(j).body_2_wrench().force().x());
            contact_pair_bottle.addFloat64(contacts.contact(i).wrench(j).body_2_wrench().force().y());
            contact_pair_bottle.addFloat64(contacts.contact(i).wrench(j).body_2_wrench().force().z());
            /* 
            msg.addFloat64(contacts.contact(i).position(j).x());
            msg.addFloat64(contacts.contact(i).position(j).y());
            msg.addFloat64(contacts.contact(i).position(j).z());
            msg.addFloat64(contacts.contact(i).normal(j).x());
            msg.addFloat64(contacts.contact(i).normal(j).y());
            msg.addFloat64(contacts.contact(i).normal(j).z());
            msg.addFloat64(contacts.contact(i).depth(j));
            */
            contact_bottle.addList() = contact_pair_bottle;
        }
        msg.addList() = contact_bottle;
    }
    if (got_collision)
    {
        output_port->prepare() = msg;
        output_port->write();
    }
    return;
}