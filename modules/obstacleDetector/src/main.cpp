/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file main.cpp
 * @authors: Valentina Vasco <valentina.vasco@iit.it>
 */

#include <cstdlib>
#include <cmath>

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Value.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Property.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RpcClient.h>

#include <yarp/dev/IRangefinder2D.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/LaserMeasurementData.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

/****************************************************************/
class Cluster
{
    Vector x,y;

public:
    double xc{0.0};
    double yc{0.0};
    double tlast{0.0};
    Cluster() {}
    Cluster(const double xc_, const double yc_, const double tlast_) : xc(xc_), yc(yc_), tlast(tlast_) { }

    /****************************************************************/
    void updateCluster(const double &xc, const double &yc,const double &tlast)
    {
        this->xc=xc;
        this->yc=yc;
        this->tlast=tlast;
        x.push_back(xc);
        y.push_back(yc);
    }

    /****************************************************************/
    double getXcenter() const
    {
        return xc;
    }

    /****************************************************************/
    double getYcenter() const
    {
        return yc;
    }

    /****************************************************************/
    double getTlast() const
    {
        return tlast;
    }

    /****************************************************************/
    int getNumberPoints() const
    {
        return x.size();
    }
};

/****************************************************************/
class ObstDetector : public RFModule
{
    double period{0.0};
    string module_name{"obstacleDetector"};
    string robot{"SIM_CER_ROBOT"};
    PolyDriver *drv{nullptr};
    IRangefinder2D *iLas{nullptr};
    double min_angle{0.0};
    double max_angle{0.0};
    double angle_step{0.0};
    double min_dist{0.0};
    double max_dist{0.0};
    int hor_res{0};
    int ver_res{0};
    double dist_thresh{1.5};
    double dist_obstacle{1.5};
    double time_to_live{1.0};
    int min_points{5};

    std::vector<Cluster> clusters;
    RpcClient navPort;

    /****************************************************************/
    bool configure(ResourceFinder& rf) override
    {
        period=rf.check("period",Value(0.1)).asDouble();
        module_name=rf.check("name",Value("obstacleDetector")).asString();
        robot=rf.check("robot",Value("SIM_CER_ROBOT")).asString();
        dist_thresh=rf.check("dist-thresh",Value(1.5)).asDouble();
        dist_obstacle=rf.check("dist-obstacle",Value(1.5)).asDouble();
        time_to_live=rf.check("time-to-live",Value(1.0)).asDouble();
        min_points=rf.check("min-points",Value(5)).asInt();

        navPort.open("/"+module_name+"/nav:rpc");
        string front_laser_port_name="/"+module_name+"/front_laser:i";

        drv=new yarp::dev::PolyDriver;
        Property options;
        options.put("device","Rangefinder2DClient");
        options.put("local",front_laser_port_name);
        options.put("remote","/"+robot+"/laser/front:o");
        bool b=drv->open(options);
        if (!b)
        {
            yError()<<"Unable to open polydriver";
            delete drv;
            return false;
        }
        drv->view(iLas);
        if (!iLas)
        {
            yError()<<"Unable to get IRangefinder2D interface";
            delete drv;
            return false;
        }
        iLas->getScanLimits(min_angle,max_angle);
        iLas->getHorizontalResolution(angle_step);
        iLas->getDistanceRange(min_dist,max_dist);
        hor_res=(int)((max_angle-min_angle)/angle_step);
        ver_res=(int)((max_dist-min_dist)/0.5);
        yInfo()<<"Reading laser with angle range ("<<min_angle<<max_angle
              <<") and distance range ("<<min_dist<<max_dist<<")";
        return true;
    }

    /****************************************************************/
    double getPeriod() override
    {
        return period;
    }

    /****************************************************************/
    bool updateModule() override
    {
        std::vector<LaserMeasurementData> data;
        if(iLas->getLaserMeasurement(data))
        {
            for (int i=0;i<data.size();i++)
            {
                double x,y;
                data[i].get_cartesian(x,y);
                if (std::isinf(x) || std::isinf(y))
                {
                    continue;
                }
                if (clusters.size()>0)
                {
                    bool found=false;
                    for (int j=0;j<clusters.size();j++)
                    {
                        double xc=clusters[j].getXcenter();
                        double yc=clusters[j].getYcenter();
                        double d=sqrt((x-xc)*(x-xc)+(y-yc)*(y-yc));
                        //we associate the point to the closest cluster
                        if (d<dist_thresh)
                        {
                            clusters[j].updateCluster(x,y,Time::now());
                            found=true;
                            break;
                        }
                    }
                    //if no cluster is found, we create a new one
                    if (!found)
                    {
                        Cluster c;
                        c.updateCluster(x,y,Time::now());
                        clusters.push_back(c);
                    }
                }
                else
                {
                    //we create a new cluster
                    Cluster c;
                    c.updateCluster(x,y,Time::now());
                    clusters.push_back(c);
                }
            }
            //we filter out clusters with few points or not updated for more than time_to_live
            for (int i=0;i<clusters.size();i++)
            {
                int n=clusters[i].getNumberPoints();
                double deltat=Time::now()-clusters[i].getTlast();
                if (n<min_points || deltat>time_to_live)
                {
                    yInfo()<<"Removing cluster"<<i<<"with number of points"<<n
                           <<"and deltat"<<deltat;
                    clusters.erase(clusters.begin()+i);
                }
            }
            yInfo()<<"Number of obstacles found"<<clusters.size();

            double xrobot,yrobot;
            if (getRobotLocation(xrobot,yrobot))
            {
                for (int i=0;i<clusters.size();i++)
                {
                    double xc=clusters[i].getXcenter();
                    double yc=clusters[i].getYcenter();
                    double d=sqrt((xrobot-xc)*(xrobot-xc)+(yrobot-yc)*(yrobot-yc));
                    if (d<dist_obstacle)
                    {
                        yInfo()<<"Found obstacle at"<<d;
                        yInfo()<<"Stopping";
                        stopNav();
                    }
                }
            }
        }
        return true;
    }

    /****************************************************************/
    bool getRobotLocation(double &x, double y)
    {
        Bottle cmd,rep;
        cmd.addString("get_state");
        if (navPort.write(cmd,rep))
        {
            Property robotState(rep.get(0).toString().c_str());
            if (Bottle *loc=robotState.find("robot-location").asList())
            {
                x=loc->get(0).asDouble();
                y=loc->get(1).asDouble();
                double theta=loc->get(2).asDouble();
                return true;
            }
        }
        return false;
    }

    /****************************************************************/
    bool stopNav()
    {
        Bottle cmd,rep;
        cmd.addString("stop");
        if (navPort.write(cmd,rep))
        {
            if (rep.get(0).asVocab()==Vocab::encode("ok"))
            {
                return true;
            }
        }
        return false;
    }

    /****************************************************************/
    bool interruptModule() override
    {
        navPort.interrupt();
        return true;
    }

    /****************************************************************/
    bool close() override
    {
        navPort.close();
        if (drv)
        {
            delete drv;
        }
        return true;
    }

};

/****************************************************************/
int main(int argc, char* argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError() << "Unable to connect to YARP server";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.configure(argc, argv);

    ObstDetector detector;
    return detector.runModule(rf);
}

