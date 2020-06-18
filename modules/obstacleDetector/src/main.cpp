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

#include <yarp/math/Math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

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
    PolyDriver *drv_front{nullptr},*drv_back{nullptr};
    IRangefinder2D *iLas_front{nullptr},*iLas_back{nullptr};
    double min_angle_front{0.0},min_angle_back{0.0};
    double max_angle_front{0.0},max_angle_back{0.0};
    double angle_step_front{0.0},angle_step_back{0.0};
    double min_dist_front{0.0},min_dist_back{0.0};
    double max_dist_front{0.0},max_dist_back{0.0};
    int hor_res_front{0},hor_res_back{0};
    int ver_res_front{0},ver_res_back{0};
    double dist_thresh{1.5};
    double dist_obstacle{1.5};
    double time_to_live{1.0};
    int min_points{5};

    std::vector<Cluster> clusters_front,clusters_back;
    RpcClient navPort;

    /****************************************************************/
    bool configure(ResourceFinder& rf) override
    {
        period=rf.check("period",Value(0.1)).asDouble();
        module_name=rf.check("name",Value("obstacleDetector")).asString();
        robot=rf.check("robot",Value("SIM_CER_ROBOT")).asString();
        dist_thresh=rf.check("dist-thresh",Value(0.5)).asDouble();
        dist_obstacle=rf.check("dist-obstacle",Value(1.5)).asDouble();
        time_to_live=rf.check("time-to-live",Value(1.0)).asDouble();
        min_points=rf.check("min-points",Value(5)).asInt();

        navPort.open("/"+module_name+"/nav:rpc");
        string front_laser_port_name="/"+module_name+"/front_laser:i";
        string back_laser_port_name="/"+module_name+"/back_laser:i";

        drv_front=new yarp::dev::PolyDriver;
        Property options;
        options.put("device","Rangefinder2DClient");
        options.put("local",front_laser_port_name);
        options.put("remote","/"+robot+"/laser/front:o");
        bool b=drv_front->open(options);
        if (!b)
        {
            yError()<<"Unable to open polydriver for front laser";
            delete drv_front;
            return false;
        }
        drv_front->view(iLas_front);
        if (!iLas_front)
        {
            yError()<<"Unable to get IRangefinder2D interface for front laser";
            delete drv_front;
            return false;
        }
        iLas_front->getScanLimits(min_angle_front,max_angle_front);
        iLas_front->getHorizontalResolution(angle_step_front);
        iLas_front->getDistanceRange(min_dist_front,max_dist_front);
        hor_res_front=(int)((max_angle_front-min_angle_front)/angle_step_front);
        ver_res_front=(int)((max_dist_front-min_dist_front)/0.5);
        yInfo()<<"Reading front laser with angle range ("<<min_angle_front<<max_angle_front
              <<") and distance range ("<<min_dist_front<<max_dist_front<<")";

        drv_back=new yarp::dev::PolyDriver;
        options.clear();
        options.put("device","Rangefinder2DClient");
        options.put("local",back_laser_port_name);
        options.put("remote","/"+robot+"/laser/back:o");
        b=drv_back->open(options);
        if (!b)
        {
            yError()<<"Unable to open polydriver for back laser";
            delete drv_back;
            return false;
        }
        drv_back->view(iLas_back);
        if (!iLas_back)
        {
            yError()<<"Unable to get IRangefinder2D interface for back laser";
            delete drv_back;
            return false;
        }
        iLas_back->getScanLimits(min_angle_back,max_angle_back);
        iLas_back->getHorizontalResolution(angle_step_back);
        iLas_back->getDistanceRange(min_dist_back,max_dist_back);
        hor_res_back=(int)((max_angle_back-min_angle_back)/angle_step_back);
        ver_res_back=(int)((max_dist_back-min_dist_back)/0.5);
        yInfo()<<"Reading front laser with angle range ("<<min_angle_back<<max_angle_back
              <<") and distance range ("<<min_dist_back<<max_dist_back<<")";

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
        std::vector<LaserMeasurementData> data_front,data_back;
        if(iLas_front->getLaserMeasurement(data_front))
        {
            clusterData(data_front,clusters_front);
            yInfo()<<"Number of obstacles found from front laser"<<clusters_front.size();
        }

        if(iLas_back->getLaserMeasurement(data_back))
        {
            clusterData(data_back,clusters_back);
            yInfo()<<"Number of obstacles found from back laser"<<clusters_back.size();
        }

        if (clusters_front.size()>0 || clusters_back.size()>0)
        {
            double d=getMinDistance();
            yInfo()<<"Found obstacle at"<<d;
            if (d<dist_obstacle)
            {
                yInfo()<<"Stopping";
                stopNav();
            }
        }
        return true;
    }

    /****************************************************************/
    void clusterData(std::vector<LaserMeasurementData> &data, std::vector<Cluster> &clusters)
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
    }

    /****************************************************************/
    double getMinDistance()
    {
        double dfront=getMinDistance(clusters_front);
        double dback=getMinDistance(clusters_back);
        if (isinf(dback))
        {
            return dfront;
        }
        else if (isinf(dfront))
        {
            return dback;
        }
        else
        {
            return min(dfront,dback);
        }
    }

    /****************************************************************/
    double getMinDistance(const std::vector<Cluster> &clusters)
    {
        if (clusters.size()>0)
        {
            Vector d(clusters.size(),numeric_limits<double>::infinity());
            for (int i=0;i<clusters.size();i++)
            {
                double xc=clusters[i].getXcenter();
                double yc=clusters[i].getYcenter();
                d[i]=sqrt(xc*xc+yc*yc);
            }
            double min_d=findMin(d);
            return min_d;
        }
        else
        {
            return numeric_limits<double>::infinity();
        }
    }

    /****************************************************************/
    bool getRobotLocation(double &x, double &y)
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
        if (drv_front)
        {
            delete drv_front;
        }
        if (drv_back)
        {
            delete drv_back;
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

