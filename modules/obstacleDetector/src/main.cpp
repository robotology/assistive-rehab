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

#include <iostream>
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

#include <opencv2/core.hpp>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace cv;

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
    double dist_thresh{1.0};
    double dist_obstacle{1.5};
    int min_points{3};

    RpcClient navPort;
    BufferedPort<Bottle> outPort;

    struct Dist
    {
        double threshold;
        Dist(const double &threshold)
        {
            this->threshold=threshold;
        }
        bool operator()(const cv::Point2d &a, const cv::Point2d &b)
        {
            return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y)) < threshold;
        }
    };

    /****************************************************************/
    bool configure(ResourceFinder& rf) override
    {
        period=rf.check("period",Value(0.1)).asDouble();
        module_name=rf.check("name",Value("obstacleDetector")).asString();
        robot=rf.check("robot",Value("SIM_CER_ROBOT")).asString();
        dist_thresh=rf.check("dist-thresh",Value(1.0)).asDouble();
        dist_obstacle=rf.check("dist-obstacle",Value(1.5)).asDouble();
        min_points=rf.check("min-points",Value(3)).asInt();

        navPort.open("/"+module_name+"/nav:rpc");
        outPort.open("/"+module_name+"/obstacle:o");
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
        double dfront=numeric_limits<double>::infinity();
        double dback=numeric_limits<double>::infinity();
        if(iLas_front->getLaserMeasurement(data_front))
        {
            // cluster data based on distance
            int nclusters;
            Matrix clusters_front=clusterData(data_front,nclusters);
            if (clusters_front.data())
            {
                // we keep only clusters with number of points > minpoints
                vector<int> ids;
                nclusters=removeOutliers(clusters_front,ids);
                yInfo()<<"Number of obstacles from front laser"<<nclusters;

                // we consider the closest cluster
                dfront=getClosestCluster(clusters_front,ids);
            }
        }

        if(iLas_back->getLaserMeasurement(data_back))
        {
            // cluster data based on distance
            int nclusters;
            Matrix clusters_back=clusterData(data_back,nclusters);
            if (clusters_back.data())
            {
                // we keep only clusters with number of points > minpoints
                vector<int> ids;
                nclusters=removeOutliers(clusters_back,ids);
                yInfo()<<"Number of obstacles from back laser"<<nclusters;

                // we consider the closest cluster
                dback=getClosestCluster(clusters_back,ids);
            }
        }

        if (!isinf(dfront) || !isinf(dback))
        {
            string laser;
            double d=getMinDistance(dfront,dback,laser);
            yInfo()<<"Found obstacle at"<<d<<"from"<<laser;
            if (d<dist_obstacle)
            {
                if (getRobotState()!="idle")
                {
                    Bottle &obstacle=outPort.prepare();
                    obstacle.clear();
                    obstacle.addDouble(d);
                    obstacle.addString(laser);
                    outPort.write();
                    yInfo()<<"Stopping";
                    stopNav();
                }
            }
        }
        else
        {
            yInfo()<<"No obstacle found";
        }
        return true;
    }

    /****************************************************************/
    double getClosestCluster(const Matrix &c, const vector<int> &ids)
    {
        int n=ids.size();
        if (n>0)
        {
            int closestid=-1;
            Vector dist(n);
            for (int i=0; i<n; i++)
            {
                int id=ids[i];
                dist[i]=getClusterDist(c,id);
            }
            closestid=(std::min_element(dist.begin(),dist.end())-dist.begin());
            return yarp::math::findMin(dist);
        }
        else
        {
            return numeric_limits<double>::infinity();
        }
    }

    /****************************************************************/
    double getClusterDist(const Matrix &c, const int &id)
    {
        cv::Point2d center{0.0,0.0};
        int n=0;
        for (int i=0;i<c.rows();i++)
        {
            if ((int)c[i][2]==id)
            {
                center.x+=c[i][0];
                center.y+=c[i][1];
                n++;
            }
        }
        center.x/=n;
        center.y/=n;
        return sqrt((center.x*center.x)+(center.y*center.y));
    }

    /****************************************************************/
    Matrix clusterData(std::vector<LaserMeasurementData> &d, int &nclusters)
    {
        vector<cv::Point2d> input=toCvFeatures(d);
        vector<int> id(input.size());
        nclusters=cv::partition(input,id,Dist(dist_thresh));
        Matrix output(input.size(),3);
        for (int i=0;i<input.size();i++)
        {
            output[i][0]=input[i].x;
            output[i][1]=input[i].y;
            output[i][2]=id[i];
        }
        return output;
    }

    /***************************************************************/
    int removeOutliers(Matrix &c, vector<int> &ids)
    {
        Vector id=c.getCol(2);
        int maxid=yarp::math::findMax(id);
        int nclusters=0;
        for (int i=0; i<=maxid; i++)
        {
            int npoints=std::count(id.begin(),id.end(),i);
            if (npoints<min_points)
            {
                update(c,i);
            }
            else
            {
                ids.push_back(i);
                nclusters++;
            }
        }
        return nclusters;
    }

    /****************************************************************/
    void update(Matrix &c, const int &id)
    {
        int j=0;
        while (true)
        {
            if ((int)c[j][2]==id)
            {
                c.removeRows(j,1);
            }
            else
            {
                j++;
            }
            if (j>=c.rows())
            {
                break;
            }
        }
    }

    /****************************************************************/
    vector<cv::Point2d> toCvFeatures(std::vector<LaserMeasurementData> &data)
    {
        vector<cv::Point2d> cvpnts;
        for (auto &d:data)
        {
            double x,y;
            d.get_cartesian(x,y);
            if (std::isinf(x) || std::isinf(y))
            {
                continue;
            }
            cvpnts.push_back(cv::Point2d(x,y));
        }
        return cvpnts;
    }

    /****************************************************************/
    double getMinDistance(const double &d1, const double &d2, string &s)
    {
        if (d1<d2)
        {
            s="front-laser";
            return d1;
        }
        else
        {
            s="back-laser";
            return d2;
        }
    }

    /****************************************************************/
    string getRobotState()
    {
        string state="";
        Bottle cmd,rep;
        cmd.addString("get_state");
        if (navPort.write(cmd,rep))
        {
            Property robotState(rep.get(0).toString().c_str());
            state=robotState.find("robot-state").asString();
        }
        return state;
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
        outPort.interrupt();
        return true;
    }

    /****************************************************************/
    bool close() override
    {
        navPort.close();
        outPort.interrupt();
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

