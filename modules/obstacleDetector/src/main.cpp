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
#include <yarp/sig/Image.h>

#include <yarp/math/Math.h>

#include <yarp/cv/Cv.h>

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
    double period{0.1};
    string module_name{"obstacleDetector"};
    string robot{"cer"};
    PolyDriver *drv_front{nullptr},*drv_rear{nullptr};
    IRangefinder2D *iLas_front{nullptr},*iLas_rear{nullptr};
    double min_angle_front{0.0},min_angle_rear{0.0};
    double max_angle_front{0.0},max_angle_rear{0.0};
    double angle_step_front{0.0},angle_step_rear{0.0};
    double min_dist_front{0.0},min_dist_rear{0.0};
    double max_dist_front{0.0},max_dist_rear{0.0};
    int hor_res_front{0},hor_res_rear{0};
    double ver_step{1.0};
    double dist_thresh{0.3};
    double dist_obstacle{1.5};
    int min_points{3};

    struct Laser2Img
    {
        cv::Rect2d range_p;
        cv::Rect range_px;
        bool mirror{false};
        Laser2Img(const cv::Rect2d &range_p_,const cv::Rect &range_px_) :
            range_p(range_p_), range_px(range_px_) { }
        void mirrorImg()
        {
            this->mirror=true;
        }
    };

    Laser2Img *l2img_front{nullptr};
    Laser2Img *l2img_rear{nullptr};
    map<int,cv::Scalar> color_map;
    int img_size{500};

    RpcClient navPort;
    BufferedPort<Bottle> outPort;
    BufferedPort<ImageOf<PixelRgb> > viewerPort;
    cv::Mat imgCv;

    struct Dist
    {
        double threshold;
        explicit Dist(const double &threshold)
        {
            this->threshold=threshold;
        }
        bool operator()(const cv::Point2d &a, const cv::Point2d &b)
        {
            return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y)) < threshold;
        }
    };

public:

    /****************************************************************/
    ObstDetector() {}

    /****************************************************************/
    bool configure(ResourceFinder& rf) override
    {
        period=rf.check("period",Value(0.1)).asDouble();
        module_name=rf.check("name",Value("obstacleDetector")).asString();
        robot=rf.check("robot",Value("cer")).asString();
        dist_thresh=rf.check("dist-thresh",Value(0.3)).asDouble();
        dist_obstacle=rf.check("dist-obstacle",Value(1.0)).asDouble();
        min_points=rf.check("min-points",Value(3)).asInt();
        ver_step=rf.check("ver-step",Value(1.0)).asDouble();

        navPort.open("/"+module_name+"/nav:rpc");
        outPort.open("/"+module_name+"/obstacle:o");
        viewerPort.open("/"+module_name+"/viewer:o");
        string front_laser_port_name="/"+module_name+"/front_laser:i";
        string rear_laser_port_name="/"+module_name+"/rear_laser:i";

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
        yInfo()<<"Reading front laser with angle range ("<<min_angle_front<<max_angle_front
              <<") and distance range ("<<min_dist_front<<max_dist_front<<")";
        img_size=500;
        cv::Rect2d range_p_front(-max_dist_front,0.0,max_dist_front,max_dist_front);
        cv::Rect range_px_front(0,0,img_size,img_size/2);
        l2img_front=new Laser2Img(range_p_front,range_px_front);
        l2img_front->mirrorImg();

        drv_rear=new yarp::dev::PolyDriver;
        options.clear();
        options.put("device","Rangefinder2DClient");
        options.put("local",rear_laser_port_name);
        options.put("remote","/"+robot+"/laser/back:o");
        b=drv_rear->open(options);
        if (!b)
        {
            yError()<<"Unable to open polydriver for rear laser";
            delete drv_rear;
            return false;
        }
        drv_rear->view(iLas_rear);
        if (!iLas_rear)
        {
            yError()<<"Unable to get IRangefinder2D interface for rear laser";
            delete drv_rear;
            return false;
        }
        iLas_rear->getScanLimits(min_angle_rear,max_angle_rear);
        iLas_rear->getHorizontalResolution(angle_step_rear);
        iLas_rear->getDistanceRange(min_dist_rear,max_dist_rear);
        hor_res_rear=(int)((max_angle_rear-min_angle_rear)/angle_step_rear);
        yInfo()<<"Reading rear laser with angle range ("<<min_angle_rear<<max_angle_rear
              <<") and distance range ("<<min_dist_rear<<max_dist_rear<<")";
        cv::Rect2d range_p_rear(-max_dist_rear,0.0,max_dist_rear,max_dist_rear);
        cv::Rect range_px_rear(0,img_size/2,img_size,img_size);
        l2img_rear=new Laser2Img(range_p_rear,range_px_rear);

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
        std::vector<LaserMeasurementData> data_front,data_rear;
        double dfront=numeric_limits<double>::infinity();
        double drear=numeric_limits<double>::infinity();
        Matrix clusters_front,clusters_rear;
        if(iLas_front->getLaserMeasurement(data_front))
        {
            // cluster data based on distance
            int nclusters;
            clusters_front=clusterData(data_front,nclusters);
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

        if(iLas_rear->getLaserMeasurement(data_rear))
        {
            // cluster data based on distance
            int nclusters;
            clusters_rear=clusterData(data_rear,nclusters);
            if (clusters_rear.data())
            {
                // we keep only clusters with number of points > minpoints
                vector<int> ids;
                nclusters=removeOutliers(clusters_rear,ids);
                yInfo()<<"Number of obstacles from rear laser"<<nclusters;

                // we consider the closest cluster
                drear=getClosestCluster(clusters_rear,ids);
            }
        }

        if (!isinf(dfront) || !isinf(drear))
        {
            string laser;
            double d=getMinDistance(dfront,drear,laser);
            yInfo()<<"Closest obstacle at"<<d<<"from"<<laser;
            if (d<dist_obstacle)
            {
                Bottle &obstacle=outPort.prepare();
                obstacle.clear();
                obstacle.addDouble(d);
                obstacle.addString(laser);
                outPort.write();
                if (getRobotState()!="idle")
                {
                    yInfo()<<"Stopping";
                    stopNav();
                }
            }
        }
        else
        {
            yInfo()<<"No obstacle found";
        }

        if (viewerPort.getOutputCount()>0)
        {
            ImageOf<PixelRgb> &imgOut=viewerPort.prepare();
            imgOut.resize(img_size,img_size);
            imgOut.zero();
            imgCv=cv::Mat::ones(img_size,img_size,CV_8UC3);
            imgCv.setTo(Scalar(200,200,200));
            cv::Point robot(imgCv.size().width/2,imgCv.size().height/2);
            cv::circle(imgCv,robot,3,cv::Scalar(0,0,0),-1);
            drawClusters(clusters_front,imgCv,*l2img_front);
            drawClusters(clusters_rear,imgCv,*l2img_rear);
            drawCircles(imgCv);
            imgOut=yarp::cv::fromCvMat<PixelRgb>(imgCv);
            viewerPort.write();
        }
        return true;
    }

    /****************************************************************/
    void drawClusters(const Matrix &c, cv::Mat &imgCv, const Laser2Img &l2img)
    {
        int radius=3;
        for (int i=0; i<c.rows(); i++)
        {
            double x=c[i][0];
            double y=c[i][1];
            int xpx=(int)((y-l2img.range_p.x)*(l2img.range_px.width-l2img.range_px.x)/(l2img.range_p.width-l2img.range_p.x)+l2img.range_px.x);
            int ypx=(int)((x-l2img.range_p.y)*(l2img.range_px.height-l2img.range_px.y)/(l2img.range_p.height-l2img.range_p.y)+l2img.range_px.y);
            if (l2img.mirror)
            {
                xpx=imgCv.size().width-xpx;
                ypx=imgCv.size().height/2-ypx;
            }
            cv::Point pt(xpx,ypx);
            int id=c[i][2];
            cv::circle(imgCv,pt,radius,color_map[id],-1);
        }
    }

    /****************************************************************/
    void drawCircles(cv::Mat &imgCv)
    {
        cv::Scalar black(0,0,0);
        cv::line(imgCv,cv::Point(0,0),cv::Point(imgCv.size().width,imgCv.size().height),black);
        cv::line(imgCv,cv::Point(imgCv.size().width,0),cv::Point(0,imgCv.size().height),black);
        cv::line(imgCv,cv::Point(imgCv.size().width/2,0),cv::Point(imgCv.size().width/2,imgCv.size().height),black);
        cv::line(imgCv,cv::Point(0,imgCv.size().height/2),cv::Point(imgCv.size().width,imgCv.size().height/2),black);
        char buff [10];
        for (float i=0; i<max_dist_front; i+=ver_step)
        {
            int y=imgCv.size().height/2-(int)(i*(imgCv.size().height/2)/max_dist_front);
            sprintf(buff,"%3.1fm",i);
            cv::putText(imgCv,buff,cv::Point(imgCv.size().width/2,y),cv::HersheyFonts::FONT_HERSHEY_PLAIN,1.5,black);
            cv::circle(imgCv,cv::Point(imgCv.size().width/2,imgCv.size().height/2),y,black);
        }
    }

    /****************************************************************/
    double getClosestCluster(const Matrix &c, const vector<int> &ids)
    {
        int n=ids.size();
        if (n>0)
        {
            Vector dist(n);
            for (int i=0; i<n; i++)
            {
                int id=ids[i];
                dist[i]=getClusterDist(c,id);
            }
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
        cv::RNG rng;
        vector<cv::Point2d> input=toCvFeatures(d);
        vector<int> id(input.size());
        nclusters=cv::partition(input,id,Dist(dist_thresh));
        Matrix output(input.size(),3);
        for (int i=0;i<input.size();i++)
        {
            output[i][0]=input[i].x;
            output[i][1]=input[i].y;
            output[i][2]=id[i];
            if (color_map.find(id[i])==end(color_map))
            {
                cv::Scalar color(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));
                color_map[id[i]]=color;
            }
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
            if (npoints<=min_points)
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
            if (std::isinf(x) || std::isinf(y) || std::isnan(x) || std::isnan(y))
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
            s="rear-laser";
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
        viewerPort.interrupt();
        return true;
    }

    /****************************************************************/
    bool close() override
    {
        navPort.close();
        outPort.interrupt();
        viewerPort.interrupt();
        delete l2img_front;
        delete l2img_rear;
        if (drv_front)
        {
            delete drv_front;
        }
        if (drv_rear)
        {
            delete drv_rear;
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

