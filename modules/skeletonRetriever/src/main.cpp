/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file main.cpp
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#include <cstdlib>
#include <memory>
#include <limits>
#include <algorithm>
#include <vector>
#include <unordered_map>
#include <iterator>
#include <utility>
#include <sstream>
#include <iostream>
#include <string>
#include <yarp/os/all.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IRGBDSensor.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/filters.h>
#include "AssistiveRehab/helpers.h"
#include "AssistiveRehab/skeleton.h"
#include "utils.h"
#include "nlp.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace assistive_rehab;

const string unknown_tag("?");

/****************************************************************/
bool is_unknown(const string &tag)
{
    return (tag.empty() || (tag==unknown_tag));
}


/****************************************************************/
class MetaSkeleton
{
    vector<shared_ptr<MedianFilter>> filter;
    unordered_map<string,shared_ptr<MedianFilter>> limbs_length;
    unordered_map<string,unsigned int> limbs_length_cnt;
    bool optimize_limblength;
    CamParamsHelper camParams;
    
    /****************************************************************/
    vector<pair<string,pair<Vector,Vector>>> optimize_limbs(const vector<string> &tags)
    {
        bool all_updated=true;
        vector<double> lengths;
        for (const auto &tag:tags)
        {
            all_updated&=(*skeleton)[tag]->isUpdated();
            const auto &cnt=limbs_length_cnt.find(tag);
            if (cnt!=limbs_length_cnt.end())
            {
                auto &flt=limbs_length[tag];
                if (cnt->second>flt->getOrder())
                {
                    lengths.push_back(flt->output()[0]);
                }
            }
        }

        vector<pair<string,pair<Vector,Vector>>> unordered;
        if (all_updated && (lengths.size()==tags.size()-1))
        {
            unordered=LimbOptimizer::optimize(camParams,(*skeleton)[tags[0]],lengths);
        }
        return unordered;
    }

public:
    const int opc_id_invalid=-1;
    double timer;
    int opc_id;
    shared_ptr<SkeletonStd> skeleton;
    vector<int> keys_acceptable_misses;
    vector<Vector> pivots;
    double name_confidence;

    /****************************************************************/
    MetaSkeleton(const CamParamsHelper &camParams_, const double t, const int filter_keypoint_order_,
                 const int filter_limblength_order_, const bool optimize_limblength_) : 
                 camParams(camParams_), timer(t), opc_id(opc_id_invalid),
                 optimize_limblength(optimize_limblength_), name_confidence(0.0)
    {
        skeleton=shared_ptr<SkeletonStd>(new SkeletonStd());
        keys_acceptable_misses.assign(skeleton->getNumKeyPoints(),0);
        pivots.assign(2,numeric_limits<double>::infinity()*ones(2));

        for (unsigned int i=0; i<skeleton->getNumKeyPoints(); i++)
        {
            filter.push_back(shared_ptr<MedianFilter>(new MedianFilter(filter_keypoint_order_,(*skeleton)[i]->getPoint())));
        }

        limbs_length[KeyPointTag::elbow_left]=shared_ptr<MedianFilter>(new MedianFilter(filter_limblength_order_,zeros(1)));
        limbs_length[KeyPointTag::hand_left]=shared_ptr<MedianFilter>(new MedianFilter(filter_limblength_order_,zeros(1)));
        limbs_length[KeyPointTag::elbow_right]=shared_ptr<MedianFilter>(new MedianFilter(filter_limblength_order_,zeros(1)));
        limbs_length[KeyPointTag::hand_right]=shared_ptr<MedianFilter>(new MedianFilter(filter_limblength_order_,zeros(1)));

        limbs_length[KeyPointTag::knee_left]=shared_ptr<MedianFilter>(new MedianFilter(filter_limblength_order_,zeros(1)));
        limbs_length[KeyPointTag::ankle_left]=shared_ptr<MedianFilter>(new MedianFilter(filter_limblength_order_,zeros(1)));
        limbs_length[KeyPointTag::foot_left]=shared_ptr<MedianFilter>(new MedianFilter(filter_limblength_order_,zeros(1)));
        limbs_length[KeyPointTag::knee_right]=shared_ptr<MedianFilter>(new MedianFilter(filter_limblength_order_,zeros(1)));
        limbs_length[KeyPointTag::ankle_right]=shared_ptr<MedianFilter>(new MedianFilter(filter_limblength_order_,zeros(1)));
        limbs_length[KeyPointTag::foot_right]=shared_ptr<MedianFilter>(new MedianFilter(filter_limblength_order_,zeros(1)));

        limbs_length_cnt[KeyPointTag::elbow_left]=limbs_length_cnt[KeyPointTag::hand_left]=
        limbs_length_cnt[KeyPointTag::elbow_right]=limbs_length_cnt[KeyPointTag::hand_right]=
        limbs_length_cnt[KeyPointTag::knee_left]=limbs_length_cnt[KeyPointTag::ankle_left]=limbs_length_cnt[KeyPointTag::foot_left]=
        limbs_length_cnt[KeyPointTag::knee_right]=limbs_length_cnt[KeyPointTag::ankle_right]=limbs_length_cnt[KeyPointTag::foot_right]=0;
    }

    /****************************************************************/
    bool init(const string &tag, const Vector &p)
    {
        int i=skeleton->getNumFromKey(tag);
        if (i>=0)
        {
            filter[i]->init(p);
            return true;
        }
        else
            return false;
    }

    /****************************************************************/
    void update(const vector<pair<string,pair<Vector,Vector>>> &unordered)
    {
        vector<pair<string,pair<Vector,Vector>>> unordered_filtered;
        for (auto &p:unordered)
        {
            int i=skeleton->getNumFromKey(p.first);
            if (i>=0)
            {
                unordered_filtered.push_back(make_pair(p.first,make_pair(filter[i]->filt(p.second.first),p.second.second)));
            }
        }
        // update 1: incorporate filtered feedback
        skeleton->update_withpixels(unordered_filtered);
        
        size_t latch_size=unordered_filtered.size();
        for (auto &it1:limbs_length)
        {
            const auto &k=(*skeleton)[it1.first];
            const auto &p=k->getParent(0);

            if (k->isUpdated() && p->isUpdated())
            {
                double d=norm(k->getPoint()-p->getPoint());

                // filter only when the skeleton shows up initially
                if (limbs_length_cnt[it1.first]<=it1.second->getOrder())
                {
                    it1.second->filt(Vector(1,d));
                    limbs_length_cnt[it1.first]++;
                }
                // print the filtered limb length once
                if (limbs_length_cnt[it1.first]==it1.second->getOrder()+1)
                {
                    yInfo()<<"Skeleton:"<<skeleton->getTag()
                           <<"limb:"<<p->getTag()<<"-"<<k->getTag()
                           <<"length:"<<it1.second->output()[0];
                    limbs_length_cnt[it1.first]++;
                }
                // seek for too long limb parts
                if (limbs_length_cnt[it1.first]>it1.second->getOrder())
                {
                    if (d>2.0*it1.second->output()[0])
                    {
                        for (auto it2=begin(unordered_filtered); it2!=end(unordered_filtered); it2++)
                        {
                            if (it2->first==it1.first)
                            {
                                unordered_filtered.erase(it2);
                                break;
                            }
                        }
                    }
                }
            }
        }
        // update 2: remove limbs' keypoints that are clearly too long
        if (unordered_filtered.size()<latch_size)
        {
            skeleton->update_withpixels(unordered_filtered);
        }

        if (optimize_limblength)
        {
            vector<pair<string,pair<Vector,Vector>>> tmp;
            tmp=optimize_limbs({KeyPointTag::shoulder_left,KeyPointTag::elbow_left,KeyPointTag::hand_left});
            unordered_filtered.insert(end(unordered_filtered),begin(tmp),end(tmp));

            tmp=optimize_limbs({KeyPointTag::shoulder_right,KeyPointTag::elbow_right,KeyPointTag::hand_right});
            unordered_filtered.insert(end(unordered_filtered),begin(tmp),end(tmp));

            tmp=optimize_limbs({KeyPointTag::hip_left,KeyPointTag::knee_left,KeyPointTag::ankle_left,KeyPointTag::foot_left});
            unordered_filtered.insert(end(unordered_filtered),begin(tmp),end(tmp));

            tmp=optimize_limbs({KeyPointTag::hip_right,KeyPointTag::knee_right,KeyPointTag::ankle_right,KeyPointTag::foot_right});
            unordered_filtered.insert(end(unordered_filtered),begin(tmp),end(tmp));

            // update 3: adjust limbs' keypoints through optimization
            skeleton->update_withpixels(unordered_filtered);
        }
    }
};


/****************************************************************/
class Retriever : public RFModule
{
    BufferedPort<Bottle> skeletonsPort;
    BufferedPort<ImageOf<PixelFloat>> depthPort;
    BufferedPort<Bottle> viewerPort;
    BufferedPort<Property> navPort;
    BufferedPort<Property> gazePort;
    RpcClient opcPort;

    PolyDriver rgbdDrv;

    Matrix navFrame, gazeFrame, rootFrame;
    bool navFrameUpdated, gazeFrameUpdated;

    ImageOf<PixelFloat> depth;

    unordered_map<string,string> keysRemap;
    vector<shared_ptr<MetaSkeleton>> skeletons;

    bool camera_configured;
    double period;
    double fov_h;
    double fov_v;
    double keys_recognition_confidence;
    double keys_recognition_percentage;
    int keys_acceptable_misses;
    double min_acceptable_path;
    int tracking_threshold;
    double time_to_live;

    bool depth_enable;
    int depth_kernel_size;
    int depth_iterations;
    float depth_min_distance;
    float depth_max_distance;

    int filter_keypoint_order;
    int filter_limblength_order;
    bool optimize_limblength;
    double t0;

    /****************************************************************/
    bool getCameraOptions()
    {
        IRGBDSensor* irgbd;
        rgbdDrv.view(irgbd);

        if (irgbd != nullptr) {
            if (irgbd->getRgbFOV(fov_h, fov_v)) {
                yInfo() << "camera fov_h (from sensor) =" << fov_h;
                yInfo() << "camera fov_v (from sensor) =" << fov_v;
                rgbdDrv.close();
                return true;
            }
        }

        return false;
    }

    /****************************************************************/
    bool getPoint3D(const int u, const int v, Vector &p) const
    {
        if ((u>=0) && (u<depth.width()) && (v>=0) && (v<depth.height()))
        {
            double f=CamParamsHelper(depth.width(),depth.height(),fov_h).get_focal();
            double d=depth(u,v);
            if ((d>0.0) && (f>0.0))
            {
                double x=u-0.5*(depth.width()-1);
                double y=v-0.5*(depth.height()-1);

                p=d*ones(3);
                p[0]*=x/f;
                p[1]*=y/f;

                return true;
            }
        }
        
        return false;
    }

    /****************************************************************/
    string getNameFromId(const int id) const
    {
        ostringstream ss;
        ss<<"#"<<hex<<id;
        return ss.str();
    }

    /****************************************************************/
    shared_ptr<MetaSkeleton> create(Bottle *keys)
    {
        shared_ptr<MetaSkeleton> s(new MetaSkeleton(CamParamsHelper(depth.width(),depth.height(),fov_h),
                                                    time_to_live,filter_keypoint_order,filter_limblength_order,
                                                    optimize_limblength));
        vector<pair<string,pair<Vector,Vector>>> unordered;
        auto foot_left=make_pair(string(""),make_pair(Vector(1),Vector(1)));
        auto foot_right=foot_left;
        vector<Vector> shoulders, hips;
        bool shoulder_center_detected=false;
        bool hip_center_detected=false;

        Vector p,pixel(2);
        for (size_t i=0; i<keys->size(); i++)
        {
            if (Bottle *k=keys->get(i).asList())
            {
                if (k->size()==4)
                {
                    string tag=k->get(0).asString();
                    int u=(int)k->get(1).asFloat64();
                    int v=(int)k->get(2).asFloat64();
                    double confidence=k->get(3).asFloat64();

                    if ((confidence>=keys_recognition_confidence) && getPoint3D(u,v,p))
                    {
                        pixel[0]=u; pixel[1]=v;
                        auto pair_=make_pair(keysRemap[tag],make_pair(p,pixel));

                        // handle foot as big-toe; if big-toe is not available,
                        // then use small-toe as fallback
                        if (keysRemap[tag]==KeyPointTag::foot_left)
                        {
                            if (foot_left.first.empty() || (tag=="LBigToe"))
                            {
                                foot_left=pair_;
                            }
                        }
                        else if (keysRemap[tag]==KeyPointTag::foot_right)
                        {
                            if (foot_right.first.empty() || (tag=="RBigToe"))
                            {
                                foot_right=pair_;
                            }
                        }
                        else
                        {
                            unordered.push_back(pair_);
                        }

                        // if [shoulder|hip]_center is not available, then
                        // use (left + right)/2 as fallback
                        if (keysRemap[tag]==KeyPointTag::shoulder_center)
                        {
                            shoulder_center_detected=true;
                            s->pivots[0]=pixel;
                        }
                        else if ((keysRemap[tag]==KeyPointTag::shoulder_left) ||
                                 (keysRemap[tag]==KeyPointTag::shoulder_right))
                        {
                            shoulders.push_back(p);
                        }
                        else if (keysRemap[tag]==KeyPointTag::hip_center)
                        {
                            hip_center_detected=true;
                            s->pivots[1]=pixel;
                        }
                        else if ((keysRemap[tag]==KeyPointTag::hip_left) ||
                                 (keysRemap[tag]==KeyPointTag::hip_right))
                        {
                            hips.push_back(p);
                        }
                    }
                }
                else if (k->size()==3)
                {
                    string tag=k->get(0).asString();
                    string name=k->get(1).asString();
                    double confidence=k->get(2).asFloat64();

                    if (tag=="Name")
                    {
                        s->skeleton->setTag(name);
                        s->name_confidence=confidence;
                    }
                }
            }
        }

        if (!foot_left.first.empty())
        {
            unordered.push_back(foot_left);
        }
        if (!foot_right.first.empty())
        {
            unordered.push_back(foot_right);
        }
        if (!shoulder_center_detected && (shoulders.size()==2))
        {
            pixel=numeric_limits<double>::quiet_NaN();
            unordered.push_back(make_pair(KeyPointTag::shoulder_center,
                                          make_pair(0.5*(shoulders[0]+shoulders[1]),pixel)));
        }
        if (!hip_center_detected && (hips.size()==2))
        {
            pixel=numeric_limits<double>::quiet_NaN();
            unordered.push_back(make_pair(KeyPointTag::hip_center,
                                          make_pair(0.5*(hips[0]+hips[1]),pixel)));
        }

        s->skeleton->update_withpixels(unordered);
        return s;
    }

    /****************************************************************/
    void update(const shared_ptr<MetaSkeleton> &src, shared_ptr<MetaSkeleton> &dest,
                vector<string> &remove_tags)
    {
        vector<pair<string,pair<Vector,Vector>>> unordered;
        for (unsigned int i=0; i<src->skeleton->getNumKeyPoints(); i++)
        {
            auto key=(*src->skeleton)[i];
            if (key->isUpdated())
            {
                const Vector &p=key->getPoint();
                unordered.push_back(make_pair(key->getTag(),make_pair(p,key->getPixel())));
                if (dest->keys_acceptable_misses[i]==0)
                    dest->init(key->getTag(),p);
                dest->keys_acceptable_misses[i]=keys_acceptable_misses;
            }
            else if (dest->keys_acceptable_misses[i]>0)
            {
                unordered.push_back(make_pair(key->getTag(),make_pair((*dest->skeleton)[i]->getPoint(),(*dest->skeleton)[i]->getPixel())));
                dest->keys_acceptable_misses[i]--;
            }
        }

        dest->update(unordered);
        dest->timer=time_to_live;
        dest->pivots=src->pivots;

        string oldTag=dest->skeleton->getTag();
        dest->skeleton->setTag(is_unknown(src->skeleton->getTag())?
                               getNameFromId(dest->opc_id):
                               src->skeleton->getTag());

        if (oldTag!=dest->skeleton->getTag())
        {
            remove_tags.push_back(oldTag);
        }
    }

    /****************************************************************/
    bool isValid(const shared_ptr<MetaSkeleton> &s) const
    {
        bool no_pivots=true;
        for (auto &pivot:s->pivots)
        {
            no_pivots=no_pivots && (norm(pivot)==numeric_limits<double>::infinity());
        }
        if (no_pivots)
        {
            return false;
        }

        unsigned int n=0;
        for (unsigned int i=0; i<s->skeleton->getNumKeyPoints(); i++)
        {
            if ((*s->skeleton)[i]->isUpdated())
            {
                n++;
            }
        }
        
        double perc=((double)n)/((double)s->skeleton->getNumKeyPoints());
        double max_path=s->skeleton->getMaxPath();
        return ((perc>=keys_recognition_percentage) && (max_path>=min_acceptable_path));
    }

    /****************************************************************/
    vector<double> computeScores(const vector<shared_ptr<MetaSkeleton>> &c,
                                 const shared_ptr<MetaSkeleton> &n)
    {
        vector<double> scores;
        for (auto &s:c)
        {
            double dist=numeric_limits<double>::infinity();
            for (size_t i=0; i<s->pivots.size(); i++)
            {
                dist=std::min(dist,norm(s->pivots[i]-n->pivots[i]));
            }
            scores.push_back(dist<=tracking_threshold?dist:
                             numeric_limits<double>::infinity());
        }

        return scores;
    }

    /****************************************************************/
    bool opcAdd(shared_ptr<MetaSkeleton> &s, const Stamp &stamp)
    {
        if (opcPort.getOutputCount())
        {
            Bottle cmd,rep;
            cmd.addVocab32("add");
            Property prop=applyTransform(s->skeleton)->toProperty();
            if (stamp.isValid())
            {
                prop.put("stamp",stamp.getTime());
            }
            cmd.addList().read(prop);
            if (opcPort.write(cmd,rep))
            {
                if (rep.get(0).asVocab32()==Vocab32::encode("ack"))
                {
                    s->opc_id=rep.get(1).asList()->get(1).asInt32();
                    if (is_unknown(s->skeleton->getTag()))
                    {
                        s->skeleton->setTag(getNameFromId(s->opc_id));
                        return opcSet(s,stamp);
                    }
                    return true;
                }
            }
        }

        return false;
    }

    /****************************************************************/
    bool opcSet(const shared_ptr<MetaSkeleton> &s, const Stamp &stamp)
    {
        if (opcPort.getOutputCount())
        {
            Bottle cmd,rep;
            cmd.addVocab32("set");
            Bottle &pl=cmd.addList();
            Property prop=applyTransform(s->skeleton)->toProperty();
            if (stamp.isValid())
            {
                prop.put("stamp",stamp.getTime());
            }
            pl.read(prop);
            Bottle id;
            Bottle &id_pl=id.addList();
            id_pl.addString("id");
            id_pl.addInt32(s->opc_id);
            pl.append(id);
            if (opcPort.write(cmd,rep))
            {
                return (rep.get(0).asVocab32()==Vocab32::encode("ack"));
            }
        }

        return false;
    }

    /****************************************************************/
    bool opcDel(const shared_ptr<MetaSkeleton> &s)
    {
        if (opcPort.getOutputCount())
        {
            Bottle cmd,rep;
            cmd.addVocab32("del");
            Bottle &pl=cmd.addList().addList();
            pl.addString("id");
            pl.addInt32(s->opc_id);
            if (opcPort.write(cmd,rep))
            {
                return (rep.get(0).asVocab32()==Vocab32::encode("ack"));
            }
        }

        return false;
    }

    /****************************************************************/
    void gc(const double dt)
    {
        vector<shared_ptr<MetaSkeleton>> skeletons_;
        for (auto &s:skeletons)
        {
            s->timer-=dt;
            if (s->timer>0.0)
            {
                skeletons_.push_back(s);
            }
            else
            {
                opcDel(s);
            }
        }

        skeletons=skeletons_;
    }

    /****************************************************************/
    void enforce_tag_uniqueness_input(const vector<shared_ptr<MetaSkeleton>> &input)
    {
        for (auto &s1:input)
        {
            string tag_pivot=s1->skeleton->getTag();
            if (!is_unknown(tag_pivot))
            {
                for (auto &s2:input)
                {
                    if ((s1!=s2) && (tag_pivot==s2->skeleton->getTag()))
                    {
                        if (s1->name_confidence>=s2->name_confidence)
                        {
                            s2->skeleton->setTag(unknown_tag);
                        }
                        else
                        {
                            s1->skeleton->setTag(unknown_tag);
                            break;
                        }
                    }
                }
            }
        }
    }

    /****************************************************************/
    void enforce_tag_uniqueness_pending(const vector<shared_ptr<MetaSkeleton>> &pending)
    {
        vector<shared_ptr<MetaSkeleton>> tbr;
        for (auto &s1:pending)
        {
            string tag_pivot=s1->skeleton->getTag();
            for (auto &s2:skeletons)
            {
                if ((s1!=s2) && (tag_pivot==s2->skeleton->getTag()))
                {
                    opcDel(s1);
                    tbr.push_back(s1);
                }
            }
        }
        for (auto &s:tbr)
        {
            for (auto it=begin(skeletons); it!=end(skeletons); it++)
            {
                if (s==*it)
                {
                    skeletons.erase(it);
                    break;
                }
            }
        }
    }

    /****************************************************************/
    bool update_nav_frame()
    {
        if (Property *p=navPort.read(false))
        {
            if (Bottle *b=p->find("robot-location").asList())
            {
                if (b->size()>=3)
                {
                    Vector rot(4,0.0);
                    rot[2]=1.0; rot[3]=(M_PI/180.0)*b->get(2).asFloat64();
                    navFrame=axis2dcm(rot);
                    navFrame(0,3)=b->get(0).asFloat64();
                    navFrame(1,3)=b->get(1).asFloat64();
                    navFrameUpdated=true;
                    return true;
                }
            }
        }

        return false;
    }

    /****************************************************************/
    bool update_gaze_frame()
    {
        if (Property* p=gazePort.read(false))
        {
            if (Bottle* b=p->find("depth_rgb").asList())
            {
                if (b->size()>=7)
                {
                    Vector pos(3);
                    for (size_t i=0; i<pos.length(); i++)
                    {
                        pos[i]=b->get(i).asFloat64();
                    }

                    Vector ax(4);
                    for (size_t i=0; i<ax.length(); i++)
                    {
                        ax[i]=b->get(pos.length()+i).asFloat64();
                    }

                    gazeFrame=axis2dcm(ax);
                    gazeFrame.setSubcol(pos,0,3);
                    gazeFrameUpdated=true;
                    return true;
                }
            }
        }

        return false;
    }

    /****************************************************************/
    shared_ptr<SkeletonStd> applyTransform(shared_ptr<SkeletonStd> &s)
    {
        if (navFrameUpdated && gazeFrameUpdated)
        {
            shared_ptr<SkeletonStd> sk=shared_ptr<SkeletonStd>(new SkeletonStd());
            sk->fromProperty(s->toProperty());
            sk->setTransformation(rootFrame);
            sk->update();
            return sk;
        }
        else
            return s;
    }

    /****************************************************************/
    void viewerUpdate(const vector<string> &remove_tags)
    {
        if (viewerPort.getOutputCount()>0)
        {
            Bottle &msg=viewerPort.prepare();
            msg.clear();
            for (auto &s:skeletons)
            {
                Property prop=applyTransform(s->skeleton)->toProperty();
                msg.addList().read(prop);
            }

            if (!remove_tags.empty())
            {
                Bottle val;
                Bottle &payload=val.addList();
                for (auto &tag:remove_tags)
                {
                    payload.addString(tag);
                }
                Property prop;
                prop.put("remove-tags",val.get(0));
                msg.addList().read(prop);
            }
            viewerPort.writeStrict();
        }
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf) override
    {
        keysRemap["Nose"]=KeyPointTag::head;
        keysRemap["Neck"]=KeyPointTag::shoulder_center;
        keysRemap["RShoulder"]=KeyPointTag::shoulder_right;
        keysRemap["RElbow"]=KeyPointTag::elbow_right;
        keysRemap["RWrist"]=KeyPointTag::hand_right;
        keysRemap["LShoulder"]=KeyPointTag::shoulder_left;
        keysRemap["LElbow"]=KeyPointTag::elbow_left;
        keysRemap["LWrist"]=KeyPointTag::hand_left;
        keysRemap["MidHip"]=KeyPointTag::hip_center;
        keysRemap["RHip"]=KeyPointTag::hip_right;
        keysRemap["RKnee"]=KeyPointTag::knee_right;
        keysRemap["RAnkle"]=KeyPointTag::ankle_right;
        keysRemap["RBigToe"]=KeyPointTag::foot_right;
        keysRemap["RSmallToe"]=KeyPointTag::foot_right;
        keysRemap["LHip"]=KeyPointTag::hip_left;
        keysRemap["LKnee"]=KeyPointTag::knee_left;
        keysRemap["LAnkle"]=KeyPointTag::ankle_left;
        keysRemap["LBigToe"]=KeyPointTag::foot_left;
        keysRemap["LSmallToe"]=KeyPointTag::foot_left;

        // default values
        period=0.01;
        keys_recognition_confidence=0.3;
        keys_recognition_percentage=0.3;
        keys_acceptable_misses=5;
        min_acceptable_path=0.5;
        tracking_threshold=50;
        time_to_live=1.0;
        depth_enable=true;
        depth_kernel_size=4;
        depth_iterations=3;
        depth_min_distance=1.0;
        depth_max_distance=4.0;
        filter_keypoint_order=3;
        filter_limblength_order=40;
        optimize_limblength=true;

        // retrieve values from config file
        Bottle &gGeneral=rf.findGroup("general");
        if (!gGeneral.isNull())
        {
            period=gGeneral.check("period",Value(period)).asFloat64();
        }

        Bottle &gSkeleton=rf.findGroup("skeleton");
        if (!gSkeleton.isNull())
        {
            keys_recognition_confidence=gSkeleton.check("keys-recognition-confidence",Value(keys_recognition_confidence)).asFloat64();
            keys_recognition_percentage=gSkeleton.check("keys-recognition-percentage",Value(keys_recognition_percentage)).asFloat64();
            keys_acceptable_misses=gSkeleton.check("keys-acceptable-misses",Value(keys_acceptable_misses)).asInt32();
            min_acceptable_path=gSkeleton.check("min-acceptable-path",Value(min_acceptable_path)).asFloat64();
            tracking_threshold=gSkeleton.check("tracking-threshold",Value(tracking_threshold)).asInt32();
            time_to_live=gSkeleton.check("time-to-live",Value(time_to_live)).asFloat64();
        }

        Bottle &gDepth=rf.findGroup("depth");
        if (!gDepth.isNull())
        {
            depth_enable=gDepth.check("enable",Value(depth_enable)).asBool();
            depth_kernel_size=gDepth.check("kernel-size",Value(depth_kernel_size)).asInt32();
            depth_iterations=gDepth.check("iterations",Value(depth_iterations)).asInt32();
            depth_min_distance=(float)gDepth.check("min-distance",Value(depth_min_distance)).asFloat64();
            depth_max_distance=(float)gDepth.check("max-distance",Value(depth_max_distance)).asFloat64();
        }

        Bottle &gFiltering=rf.findGroup("filtering");
        if (!gFiltering.isNull())
        {
            filter_keypoint_order=gFiltering.check("filter-keypoint-order",Value(filter_keypoint_order)).asInt32();
            filter_limblength_order=gFiltering.check("filter-limblength-order",Value(filter_limblength_order)).asInt32();
            optimize_limblength=gFiltering.check("optimize-limblength",Value(optimize_limblength)).asBool();
        }

        string camera_remote = "/depthCamera";
        Bottle &gCamera=rf.findGroup("camera");
        if (!gCamera.isNull())
        {
            if (gCamera.check("fov"))
            {
                if (Bottle *fov=gCamera.find("fov").asList())
                {
                    if (fov->size()>=2)
                    {
                        camera_configured=true;
                        fov_h=fov->get(0).asFloat64();
                        fov_v=fov->get(1).asFloat64();
                        yInfo()<<"camera fov_h (from file) ="<<fov_h;
                        yInfo()<<"camera fov_v (from file) ="<<fov_v;
                    }
                }
            }
            if (gCamera.check("remote"))
            {
                camera_remote = gCamera.find("remote").asString();
            }
        }

        Property rgbdOpts;
        rgbdOpts.put("device", "RGBDSensorClient");

        rgbdOpts.put("remoteImagePort", camera_remote + "/rgbImage:o");
        rgbdOpts.put("remoteDepthPort", camera_remote + "/depthImage:o");
        rgbdOpts.put("remoteRpcPort", camera_remote + "/rpc:i");

        rgbdOpts.put("localImagePort", "/" + getName() + "/cam/rgb");
        rgbdOpts.put("localDepthPort", "/" + getName() + "/cam/depth");
        rgbdOpts.put("localRpcPort", "/" + getName() + "/cam/rpc");

        rgbdOpts.put("ImageCarrier", "mjpeg");
        rgbdOpts.put("DepthCarrier", "fast_tcp");

        if (!rgbdDrv.open(rgbdOpts)) {
            yError() << "Unable to talk to depthCamera!";
            return false;
        }

        skeletonsPort.open("/skeletonRetriever/skeletons:i");
        depthPort.open("/skeletonRetriever/depth:i");
        viewerPort.open("/skeletonRetriever/viewer:o");
        navPort.open("/skeletonRetriever/nav:i");
        gazePort.open("/skeletonRetriever/gaze:i");
        opcPort.open("/skeletonRetriever/opc:rpc");

        navFrame=gazeFrame=eye(4,4);

        t0=Time::now();
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
        const double t=Time::now();
        const double dt=t-t0;
        t0=t;

        if (ImageOf<PixelFloat> *depth=depthPort.read(false))
        {
            if (depth_enable)
            {
                filterDepth(*depth,this->depth,depth_kernel_size,depth_iterations,
                            depth_min_distance,depth_max_distance);
            }
            else
            {
                this->depth=*depth;
            }
        }

        if (!camera_configured)
        {
            camera_configured=getCameraOptions();
        }

        // garbage collector
        gc(dt);

        // update external frames
        update_nav_frame();
        update_gaze_frame();
        rootFrame=navFrame*gazeFrame;

        // handle skeletons acquired from detector
        if (Bottle *b1=skeletonsPort.read(false))
        {
            if (Bottle *b2=b1->get(0).asList())
            {
                // acquire skeletons with sufficient number of key-points
                vector<shared_ptr<MetaSkeleton>> new_accepted_skeletons;
                for (size_t i=0; i<b2->size(); i++)
                {
                    Bottle *b3=b2->get(i).asList();
                    if ((depth.width()>0) && (depth.height()>0) && (b3!=nullptr))
                    {
                        shared_ptr<MetaSkeleton> s=create(b3);
                        if (isValid(s))
                        {
                            new_accepted_skeletons.push_back(s);
                        }
                    }
                }

                // update existing skeletons / create new skeletons
                if (!new_accepted_skeletons.empty())
                {
                    Stamp stamp;
                    skeletonsPort.getEnvelope(stamp);

                    enforce_tag_uniqueness_input(new_accepted_skeletons);

                    vector<string> viewer_remove_tags;
                    vector<shared_ptr<MetaSkeleton>> pending=skeletons;
                    for (auto &n:new_accepted_skeletons)
                    {
                        vector<double> scores=computeScores(pending,n);
                        auto it=min_element(scores.begin(),scores.end());
                        if (it!=scores.end())
                        {
                            if (*it<numeric_limits<double>::infinity())
                            {
                                auto i=distance(scores.begin(),it);
                                auto &s=pending[i];
                                update(n,s,viewer_remove_tags);
                                opcSet(s,stamp);
                                pending.erase(pending.begin()+i);
                                continue;
                            }
                        }

                        if (opcAdd(n,stamp))
                        {
                            skeletons.push_back(n);
                        }
                    }

                    enforce_tag_uniqueness_pending(pending);
                    viewerUpdate(viewer_remove_tags);
                }
            }
        }

        return true;
    }

    /****************************************************************/
    bool close() override
    {
        // remove all skeletons from OPC
        gc(numeric_limits<double>::infinity());

        // rgbdDrv.close();
        skeletonsPort.close();
        depthPort.close();
        viewerPort.close();
        navPort.close();
        gazePort.close();
        opcPort.close();

        return true;
    }

/****************************************************************/
public:

    /****************************************************************/
    Retriever() : camera_configured(false), navFrameUpdated(false),
        gazeFrameUpdated(false) { }

};


/****************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"Unable to find Yarp server!";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.setDefaultContext("skeletonRetriever");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);

    Retriever retriever;
    return retriever.runModule(rf);
}

