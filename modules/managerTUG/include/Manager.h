#pragma once

#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/math/Math.h>
#include <yarp/os/LogComponent.h>

#include <condition_variable>
#include <memory>
#include <mutex>
#include <cmath>

#include <AssistiveRehab/skeleton.h>

#include "src/managerTUG_IDL.h"

// local classes
#include "AnswerManager.h"
#include "HandManager.h"
#include "helpers.h"
#include "ObstacleManager.h"
#include "Speech.h"
#include "SpeechParam.h"
#include "TriggerManager.h"

namespace
{
    YARP_LOG_COMPONENT(MANAGERTUG, "managerTUG")
}

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace assistive_rehab;


class Manager : public yarp::os::RFModule, public managerTUG_IDL
{
private:
    //params
    std::string module_name;
    std::string speak_file;
    double period;
    double pointing_time,arm_thresh;
    bool detect_hand_up;
    yarp::sig::Vector starting_pose;
    yarp::sig::Vector pointing_home,pointing_start,pointing_finish;
    bool simulation,lock;
    yarp::sig::Vector target_sim;
    std::string human_state;
    std::vector<std::string> laser_adverb;
    std::vector<double> engage_distance,engage_azimuth;

    const int ok=Vocab32::encode("ok");
    const int fail=Vocab32::encode("fail");
    enum class State { stopped,
                       idle,
                       obstacle,
                       lock,
                       seek_locked,
                       seek_skeleton,
                       follow, frozen,
                       assess_standing,
                       assess_crossing,
                       line_crossed,
                       engaged, point_start,
                       explain, point_line,
                       reach_line,
                       questions,
                       starting,
                       not_passed,
                       finished } state;
    State prev_state;
    std::string tag;
    double t0,tstart,t;
    double question_time_tstart;
    int encourage_cnt,reinforce_engage_cnt;
    int reinforce_obstacle_cnt;
    std::unordered_map<std::string,std::string> speak_map;
    std::unordered_map<std::string,int> speak_count_map;
    bool interrupting;
    std::mutex mtx;
    bool start_ex,ok_go,connected,params_set;
    std::string success_status;
    bool test_finished;

    Vector finishline_pose;
    double line_length;
    bool world_configured;

    //ports
    RpcClient analyzerPort;
    RpcClient speechRpcPort;
    RpcClient attentionPort;
    RpcClient navigationPort;
    BufferedPort<Bottle> speechStreamPort;
    RpcServer cmdPort;
    RpcClient leftarmPort;
    RpcClient rightarmPort;
    BufferedPort<Bottle> opcPort;
    RpcClient lockerPort;
    RpcClient triggerPort;
    RpcClient gazeboPort;
    RpcClient collectorPort;
    BufferedPort<Bottle> obstaclePort;

    std::unique_ptr<AnswerManager> answer_manager;
    std::unique_ptr<HandManager> hand_manager;
    std::unique_ptr<TriggerManager> trigger_manager;
    std::unique_ptr<ObstacleManager> obstacle_manager;
public:

    Manager();

    bool attach(RpcServer &source) override;

    bool load_speak(const std::string &context, const std::string &speak_file);

    bool speak(Speech &s);

    std::string get_sentence(std::string &value, const std::vector<std::shared_ptr<SpeechParam>> &p) const;

    std::string get_animation();

    bool play_animation(const std::string &name);

    bool play_from_last();

    bool set_auto();

    bool send_stop(const RpcClient &port);

    bool is_navigating();

    bool disengage();

    void resume_animation();

    bool go_to(const Vector &target, const bool &wait);

    bool remove_locked();

    bool start() override;

    bool trigger() override;

    bool set_target(const double x, const double y, const double theta) override;

    double get_measured_time() override;

    std::string get_success_status() override;

    bool has_finished() override;

    bool stop() override;

    bool configure(ResourceFinder &rf) override;

    double getPeriod() override;

    bool updateModule() override;

    void follow(const std::string &follow_tag);

    void encourage(const double &timeout);

    void start_interaction();

    bool start_collection();

    bool set_analyzer_param(const std::string & option, const std::string & arg);

    void get_walking_speed(const Bottle &r);

    bool is_active();

    std::string which_part();

    bool point(const Vector &target, const std::string &part, const bool wait);

    bool opcRead(const std::string &t, Property &prop, const std::string &tval="");

    bool hasLine(Property &prop);

    bool getWorld(const Property &prop);

    bool findLocked(std::string &t);

    bool interruptModule() override;

    bool close() override;
};
