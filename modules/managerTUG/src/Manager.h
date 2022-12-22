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

#include <mutex>
#include <cmath>

#include <AssistiveRehab/skeleton.h>

#include "src/managerTUG_IDL.h"

namespace
{
    YARP_LOG_COMPONENT(MANAGERTUG, "managerTUG")
}

using namespace std;
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
    yarp::sig::Vector starting_pose,
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
                       starting, 
                       not_passed, 
                       finished } state;
    State prev_state;
    string tag;
    double t0,tstart,t;
    int encourage_cnt,reinforce_engage_cnt;
    int reinforce_obstacle_cnt;
    unordered_map<string,string> speak_map;
    unordered_map<string,int> speak_count_map;
    bool interrupting;
    mutex mtx;
    bool start_ex,ok_go,connected,params_set;
    string success_status;
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

    AnswerManager *answer_manager;
    HandManager *hand_manager;
    TriggerManager *trigger_manager;
    ObstacleManager *obstacle_manager;
public:

    Manager()

    bool attach(RpcServer &source) override;
    
    bool load_speak(const string &context, const string &speak_file);

    bool speak(Speech &s);
    
    string get_sentence(string &value, const vector<shared_ptr<SpeechParam>> &p) const;
    
    string get_animation();
    
    bool play_animation(const string &name);
    
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

    string get_success_status() override;

    bool has_finished() override;

    bool stop() override;

    bool configure(ResourceFinder &rf) override;

    double getPeriod() override;

    bool updateModule() override;

    void follow(const string &follow_tag);

    void encourage(const double &timeout);

    void start_interaction();

    bool start_collection();
 
    void set_walking_speed(const Bottle &r);

    bool is_active();

    string which_part();

    bool point(const Vector &target, const string &part, const bool wait);

    bool opcRead(const string &t, Property &prop, const string &tval="");

    bool hasLine(Property &prop);

    bool getWorld(const Property &prop);

    bool findLocked(string &t);

    bool interruptModule() override;

    bool close() override;
};
