class Manager : public yarp::os::RFModule, public managerTUG_IDL
{
private:
    //params
    std::string module_name;
    std::string speak_file;
    double period;
    double pointing_time,arm_thresh;
    bool detect_hand_up;
    Vector starting_pose,
    Vector pointing_home,pointing_start,pointing_finish;
    bool simulation,lock;
    Vector target_sim;
    string human_state;
    vector<string> laser_adverb;
    vector<double> engage_distance,engage_azimuth;

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
};