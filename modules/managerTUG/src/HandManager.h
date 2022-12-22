#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcClient.h>
#include <AssistiveRehab/skeleton.h>
#include <yarp/os/Thread.h>




class HandManager : public yarp::os::Thread
{
    std::string module_name;
    yarp::os::BufferedPort<yarp::os::Bottle> *opcPort;
    yarp::os::RpcClient *triggerPort;
    assistive_rehab::Skeleton *skeleton;
    double arm_thresh;
    std::string tag,part;
    bool isArmLifted;

public:

    HandManager(const std::string &module_name_, const double &arm_thresh_);

    void set_tag(const std::string &tag);
    
    void setPorts(yarp::os::BufferedPort<yarp::os::Bottle> *opcPort, yarp::os::RpcClient *triggerPort);

    void run() override;

    void onStop() override;

    void get_skeleton();

    bool is_with_raised_hand();

    bool is_raised() const;

    bool is_raised(const std::string &p) const;
};
