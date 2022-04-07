# main.py
import sys
import yarp
import time
import random
import math
import os
import glob
import shutil

def connect(from_port, to_port, *protocol):
    """
    Connect two YARP ports (using YARP APIs). If the connection is already
    established, do nothing. Return whether the ports are connected.

    Arguments:
    from_port -- name of the source port
    to_port -- name of the destination port
    protocol -- (optional) YARP network protocol: tcp (default), udp, unix_stream, mcast, etc.

    Return values:
    success -- boolean indicating whether the ports are connected or not
    """

    success = False

    if yarp.Network.isConnected(from_port, to_port):
        # already connected, nothing to do
        success = True
        return success

    if protocol:
        success = yarp.Network.connect(from_port, to_port, str(protocol[0]))
    else:
        success = yarp.Network.connect(from_port, to_port)

    return success


class RiskAnalysis():
    # here we want to initialize everything, start up the deployment, check if all is ok
    def __init__(self):

        # connect to tug_input_port RPC server - get the position from gazebo (correct odometry) / change speed / get ground truth
        self.gazeboControl_rpc = yarp.RpcClient()
        local_name = '/riskAnalyzer/tug_input_port:rpc'
        remote_name = '/tug_input_port'
        self.gazeboControl_rpc.open(local_name)
        _ = connect(local_name, remote_name)

        # connect to navController RPC to make the robot navigate
        self.navController_rpc = yarp.RpcClient()
        local_name = '/riskAnalyzer/navController:rpc'
        remote_name = '/navController/rpc'
        self.navController_rpc.open(local_name)
        _ = connect(local_name, remote_name)

        # connect to obstacleDetector RPC to have info about obstacles
        self.obstacleDetector_rpc = yarp.RpcClient()
        local_name = '/riskAnalyzer/obstacleDetector:rpc'
        remote_name = '/obstacleDetector/rpc'
        self.obstacleDetector_rpc.open(local_name)
        _ = connect(local_name, remote_name)

        # connect to baseControl RPC to set the robot speed
        self.baseController_rpc = yarp.RpcClient()
        local_name = '/riskAnalyzer/baseControl:rpc'
        remote_name = '/baseControl/rpc'
        self.baseController_rpc.open(local_name)
        _ = connect(local_name, remote_name)

        # setting the baseControl maximum speed
        print(f"Setting maximum speed to {maximum_speed}")
        cmd = yarp.Bottle()
        cmd.clear()
        cmd.addString('set_max_lin_vel')
        cmd.addDouble(application_speed)
        rep = yarp.Bottle()
        rep.clear()
        self.baseController_rpc.write(cmd, rep)

    def measure_time_to_stop_obstacle(self, num_trials, speed):
        ###################################################
        # - we measure the time to send a stop signal to  #
        #  dnavigation when an obstacle is found          #
        # - we place an obstacle in front of the robot    #
        #  at fixed distance of 2.5 m                     #
        ###################################################
        print("#################################")            
        print("# measure time to stop obstacle #")
        print("#################################") 
        trial = 0
        avg_time_to_stop = 0.0
        while trial < num_trials:
            print("Starting trial", trial, "with speed", speed)

            # first we set the speed
            cmd = yarp.Bottle()
            cmd.clear()
            cmd.addString('set_linear_velocity')
            cmd.addDouble(speed)
            rep = yarp.Bottle()
            rep.clear()
            self.navController_rpc.write(cmd, rep)

            #########################################################
            # update odometry of the robot with real world odometry #
            #########################################################
            cmd = yarp.Bottle()
            cmd.clear()
            cmd.addString('getModelPos')
            cmd.addString('SIM_CER_ROBOT')
            rep = yarp.Bottle()
            rep.clear()
            self.gazeboControl_rpc.write(cmd, rep)
             
            if rep.check("SIM_CER_ROBOT"):
                sim_cer_robot = rep.find("SIM_CER_ROBOT")
                pose_world = sim_cer_robot.find("pose_world").asList()
                _x = pose_world.get(0).asDouble()
                _y = pose_world.get(1).asDouble()
                _z = pose_world.get(2).asDouble()
                _ax = pose_world.get(3).asDouble()
                _ay = pose_world.get(4).asDouble()
                _az = pose_world.get(5).asDouble()
                _angle = math.degrees(pose_world.get(6).asDouble())
            else:
                continue

            # set odometry
            cmd = yarp.Bottle()
            cmd.clear()
            cmd.addString('reset_odometry')
            cmd.addDouble(_x)
            cmd.addDouble(_y)
            cmd.addDouble(_angle)
            print(cmd.toString())
            rep = yarp.Bottle()
            rep.clear()

            self.navController_rpc.write(cmd, rep)
            print("navController odometry reset reply was:", rep.toString())
            time.sleep(0.5)
            #########################################################

            # move the robot to the initial position
            print("Going back to initial position")   
            cmd = yarp.Bottle()
            cmd.clear()
            cmd.addString('go_to_wait')
            cmd.addDouble(2.5)
            cmd.addDouble(0.0)
            cmd.addDouble(180.0)
            cmd.addInt(1)
            rep = yarp.Bottle()
            rep.clear()
            self.navController_rpc.write(cmd, rep)

            # start to detect obstacles
            cmd = yarp.Bottle()
            cmd.clear()
            cmd.addString('start')
            rep = yarp.Bottle()
            rep.clear()
            self.obstacleDetector_rpc.write(cmd, rep)

            print("robot is moving")
            # move the robot forward
            cmd = yarp.Bottle()
            cmd.clear()
            cmd.addString('go_to_dontwait')
            cmd.addDouble(0.0)
            cmd.addDouble(0.0)
            cmd.addDouble(180.0)
            rep = yarp.Bottle()
            rep.clear()
            self.navController_rpc.write(cmd, rep)

            # now we check when the robot has stopped
            is_navigating = "[ok]"
            while is_navigating == "[ok]":
                cmd = yarp.Bottle()
                cmd.clear()
                cmd.addString('is_navigating')
                rep = yarp.Bottle()
                rep.clear()
                self.navController_rpc.write(cmd, rep)
                
                is_navigating = rep.toString()
                time.sleep(0.1)
            
            print("The robot is not navigating anymore...")   

            # get the time from obstacleDetector 
            cmd = yarp.Bottle()
            cmd.clear()
            cmd.addString('get_time_to_stop')
            rep = yarp.Bottle()
            rep.clear()
            self.obstacleDetector_rpc.write(cmd, rep)
            _measured_time_to_stop = rep.get(0).asDouble()
            avg_time_to_stop += _measured_time_to_stop

            print("Measured time for trial", trial, "is:", _measured_time_to_stop)   

            # stop to detect obstacles
            cmd = yarp.Bottle()
            cmd.clear()
            cmd.addString('stop')
            rep = yarp.Bottle()
            rep.clear()
            self.obstacleDetector_rpc.write(cmd, rep)

            file_name="data_experiment_obstacle_" + str(speed) + ".txt"
            with open(file_name, "a") as myfile:
                if trial == 0:
                    data_to_write = "" + str(trial) 
                else:
                    data_to_write = "\n" + str(trial)
                data_to_write += " " + str(_measured_time_to_stop)
                myfile.write(data_to_write)

            trial += 1

        # we compute the average time over the number of trials
        avg_time_to_stop /= num_trials
        print("The average time to stop for ", num_trials, "is:", avg_time_to_stop)

        with open(file_name, "a") as myfile:
            data_to_write = "\n" + "average_time_to_stop:" + str(avg_time_to_stop)
            myfile.write(data_to_write)


    def measure_time_to_stop(self, num_trials, speed):
        ###################################################
        # - we measure the time between sending a stop    #
        #  command and the robot physically stopping      #
        # - we instruct the robot to move 2 meters and    #
        #  we ask him to stop 5 seconds after             #
        ###################################################
        print("################################")            
        print("#     measure time to stop     #")
        print("################################") 
        trial = 0
        avg_time_to_stop = 0.0
        while trial < num_trials:
            print("Starting trial", trial, "with speed", speed)

            # first we set the speed
            cmd = yarp.Bottle()
            cmd.clear()
            cmd.addString('set_linear_velocity')
            cmd.addDouble(speed)
            rep = yarp.Bottle()
            rep.clear()
            self.navController_rpc.write(cmd, rep)

            #########################################################
            # update odometry of the robot with real world odometry #
            #########################################################
            cmd = yarp.Bottle()
            cmd.clear()
            cmd.addString('getModelPos')
            cmd.addString('SIM_CER_ROBOT')
            rep = yarp.Bottle()
            rep.clear()
            self.gazeboControl_rpc.write(cmd, rep)
             
            if rep.check("SIM_CER_ROBOT"):
                sim_cer_robot = rep.find("SIM_CER_ROBOT")
                pose_world = sim_cer_robot.find("pose_world").asList()
                _x = pose_world.get(0).asDouble()
                _y = pose_world.get(1).asDouble()
                _z = pose_world.get(2).asDouble()
                _ax = pose_world.get(3).asDouble()
                _ay = pose_world.get(4).asDouble()
                _az = pose_world.get(5).asDouble()
                _angle = math.degrees(pose_world.get(6).asDouble())
            else:
                continue

            # set odometry
            cmd = yarp.Bottle()
            cmd.clear()
            cmd.addString('reset_odometry')
            cmd.addDouble(_x)
            cmd.addDouble(_y)
            cmd.addDouble(_angle)
            print(cmd.toString())
            rep = yarp.Bottle()
            rep.clear()

            self.navController_rpc.write(cmd, rep)
            print("navController odometry reset reply was:", rep.toString())
            time.sleep(0.5)
            #########################################################

            # move the robot to the initial position
            print("Going back to initial position")   
            cmd = yarp.Bottle()
            cmd.clear()
            cmd.addString('go_to_wait')
            cmd.addDouble(3.5)
            cmd.addDouble(0.0)
            cmd.addDouble(180.0)
            cmd.addInt(1)
            rep = yarp.Bottle()
            rep.clear()
            self.navController_rpc.write(cmd, rep)

            # move the robot forward
            cmd = yarp.Bottle()
            cmd.clear()
            cmd.addString('go_to_dontwait')
            cmd.addDouble(0.0)
            cmd.addDouble(0.0)
            cmd.addDouble(180.0)
            rep = yarp.Bottle()
            rep.clear()
            self.navController_rpc.write(cmd, rep)
            print("robot is moving")
            
            # we should allow the robot to get some speed
            time.sleep(10.0)

            # now we stop the robot
            print("Stopping the robot")
            cmd = yarp.Bottle()
            cmd.clear()
            cmd.addString('stop')
            rep = yarp.Bottle()
            rep.clear()
            self.navController_rpc.write(cmd, rep)

            # now we get the time
            cmd = yarp.Bottle()
            cmd.clear()
            cmd.addString('get_time_to_stop')
            rep = yarp.Bottle()
            rep.clear()
            self.navController_rpc.write(cmd, rep)
            print("bottle was:", rep.toString())
            _measured_time_to_stop = rep.get(0).asDouble()
            avg_time_to_stop += _measured_time_to_stop        

            print("Measured time for trial", trial, "is:", _measured_time_to_stop)   

            file_name="data_experiment_stop_" + str(speed) + ".txt"
            with open(file_name, "a") as myfile:
                if trial == 0:
                    data_to_write = "" + str(trial) 
                else:
                    data_to_write = "\n" + str(trial)
                data_to_write += " " + str(_measured_time_to_stop)
                myfile.write(data_to_write)

            trial += 1

        # we compute the average time over the number of trials
        avg_time_to_stop /= num_trials
        print("The average time to stop for ", num_trials, "is:", avg_time_to_stop)

        with open(file_name, "a") as myfile:
            data_to_write = "\n" + "average_time_to_stop:" + str(avg_time_to_stop)
            myfile.write(data_to_write)

    def measure_collisions(self, num_trials, speed):
        #####################
        #                   #
        #####################
        print("################################")            
        print("#                              #")
        print("################################")     
        trial = 0
        while trial < num_trials:
            print("Starting trial", trial, "with speed", speed)

            # first we set the speed
            cmd = yarp.Bottle()
            cmd.clear()
            cmd.addString('set_linear_velocity')
            cmd.addDouble(speed)
            rep = yarp.Bottle()
            rep.clear()
            self.navController_rpc.write(cmd, rep)

            # move the robot forward 
            cmd = yarp.Bottle()
            cmd.clear()
            cmd.addString('go_to_dontwait')
            cmd.addDouble(3.0)
            cmd.addDouble(0.0)
            cmd.addDouble(0.0)
            rep = yarp.Bottle()
            rep.clear()
            self.navController_rpc.write(cmd, rep)

            trial += 1

        

if __name__ == '__main__':

    yarp.Network.init()

    rf = yarp.ResourceFinder()
    rf.setVerbose(True)
    rf.setDefaultConfigFile('conf.ini')
    rf.configure(sys.argv)

    maximum_speed = rf.find("maximum_speed").asDouble()
    application_speed = rf.find("application_speed").asDouble()

    # the function should be called like "python risk_analysis.py ${num_trials}"
    arg_list=sys.argv
    analysis=RiskAnalysis()
    analysis.measure_time_to_stop_obstacle(int(arg_list[1]), application_speed) 
    analysis.measure_time_to_stop_obstacle(int(arg_list[1]), maximum_speed) 
    analysis.measure_time_to_stop(int(arg_list[1]), application_speed) 
    analysis.measure_time_to_stop(int(arg_list[1]), maximum_speed) 
    # analysis.measure_collisions(int(arg_list[1]), application_speed)
    # analysis.measure_collisions(int(arg_list[1]), maximum_speed)
    
    yarp.Network.fini()
