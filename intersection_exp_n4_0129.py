import sys
sys.path.append('/Users/estel/102_passive_cf_python')

"""
this is the code for testing the convergence control for a swarm of drones
"""
# TODO: log the real time stamp, and the status flag.
# TODO: do we need a low pass filter?
import logging
import time
import math
import numpy as np
from mat4py import loadmat
from phe import paillier

# libs from crazyflie
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.swarm import CachedCfFactory # -> for swarm
from cflib.crazyflie.swarm import Swarm # -> for swarm

# libs from rislab
from rislab_lib.mocap_udp import UdpReceiver
from rislab_lib.Joystick import joystick
from rislab_lib.savemat import savemat
from rislab_lib.controllers import pid_3d
from rislab_lib.data_processor import GeneralFcn
from rislab_lib.controllers import trajectory_loader

# libs from Estel
from cooperative_transport.controllers import PID_ControllerThreeAixs, Creat_Agent, crazyflie_connect_check, joystick_amend
from cooperative_transport.wheel_functions import limit_angular, string_list_format_converter

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR) 

# ----- functions block -----
# saturation funciton for thrust
def limit_thrust(thrust_cmd):
    if thrust_cmd > 65535:
        thrust_cmd = 65535
    if thrust_cmd < 0:
        thrust_cmd = 0
    return int(thrust_cmd)
  

# get the commands for the roll, pitch, yaw and thrust
def get_rpyt_cmd(u_x, u_y, u_z, drone):
    drone.pitch.ctrl_cmd = -(u_x * math.cos(drone.yaw.fdk) + u_y * math.sin(drone.yaw.fdk)) / (math.cos(drone.yaw.fdk) * math.cos(drone.yaw.fdk) 
                          + math.sin(drone.yaw.fdk) * math.sin(drone.yaw.fdk))
    drone.roll.ctrl_cmd = (u_y * math.cos(drone.yaw.fdk) - u_x * math.sin(drone.yaw.fdk)) / (math.cos(drone.yaw.fdk) * math.cos(drone.yaw.fdk) 
                        + math.sin(drone.yaw.fdk) * math.sin(drone.yaw.fdk))
    drone.thrus_cmd.ctrl_cmd = limit_thrust(u_z)

    drone.yaw.err = limit_angular(limit_angular(drone.yaw.des) - limit_angular(drone.yaw.fdk))
    drone.yaw.ctrl_cmd = -drone.yaw.err * 50

# use joystick to control the trajectory
def joystick_traj_ctrl(drone):
    if joystick.get_button_lb():
        # traj ini, to the start point but hover there
        if joystick.get_button_a():
            drone.traj_ini_flag = True

        # traj start, start the trajectory               
        if drone.traj_ini_flag and joystick.get_button_b():
            drone.traj_start_flag = True
            drone.traj_end_flag = False
            drone.start_time = time.time()

        # traj end, stop the trajectory
        if drone.traj_start_flag and joystick.get_button_y():
            drone.traj_end_flag = True
            drone.traj_start_flag = False
    
    if joystick.get_button_rb():
        # hold the trajectory
        if drone.traj_start_flag and joystick.get_button_a():
            drone.traj_hold_flag = True
        
        # continue the trajectory
        if drone.traj_start_flag and joystick.get_button_b():
            drone.traj_hold_flag = False

# general joystick to control landing and terminating
def joystick_general(drone, axis_act_thre, axis_deact_thre):
    if joystick.get_button_lb() > axis_act_thre and joystick.get_button_rb() > axis_act_thre:
        drone.land_flag = True
        drone.event_flag = int(2)
        drone.land_time = time.time()

    if joystick.get_button_x() and joystick.get_button_lb() > axis_act_thre and joystick.get_button_rb() < axis_deact_thre:
        drone.terminate_flag = True
        drone.terminate_time = time.time()

# joystick to control the drone one, send the take off, land and terminate flag
def joystick_drone_one(drone_one, axis_act_thre, axis_deact_thre):
    if joystick.get_button_l() and joystick.get_button_lb() < axis_deact_thre and joystick.get_button_rb() < axis_deact_thre:
        drone_one.take_off_flag = True
        drone_one.event_flag = int(1)
        drone_one.take_off_time = time.time()
    
    joystick_general(drone_one, axis_act_thre, axis_deact_thre)

# joystick to control the drone two, send the take off, land and terminate flag
def joystick_drone_two(drone_two, axis_act_thre, axis_deact_thre):
    if joystick.get_button_r() and joystick.get_button_lb() > axis_act_thre and joystick.get_button_rb() < axis_deact_thre:
        drone_two.take_off_flag = True
        drone_two.event_flag = int(1)
        drone_two.take_off_time = time.time()
    
    joystick_general(drone_two, axis_act_thre, axis_deact_thre)

# joystick to control the drone two, send the take off, land and terminate flag
def joystick_drone_three(drone_three, axis_act_thre, axis_deact_thre):
    if joystick.get_button_r() and joystick.get_button_lb() < axis_act_thre and joystick.get_button_rb() > axis_deact_thre:
        drone_three.take_off_flag = True
        drone_three.event_flag = int(1)
        drone_three.take_off_time = time.time()
    
    joystick_general(drone_three, axis_act_thre, axis_deact_thre)

def joystick_drone_four(drone_four, axis_act_thre, axis_deact_thre):
    if joystick.get_button_r() and joystick.get_button_lb() < axis_act_thre and joystick.get_button_rb() > axis_deact_thre:
        drone_three.take_off_flag = True
        drone_three.event_flag = int(1)
        drone_three.take_off_time = time.time()
    
    joystick_general(drone_four, axis_act_thre, axis_deact_thre)    

def traj_index_update(drone):
    if drone.traj_start_flag:
        if drone.traj_hold_flag:
            drone.traj_index += 0
        else:
            drone.traj_index += 1

        if drone.traj_index >= drone.traj_index_terminate:
            drone.traj_index = drone.traj_index_terminate
            drone.traj_end_flag = True
            drone.traj_start_flag = False
                                
            print('trajectory finished')

def traj_pos_update(drone, traj_data):
    if drone.traj_ini_flag and not drone.traj_start_flag and not drone.traj_end_flag:
        temp_x = traj_data["pos_x_traj"][0]
        temp_y = traj_data["pos_y_traj"][0]
        temp_z = traj_data["pos_z_traj"][0]
                    
    if drone.traj_start_flag:
        temp_x = traj_data["pos_x_traj"][drone.traj_index]
        temp_y = traj_data["pos_y_traj"][drone.traj_index]
        temp_z = traj_data["pos_z_traj"][drone.traj_index]

    if drone.traj_end_flag:
        temp_x = traj_data["pos_x_traj"][drone.traj_index]
        temp_y = traj_data["pos_y_traj"][drone.traj_index]
        temp_z = traj_data["pos_z_traj"][drone.traj_index]
    
    #update the desired value
    if drone.traj_start_flag or drone.traj_ini_flag and not drone.traj_end_flag:
        drone.pos_x.des = temp_x
        drone.pos_y.des = temp_y
        drone.pos_z.des = temp_z    

def terminate_sequence(scf):
    scf.cf.commander.send_setpoint(0, 0, 0, 3000)
    time.sleep(0.5)
    scf.cf.commander.send_setpoint(0, 0, 0, 0)

def standby_sequence(scf):
    scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 10000)

def land_sequence(drone, scf):
    if time.time() - drone.land_time <= 3.0:
        scf.cf.commander.send_setpoint(0.0, 0, 0, 40000)
    elif time.time() - drone.land_time <= 4.0:
        scf.cf.commander.send_setpoint(0.0, 0, 0, 36000)
    elif time.time() - drone.land_time <= 6.0:
        scf.cf.commander.send_setpoint(0.0, 0, 0, 33000)
    else:
        scf.cf.commander.send_setpoint(0.0, 0, 0, 5000)    


def pole_to_carterian(R, theta):
    # theta must be in radians
    x = R * math.cos(theta)
    y = R * math.sin(theta)
    z = 0.8

    return x , y, z


# calculate the control signal
def cal_sat(input):
    if np.abs(input) <= 1:
        output = input
    else:
        output = np.sign(input)
    
    return output

def cal_U(drone_ID, angular_velocity, ctrl_gain_mu, ucov, ucol):
    global global_phi

    i = drone_ID - 1

    U = -angular_velocity + ctrl_gain_mu*cal_sat((ucov + ucol)/global_phi[i]) 

    return U

# set the initial angles, calculate the initial positions 
mission_radius = 0.1
#finaldes1= np.array([-0.8,-0.5,0.7])
#finaldes2= np.array([0.8,0.5,0.7])
finaldes1= np.array([0.8,-0.6,0.7])
finaldes2= np.array([0.6,0.8,0.7])
finaldes3= np.array([-0.6,-0.8,0.7])
finaldes4= np.array([-0.8,0.6,0.7])

printcount=0
maxprintcount=30


# ----- function block end -----

# crazyflie_control is the main control function for the swarm control which include the drone_one and follower 1
def crazyflie_control(scf, control_which_flag):
    """
    set the controller for this carzyflie in the firmware
    1 -> PID controllerh
    2 -> Mellinger controller
    3 -> INDI controller
    """
    global mission_radius
    global printcount
    global maxprintcount
    global finaldes1
    global finaldes2
    global finaldes3
    global finaldes4    
    global p_center1
    global p_center2
    global p_center3
    global p_center4    

    # set the controller for all the crazyflies in the swarm to be PID 
    scf.cf.param.set_value('stabilizer.controller', '1')
    time.sleep(0.1)
    # unlock all the motors, this step is very significant
    scf.cf.commander.send_setpoint(0, 0, 0, 0)
    time.sleep(0.1)

    axis_act_thre = 0.95
    axis_deact_thre = 0.05
    
    # set the rigidbody ID
    rigid_body_ID_drone_one = 1
    rigid_body_ID_drone_two = 2
    rigid_body_ID_drone_three = 3
    rigid_body_ID_drone_four = 4

    Filter_x_1 = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=15, fs=sample_rate)
    Filter_y_1 = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=15, fs=sample_rate)
    Filter_v_1 = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=15, fs=sample_rate)
    Filter_z = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=15, fs=sample_rate)

    Filter_x_2 = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=15, fs=sample_rate)
    Filter_y_2 = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=15, fs=sample_rate)
    Filter_v_2 = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=15, fs=sample_rate)

    Filter_x_3 = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=15, fs=sample_rate)
    Filter_y_3 = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=15, fs=sample_rate)
    Filter_v_3 = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=15, fs=sample_rate)

    Filter_x_4 = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=15, fs=sample_rate)
    Filter_y_4 = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=15, fs=sample_rate)
    Filter_v_4 = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=15, fs=sample_rate)    
    # Filter_pad_rx = GeneralFcn.IIR2Filter(4, [4], 'lowpass', design='cheby2', rs=50, fs=sample_rate)

    time_previous = 0

    # initialize the parameters for all drones
    #number_of_agents = 3

    # global variables for the convergence swarm control
    n=4 #agent number

    
    max_vel_lambda = np.array([15, 5, 9, 0.8, 1.4])

    ctrl_gain_k = 2
    ctrl_gain_gamma=1

    ctrl_gain_kx = 0.5 
    ctrl_gain_ky = 0.5#1  
    ctrl_gain_kp=0.5 
    ctrl_gain_a = [1,1,1,1]
    ctrl_gain_m = 1
    epsilon = 0.01
    u_max=1

    u = np.array([0, 0, 0, 0, 0])
    v = np.array([0, 0, 0, 0, 0])

    # initialize ends
    # time_amed = 0.5
    time_amend = 0.7

    #theta_ini = np.array([10, 20, 30])/180*math.pi

    u_1_prev = np.array([0, 0, 0])
    u_2_prev = np.array([0, 0, 0])
    u_3_prev = np.array([0, 0, 0])
    u_4_prev = np.array([0, 0, 0])    
    u_1 = np.array([0, 0, 0])
    u_2 = np.array([0, 0, 0])
    u_3 = np.array([0, 0, 0])
    u_4 = np.array([0, 0, 0])    

    # q_1_prev = theta_ini[0]
    # q_2_prev = theta_ini[1]
    # q_3_prev = theta_ini[2]

    # q_1_des_prev = q_1_prev
    # q_2_des_prev = q_2_prev
    # q_3_des_prev = q_3_prev
    # ------- DRONE ONE CONTROL BRANCH -------
    if control_which_flag[0] == 1.0:

        # dp data processor
        dp_drone_one = GeneralFcn.RealTimeProcessor() # for controlling the drone one
        dp_drone_two = GeneralFcn.RealTimeProcessor() # for recording the drone two
        dp_drone_three = GeneralFcn.RealTimeProcessor() # for recording the drone three
        dp_drone_four = GeneralFcn.RealTimeProcessor() # for recording the drone four

        # set the position controller
        position_controller_drone_one = PID_ControllerThreeAixs(sample_time,
                                                  12, 2.5, 25, 0,
                                                  12, 2.5, 25, 0,
                                                  15000, 2200, 8000, 37000, )

        # the logging data list, for the drone_one
        lg_stab_drone_one = LogConfig(name='Stabilizer', period_in_ms=sample_rate_in_ms)
        lg_stab_drone_one.add_variable('stabilizer.thrust', 'float')
        
        with SyncLogger(scf, lg_stab_drone_one) as logger_drone_one:
            # while not flight_terminate_flag:
            for log_entry in logger_drone_one:
                

                time.sleep(sample_time)
                # time_diff = time.time() - time_previous
                # read from the MoCap
                data_all_agents = receiver.get_data()

                # get the data from MoCap based on rigid body ID
                dp_drone_one.step(data_all_agents[(rigid_body_ID_drone_one-1)*14 : rigid_body_ID_drone_one*14])
                dp_drone_two.step(data_all_agents[(rigid_body_ID_drone_two-1)*14 : rigid_body_ID_drone_two*14])
                dp_drone_three.step(data_all_agents[(rigid_body_ID_drone_three-1)*14 : rigid_body_ID_drone_three*14])
                dp_drone_four.step(data_all_agents[(rigid_body_ID_drone_four-1)*14 : rigid_body_ID_drone_four*14])
                
                # save the data from the mocap 
                    
                # get the feedback from mocap
                drone_one.yaw.fdk = math.atan2(2 * (dp_drone_one.QW * dp_drone_one.QZ + dp_drone_one.QX * dp_drone_one.QY), 1 - 2 * (dp_drone_one.QZ * dp_drone_one.QZ + dp_drone_one.QY * dp_drone_one.QY))
                drone_one.pos_x.fdk = Filter_x_1.filter(dp_drone_one.X)
                drone_one.pos_y.fdk = Filter_y_1.filter(dp_drone_one.Y)
                drone_one.pos_z.fdk = Filter_z.filter(dp_drone_one.Z)

                drone_two.pos_x.fdk = Filter_x_2.filter(dp_drone_two.X)
                drone_two.pos_y.fdk = Filter_y_2.filter(dp_drone_two.Y)
                drone_two.pos_z.fdk = Filter_z.filter(dp_drone_two.Z)

                drone_three.pos_x.fdk = Filter_x_3.filter(dp_drone_three.X)
                drone_three.pos_y.fdk = Filter_y_3.filter(dp_drone_three.Y)
                drone_three.pos_z.fdk = Filter_z.filter(dp_drone_three.Z) 

                drone_four.pos_x.fdk = Filter_x_4.filter(dp_drone_four.X)
                drone_four.pos_y.fdk = Filter_y_4.filter(dp_drone_four.Y)
                drone_four.pos_z.fdk = Filter_z.filter(dp_drone_four.Z)               
                
                drone_one_posxy=np.array([drone_one.pos_x.fdk,drone_one.pos_y.fdk])
                drone_two_posxy=np.array([drone_two.pos_x.fdk,drone_two.pos_y.fdk])
                drone_three_posxy=np.array([drone_three.pos_x.fdk,drone_three.pos_y.fdk])
                drone_four_posxy=np.array([drone_four.pos_x.fdk,drone_four.pos_y.fdk])

                # read from the joystick  
                joystick.step()
                joystick_drone_one(drone_one, axis_act_thre, axis_deact_thre) # commands from joystick
                joystick_traj_ctrl(drone_one) # differnet stages for the trajectory, control by the joystick
                
                
                if drone_one.traj_start_flag and not drone_one.traj_end_flag:
                    # time_in_traj = time.time() - drone_one.start_time
                    
                    # update the desired X and Y positions for the drone one based on the convergence ctrl law.
                    #p2l=abs((p_dest(i,2)-p_center(i,2))*p(i,1)-(p_dest(i,1)-p_center(i,1))*p(i,2)+(p_dest(i,1)-p_center(i,1))*p_center(i,2)-(p_dest(i,2)-p_center(i,2))*p_center(i,1))/sqrt((p_dest(i,2)-p_center(i,2))^2+(p_dest(i,1)-p_center(i,1))^2);
                    #p2l=abs((finaldes1[1]-p_center1[1])*drone_one.pos_x.fdk-(finaldes1[0]-p_center1[0])*drone_one.pos_y.fdk+(finaldes1[0]-p_center1[0])*p_center1[1]-(finaldes1[1]-p_center1[1])*p_center1[0])/math.sqrt((finaldes1[1]-p_center1[1])**2+(finaldes1[0]-p_center1[0])**2)
                    #print('p2l=',p2l)
                    #temp=[(p_dest(i,1)-p_center(i,1)) -(p_dest(i,2)-p_center(i,2));(p_dest(i,2)-p_center(i,2)) (p_dest(i,1)-p_center(i,1))]/norm(p_dest(i,:)-p_center(i,:))*[kx*norm(p_dest(i,:)-p(i,:));ky*p2l^m];
                    #temp=np.mat([[(finaldes1[0]-p_center1[0]), -(finaldes1[1]-p_center1[1])],[(finaldes1[1]-p_center1[1]), (finaldes1[0]-p_center1[0])]])/np.linalg.norm(finaldes1-p_center1)*np.mat([[ctrl_gain_kx*np.linalg.norm(finaldes1-p_center1)],[ctrl_gain_ky*p2l**ctrl_gain_m]])
                    
                    #temp=ctrl_gain_kp*np.array([finaldes1[0]-drone_one_posxy[0],finaldes1[1]-drone_one_posxy[1]])
                    temp=ctrl_gain_kp*np.mat([[finaldes1[0]-drone_one_posxy[0]],[finaldes1[1]-drone_one_posxy[1]]])
                    print('temp=',temp)
                    #u_1=np.transpose(temp)
                    # u_1[0], u_1[1]=temp[0,0],temp[1,0]
                    mode_one=0
                    u_1= [temp[0,0],temp[1,0], 0]

                    print('u_1=',u_1)
                    
                    # q_1 = math.atan2(drone_one.pos_y.fdk, drone_one.pos_x.fdk)
                    # q_2 = math.atan2(Filter_y_2.filter(dp_drone_two.Y), Filter_x_2.filter(dp_drone_two.X))
                    # q_3 = math.atan2(Filter_y_3.filter(dp_drone_three.Y), Filter_x_3.filter(dp_drone_three.X))

                    #if np.dot((drone_two_posxy-drone_one_posxy),(np.array(finaldes1[0],finaldes1[1])-drone_one_posxy))>0: 
                    print('drone_one_posxy=',drone_one_posxy)
                    print('drone_two_posxy=',drone_two_posxy)
                    print('drone_one_posxy-drone_two_posxy=',drone_one_posxy-drone_two_posxy) 
                    print('finaldes1[0]=',finaldes1[0])   
                    print('finaldes1[1]=',finaldes1[1]) 
                    print('finaldes1[0],finaldes1[1]-drone_two_posxy=',np.array([finaldes1[0]-drone_two_posxy[0],finaldes1[1]-drone_two_posxy[1]]))

                    if np.dot((drone_one_posxy-drone_two_posxy),(np.array([finaldes1[0],finaldes1[1]])-drone_two_posxy))<0:
                        if np.dot(u_1_prev,u_2_prev)<=0:
                            mode_one=1

                    if np.dot((drone_one_posxy-drone_three_posxy),(np.array([finaldes1[0],finaldes1[1]])-drone_three_posxy))<0:
                        if np.dot(u_1_prev,u_3_prev)<=0:
                            mode_one=1

                    if np.dot((drone_one_posxy-drone_four_posxy),(np.array([finaldes1[0],finaldes1[1]])-drone_four_posxy))<0:
                        if np.dot(u_1_prev,u_4_prev)<=0:  
                            mode_one=1     

                    if mode_one==1:
                        p_center1xy=np.array([p_center1[0],p_center1[1]])
                        dist2R=mission_radius**2-np.linalg.norm(drone_one_posxy-p_center1xy)**2
                        #temp=k*[gamma*dist2R(i) -1;1 gamma*dist2R(i)]*(p(i,:)-p_center(i,:))'
                            
                        temp=ctrl_gain_k*np.mat([[ctrl_gain_gamma*dist2R, -1],[1, ctrl_gain_gamma*dist2R]])*np.transpose(np.mat(drone_one_posxy-p_center1xy))
                        print('temp_cir=',temp)
                        #u_1=np.transpose(temp)
                        u_1= [temp[0,0],temp[1,0], 0]
                        #u_1[0],u_1[1]=temp[0,0],temp[1,0]
                        print('u_1_cir=',u_1)                        
                    
                    #u_1=(max_vel_lambda[n-1]+max_vel_lambda[0])*min(rc,real_d[0])-(max_vel_lambda[0]+max_vel_lambda[1])*min(rc,real_d[n-1]) 

                    print('mode_one',mode_one)

                    if np.linalg.norm(u_1)>u_max:
                        u_1[0]=u_1[0]/np.linalg.norm(u_1)*u_max
                        u_1[1]=u_1[1]/np.linalg.norm(u_1)*u_max
                        

                    #u_1=max_vel_lambda[0]*cal_sat(max(0,u_1))
                    if printcount<maxprintcount:
                        print(printcount+1)
                        print("1Control input: ", u_1)
                        #print("1Incremental angle in radian: " , u_1*(sample_time*time_amend))
                        print("1Current position: ", drone_one_posxy)                     
                    # CAUTION: remember to reset the index value when the number of agnets cahnges
                    #u_1=1
                    #q_1_des = v_1*(sample_time*time_amend) + 0.5*u_1*(sample_time*time_amend)**2 + q_1
                    # if q_1>=q_1_prev:
                    #     q_1_des = u_1*(sample_time*time_amend*2) + q_1
                    # else:
                    #     q_1_des =  u_1*(sample_time*time_amend*2) + q_1_des_prev
                    #     q_1_des = min(q_1+15/180*np.pi, q_1_des)

                    drone_onedes=np.array([drone_one.pos_x.des,drone_one.pos_y.des])
                    drone_onedes[0]=drone_onedes[0]+u_1[0]*(sample_time*time_amend)
                    drone_onedes[1]=drone_onedes[1]+u_1[1]*(sample_time*time_amend)
                    
                    #print(drone_onedes[0])

                    if printcount<maxprintcount:
                        print("1Target position: " , drone_onedes)                        

                    # drone_one.pos_x.des = math.cos(q_1_des)*mission_radius
                    # drone_one.pos_y.des = math.sin(q_1_des)*mission_radius

                    # q_1_prev = q_1
                    # q_2_prev = q_2
                    # q_3_prev = q_3

                    u_1_prev = u_1
                    #u_2_prev = u_2

                    #p_center(i,:)=p_center(i,:)+epsilon*a(i)*(p_center(j,:)-p_center(i,:))
                    p_center1=p_center1+epsilon*ctrl_gain_a[0]*(p_center2-p_center1+p_center3-p_center1+p_center4-p_center1)

                    print(u_1)


                    #q_1_des_prev = q_1_des

                    #print("using convergence ctrl")


                    # for save the data, modify later
                    u_1_0 = u_1[0]
                    u_1_1 = u_1[1]
                    drone_one_posxy_x = drone_one_posxy[0]
                    drone_one_posxy_y = drone_one_posxy[1]
                    drone_onedes_x = drone_onedes[0]
                    drone_onedes_y = drone_onedes[1]
                    p_center1_x = p_center1[0]
                    p_center1_y = p_center1[1]

                    time_in_traj = time.time() - drone_one.start_time

                    # saver_data_drone_1.add_elements(*[eval(val_name) for val_name in data_to_save_1])


                # saver for the robot 1 
                    # saver_agents.add_elements(
                    #     dp_drone_one.X, dp_drone_one.Y, dp_drone_one.Z,
                    #     dp_drone_one.QW, dp_drone_one.QX, dp_drone_one.QY, dp_drone_one.QZ,
                    #     dp_drone_two.X, dp_drone_two.Y, dp_drone_two.Z,
                    #     dp_drone_two.QW, dp_drone_two.QX, dp_drone_two.QY, dp_drone_two.QZ,
                    #     #dp_drone_three.X, dp_drone_three.Y, dp_drone_three.Z,
                    #     #dp_drone_three.QW, dp_drone_three.QX, dp_drone_three.QY, dp_drone_three.QZ,
                    #     abs_time
                    # )  

                    saver_data_drone_1.add_elements(
                        u_1_0, u_1_1, drone_one_posxy_x, drone_one_posxy_y,
                        drone_onedes_x, drone_onedes_y, p_center1_x, p_center1_y,
                        mode_one,
                        drone_one.pos_x.fdk, drone_one.pos_y.fdk, drone_one.pos_z.fdk,
                        time_in_traj,
                    )

                    drone_one.pos_x.des=drone_onedes[0]
                    drone_one.pos_y.des=drone_onedes[1]
                    drone_one.pos_z.des=0.7
                    '''
                    if printcount<maxprintcount:
                        print("1Target position2_onedes: " , drone_onedes)  
                    if printcount<maxprintcount:
                        print("1Target position_pos_x.des: " , np.array([drone_one.pos_x.des,drone_one.pos_y.des,drone_one.pos_z.des]))  
'''
                # update the reference and get the controller output for the drone_one
                position_controller_drone_one.update_reference(drone_one.pos_x.des, drone_one.pos_y.des, drone_one.pos_z.des)
                # get the control output
                u_x_drone_one, u_y_drone_one, u_z_drone_one = position_controller_drone_one.update_error(drone_one.pos_x.fdk, drone_one.pos_y.fdk, drone_one.pos_z.fdk)
                # get the desired roll pitch yaw and thrust for drone one
                get_rpyt_cmd(u_x_drone_one, u_y_drone_one, u_z_drone_one, drone_one)

               

                # debug
                # time_previous = time.time()
                # print(drone_one.yaw.fdk, drone_one.pos_x.fdk, drone_one.pos_y.fdk, drone_one.pos_z.fdk)
                # print(drone_one.yaw.des, drone_one.pos_x.des, drone_one.pos_y.des, drone_one.pos_z.des)
                # print(time_diff)
                # print("+=========+")
                
                # send command to the crazyflie
                if drone_one.terminate_flag:
                    # terminate the flight
                    terminate_sequence(scf)
                    break
                else:
                    if not drone_one.take_off_flag:
                        standby_sequence(scf)

                    if drone_one.take_off_flag and not drone_one.land_flag:
                        scf.cf.commander.send_setpoint(drone_one.roll.ctrl_cmd, drone_one.pitch.ctrl_cmd, drone_one.yaw.ctrl_cmd, drone_one.thrus_cmd.ctrl_cmd)
                        print('Drone ONE:', 'desired Z is', drone_one.pos_z.des, 'actual Z is', drone_one.pos_z.fdk)

                    if drone_one.land_flag:
                        land_sequence(drone_one, scf)


    # ------- DRONE TWO CONTROL BRANCH -------
    elif control_which_flag[0] == 2.0:
        
        # dp data processor
        dp_drone_one = GeneralFcn.RealTimeProcessor() # for controlling the drone one
        dp_drone_two = GeneralFcn.RealTimeProcessor() # for recording the drone two
        dp_drone_three = GeneralFcn.RealTimeProcessor() # for recording the drone three
        dp_drone_four = GeneralFcn.RealTimeProcessor() # for recording the drone four

        # set the position controller
        position_controller_drone_two = PID_ControllerThreeAixs(sample_time,
                                                  12, 2.5, 25, 0,
                                                  12, 2.5, 25, 0,
                                                  16000, 2200, 7000, 45000, )

        # the logging data list, for the drone_two
        lg_stab_drone_two = LogConfig(name='Stabilizer', period_in_ms=sample_rate_in_ms)
        lg_stab_drone_two.add_variable('stabilizer.thrust', 'float')
        
        with SyncLogger(scf, lg_stab_drone_two) as logger_drone_two:
            # while not flight_terminate_flag:
            for log_entry in logger_drone_two:
                
                time.sleep(sample_time)
                # read from the MoCap
                data_all_agents = receiver.get_data()

                dp_drone_one.step(data_all_agents[(rigid_body_ID_drone_one-1)*14 : rigid_body_ID_drone_one*14])
                dp_drone_two.step(data_all_agents[(rigid_body_ID_drone_two-1)*14 : rigid_body_ID_drone_two*14])
                dp_drone_three.step(data_all_agents[(rigid_body_ID_drone_three-1)*14 : rigid_body_ID_drone_three*14])
                dp_drone_four.step(data_all_agents[(rigid_body_ID_drone_four-1)*14 : rigid_body_ID_drone_four*14])
                
                # get the feedback from mocap
                drone_two.yaw.fdk = math.atan2(2 * (dp_drone_two.QW * dp_drone_two.QZ + dp_drone_two.QX * dp_drone_two.QY), 1 - 2 * (dp_drone_two.QZ * dp_drone_two.QZ + dp_drone_two.QY * dp_drone_two.QY))
                drone_two.pos_x.fdk = Filter_x_2.filter(dp_drone_two.X)
                drone_two.pos_y.fdk = Filter_y_2.filter(dp_drone_two.Y)
                drone_two.pos_z.fdk = Filter_z.filter(dp_drone_two.Z) 

                drone_one.pos_x.fdk = Filter_x_1.filter(dp_drone_one.X)
                drone_one.pos_y.fdk = Filter_y_1.filter(dp_drone_one.Y)
                drone_one.pos_z.fdk = Filter_z.filter(dp_drone_one.Z)

                drone_three.pos_x.fdk = Filter_x_3.filter(dp_drone_three.X)
                drone_three.pos_y.fdk = Filter_y_3.filter(dp_drone_three.Y)
                drone_three.pos_z.fdk = Filter_z.filter(dp_drone_three.Z) 

                drone_four.pos_x.fdk = Filter_x_4.filter(dp_drone_four.X)
                drone_four.pos_y.fdk = Filter_y_4.filter(dp_drone_four.Y)
                drone_four.pos_z.fdk = Filter_z.filter(dp_drone_four.Z)               
                
                drone_one_posxy=np.array([drone_one.pos_x.fdk,drone_one.pos_y.fdk])
                drone_two_posxy=np.array([drone_two.pos_x.fdk,drone_two.pos_y.fdk])
                drone_three_posxy=np.array([drone_three.pos_x.fdk,drone_three.pos_y.fdk])
                drone_four_posxy=np.array([drone_four.pos_x.fdk,drone_four.pos_y.fdk])            
                # read from the joystick  
                joystick.step()
                joystick_drone_two(drone_two, axis_act_thre, axis_deact_thre) # commands from joystick
                joystick_traj_ctrl(drone_two) # differnet stages for the trajectory, control by the joystick
                
                if drone_two.traj_start_flag and not drone_two.traj_end_flag:
                    #p2l=abs((finaldes2[1]-p_center2[1])*drone_two.pos_x.fdk-(finaldes2[0]-p_center2[0])*drone_two.pos_y.fdk+(finaldes2[0]-p_center2[0])*p_center2[1]-(finaldes2[1]-p_center2[1])*p_center2[0])/math.sqrt((finaldes2[1]-p_center2[1])**2+(finaldes2[0]-p_center2[0])**2)
                    #temp=[(p_dest(i,1)-p_center(i,1)) -(p_dest(i,2)-p_center(i,2));(p_dest(i,2)-p_center(i,2)) (p_dest(i,1)-p_center(i,1))]/norm(p_dest(i,:)-p_center(i,:))*[kx*norm(p_dest(i,:)-p(i,:));ky*p2l^m];
                    #temp=np.mat([[(finaldes2[0]-p_center2[0]), -(finaldes2[1]-p_center2[1])],[(finaldes2[1]-p_center2[1]) ,(finaldes2[0]-p_center2[0])]])/np.linalg.norm(finaldes2-p_center2)*np.mat([[ctrl_gain_kx*np.linalg.norm(finaldes2-p_center2)],[ctrl_gain_ky*p2l**ctrl_gain_m]])
                    #u_2=np.transpose(temp)
                    # u_2[0],u_2[1]=temp[0,0],temp[1,0]
                    temp=ctrl_gain_kp*np.mat([[finaldes2[0]-drone_two_posxy[0]],[finaldes2[1]-drone_two_posxy[1]]])
                    mode_two=0
                    u_2 = [temp[0,0],temp[1,0], 0]



                    #if np.dot((drone_one_posxy-drone_two_posxy),(np.array(finaldes2[0],finaldes2[1])-drone_two_posxy))>0:
                    
                    if np.dot((drone_two_posxy-drone_one_posxy),(np.array([finaldes2[0],finaldes2[1]])-drone_one_posxy))<0:
                        if np.dot(u_1_prev,u_2_prev)<=0:
                            mode_two=1

                    if np.dot((drone_two_posxy-drone_three_posxy),(np.array([finaldes2[0],finaldes2[1]])-drone_three_posxy))<0:
                        if np.dot(u_3_prev,u_2_prev)<=0:
                            mode_two=1

                    if np.dot((drone_two_posxy-drone_four_posxy),(np.array([finaldes2[0],finaldes2[1]])-drone_four_posxy))<0:
                        if np.dot(u_4_prev,u_2_prev)<=0:  
                            mode_two=1

                    if mode_two==1:
                        p_center2xy=np.array([p_center2[0],p_center2[1]])
                        dist2R=mission_radius**2-np.linalg.norm(drone_two_posxy-p_center2xy)**2
                        temp=ctrl_gain_k*np.mat([[ctrl_gain_gamma*dist2R, -1],[1, ctrl_gain_gamma*dist2R]])*np.transpose(np.mat(drone_two_posxy-p_center2xy))
                        u_2 = [temp[0,0],temp[1,0], 0]                        
                    
                    
                    print('mode_two',mode_two)

                    if np.linalg.norm(u_2)>u_max:
                        u_2[0]=u_2[0]/np.linalg.norm(u_2)*u_max
                        u_2[1]=u_2[1]/np.linalg.norm(u_2)*u_max
                        

                    if printcount<maxprintcount:
                        print(printcount+1)
                        print("2Control input: ", u_2)
                        #print("1Incremental angle in radian: " , u_1*(sample_time*time_amend))
                        print("2Current position: ", drone_two_posxy)                    
                    #q_2_des =  v_2*(sample_time*time_amend) + 0.5*u_2*(sample_time*time_amend)**2 + q_2
                    # if q_2>=q_2_prev:
                    #     q_2_des = u_2*(sample_time*time_amend*2) + q_2
                    # else:
                    #     q_2_des =  u_2*(sample_time*time_amend*2) + q_2_des_prev
                    #     q_2_des = min(q_2+6/180*np.pi, q_2_des)
                    drone_twodes=np.array([drone_two.pos_x.des,drone_two.pos_y.des])
                    drone_twodes[0]=drone_twodes[0]+u_2[0]*(sample_time*time_amend)
                    drone_twodes[1]=drone_twodes[1]+u_2[1]*(sample_time*time_amend)

                    if printcount<maxprintcount:
                        print("2Target position: " , drone_twodes)                        

                    # drone_one.pos_x.des = math.cos(q_1_des)*mission_radius
                    # drone_one.pos_y.des = math.sin(q_1_des)*mission_radius

                    # q_1_prev = q_1
                    # q_2_prev = q_2
                    # q_3_prev = q_3

                    #u_1_prev = u_1
                    u_2_prev = u_2



                    #p_center(i,:)=p_center(i,:)+epsilon*a(i)*(p_center(j,:)-p_center(i,:))
                    p_center2=p_center2+epsilon*ctrl_gain_a[1]*(p_center1-p_center2+p_center3-p_center2+p_center4-p_center2)

                    # for save the data, modify later
                    u_2_0 = u_2[0]
                    u_2_1 = u_2[1]
                    drone_two_posxy_x = drone_two_posxy[0]
                    drone_two_posxy_y = drone_two_posxy[1]
                    drone_twodes_x = drone_twodes[0]
                    drone_twodes_y = drone_twodes[1]
                    p_center2_x = p_center2[0]
                    p_center2_y = p_center2[1]

                    # saver_data_drone_2.add_elements(*[eval(val_name) for val_name in data_to_save_2])
                    saver_data_drone_2.add_elements(
                        u_2_0, u_2_1, drone_two_posxy_x, drone_two_posxy_y,
                        drone_twodes_x, drone_twodes_y, p_center2_x, p_center2_y,
                        mode_two,
                        drone_two.pos_x.fdk, drone_two.pos_y.fdk, drone_two.pos_z.fdk,
                        time_in_traj,
                    )

                    drone_two.pos_x.des=drone_twodes[0]
                    drone_two.pos_y.des=drone_twodes[1]
                    drone_two.pos_z.des=0.7

                # update the reference and get the controller output for the drone_two
                position_controller_drone_two.update_reference(drone_two.pos_x.des, drone_two.pos_y.des, drone_two.pos_z.des)
                # get the control output
                u_x_drone_two, u_y_drone_two, u_z_drone_two = position_controller_drone_two.update_error(drone_two.pos_x.fdk, drone_two.pos_y.fdk, drone_two.pos_z.fdk)
                # get the desired roll pitch yaw and thrust for drone one
                get_rpyt_cmd(u_x_drone_two, u_y_drone_two, u_z_drone_two, drone_two)


                
                # send command to the crazyflie
                if drone_two.terminate_flag:
                    # terminate the flight
                    terminate_sequence(scf)
                    break
                else:
                    if not drone_two.take_off_flag:
                        standby_sequence(scf)

                    if drone_two.take_off_flag and not drone_two.land_flag:
                        scf.cf.commander.send_setpoint(drone_two.roll.ctrl_cmd, drone_two.pitch.ctrl_cmd, drone_two.yaw.ctrl_cmd, drone_two.thrus_cmd.ctrl_cmd)
                        #print('Drone ONE:', 'desired Z is', drone_one.pos_z.des, 'actual Z is', drone_one.pos_z.fdk)
                        print('Drone TWO:', 'desired Z is', drone_two.pos_z.des, 'actual Z is', drone_two.pos_z.fdk)
                    if drone_two.land_flag:
                        land_sequence(drone_two, scf)

    # ------- DRONE THREE CONTROL BRANCH -------
    elif control_which_flag[0] == 3.0:
        
        # dp data processor
        dp_drone_one = GeneralFcn.RealTimeProcessor() # for controlling the drone one
        dp_drone_two = GeneralFcn.RealTimeProcessor() # for recording the drone two
        dp_drone_three = GeneralFcn.RealTimeProcessor() # for recording the drone three
        dp_drone_four = GeneralFcn.RealTimeProcessor() # for recording the drone four

        # set the position controller
        position_controller_drone_three = PID_ControllerThreeAixs(sample_time,
                                                  12, 2.5, 25, 0,
                                                  12, 2.5, 25, 0,
                                                  16000, 2200, 7000, 45000, )

        # the logging data list, for the drone_two
        lg_stab_drone_three = LogConfig(name='Stabilizer', period_in_ms=sample_rate_in_ms)
        lg_stab_drone_three.add_variable('stabilizer.thrust', 'float')
        
        with SyncLogger(scf, lg_stab_drone_three) as logger_drone_three:
            # while not flight_terminate_flag:
            for log_entry in logger_drone_three:
                
                time.sleep(sample_time)
                # read from the MoCap
                data_all_agents = receiver.get_data()

                dp_drone_one.step(data_all_agents[(rigid_body_ID_drone_one-1)*14 : rigid_body_ID_drone_one*14])
                dp_drone_two.step(data_all_agents[(rigid_body_ID_drone_two-1)*14 : rigid_body_ID_drone_two*14])
                dp_drone_three.step(data_all_agents[(rigid_body_ID_drone_three-1)*14 : rigid_body_ID_drone_three*14])
                dp_drone_four.step(data_all_agents[(rigid_body_ID_drone_four-1)*14 : rigid_body_ID_drone_four*14])
                
                # get the feedback from mocap
                drone_three.yaw.fdk = math.atan2(2 * (dp_drone_three.QW * dp_drone_three.QZ + dp_drone_three.QX * dp_drone_three.QY), 1 - 2 * (dp_drone_three.QZ * dp_drone_three.QZ + dp_drone_three.QY * dp_drone_three.QY))
                drone_three.pos_x.fdk = Filter_x_3.filter(dp_drone_three.X)
                drone_three.pos_y.fdk = Filter_y_3.filter(dp_drone_three.Y)
                drone_three.pos_z.fdk = Filter_z.filter(dp_drone_three.Z) 

                drone_one.pos_x.fdk = Filter_x_1.filter(dp_drone_one.X)
                drone_one.pos_y.fdk = Filter_y_1.filter(dp_drone_one.Y)
                drone_one.pos_z.fdk = Filter_z.filter(dp_drone_one.Z)                

                drone_two.pos_x.fdk = Filter_x_2.filter(dp_drone_two.X)
                drone_two.pos_y.fdk = Filter_y_2.filter(dp_drone_two.Y)
                drone_two.pos_z.fdk = Filter_z.filter(dp_drone_two.Z) 

                drone_four.pos_x.fdk = Filter_x_4.filter(dp_drone_four.X)
                drone_four.pos_y.fdk = Filter_y_4.filter(dp_drone_four.Y)
                drone_four.pos_z.fdk = Filter_z.filter(dp_drone_four.Z)               
                
                drone_one_posxy=np.array([drone_one.pos_x.fdk,drone_one.pos_y.fdk])
                drone_two_posxy=np.array([drone_two.pos_x.fdk,drone_two.pos_y.fdk])
                drone_three_posxy=np.array([drone_three.pos_x.fdk,drone_three.pos_y.fdk])
                drone_four_posxy=np.array([drone_four.pos_x.fdk,drone_four.pos_y.fdk])             
                # read from the joystick  
                joystick.step()
                joystick_drone_three(drone_three, axis_act_thre, axis_deact_thre) # commands from joystick
                joystick_traj_ctrl(drone_three) # differnet stages for the trajectory, control by the joystick
                
                if drone_three.traj_start_flag and not drone_three.traj_end_flag:
                    temp=ctrl_gain_kp*np.array([[finaldes3[0]-drone_three_posxy[0]],[finaldes3[1]-drone_three_posxy[1]]])
                    mode_two=0
                    u_3 = [temp[0,0],temp[1,0], 0]



                    #if np.dot((drone_one_posxy-drone_two_posxy),(np.array(finaldes2[0],finaldes2[1])-drone_two_posxy))>0:
                    
                    if np.dot((drone_three_posxy-drone_one_posxy),(np.array([finaldes3[0],finaldes3[1]])-drone_one_posxy))<0:
                        if np.dot(u_1_prev,u_3_prev)<=0:
                            mode_three=1

                    if np.dot((drone_three_posxy-drone_two_posxy),(np.array([finaldes3[0],finaldes3[1]])-drone_two_posxy))<0:
                        if np.dot(u_2_prev,u_3_prev)<=0:
                            mode_three=1

                    if np.dot((drone_three_posxy-drone_four_posxy),(np.array([finaldes3[0],finaldes3[1]])-drone_four_posxy))<0:
                        if np.dot(u_4_prev,u_3_prev)<=0:      
                            mode_three=1   

                    if mode_three==1:
                        p_center3xy=np.array([p_center3[0],p_center3[1]])
                        dist2R=mission_radius**2-np.linalg.norm(drone_three_posxy-p_center3xy)**2
                        temp=ctrl_gain_k*np.mat([[ctrl_gain_gamma*dist2R, -1],[1, ctrl_gain_gamma*dist2R]])*np.transpose(np.mat(drone_three_posxy-p_center3xy))
                        mode_three=1
                        u_3 = [temp[0,0],temp[1,0], 0]                        
                                                                   
                    
                    print('mode_three',mode_three)

                    if np.linalg.norm(u_3)>u_max:
                        u_3[0]=u_3[0]/np.linalg.norm(u_3)*u_max
                        u_3[1]=u_3[1]/np.linalg.norm(u_3)*u_max
                        

                    if printcount<maxprintcount:
                        print(printcount+1)
                        print("3Control input: ", u_3)
                        #print("1Incremental angle in radian: " , u_1*(sample_time*time_amend))
                        print("3Current position: ", drone_three_posxy)                    
                    #q_2_des =  v_2*(sample_time*time_amend) + 0.5*u_2*(sample_time*time_amend)**2 + q_2
                    # if q_2>=q_2_prev:
                    #     q_2_des = u_2*(sample_time*time_amend*2) + q_2
                    # else:
                    #     q_2_des =  u_2*(sample_time*time_amend*2) + q_2_des_prev
                    #     q_2_des = min(q_2+6/180*np.pi, q_2_des)
                    drone_threedes=np.array([drone_three.pos_x.des,drone_three.pos_y.des])
                    drone_threedes[0]=drone_threedes[0]+u_3[0]*(sample_time*time_amend)
                    drone_threedes[1]=drone_threedes[1]+u_3[1]*(sample_time*time_amend)

                    if printcount<maxprintcount:
                        print("3Target position: " , drone_threedes)                        

                    # drone_one.pos_x.des = math.cos(q_1_des)*mission_radius
                    # drone_one.pos_y.des = math.sin(q_1_des)*mission_radius

                    # q_1_prev = q_1
                    # q_2_prev = q_2
                    # q_3_prev = q_3

                    #u_1_prev = u_1
                    u_3_prev = u_3



                    #p_center(i,:)=p_center(i,:)+epsilon*a(i)*(p_center(j,:)-p_center(i,:))
                    p_center3=p_center3+epsilon*ctrl_gain_a[2]*(p_center1-p_center3+p_center2-p_center3+p_center4-p_center3)

                    # for save the data, modify later
                    u_3_0 = u_3[0]
                    u_3_1 = u_3[1]
                    drone_three_posxy_x = drone_three_posxy[0]
                    drone_three_posxy_y = drone_three_posxy[1]
                    drone_threedes_x = drone_threedes[0]
                    drone_threedes_y = drone_threedes[1]
                    p_center3_x = p_center3[0]
                    p_center3_y = p_center3[1]

                    # saver_data_drone_2.add_elements(*[eval(val_name) for val_name in data_to_save_2])
                    saver_data_drone_3.add_elements(
                        u_3_0, u_3_1, drone_three_posxy_x, drone_three_posxy_y,
                        drone_threedes_x, drone_threedes_y, p_center3_x, p_center3_y,
                        mode_three,
                        drone_three.pos_x.fdk, drone_three.pos_y.fdk, drone_three.pos_z.fdk,
                        time_in_traj,                        
                    )

                    drone_three.pos_x.des=drone_threedes[0]
                    drone_three.pos_y.des=drone_threedes[1]
                    drone_three.pos_z.des=0.7

                # update the reference and get the controller output for the drone_two
                position_controller_drone_three.update_reference(drone_three.pos_x.des, drone_three.pos_y.des, drone_three.pos_z.des)
                # get the control output
                u_x_drone_three, u_y_drone_three, u_z_drone_three = position_controller_drone_three.update_error(drone_three.pos_x.fdk, drone_three.pos_y.fdk, drone_three.pos_z.fdk)
                # get the desired roll pitch yaw and thrust for drone one
                get_rpyt_cmd(u_x_drone_three, u_y_drone_three, u_z_drone_three, drone_three)


                
                # send command to the crazyflie
                if drone_three.terminate_flag:
                    # terminate the flight
                    terminate_sequence(scf)
                    break
                else:
                    if not drone_three.take_off_flag:
                        standby_sequence(scf)

                    if drone_three.take_off_flag and not drone_three.land_flag:
                        scf.cf.commander.send_setpoint(drone_three.roll.ctrl_cmd, drone_three.pitch.ctrl_cmd, drone_three.yaw.ctrl_cmd, drone_three.thrus_cmd.ctrl_cmd)
                        #print('Drone ONE:', 'desired Z is', drone_one.pos_z.des, 'actual Z is', drone_one.pos_z.fdk)
                        print('Drone THREE:', 'desired Z is', drone_three.pos_z.des, 'actual Z is', drone_three.pos_z.fdk)
                    if drone_three.land_flag:
                        land_sequence(drone_three, scf)    

    # ------- DRONE FOUR CONTROL BRANCH -------
    elif control_which_flag[0] == 4.0:
        
        # dp data processor
        dp_drone_one = GeneralFcn.RealTimeProcessor() # for controlling the drone one
        dp_drone_two = GeneralFcn.RealTimeProcessor() # for recording the drone two
        dp_drone_three = GeneralFcn.RealTimeProcessor() # for recording the drone three
        dp_drone_four = GeneralFcn.RealTimeProcessor() # for recording the drone four

        # set the position controller
        position_controller_drone_four = PID_ControllerThreeAixs(sample_time,
                                                  12, 2.5, 25, 0,
                                                  12, 2.5, 25, 0,
                                                  16000, 2200, 7000, 45000, )

        # the logging data list, for the drone_two
        lg_stab_drone_four = LogConfig(name='Stabilizer', period_in_ms=sample_rate_in_ms)
        lg_stab_drone_four.add_variable('stabilizer.thrust', 'float')
        
        with SyncLogger(scf, lg_stab_drone_four) as logger_drone_four:
            # while not flight_terminate_flag:
            for log_entry in logger_drone_four:
                
                time.sleep(sample_time)
                # read from the MoCap
                data_all_agents = receiver.get_data()

                dp_drone_one.step(data_all_agents[(rigid_body_ID_drone_one-1)*14 : rigid_body_ID_drone_one*14])
                dp_drone_two.step(data_all_agents[(rigid_body_ID_drone_two-1)*14 : rigid_body_ID_drone_two*14])
                dp_drone_three.step(data_all_agents[(rigid_body_ID_drone_three-1)*14 : rigid_body_ID_drone_three*14])
                dp_drone_four.step(data_all_agents[(rigid_body_ID_drone_four-1)*14 : rigid_body_ID_drone_four*14])
                
                # get the feedback from mocap
                drone_four.yaw.fdk = math.atan2(2 * (dp_drone_four.QW * dp_drone_four.QZ + dp_drone_four.QX * dp_drone_four.QY), 1 - 2 * (dp_drone_four.QZ * dp_drone_four.QZ + dp_drone_four.QY * dp_drone_four.QY))
                drone_four.pos_x.fdk = Filter_x_4.filter(dp_drone_four.X)
                drone_four.pos_y.fdk = Filter_y_4.filter(dp_drone_four.Y)
                drone_four.pos_z.fdk = Filter_z.filter(dp_drone_four.Z) 

                drone_one.pos_x.fdk = Filter_x_1.filter(dp_drone_one.X)
                drone_one.pos_y.fdk = Filter_y_1.filter(dp_drone_one.Y)
                drone_one.pos_z.fdk = Filter_z.filter(dp_drone_one.Z) 

                drone_two.pos_x.fdk = Filter_x_2.filter(dp_drone_two.X)
                drone_two.pos_y.fdk = Filter_y_2.filter(dp_drone_two.Y)
                drone_two.pos_z.fdk = Filter_z.filter(dp_drone_two.Z)

                drone_three.pos_x.fdk = Filter_x_3.filter(dp_drone_three.X)
                drone_three.pos_y.fdk = Filter_y_3.filter(dp_drone_three.Y)
                drone_three.pos_z.fdk = Filter_z.filter(dp_drone_three.Z) 

             
                
                drone_one_posxy=np.array([drone_one.pos_x.fdk,drone_one.pos_y.fdk])
                drone_two_posxy=np.array([drone_two.pos_x.fdk,drone_two.pos_y.fdk])
                drone_three_posxy=np.array([drone_three.pos_x.fdk,drone_three.pos_y.fdk])
                drone_four_posxy=np.array([drone_four.pos_x.fdk,drone_four.pos_y.fdk])            
                # read from the joystick  
                joystick.step()
                joystick_drone_four(drone_four, axis_act_thre, axis_deact_thre) # commands from joystick
                joystick_traj_ctrl(drone_four) # differnet stages for the trajectory, control by the joystick
                
                if drone_four.traj_start_flag and not drone_four.traj_end_flag:
                    #p2l=abs((finaldes2[1]-p_center2[1])*drone_two.pos_x.fdk-(finaldes2[0]-p_center2[0])*drone_two.pos_y.fdk+(finaldes2[0]-p_center2[0])*p_center2[1]-(finaldes2[1]-p_center2[1])*p_center2[0])/math.sqrt((finaldes2[1]-p_center2[1])**2+(finaldes2[0]-p_center2[0])**2)
                    #temp=[(p_dest(i,1)-p_center(i,1)) -(p_dest(i,2)-p_center(i,2));(p_dest(i,2)-p_center(i,2)) (p_dest(i,1)-p_center(i,1))]/norm(p_dest(i,:)-p_center(i,:))*[kx*norm(p_dest(i,:)-p(i,:));ky*p2l^m];
                    #temp=np.mat([[(finaldes2[0]-p_center2[0]), -(finaldes2[1]-p_center2[1])],[(finaldes2[1]-p_center2[1]) ,(finaldes2[0]-p_center2[0])]])/np.linalg.norm(finaldes2-p_center2)*np.mat([[ctrl_gain_kx*np.linalg.norm(finaldes2-p_center2)],[ctrl_gain_ky*p2l**ctrl_gain_m]])
                    #u_2=np.transpose(temp)
                    # u_2[0],u_2[1]=temp[0,0],temp[1,0]
                    temp=ctrl_gain_kp*np.array([[finaldes4[0]-drone_four_posxy[0]],[finaldes4[1]-drone_four_posxy[1]]])
                    mode_four=0
                    u_4 = [temp[0,0],temp[1,0], 0]



                    #if np.dot((drone_one_posxy-drone_two_posxy),(np.array(finaldes2[0],finaldes2[1])-drone_two_posxy))>0:
                    
                    if np.dot((drone_four_posxy-drone_one_posxy),(np.array([finaldes4[0],finaldes4[1]])-drone_one_posxy))<0:
                        if np.dot(u_1_prev,u_4_prev)<=0:
                            mode_four=1
                    

                    if np.dot((drone_four_posxy-drone_two_posxy),(np.array([finaldes4[0],finaldes4[1]])-drone_two_posxy))<0:
                        if np.dot(u_2_prev,u_4_prev)<=0:
                            mode_four=1

                    if np.dot((drone_four_posxy-drone_three_posxy),(np.array([finaldes4[0],finaldes4[1]])-drone_three_posxy))<0:
                        if np.dot(u_3_prev,u_4_prev)<=0:
                            mode_four=1

                    if mode_four==1:
                        p_center4xy=np.array([p_center4[0],p_center4[1]])
                        dist2R=mission_radius**2-np.linalg.norm(drone_four_posxy-p_center4xy)**2
                        temp=ctrl_gain_k*np.mat([[ctrl_gain_gamma*dist2R, -1],[1, ctrl_gain_gamma*dist2R]])*np.transpose(np.mat(drone_four_posxy-p_center4xy))
                        u_4 = [temp[0,0],temp[1,0], 0]                        


                    print('mode_four',mode_four)

                    if np.linalg.norm(u_4)>u_max:
                        u_4[0]=u_4[0]/np.linalg.norm(u_4)*u_max
                        u_4[1]=u_4[1]/np.linalg.norm(u_4)*u_max
                        

                    if printcount<maxprintcount:
                        print(printcount+1)
                        print("4Control input: ", u_4)
                        #print("1Incremental angle in radian: " , u_1*(sample_time*time_amend))
                        print("4Current position: ", drone_four_posxy)                    
                    #q_2_des =  v_2*(sample_time*time_amend) + 0.5*u_2*(sample_time*time_amend)**2 + q_2
                    # if q_2>=q_2_prev:
                    #     q_2_des = u_2*(sample_time*time_amend*2) + q_2
                    # else:
                    #     q_2_des =  u_2*(sample_time*time_amend*2) + q_2_des_prev
                    #     q_2_des = min(q_2+6/180*np.pi, q_2_des)
                    drone_fourdes=np.array([drone_four.pos_x.des,drone_four.pos_y.des])
                    drone_fourdes[0]=drone_fourdes[0]+u_4[0]*(sample_time*time_amend)
                    drone_fourdes[1]=drone_fourdes[1]+u_4[1]*(sample_time*time_amend)

                    if printcount<maxprintcount:
                        print("4Target position: " , drone_fourdes)                        

                    # drone_one.pos_x.des = math.cos(q_1_des)*mission_radius
                    # drone_one.pos_y.des = math.sin(q_1_des)*mission_radius

                    # q_1_prev = q_1
                    # q_2_prev = q_2
                    # q_3_prev = q_3

                    #u_1_prev = u_1
                    u_4_prev = u_4



                    #p_center(i,:)=p_center(i,:)+epsilon*a(i)*(p_center(j,:)-p_center(i,:))
                    p_center4=p_center4+epsilon*ctrl_gain_a[3]*(p_center1-p_center4+p_center2-p_center4+p_center3-p_center4)

                    # for save the data, modify later
                    u_4_0 = u_4[0]
                    u_4_1 = u_4[1]
                    drone_four_posxy_x = drone_four_posxy[0]
                    drone_four_posxy_y = drone_four_posxy[1]
                    drone_fourdes_x = drone_fourdes[0]
                    drone_fourdes_y = drone_fourdes[1]
                    p_center4_x = p_center4[0]
                    p_center4_y = p_center4[1]

                    # saver_data_drone_2.add_elements(*[eval(val_name) for val_name in data_to_save_2])
                    saver_data_drone_4.add_elements(
                        u_4_0, u_4_1, drone_four_posxy_x, drone_four_posxy_y,
                        drone_fourdes_x, drone_fourdes_y, p_center4_x, p_center4_y,
                        mode_four,
                        drone_four.pos_x.fdk, drone_four.pos_y.fdk, drone_four.pos_z.fdk,
                        time_in_traj,                        
                    )

                    drone_four.pos_x.des=drone_fourdes[0]
                    drone_four.pos_y.des=drone_fourdes[1]
                    drone_four.pos_z.des=0.7

                # update the reference and get the controller output for the drone_two
                position_controller_drone_four.update_reference(drone_four.pos_x.des, drone_four.pos_y.des, drone_four.pos_z.des)
                # get the control output
                u_x_drone_four, u_y_drone_four, u_z_drone_four = position_controller_drone_four.update_error(drone_four.pos_x.fdk, drone_four.pos_y.fdk, drone_four.pos_z.fdk)
                # get the desired roll pitch yaw and thrust for drone one
                get_rpyt_cmd(u_x_drone_four, u_y_drone_four, u_z_drone_four, drone_four)


                
                # send command to the crazyflie
                if drone_four.terminate_flag:
                    # terminate the flight
                    terminate_sequence(scf)
                    break
                else:
                    if not drone_four.take_off_flag:
                        standby_sequence(scf)

                    if drone_four.take_off_flag and not drone_four.land_flag:
                        scf.cf.commander.send_setpoint(drone_four.roll.ctrl_cmd, drone_four.pitch.ctrl_cmd, drone_four.yaw.ctrl_cmd, drone_four.thrus_cmd.ctrl_cmd)
                        print('Drone FOUR:', 'desired Z is', drone_four.pos_z.des, 'actual Z is', drone_four.pos_z.fdk)

                    if drone_four.land_flag:
                        land_sequence(drone_four, scf)                                                 

# main block
if __name__ == '__main__':

    cflib.crtp.init_drivers(enable_debug_driver=False)
    
    receiver = UdpReceiver.UdpRigidBodies()

    # set the sample here, hz
    sample_rate = 100
    sample_time = 1 / sample_rate
    sample_rate_in_ms = int(sample_time * 1000)
    receiver.start_thread()

    drone_one = Creat_Agent()
    drone_two = Creat_Agent()
    drone_three = Creat_Agent()
    drone_four = Creat_Agent()

    # mission_radius = 1


    # theta_ini = np.array([10, 135, 300, 290, 330])/180*math.pi

    # q_1_prev = theta_ini[0]
    # q_2_prev = theta_ini[1]
    # q_3_prev = theta_ini[2]

    # v_1_prev = 0
    # v_2_prev = 0
    # v_3_prev = 0

    # set the initial position of the agents, mainly including the three robots
    # drone_one.pos_x.des = 0.7
    # drone_one.pos_y.des = -0.5
    # drone_one.pos_z.des = 0.5

    #drone_one.pos_x.des, drone_one.pos_y.des, drone_one.pos_z.des = pole_to_carterian(mission_radius, theta_ini[0])
    drone_one.pos_x.des, drone_one.pos_y.des, drone_one.pos_z.des = -0.8,0.8,0.7

    #drone_two.pos_x.des, drone_two.pos_y.des, drone_two.pos_z.des = pole_to_carterian(mission_radius, theta_ini[1])
    drone_two.pos_x.des, drone_two.pos_y.des, drone_two.pos_z.des = -0.8,-0.8,0.7
    drone_three.pos_x.des, drone_three.pos_y.des, drone_three.pos_z.des = 0.8,0.8,0.7
    drone_four.pos_x.des, drone_four.pos_y.des, drone_four.pos_z.des = 0.8,-0.8,0.7

    p_center1=0.5*(np.array([drone_one.pos_x.des,drone_one.pos_y.des,drone_one.pos_z.des])+finaldes1)
    p_center2=0.5*(np.array([drone_two.pos_x.des,drone_two.pos_y.des,drone_two.pos_z.des])+finaldes2)   
    p_center3=0.5*(np.array([drone_three.pos_x.des,drone_three.pos_y.des,drone_three.pos_z.des])+finaldes3)   
    p_center4=0.5*(np.array([drone_four.pos_x.des,drone_four.pos_y.des,drone_four.pos_z.des])+finaldes4)   

    print("p_center1: ", p_center1)
    print("p_center2: ", p_center2) 
    print("p_center3: ", p_center3)
    print("p_center4: ", p_center4)

    #drone_three.pos_x.des, drone_three.pos_y.des, drone_three.pos_z.des = pole_to_carterian(mission_radius, theta_ini[2])

    # drone_two.pos_x.des = 0.7
    # drone_two.pos_y.des = 0.5
    # drone_two.pos_z.des = 0.5

    # drone_three.pos_x.des, drone_three.pos_y.des, drone_three.pos_z.des = 0.0, 0.0, 0.8
    # drone_four.pos_x.des, drone_four.pos_y.des, drone_four.pos_z.des = 0.6, 0.6, 0.8


    uri_drone_one = 'radio://0/90/2M'  
    uri_drone_two = 'radio://0/10/2M'
    uri_drone_three = 'radio://1/80/2M'
    uri_drone_four = 'radio://1/40/2M'


    control_flag_args = {
        uri_drone_one : [[1.0, 1.0, 1.0]],
        uri_drone_two: [[2.0, 2.0, 2.0]],
        uri_drone_three: [[3.0, 3.0, 3.0]],
        uri_drone_four: [[4.0, 4.0, 4.0]]
    }


    URIS_swarm = {uri_drone_one, uri_drone_two, uri_drone_three, uri_drone_four}
    #URIS_swarm = {uri_drone_one, uri_drone_two, uri_drone_three}
    # URIS_swarm = {uri_drone_one}


    factory = CachedCfFactory(rw_cache='./cache')

    # saver_agents = savemat.DataSaver(
    #     'drone_one_X_pos', 'drone_one_Y_pos', 'drone_one_Z_pos', 
    #     'drone_one_QW', 'drone_one_QX', 'drone_one_QY', 'drone_one_QZ',
    #     'drone_two_X_pos', 'drone_two_Y_pos', 'drone_two_Z_pos', 
    #     'drone_two_QW', 'drone_two_QX', 'drone_two_QY', 'drone_two_QZ',
    #     #'drone_three_X_pos', 'drone_three_Y_pos', 'drone_three_Z_pos', 
    #     #'drone_three_QW', 'drone_three_QX', 'drone_three_QY', 'drone_three_QZ',
    #     'abs_time'
    #     )

    # data_to_save_1 = ["u_1[0]", "u_1[1]", "drone_one_posxy[0]", "drone_one_posxy[1]",
    #                   "drone_onedes[0]", "drone_onedes[1]",
    #                   "p_center1[0]", "p_center1[1]", 'mode_one', 
    #                   'drone_one.pos_x.fdk', 'drone_one.pos_y.fdk', 'drone_one.pos_z.fdk',
    #                   'drone_two.pos_x.fdk', 'drone_two.pos_y.fdk', 'drone_two.pos_z.fdk',
    #                   'time_in_traj'
    #                   ]
    # data_to_save_2 = ["u_2[0]", "u_2[1]", "drone_two_posxy[0]", "drone_two_posxy[1]",
    #                   "drone_twodes[0]", "drone_twodes[1]",
    #                   "p_center2[0]", "p_center2[1]", 'mode_two', ]

    data_to_save_1 = ['u_1_0',
                        'u_1_1',
                        'drone_one_posxy_x', 
                        'drone_one_posxy_y',
                        'drone_onedes_x', 
                        'drone_onedes_y', 
                        'p_center1_x', 
                        'p_center1_y',
                        'mode_one', 
                      'drone_one.pos_x.fdk', 'drone_one.pos_y.fdk', 'drone_one.pos_z.fdk',
                      'time_in_traj'
                      ]
    data_to_save_2 = ['u_2_0',
                    'u_2_1',
                    'drone_two_posxy_x', 
                    'drone_two_posxy_y',
                    'drone_twodes_x', 
                    'drone_twodes_y', 
                    'p_center2_x', 
                    'p_center2_y', 
                    'mode_two',
                    'drone_two.pos_x.fdk', 'drone_two.pos_y.fdk', 'drone_two.pos_z.fdk',
                    'time_in_traj'
                      ]
    
    data_to_save_3 = ['u_3_0',
                    'u_3_1',
                    'drone_three_posxy_x', 
                    'drone_three_posxy_y',
                    'drone_threedes_x', 
                    'drone_threedes_y', 
                    'p_center3_x', 
                    'p_center3_y', 
                    'mode_three',
                    'drone_three.pos_x.fdk', 'drone_three.pos_y.fdk', 'drone_three.pos_z.fdk',
                    'time_in_traj'
                      ]                     
    data_to_save_4 = ['u_4_0',
                    'u_4_1',
                    'drone_four_posxy_x', 
                    'drone_four_posxy_y',
                    'drone_fourdes_x', 
                    'drone_fourdes_y', 
                    'p_center4_x', 
                    'p_center4_y', 
                    'mode_four',
                    'drone_four.pos_x.fdk', 'drone_four.pos_y.fdk', 'drone_four.pos_z.fdk',
                    'time_in_traj'
                      ]                           
    saver_data_drone_1 = savemat.DataSaver(*string_list_format_converter(data_to_save_1))
    saver_data_drone_2 = savemat.DataSaver(*string_list_format_converter(data_to_save_2))
    saver_data_drone_3 = savemat.DataSaver(*string_list_format_converter(data_to_save_3))
    saver_data_drone_4 = savemat.DataSaver(*string_list_format_converter(data_to_save_4))

    print(*string_list_format_converter(data_to_save_1))
    print(*string_list_format_converter(data_to_save_2))
    print(*string_list_format_converter(data_to_save_3))
    print(*string_list_format_converter(data_to_save_4))    

    # starts the swarm!!!
    with Swarm(URIS_swarm, factory=factory) as swarm:
        print('There are' , len(URIS_swarm), ' crazyflies in the swarm')
        swarm.parallel_safe(crazyflie_connect_check)
        swarm.parallel_safe(crazyflie_control, args_dict = control_flag_args)
    
    receiver.stop_thread()
    # saver_agents.save2mat('DATA_CONVERGENCE_CTRL/')
    saver_data_drone_1.save2mat('convergence_control/DATA_CONVERGENCE_CTRL/')
    time.sleep(3)
    saver_data_drone_2.save2mat('convergence_control/DATA_CONVERGENCE_CTRL/')
    time.sleep(3)
    saver_data_drone_3.save2mat('convergence_control/DATA_CONVERGENCE_CTRL/')
    time.sleep(3)
    saver_data_drone_4.save2mat('convergence_control/DATA_CONVERGENCE_CTRL/')


    joystick.quit()

