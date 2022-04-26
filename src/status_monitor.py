#!/usr/bin/env python3
from numpy import append
import rospy
from std_msgs.msg import String
from driver_status.msg import RobotStatus
from actionlib_msgs.msg import GoalStatusArray
from roboteq_motor_controller_driver.msg import channel_values

def check_states(status_names, state):
    """[Finding status name from binary array]
    Args:
        status_names(list):[Name of status]
        states(list):[Binary array]
    Returns:
        out_states(list):[list of name states]
    """
    states = [int(state) for state in format(state, "08b")]
    states.reverse()
    out_states = []
    for index, value in enumerate(states):
        if value:
            out_states.append(status_names[index])
    return out_states


def fault_flags_callback(value):
    """
        [Callback function of Fault flag topic]
        Arg:
            value(int64[]):fault_flag_number(2^(n-1))
        Publishes Name of fault to topic '/robot_status' 
    """
    status_naming = ["Roboteq_Driver/Overheat", "Roboteq_Driver/Overvoltage", "Roboteq_Driver/Undervoltage", "Roboteq_Driver/Short_Circuit", "Roboteq_Driver/Emergancy_Stop", "Roboteq_Driver/Motor/Sensor setup fault", "Roboteq_Driver/MosFet failure", "Roboteq_Driver/Default configuration loaded at startup"]
    if(value.value[0] == 0):
        status=["Roboteq_driver_fault_flag/Normal"]
    else:
        status = check_states(status_naming, state=value.value[0])
    #rospy.loginfo(status)
    status_pub.publish(status)

def status_flags_callback(value):
    """
        [Callback function of Status flags topic]
        Arg:
            value(int64[]):fault_flag_number(2^(n-1))
        Publishes Name of status each wheel to topic '/robot_status' 
    """
    status_naming_right = ["Motor_right/Amps Limit currently active", "Motor_right/Motor stalled", "Motor_right/Loop Error detected", "Motor_right/Safety Stop active", "Motor_right/Forward Limit triggered", "Motor_right/Reverse Limit triggered", "Motor_right/Amps Trigger activated"]
    status_naming_left = ["Motor_left/Amps Limit currently active", "Motor_left/Motor stalled", "Motor_left/Loop Error detected", "Motor_left/Safety Stop active", "Motor_left/Forward Limit triggered", "Motor_left/Reverse Limit triggered", "Motor_left/Amps Trigger activated"]
    if(value.value[0] == 0):
        status_left = ["Left_Motor_status_flag/Normal"]
    else:
        status_left = check_states(status_naming_left, state=value.value[0])
    if(value.value[1] == 0):
        status_right = ["Right_Motor_status_flag/Normal"]
    else:
        status_right = check_states(status_naming_right, state=value.value[1])
    
    #rospy.loginfo(status_left)
    status_pub.publish(status_left)
    #rospy.loginfo(status_right)
    status_pub.publish(status_right)


def bms_status_flags_callback(data):
    """
        [Callback function of Fault flag topic]
        Arg:
            data(str):fault_flag_number(2^(n-1))
        Publishes Name of fault to topic 'robot_status' 
    """
    status_naming = ["roboteq_bms/Unsafe Temperature", "roboteq_bms/Over or Under Voltage Error Set", "roboteq_bms/Amp Trigger Set", "roboteq_bms/Over Current Error set", "roboteq_bms/Short Load or Inv Charger", "roboteq_bms/Bad State of Health", "roboteq_bms/Config Error", "roboteq_bms/Internal Fault"]
    #print(data.data)
    if(int(data.data) == 0):
        status = ["BMS_Status_flag/Normal"]
    else:
        status = check_states(status_naming, state=int(data.data))    
    #rospy.loginfo(status)
    status_pub.publish(status)

def move_base_status_callback(msg):
    """
        [Callback function of Move_Base_Status topic]
        Arg:
            msg(int): 
                uint8 PENDING=0
                uint8 ACTIVE=1
                uint8 PREEMPTED=2
                uint8 SUCCEEDED=3
                uint8 ABORTED=4
                uint8 REJECTED=5
                uint8 PREEMPTING=6
                uint8 RECALLING=7
                uint8 RECALLED=8
                uint8 LOST=9
        Publishes Name of status to topic 'robot_status' 
    """   
    status_msg = GoalStatusArray()
    status_msg = msg
    status = []
    status_naming_list = ["Move_base/PENDING", "Move_base/ACTIVE", "Move_base/PREEMPTED", "Move_base/SUCCEEDED", "Move_base/ABORTED", "Move_base/REJECTED", "Move_base/PREEMPTING", "Move_base/RECALLING", "Move_base/RECALLED", "Move_base/LOST"]
    status_index = status_msg.status_list[0].status
    status.append(status_naming_list[status_index])
    #rospy.loginfo(status)
    status_pub.publish(status)

def listener():
    """
        Initialize node and Subscribes topics
    """
    global status_pub

    rospy.init_node('Driver_status', anonymous=True)
    
    status_pub = rospy.Publisher(rospy.get_param('~robot_status'), RobotStatus, queue_size=10)   
    rospy.Subscriber(rospy.get_param('~driver_fault_flag'), channel_values, fault_flags_callback)
    rospy.Subscriber(rospy.get_param('~driver_status_flag'), channel_values, status_flags_callback)
    rospy.Subscriber(rospy.get_param('~bms_status_flag'), String, bms_status_flags_callback)
    rospy.Subscriber(rospy.get_param('~move_base_status'), GoalStatusArray, move_base_status_callback)
    rospy.spin()
   
if __name__ == '__main__':
    listener()