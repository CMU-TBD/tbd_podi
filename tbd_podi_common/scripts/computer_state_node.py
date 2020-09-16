#!/usr/bin/env python

import rospy
import os.path
import alloy.ros
from sensor_msgs.msg import (
    BatteryState
)

class PodiComputerState():

    _battery_msg    : BatteryState
    _loop_rate      : float
    _battery_dir    : str

    def __init__(self):

        # get parameters
        self._loop_rate = rospy.get_param("~battery_pub_rate_in_hz", 1) # how many seconds to publish battery state
        self._battery_dir = rospy.get_param("~battery_dir", "/sys/class/power_supply/BAT0")
        # create default battery message
        self._battery_msg = BatteryState(power_supply_technology=BatteryState.POWER_SUPPLY_TECHNOLOGY_LION, power_supply_health=BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN)
        # create battery state publisher
        self._battery_state_pub = rospy.Publisher("computer_battery_state", BatteryState, queue_size=1)
        
    def spin(self) -> None:
        rate = rospy.Rate(self._loop_rate)
        # loop until close down
        while not rospy.is_shutdown():
            # loop
            rate.sleep()
            # try to read the current state
            capacity = open(os.path.join(self._battery_dir, "capacity")).readline().strip()
            charge_full = open(os.path.join(self._battery_dir, "charge_full")).readline().strip()
            charge_now = open(os.path.join(self._battery_dir, "charge_now")).readline().strip()
            status = open(os.path.join(self._battery_dir, "status")).readline().strip()
            voltage_now = open(os.path.join(self._battery_dir, "voltage_now")).readline().strip() #represented in microvolts
            # now we add to message
            self._battery_msg.header = alloy.ros.create_ros_header(rospy)
            self._battery_msg.voltage = float(voltage_now)/1000000 # conver to just volts
            self._battery_msg.percentage = float(capacity)
            self._battery_msg.design_capacity = float(charge_full)
            self._battery_msg.capacity = float(charge_now)
            # status
            if (status == "Discharging"):
                self._battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            elif (status == "Charging"):
                self._battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
            elif (status == "Full"):
                self._battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
            elif (status == "Not charging"):
                self._battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
            else:    
                self._battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
            # publish message
            self._battery_state_pub.publish(self._battery_msg)


if __name__ == "__main__":
    rospy.init_node("computer_state_node")
    pcs = PodiComputerState()
    rospy.loginfo("Starting podi computer state node")
    pcs.spin()
