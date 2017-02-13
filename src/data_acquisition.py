#!/usr/bin/python

import os
import time
import signal
import sys
import shutil
import threading

import numpy as np
import scipy.io as scio

import rospy

from ble_observation.srv import BLEArrayService
from ble_observation.srv import BLEArrayServiceResponse
from ble_observation.msg import BLEMessage

from wifi_observation.srv import WifiArrayService
from wifi_observation.srv import WifiArrayServiceResponse
from wifi_observation.msg import WifiMessage

from xbee_observation.srv import XBeeArrayService
from xbee_observation.srv import XBeeArrayServiceResponse
from xbee_observation.msg import XBeeMessage

from std_msgs.msg import Header
from std_msgs.msg import String
from std_msgs.msg import Int64
from std_srvs.srv import Empty

class data_acquisition(object):
    """docstring for data_acquisition."""
    def __init__(self):
        super(data_acquisition, self).__init__()
        rospy.init_node('data_acquisition')
        self.rate             = rospy.Rate(10) # 10hz
        self.folder_location  = None
        self.db_name          = None
        self.is_txt           = None
        self.is_npy           = None
        self.is_mat           = None
        self.db_exists        = None
        self.epoch            = None
        self.rostime          = None
        self.ble_response     = None
        self.wifi_response    = None
        self.rf_response      = None
        self.pose             = None
        self.obs_array        = np.zeros((1,26), dtype=np.float64)
        self.port             = None
        self.baud             = None
        self.wifi_prefix      = None
        self.ble_prefix       = None
        self.ble_suffix       = None
        self.n_wifi_AP        = None
        self.n_ble_beacon     = None
        # self.rf_prefix        = None
        # self.n_rf             = None
        self.n_obs            = None
        self.db_location      = None
        self.db_name          = None
        self.db_fmt           = ".npy"
        self.parent_directory = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir))
        self.full_db          = None
        self.pos_idx          = 0

        self.get_params()

        print self.construct_full_location()

        self.s0 = rospy.Service('collect_data', Empty, self.collect_data)
        self.s1 = rospy.Service('save_data', Empty, self.save_data)
        self.obs_array = np.delete(self.obs_array, (0), axis=0)

        signal.signal(signal.SIGINT, self.signal_handler)

        rospy.spin()

    def get_params(self):
        self.port         = rospy.get_param("/port_wifi")
        self.baud         = rospy.get_param("/baud_wifi")
        self.wifi_prefix  = rospy.get_param("/wifi_prefix")
        self.ble_prefix   = rospy.get_param("/ble_prefix")
        self.ble_suffix   = rospy.get_param("/ble_suffix")
        self.n_wifi_AP    = rospy.get_param("/n_wifi_ap")
        self.n_ble_beacon = rospy.get_param("/n_ble_beacon")
        # self.rf_prefix    = rospy.get_param("/rf_prefix")
        # self.n_rf         = rospy.get_param("/n_rf")
        self.n_obs        = rospy.get_param("/n_obs")
        self.db_location  = rospy.get_param("/db_location")
        self.db_name      = rospy.get_param("/db_name")
        self.db_fmt       = rospy.get_param("/db_fmt")
        self.xbee_addr    = rospy.get_param("/xbee_addresses")
    def get_epoch(self):
        return int(time.time()), rospy.get_rostime()

    def construct_full_location(self):
        _, epoch = self.get_epoch()
        return self.parent_directory + '/' + self.db_location + self.db_name + "_" + str(epoch) + self.db_fmt

    def save_data(self, data):
        self.save_db(self.obs_array, location=self.construct_full_location(), fmt=self.db_fmt)
        return []

    def collect_data(self, data):
        rospy.loginfo("Service request captured")

        ble_obs = self.get_ble_observation()
        filtered_ble_observations = self.filter_ble_observations(ble_obs)
        ble_vector = self.parse_ble_observation(filtered_ble_observations, self.n_ble_beacon)

        for i in range(self.n_obs):
            self.epoch, self.rostime = self.get_epoch()
            obs_vector = np.zeros((1,26), dtype=np.float64)
            # print type(self.rostime), type(self.rostime.to_nsec()), self.rostime.to_nsec, np.float64(self.rostime.to_nsec())
            obs_vector[0,0] = np.float64(self.rostime.to_nsec())

            wifi_obs = self.get_wifi_observation()
            filtered_wifi_observations = self.filter_wifi_observations(wifi_obs)
            wifi_vector = self.parse_wifi_observation(filtered_wifi_observations, self.n_wifi_AP)

            xbee_obs = self.get_xbee_observation()
            xbee_vector = self.parse_xbee_observation(xbee_obs)

            obs_vector[0,1:9] = wifi_vector
            obs_vector[0,9:17] = ble_vector
            obs_vector[0,17:25] = xbee_vector
            obs_vector[0,25] = self.pos_idx

            self.obs_array = np.vstack((self.obs_array, obs_vector))
            print self.obs_array, self.obs_array.shape
            time.sleep(0.5)
        self.pos_idx += 1
        return []

    def parse_xbee_observation(self, obslist):
        n_obs = len(obslist.observations)
        obs_array = np.zeros(n_obs, dtype=np.float64)

        for i in range(n_obs):
            obs_array[i] = obslist.observations[i].rss
        return obs_array

    def parse_wifi_observation(self, obslist, n_total_obs):
        n_obs = len(obslist)
        obs_array = np.zeros(n_total_obs, dtype=np.float64)

        for i in range(n_obs):
            chunks = obslist[i].ssid.split("_")
            obs_array[int(chunks[-1])] = obslist[i].rss
        return obs_array

    def parse_ble_observation(self, obslist, n_total_obs):
        n_obs = len(obslist)
        obs_array = np.zeros(n_total_obs, dtype=np.float64)
        counter = np.zeros(n_total_obs, dtype=np.float64)

        for i in range(n_obs):
            chunks = obslist[i].ssid.split("_")
            idx = int(chunks[-2])
            obs_array[idx] += obslist[i].rss
            counter[idx] += 1

        with np.errstate(divide='ignore',invalid='ignore'):
            mean_obs = obs_array / counter

        mean_obs = np.nan_to_num(mean_obs)

        return mean_obs

    def filter_wifi_observations(self, obslist):
        filtered_observations = list()
        ssidList = list()
        sorted_observations = list()
        for i in range(len(obslist.observations)):
            ssid = obslist.observations[i].ssid
            if len(ssid)<len(self.wifi_prefix):
                continue

            if ssid[0:len(self.wifi_prefix)] == self.wifi_prefix:
                ssidList.append(ssid)
                rospy.loginfo("Discovered Node: %s, RSSI: %s", obslist.observations[i].ssid, obslist.observations[i].rss)
                filtered_observations.append(obslist.observations[i])
        return filtered_observations

    def filter_ble_observations(self, obslist):
        filtered_observations = list()
        ssidList = list()
        sorted_observations = list()
        for i in range(len(obslist.observations)):
            ssid = obslist.observations[i].ssid
            if len(ssid)<len(self.ble_prefix):
                continue

            if ssid[0:len(self.ble_prefix)] == self.ble_prefix:
                ssidList.append(ssid)
                rospy.loginfo("Discovered Node: %s, RSSI: %s", obslist.observations[i].ssid, obslist.observations[i].rss)
                filtered_observations.append(obslist.observations[i])
        return filtered_observations

    def get_xbee_observation(self):
        rospy.wait_for_service('xbee_observation')
        xbee_observation = rospy.ServiceProxy('xbee_observation', XBeeArrayService)
        obs_response = None
        try:
            obs_response = xbee_observation()
            rospy.loginfo("XBee observations acquired with return code and message %s, %s.\n Total number of observations acquired: %d", obs_response.success, obs_response.message, len(obs_response.observations))
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: %s" + str(exc))
        return obs_response

    def get_wifi_observation(self):
        rospy.wait_for_service('wifi_observation')
        wifi_observation = rospy.ServiceProxy('wifi_observation', WifiArrayService)
        obs_response = None
        try:
            obs_response = wifi_observation()
            rospy.loginfo("WiFi observations acquired with return code and message %s, %s.\n Total number of observations acquired: %d", obs_response.success, obs_response.message, len(obs_response.observations))
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: %s" + str(exc))
        return obs_response

    def get_ble_observation(self):
        rospy.wait_for_service('ble_observation')
        ble_observation = rospy.ServiceProxy('ble_observation', BLEArrayService)
        obs_response = None
        try:
            obs_response = ble_observation()
            rospy.loginfo("BLE observations acquired with return code and message %s, %s.\n Total number of observations acquired: %d", obs_response.success, obs_response.message, len(obs_response.observations))
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: %s" + str(exc))
        return obs_response

    def save_db(self, array, location, fmt):
        if fmt == ".mat":
            scipy.io.savemat(location, {'data':array})
        elif fmt == ".npy":
            np.save(location, array)
        elif fmt == ".txt":
            np.savetxt(location, array, delimiter=',')
        else:
            rospy.loginfo("Wrong file format")
            return False

    def signal_handler(self, signal, frame):
        print "Gracefully exited"
        rospy.signal_shutdown("SIGINT caught, I am trying to gracefully exit.")
        sys.exit(0)

if __name__ == "__main__":
    acq = data_acquisition()
