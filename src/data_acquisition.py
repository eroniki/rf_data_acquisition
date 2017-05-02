#!/usr/bin/python

import os
import time
import signal
import sys

import numpy as np
import scipy.io as scio

import rospy
import matplotlib.pyplot as plt

from xbee_observation.srv import XBeeArrayService
from xbee_observation.srv import XBeeArrayServiceResponse
from xbee_observation.msg import XBeeMessage

from std_srvs.srv import Empty


class observation(object):
    """docstring for observation."""

    def __init__(self, fname, n_anchor):
        super(observation, self).__init__()
        self._fname = fname
        self._n_anchor = n_anchor
        self._measurements = None

    def append_observation(self, observation):
        if self.measurements is None:
            self.measurements = observation
        else:
            self.measurements = np.vstack((self.measurements, observation))

    def save_db(self, array, location, fmt):
        if fmt == ".mat":
            scipy.io.savemat(location, {'data': array})
        elif fmt == ".npy":
            np.save(location, array)
        elif fmt == ".txt":
            np.savetxt(location, array, delimiter=',')
        else:
            rospy.loginfo("Wrong file format")
            return False

    @property
    def measurements(self):
        doc = "The measurements property."
        return self._measurements

    @measurements.setter
    def measurements(self, value):
        if self._measurements is None:
            self._measurements = value
        else:
            self._measurements = np.vstack((self.measurements, value))

    @measurements.deleter
    def measurements(self):
        del self._measurements

    @property
    def n_anchor(self):
        doc = "The n_anchor property."
        return self._n_anchor

    @n_anchor.setter
    def n_anchor(self, value):
        pass

    @n_anchor.deleter
    def n_anchor(self):
        del self._n_anchor

    @property
    def fname(self):
        doc = "The fname property."
        return self._fname

    @fname.setter
    def fname(self, value):
        pass

    @fname.deleter
    def fname(self):
        del self._fname


class data_acquisition(object):
    """docstring for data_acquisition."""

    def __init__(self):
        super(data_acquisition, self).__init__()
        rospy.init_node('data_acquisition')
        self.rate = rospy.Rate(100)  # 10hz
        self.folder_location = None
        self.db_name = None
        self.is_txt = None
        self.is_npy = None
        self.is_mat = None
        self.epoch = None
        self.rostime = None
        self.wifi_response = None
        self.rf_response = None
        self.pose = None

        self.debug = None
        self.n_obs = None
        self.db_location = None
        self.db_name = None
        self.db_fmt = None
        self.seq_id = 0

        self.get_params()

        self.sensors = ["wifi", "lora"]
        self.databases = list()

        self.create_databases()
        if self.debug:
            for database in self.databases:
                if database is not None:
                    rospy.loginfo(
                        " ".join((database.fname, str(database.n_anchor))))

        if self.visualize:
            # plt.ion()
            self.figure_rssi, self.axes_rssi = self.init_visualization(1)
            self.figure_rtt, self.axes_rtt = self.init_visualization(2)
            plt.tight_layout()
            plt.show()

        self.s0 = rospy.Service('collect_data', Empty, self.collect_data)
        self.s1 = rospy.Service('save_data', Empty, self.save_data)

        signal.signal(signal.SIGINT, self.signal_handler)

        rospy.spin()

    def get_params(self):
        try:
            self.n_obs = rospy.get_param("/n_obs")
            self.n_wifi = rospy.get_param("/n_wifi")
            self.n_lora = rospy.get_param("/n_lora")
            self.db_location = rospy.get_param("/db_location")
            self.db_name = rospy.get_param("/db_name")
            self.db_fmt = rospy.get_param("/db_fmt")
            self.debug = rospy.get_param("/debug")
            self.visualize = rospy.get_param("/visualize")
            self.wifi_namespace = rospy.get_param("/wifi_namespace")
            self.lora_namespace = rospy.get_param("/lora_namespace")
            self.wifi_service = rospy.get_param("/")

            self.n_sensors = [self.n_wifi, self.n_lora]
            self.namespaces = [self.wifi_namespace, self.lora_namespace]
            self.service_names = ["".join(("/", ns, "/xbee_observation"))
                                  for ns in self.namespaces]

            if self.debug:
                self.print_params()

        except KeyError as e:
            rospy.logerr("Parameters couldn't parsed \n\r%s", e)

    def print_params(self):
        params = "".join(("WiFi Anchor Nodes: ", str(self.n_wifi), "\n",
                          "LoRa Anchor Nodes: ", str(self.n_lora), "\n",
                          "DB Name: ", str(self.db_name), "\n",
                          "DB Location: ", str(self.db_location), "\n",
                          "DB File Format: ", str(self.db_fmt), "\n",
                          "Debug: ", str(self.debug), "\n",
                          "Visualize: ", str(self.visualize), "\n",
                          "WiFi Namespace:", str(self.wifi_namespace), "\n",
                          "LoRa Namespace:", str(self.lora_namespace), "\n",
                          "Succesive Measurements: ", str(self.n_obs)
                          ))

        rospy.loginfo(
            "Params Acquired from Parameter Server \n\r%s", params)

    def create_databases(self):
        _, epoch = self.get_epoch()
        suffixes = ["_rssi", "_rtt"]
        for i, sensor in enumerate(self.sensors):
            for suffix in suffixes:
                directory = os.path.abspath(os.path.join(os.path.dirname(
                    os.path.realpath(__file__)), os.pardir))
                s = "".join((sensor, suffix))
                fname = self.construct_full_location(
                    epoch=epoch, parent_directory=directory, suffix=s)

                self.databases.append(
                    self.init_measurement_space(fname=fname,
                                                n_anchor=self.n_sensors[i]))

    def init_measurement_space(self, fname, n_anchor):
        return observation(fname=fname, n_anchor=n_anchor)

    def get_epoch(self):
        return int(time.time()), rospy.get_rostime()

    def construct_full_location(self, epoch, parent_directory, suffix):
        fname = "".join((parent_directory,
                         self.db_location,
                         self.db_name,
                         "_",
                         suffix,
                         "_",
                         str(epoch),
                         str(self.db_fmt)
                         ))
        return fname

    # TODO: FIX THIS, WE NOW HAVE A DB CLASS
    def save_data(self, data):
        self.save_db(self.obs_array, location=self.full_location,
                     fmt=self.db_fmt)
        return []

    def collect_data(self, data):
        rospy.loginfo("Service request captured")
        # TODO: pos obtain
        # pos =
        for i in range(self.n_obs):
            for sensor_id, sensor in enumerate(self.sensors):
                self.epoch, self.rostime = self.get_epoch()
                xbee_obs = self.get_xbee_observation(
                    service_name=self.service_names[sensor_id])
                rss, rtt = self.parse_xbee_observation(xbee_obs)
                self.databases[(sensor_id * 2)].append_observation(rss)
                self.databases[(sensor_id * 2) + 1].append_observation(rtt)
            print self.databases[0].measurements
            print self.databases[1].measurements

            self.seq_id += 1
            time.sleep(0.5)
            # print i, db_id, database.fname, database.n_anchor,
            # np.float64(self.rostime.to_nsec())

            # self.plot_measurements(
            #     self.figure_rssi, wifi_vector, ble_vector, xbee_vector)
            # self.obs_array = np.vstack((self.obs_array, obs_vector))
        return []

    # TODO: ALL VISUALIZATION SHALL BE IMPLEMENTED AGAIN!
    def init_visualization(self, id):
        pass

    def plot_measurements(self, figure, axes,
                          wifi_vector, lora_vector):
        figure.clear()
        figure, axes = self.init_visualization()
        axes[0].bar(range(wifi_vector.size), wifi_vector)
        axes[0].set_xlim([0, wifi_vector.size])
        axes[0].grid(True)
        axes[1].bar(range(ble_vector.size), ble_vector)
        axes[1].set_xlim([0, ble_vector.size])
        axes[1].grid(True)
        axes[2].bar(range(xbee_vector.size), xbee_vector)
        axes[2].set_xlim([0, xbee_vector.size])
        axes[2].grid(True)
        figure.canvas.draw()
        # plt.draw()
        # self.axes[2].draw()

    def parse_xbee_observation(self, obslist):
        n_obs = len(obslist.observations)
        rss = np.zeros(n_obs, dtype=np.float64)
        rtt = np.zeros(n_obs, dtype=np.float64)

        for i in range(n_obs):
            rss[i] = obslist.observations[i].rss
            rtt[i] = obslist.observations[i].rtt.to_nsec()
        return rss, rtt

    def get_xbee_observation(self, service_name):
        rospy.wait_for_service(service_name)
        xbee_observation = rospy.ServiceProxy(service_name, XBeeArrayService)
        obs_response = None
        try:
            obs_response = xbee_observation()
            msg = "".join(("XBee observations acquired with return code: ",
                           str(obs_response.success), "\n",
                           "Return Message: ", str(
                               obs_response.message), "\n",
                           "Total number of observations: ",
                           str(len(obs_response.observations)), "\n"))

            rospy.loginfo(msg)

        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: %s", str(exc))
        return obs_response

    def signal_handler(self, signal, frame):
        rospy.loginfo("Gracefully exited")
        if self.visualize is True:
            try:
                plt.close(self.figure_rtt)
                plt.close(self.figure_rssi)
            except Exception as e:
                rospy.logerr(e)
        rospy.signal_shutdown("SIGINT caught, I am trying to gracefully exit.")
        sys.exit(0)


if __name__ == "__main__":
    acq = data_acquisition()
