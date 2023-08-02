import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import TimesyncStatus
import logging
import time
import numpy as np
from threading import Timer
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
import tf.transformations
import os
import csv

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


class MoCapPubSub:
    def __init__(self):
        rospy.init_node('px4_mocap_pubsub')  # Initialize node

        self.mocap_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)

        rospy.loginfo("PX4 mocap pub-sub node initialized")

        cflib.crtp.init_drivers()

        self._cf = Crazyflie(rw_cache='./cache')

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % uri)

        self._cf.open_link(uri)

        self.is_connected = True

    def _connected(self, link_uri):
        print('Connected to %s' % link_uri)

        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
        self._lg_stab.add_variable('stateEstimate.x', 'float')
        self._lg_stab.add_variable('stateEstimate.y', 'float')
        self._lg_stab.add_variable('stateEstimate.z', 'float')
        self._lg_stab.add_variable('stabilizer.roll', 'float')
        self._lg_stab.add_variable('stabilizer.pitch', 'float')
        self._lg_stab.add_variable('stabilizer.yaw', 'float')

        try:
            self._cf.log.add_config(self._lg_stab)
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

    def _stab_log_error(self, logconf, msg):
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        x = data['stateEstimate.x']
        y = data['stateEstimate.y']
        z = data['stateEstimate.z']
        px4_z = -z + 0.18

        roll = data['stabilizer.roll']
        pitch = data['stabilizer.pitch']
        yaw = data['stabilizer.yaw']

        roll_rad = np.deg2rad(roll)
        pitch_rad = np.deg2rad(pitch)
        yaw_rad = np.deg2rad(yaw)
        quaternion = tf.transformations.quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = y
        pose_msg.pose.position.y = x
        pose_msg.pose.position.z = px4_z
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]
        #pose_msg.pose.orientation.x = NaN
        #pose_msg.pose.orientation.y = NaN
        #pose_msg.pose.orientation.z = NaN
        #pose_msg.pose.orientation.w = NaN
        # self.mocap_pub.publish(pose_msg)

        # Save the data to the CSV file
        self.save_to_csv(pose_msg.header.stamp, pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z, pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w)

        print(f'quaternion=({quaternion[0]:.2f},{quaternion[1]:.2f},{quaternion[2]:.2f},{quaternion[3]:.2f})')

        print(f'position: x=({y:.2f}, y={y:.2f}, z_cf={z:.2f}, z_px4={px4_z:.2f}) Euler_Angles=({roll:.2f}, {pitch:.2f}, {yaw:.2f})')

    def _connection_failed(self, link_uri, msg):
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        print('Disconnected from %s' % link_uri)
        self.is_connected = False

    def save_to_csv(self, timestamp, x, y, z, qx, qy, qz, qw):
        """
        Save the position data to a CSV file.

        Parameters:
        - timestamp: The timestamp of the position data.
        - x: X-coordinate of the position.
        - y: Y-coordinate of the position.
        - z: Z-coordinate of the position.

        Note: The data is appended to the CSV file with the following format:
              Timestamp, X, Y, Z
        """
        try:
            file_path = os.path.join(os.path.expanduser('~'), 'position_data_crazyflie.csv')
            rospy.loginfo(f"Saving file to: {file_path}")

            file_exists = os.path.isfile(file_path)

            # If the file does not exist, create a new one and write the header row
            if not file_exists:
                with open(file_path, mode='w', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(['Timestamp', 'X', 'Y', 'Z', 'qX', 'qY', 'qZ', 'qW'])

            # Open the CSV file in append mode and write the data to a row
            with open(file_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([timestamp, x, y, z])
        except Exception as e:
            rospy.logerr('Error: ' + str(e))


def main():
    pub_sub = MoCapPubSub()
    while pub_sub.is_connected and not rospy.is_shutdown():
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print('Exiting...')
            break


if __name__ == '__main__':
    main()

