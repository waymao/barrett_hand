#!/usr/bin/env python3
from collections import defaultdict
import gzip
from datetime import datetime
import pathlib
import numpy as np

import rospy
from bhand_controller.msg import TactileArray
from sensor_msgs.msg import JointState

JOINT_KEYS = {key: i for i, key in enumerate(["bh_j11_joint", "bh_j12_joint", "bh_j22_joint", "bh_j32_joint"])}
JOINT_KEYS_NUM = len(JOINT_KEYS.keys())

class DataRecorder:
    def __init__(self, run_name="log"):
        run_start_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.file_path = pathlib.Path(__file__).parent.parent / "logs" / f"r{run_start_time}_{run_name}"

        self.joint_subscriber = rospy.Subscriber("/joint_states", JointState, self.joint_state_cb)
        self.tactile_subscriber = rospy.Subscriber("/bhand_node/tact_array", TactileArray, self.tact_cb)
        self.data_log = defaultdict(lambda: {"tact_max": None, "joint_pos": None, "joint_effort": None})

    def tact_cb(self, data: TactileArray):
        palm_max = np.max(data.palm)
        finger1_max = np.max(data.finger1)
        finger2_max = np.max(data.finger2)
        finger3_max = np.max(data.finger3)
        time = data.header.stamp
        # print("tact", time, palm_max, finger1_max, finger2_max, finger3_max)
        self.data_log[time]['tact_max'] = (palm_max, finger1_max, finger2_max, finger3_max)

    def joint_state_cb(self, data: JointState):
        time = data.header.stamp
        # print("jont", time, data.name, data.position, data.effort)
        pos_result = np.zeros((JOINT_KEYS_NUM,))
        eff_result = np.zeros((JOINT_KEYS_NUM,))
        for i, name in enumerate(data.name):
            if name in JOINT_KEYS:
                pos_result[JOINT_KEYS[name]] = data.position[i]
                eff_result[JOINT_KEYS[name]] = data.effort[i]
        self.data_log[time]["pos"] = pos_result
        self.data_log[time]["eff"] = eff_result
    
    def save_data(self, file_path=None):
        if file_path is None:
            file_path = self.file_path
        all_data_arr = [
            np.concatenate([
                [str(time)], 
                data.get('tact_max', np.zeros((JOINT_KEYS_NUM,))), 
                data.get('pos', np.zeros((JOINT_KEYS_NUM,))), 
                data.get('eff', np.zeros((JOINT_KEYS_NUM,))), 
            ]) 
            for time, data in self.data_log.items()
        ]
        data_result = np.array(all_data_arr)
        np.savez_compressed(file_path, data=data_result)
        print("data saved to", self.file_path)

        

if __name__ == "__main__":
    rospy.init_node('bhand_collect', anonymous=True)
    data_recorder = DataRecorder()
    rospy.spin()
    data_recorder.save_data()
