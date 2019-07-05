#! /usr/bin/env python3

import os
import sys
import json
import glob

import pickle

import pandas as pd


def file_n_lines(file_path):
    return len([l for l in open(file_path, 'r')])


def dir_prepare(log_dir):

    # In case of old mavproxy on target
    # dataflash_path = "dataflash/"
    # with open(base_dir + dataflash_path + "LASTLOG.TXT") as last_bin_log:
    #     sh.copy(base_dir + dataflash_path + last_bin_log.read() + ".BIN", log_dir)

    if not os.path.isfile(log_dir + "data_json.pcl"):

        log_paths = [log_path for log_path in glob.glob(log_dir + "*.BIN")] + [log_dir + "flight.tlog"]

        data_json = {}

        for log_path in log_paths:
            log_key = log_path.split("/")[-1].split(".")[0]

            data_json[log_key] = {}

            json_log_path = log_dir + log_key + ".json"

            if not os.path.isfile(json_log_path):
                os.system("mavlogdump.py --format=json " + log_path + " > " + json_log_path)

            data_json[log_key]["data"] = [json.loads(line) for line in open(json_log_path, 'r')]
            data_json[log_key]["messages"] = sorted(set([v["meta"]["type"] for v in data_json[log_key]["data"]]))

            print(data_json[log_key]["messages"])

            for msg_type in data_json[log_key]["messages"]:
                csv_log_path = log_dir + log_key + "." + msg_type + ".csv"
                if not os.path.isfile(csv_log_path):
                    os.system("mavlogdump.py --format=csv --types=" + msg_type + " " + log_path + " > " + csv_log_path)

        with open(log_dir + "data_json.pcl", 'wb') as of:
            pickle.dump(data_json, of, protocol=pickle.HIGHEST_PROTOCOL)

    else:
        with open(log_dir + "data_json.pcl", 'rb') as of:
            data_json = pickle.load(of)

    return data_json


def dir_load(log_dir):

    if not os.path.isfile(log_dir + "data_raw.pcl"):
        csv_log_paths = glob.glob(log_dir + "*.csv")

        data_raw = {}

        for csv_log_path in csv_log_paths:
            if file_n_lines(csv_log_path) < 2:
                continue

            log_key = csv_log_path.split("/")[-1].split(".")[0]
            msg_type = csv_log_path.split("/")[-1].split(".")[1]

            if log_key not in data_raw.keys():
                data_raw[log_key] = {}

            data_raw[log_key][msg_type] = pd.read_csv(csv_log_path, index_col=None, header=0)

        with open(log_dir + "data_raw.pcl", 'wb') as of:
            pickle.dump(data_raw, of, protocol=pickle.HIGHEST_PROTOCOL)

    else:
        with open(log_dir + "data_raw.pcl", 'rb') as of:
            data_raw = pickle.load(of)

    return data_raw


if __name__ == '__main__':
    if len(sys.argv) == 2:
        dir_prepare(sys.argv[1])
    elif len(sys.argv) == 1:
        dir_prepare(os.getcwd())
    else:
        raise Exception("Wrong usage, please specify dir or run in dir with no args")
