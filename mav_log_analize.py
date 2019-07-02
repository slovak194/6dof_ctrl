# %%
import os
import sys

import json

import pandas as pd

from plotly import tools
import plotly.graph_objs as go

from webplot import WebPlot

Ts = 0.04  # Most probably

path = "./"

#
# "log_13_2019-7-1-20-34-24.bin",
# "log_14_2019-7-2-09-20-34.bin",

bin_log_names = [
    "log_1_2019-7-2-10-21-48.bin"
]

log_keys = [bin_log_name.split(".")[0] for bin_log_name in bin_log_names]

msg_types = [
    "ATT",
    "IMU"
]

data_selected = {}
data_json = {}
min_ts = sys.maxsize


def file_n_lines(file_path):
    return len([l for l in open(file_path, 'r')])


for bin_log_name in bin_log_names:
    log_key = bin_log_name.split(".")[0]

    data_selected[log_key] = {}
    data_json[log_key] = {}

    bin_log_path = path + bin_log_name
    json_log_path = path + bin_log_name + "." + "json"

    os.system("mavlogdump.py --format=json " + bin_log_path + " > " + json_log_path)

    data_json[log_key]["data"] = [json.loads(line) for line in open(json_log_path, 'r')]
    data_json[log_key]["messages"] = sorted(set([v["meta"]["type"] for v in data_json[log_key]["data"]]))

    print(data_json[log_key]["messages"])

    for msg_type in data_json[log_key]["messages"]:
        csv_log_path = path + bin_log_name + "." + msg_type + ".csv"
        os.system("mavlogdump.py --format=csv --types="
                  + msg_type + " " + bin_log_path + " > " + csv_log_path)

        if file_n_lines(csv_log_path) < 2:
            continue

        data_selected[log_key][msg_type] = pd.read_csv(csv_log_path, index_col=None, header=0)

        if "TimeUS" in data_selected[log_key][msg_type].keys():
            if min_ts > data_selected[log_key][msg_type]["TimeUS"][0]:
                min_ts = data_selected[log_key][msg_type]["TimeUS"][0]

data_prep = data_selected

us2s = 10**-6
plt = WebPlot()


def get_ts_scatter(df, lmsg_name, ltrace_name):
    return go.Scatter(x=(df[lmsg_name]["TimeUS"]-min_ts)*us2s, y=df[lmsg_name][ltrace_name], name=ltrace_name, mode='lines+markers')


def make_subplots():
    f = tools.make_subplots(rows=3, cols=1, print_grid=False)
    f["layout"]["title"]["text"] = k
    f["layout"]["paper_bgcolor"] = "rgb(0,0,0)"
    f["layout"]["plot_bgcolor"] = "rgb(0,0,0)"
    f["layout"]["legend"] = dict(orientation="h")
    f["layout"]["xaxis"] = dict(autorange=True)
    f["layout"]["xaxis"] = dict(tickformat=".3f")
    return f
# %%


for k, v in data_prep.items():
    fig = tools.make_subplots(rows=3, cols=1, print_grid=False)
    fig["layout"]["title"]["text"] = k
    fig["layout"]["paper_bgcolor"] = "rgb(0,0,0)"
    fig["layout"]["plot_bgcolor"] = "rgb(0,0,0)"
    fig["layout"]["legend"] = dict(orientation="h")
    fig["layout"]["xaxis"] = dict(autorange=True)
    fig["layout"]["xaxis"] = dict(tickformat=".3f")

    fig.append_trace(get_ts_scatter(v, "ATT", "Roll"), 1, 1)
    fig.append_trace(get_ts_scatter(v, "ATT", "Pitch"), 1, 1)
    fig.append_trace(get_ts_scatter(v, "ATT", "Yaw"), 1, 1)

    fig.append_trace(get_ts_scatter(v, "IMU", "AccX"), 2, 1)
    fig.append_trace(get_ts_scatter(v, "IMU", "AccY"), 2, 1)
    fig.append_trace(get_ts_scatter(v, "IMU", "AccZ"), 2, 1)

    fig.append_trace(get_ts_scatter(v, "IMU", "GyrX"), 3, 1)
    fig.append_trace(get_ts_scatter(v, "IMU", "GyrY"), 3, 1)
    fig.append_trace(get_ts_scatter(v, "IMU", "GyrZ"), 3, 1)

    plt.plot(fig)

plt.show()


# %%


for k, v in data_prep.items():
    fig = tools.make_subplots(rows=3, cols=1, print_grid=False)
    fig["layout"]["title"]["text"] = k
    fig["layout"]["paper_bgcolor"] = "rgb(0,0,0)"
    fig["layout"]["plot_bgcolor"] = "rgb(0,0,0)"
    fig["layout"]["legend"] = dict(orientation="h")
    fig["layout"]["xaxis"] = dict(autorange=True)
    fig["layout"]["xaxis"] = dict(tickformat=".3f")

    fig.append_trace(get_ts_scatter(v, "ATT", "Roll"), 1, 1)
    fig.append_trace(get_ts_scatter(v, "ATT", "Pitch"), 1, 1)
    fig.append_trace(get_ts_scatter(v, "ATT", "Yaw"), 1, 1)

    fig.append_trace(get_ts_scatter(v, "IMU", "AccX"), 2, 1)
    fig.append_trace(get_ts_scatter(v, "IMU", "AccY"), 2, 1)
    fig.append_trace(get_ts_scatter(v, "IMU", "AccZ"), 2, 1)

    fig.append_trace(get_ts_scatter(v, "IMU", "GyrX"), 3, 1)
    fig.append_trace(get_ts_scatter(v, "IMU", "GyrY"), 3, 1)
    fig.append_trace(get_ts_scatter(v, "IMU", "GyrZ"), 3, 1)

    plt.plot(fig)

plt.show()



# %%

# NKF6 -
# MOTB
# IMU
# NKF7
# RATE
# NKF9
# DU32
# NKF2
# CTRL
# PM
# NKQ2
# NKF5
# IMU2
# PARM
# AHR2
# NKQ1
# CTUN
# POWR
# BARO
# MAG
# NKF4
# MSG
# ATT
# RCOU
# VIBE
# FMT
# NKF1
# NKF3
# NKF8
# RCIN
# EV
# MODE

A = set(data_json["log_13_2019-7-1-20-34-24"]["messages"])
B = set(data_json["log_14_2019-7-2-09-20-34"]["messages"])

# print(A)
# print(B)

print(A.difference(B))
print(B.difference(A))

A.symmetric_difference(B)


# %%

from pprint import pprint

msg_struct = {msg_name: {trace_name: '' for trace_name, trace_value in msg_value.items()} for msg_name, msg_value in data_selected[log_keys[0]].items()}

pprint(msg_struct)