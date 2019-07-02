# %%
import os
import sys
import json
from pprint import pprint

import numpy as np
import pandas as pd
from plotly import tools
import plotly.graph_objs as go
import matplotlib.pyplot as mplot

from webplot import WebPlot

Ts = 0.04  # Most probably

path = "./"

bin_log_names = [
    "log_1_2019-7-2-10-21-48.bin",
    "mav.tlog"
]

log_keys = [bin_log_name.split(".")[0] for bin_log_name in bin_log_names]

data_selected = {}
data_json = {}
min_ts_fast = sys.maxsize


def file_n_lines(file_path):
    return len([l for l in open(file_path, 'r')])


for bin_log_name in bin_log_names:
    log_key = bin_log_name.split(".")[0]

    data_selected[log_key] = {}
    data_json[log_key] = {}

    bin_log_path = path + bin_log_name
    json_log_path = path + "tmp_" + bin_log_name + "." + "json"

    os.system("mavlogdump.py --format=json " + bin_log_path + " > " + json_log_path)

    data_json[log_key]["data"] = [json.loads(line) for line in open(json_log_path, 'r')]
    data_json[log_key]["messages"] = sorted(set([v["meta"]["type"] for v in data_json[log_key]["data"]]))

    print(data_json[log_key]["messages"])

    for msg_type in data_json[log_key]["messages"]:
        csv_log_path = path + "tmp_" + bin_log_name + "." + msg_type + ".csv"
        if not os.path.isfile(csv_log_path):
            os.system("mavlogdump.py --format=csv --types=" + msg_type + " " + bin_log_path + " > " + csv_log_path)

        if file_n_lines(csv_log_path) < 2:
            continue

        data_selected[log_key][msg_type] = pd.read_csv(csv_log_path, index_col=None, header=0)

        if "timestamp" in data_selected[log_key][msg_type].keys() and "FMT" not in msg_type:
            if min_ts_fast > data_selected[log_key][msg_type]["timestamp"][0]:
                min_ts_fast = data_selected[log_key][msg_type]["timestamp"][0]

data_prep = data_selected
us2s = 10**-6


def get_ts_scatter(df, lmsg_name, ltrace_name):
    if "timestamp" in df[lmsg_name].keys():
        return go.Scatter(x=(df[lmsg_name]["timestamp"] - min_ts_fast), y=df[lmsg_name][ltrace_name], name=lmsg_name + ":" + ltrace_name, mode='lines+markers')


def make_subplots(title, rows=1, cols=1):
    f = tools.make_subplots(rows=rows, cols=cols, print_grid=False)
    f["layout"]["title"]["text"] = title
    f["layout"]["paper_bgcolor"] = "rgb(0,0,0)"
    f["layout"]["plot_bgcolor"] = "rgb(0,0,0)"
    f["layout"]["legend"] = dict(orientation="h")
    f["layout"]["xaxis"] = dict(autorange=True)
    f["layout"]["xaxis"] = dict(tickformat=".3f")
    return f


# %%
plt = WebPlot()

for k, v in data_prep.items():

    if "ATT" not in v.keys():
        continue

    fig = make_subplots(k, rows=3, cols=1)

    print(k)
    print(v.keys())

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

plt = WebPlot()

for k, v in data_prep.items():

    if "RCIN" not in v.keys():
        continue

    fig = make_subplots(k, rows=6, cols=1)

    for idx in range(1, 7):
        fig.append_trace(get_ts_scatter(v, "RCIN", "C" + str(idx)), idx, 1)
        fig.append_trace(get_ts_scatter(v, "RCOU", "C" + str(idx)), idx, 1)

    plt.plot(fig)

plt.show()


# %%
plt = WebPlot()

for k, v in data_prep.items():
    if 'ATTITUDE' not in v.keys():
        continue

    fig = make_subplots(k, rows=3, cols=1)

    fig.append_trace(get_ts_scatter(v, "ATTITUDE", "ATTITUDE.roll"), 1, 1)
    fig.append_trace(get_ts_scatter(v, "ATTITUDE", "ATTITUDE.pitch"), 1, 1)
    fig.append_trace(get_ts_scatter(v, "ATTITUDE", "ATTITUDE.yaw"), 1, 1)

    fig.append_trace(get_ts_scatter(v, "ATTITUDE", "ATTITUDE.rollspeed"), 2, 1)
    fig.append_trace(get_ts_scatter(v, "ATTITUDE", "ATTITUDE.pitchspeed"), 2, 1)
    fig.append_trace(get_ts_scatter(v, "ATTITUDE", "ATTITUDE.yawspeed"), 2, 1)

    plt.plot(fig)

plt.show()

# %%

exit(0)

# %%

msg_struct = {msg_name: {trace_name: '' for trace_name, trace_value in msg_value.items()} for msg_name, msg_value in data_selected[log_keys[1]].items()}

pprint(msg_struct)


# %%

timestamps = (data_prep["mav"]["ATTITUDE"]["timestamp"] - data_prep["mav"]["ATTITUDE"]["timestamp"][0])
tdiff = np.diff(data_prep["mav"]["ATTITUDE"]["timestamp"])

mplot.plot(tdiff, '.-')

mplot.show()
