# %% Some preparations
import shutil as sh
import os
from datetime import date
from pprint import pprint

import numpy as np

import plot_helper as ph
import mav_log_helper as mlh

Ts = 0.04  # Most probably

base_dir = "/home/slovak/fishbot/test/water_tank/"
test_day_dir = "fishbot/logs/" + date.today().strftime("%Y-%m-%d")
latest_flight = sorted(os.listdir(base_dir + test_day_dir))[-1]
log_dir = test_day_dir + "/" + latest_flight + "/"

# Or set explicitly
# log_dir = "fishbot/logs/2019-07-04/flight4/"

log_dir = base_dir + log_dir

print(log_dir)

data_json = mlh.dir_prepare(log_dir)
data_raw = mlh.dir_load(log_dir)

if not os.path.isfile(log_dir + "plotly-latest.min.js"):
    sh.copy("plotly-latest.min.js", log_dir)


# %% ATTITUDE
plt = ph.WebPlot(log_dir + "attitude.html")

for k, v in data_raw.items():

    if "ATT" not in v.keys():
        continue

    fig = ph.make_subplots(k, rows=1, cols=1)

    print(k)
    print(v.keys())

    fig.append_trace(ph.get_ts_scatter(v, "ATT", "Roll", scaler=np.pi / 180), 1, 1)
    fig.append_trace(ph.get_ts_scatter(v, "ATT", "Pitch", scaler=np.pi / 180), 1, 1)
    fig.append_trace(ph.get_ts_scatter(v, "ATT", "Yaw", scaler=np.pi / 180), 1, 1)
    #
    # fig.append_trace(wp.get_ts_scatter(v, "IMU", "AccX"), 2, 1)
    # fig.append_trace(wp.get_ts_scatter(v, "IMU", "AccY"), 2, 1)
    # fig.append_trace(wp.get_ts_scatter(v, "IMU", "AccZ"), 2, 1)
    #
    # fig.append_trace(wp.get_ts_scatter(v, "IMU", "GyrX"), 3, 1)
    # fig.append_trace(wp.get_ts_scatter(v, "IMU", "GyrY"), 3, 1)
    # fig.append_trace(wp.get_ts_scatter(v, "IMU", "GyrZ"), 3, 1)

    plt.plot(fig)


for k, v in data_raw.items():
    if 'ATTITUDE' not in v.keys():
        continue

    fig = ph.make_subplots(k, rows=1, cols=1)

    fig.append_trace(ph.get_ts_scatter(v, "ATTITUDE", "ATTITUDE.roll"), 1, 1)
    fig.append_trace(ph.get_ts_scatter(v, "ATTITUDE", "ATTITUDE.pitch"), 1, 1)
    fig.append_trace(ph.get_ts_scatter(v, "ATTITUDE", "ATTITUDE.yaw"), 1, 1)
    #
    # fig.append_trace(wp.get_ts_scatter(v, "ATTITUDE", "ATTITUDE.rollspeed"), 2, 1)
    # fig.append_trace(wp.get_ts_scatter(v, "ATTITUDE", "ATTITUDE.pitchspeed"), 2, 1)
    # fig.append_trace(wp.get_ts_scatter(v, "ATTITUDE", "ATTITUDE.yawspeed"), 2, 1)

    plt.plot(fig)

plt.show()

exit(0)


# %% RC ins and outs

plt = ph.WebPlot(log_dir + "rc_in_out.html")

for k, v in data_raw.items():

    if "RCIN" not in v.keys():
        continue

    fig = ph.make_subplots(k, rows=6, cols=1)

    for idx in range(1, 7):
        fig.append_trace(ph.get_ts_scatter(v, "RCIN", "C" + str(idx)), idx, 1)
        fig.append_trace(ph.get_ts_scatter(v, "RCOU", "C" + str(idx)), idx, 1)

    plt.plot(fig)

plt.show()


# %% Stats for messages
def get_stats(msg_value):
    if "timestamp" in msg_value.keys() and len(msg_value["timestamp"]) > 2:
        return [
            round(1/np.median(np.diff(msg_value["timestamp"]))),
            # round(1/np.min(np.diff(msg_value["timestamp"]))),
            # round(1/np.max(np.diff(msg_value["timestamp"]))),
        ]
    else:
        return ''


# %% All
msg_struct = [{msg_name: {trace_name: get_stats(msg_value) for trace_name, trace_value in msg_value.items()} for msg_name, msg_value in
               data_raw[key].items()} for key in data_raw.keys()]
pprint(msg_struct)

# %% Dataflash
msg_struct = [{msg_name: {trace_name: get_stats(msg_value) for trace_name, trace_value in msg_value.items()} for msg_name, msg_value in
               data_raw[key].items()} for key in ["1"]]
pprint(msg_struct)

# %% Mavlink

msg_struct = [{msg_name: {trace_name: get_stats(msg_value) for trace_name, trace_value in msg_value.items()} for msg_name, msg_value in
               data_raw[key].items()} for key in ["flight"]]
pprint(msg_struct)

# %% Something more

