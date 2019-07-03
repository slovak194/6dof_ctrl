# %%
import shutil as sh
from pprint import pprint

from webplot import WebPlot, get_ts_scatter, make_subplots
import mav_log_helper as mlh


Ts = 0.04  # Most probably

base_dir = "/home/slovak/fishbot/test/water_tank/"
log_dir = "fishbot/logs/2019-07-03/flight1/"

log_dir = base_dir + log_dir

data_json = mlh.dir_prepare(log_dir)
data_prep = mlh.dir_load(log_dir)

sh.copy("plotly-latest.min.js", log_dir)


# %%
plt = WebPlot(log_dir + "attitude.html")

for k, v in data_prep.items():

    if "ATT" not in v.keys():
        continue

    fig = make_subplots(k, rows=1, cols=1)

    print(k)
    print(v.keys())

    fig.append_trace(get_ts_scatter(v, "ATT", "Roll"), 1, 1)
    fig.append_trace(get_ts_scatter(v, "ATT", "Pitch"), 1, 1)
    fig.append_trace(get_ts_scatter(v, "ATT", "Yaw"), 1, 1)
    #
    # fig.append_trace(get_ts_scatter(v, "IMU", "AccX"), 2, 1)
    # fig.append_trace(get_ts_scatter(v, "IMU", "AccY"), 2, 1)
    # fig.append_trace(get_ts_scatter(v, "IMU", "AccZ"), 2, 1)
    #
    # fig.append_trace(get_ts_scatter(v, "IMU", "GyrX"), 3, 1)
    # fig.append_trace(get_ts_scatter(v, "IMU", "GyrY"), 3, 1)
    # fig.append_trace(get_ts_scatter(v, "IMU", "GyrZ"), 3, 1)

    plt.plot(fig)


for k, v in data_prep.items():
    if 'ATTITUDE' not in v.keys():
        continue

    fig = make_subplots(k, rows=1, cols=1)

    fig.append_trace(get_ts_scatter(v, "ATTITUDE", "ATTITUDE.roll"), 1, 1)
    fig.append_trace(get_ts_scatter(v, "ATTITUDE", "ATTITUDE.pitch"), 1, 1)
    fig.append_trace(get_ts_scatter(v, "ATTITUDE", "ATTITUDE.yaw"), 1, 1)
    #
    # fig.append_trace(get_ts_scatter(v, "ATTITUDE", "ATTITUDE.rollspeed"), 2, 1)
    # fig.append_trace(get_ts_scatter(v, "ATTITUDE", "ATTITUDE.pitchspeed"), 2, 1)
    # fig.append_trace(get_ts_scatter(v, "ATTITUDE", "ATTITUDE.yawspeed"), 2, 1)

    plt.plot(fig)

plt.show()

# %%

plt = WebPlot(log_dir + "rc_in_out.html")

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

msg_struct = [{msg_name: {trace_name: '' for trace_name, trace_value in msg_value.items()} for msg_name, msg_value in
               data_prep[key].items()} for key in data_prep.keys()]
pprint(msg_struct)

