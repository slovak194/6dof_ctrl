# %%
import os
import sys
import pandas as pd

from plotly import tools
import plotly.graph_objs as go

from webplot import WebPlot

path = "./"

bin_log_names = [
    "log_13_2019-7-1-20-34-24.bin",
    "log_13_2019-7-1-20-34-25.bin"
]

msg_types = [
    "ATT",
    "IMU"
]

din = {}
min_ts = sys.maxsize

for bin_log_name in bin_log_names:
    din[bin_log_name.split(".")[0]] = {}
    for msg_type in msg_types:
        csv_log_path = path + bin_log_name + "." + msg_type + ".csv"
        bin_log_path = path + bin_log_name
        os.system("mavlogdump.py --format=csv --types="
                  + msg_type + " " + bin_log_path + " > " + csv_log_path)

        din[bin_log_name.split(".")[0]][msg_type] = pd.read_csv(csv_log_path, index_col=None, header=0)

        if min_ts > din[bin_log_name.split(".")[0]][msg_type]["TimeUS"][0]:
            min_ts = din[bin_log_name.split(".")[0]][msg_type]["TimeUS"][0]

data_prep = din


# %%

us2s = 10**-6
plt = WebPlot()


def get_ts_scatter(df, lmsg_name, ltrace_name):
    return go.Scatter(x=(df[lmsg_name]["TimeUS"]-min_ts)*us2s, y=df[lmsg_name][ltrace_name], name=ltrace_name, mode='lines+markers')


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
