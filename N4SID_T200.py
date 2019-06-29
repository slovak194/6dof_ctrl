# %%
import numpy as np
import pandas as pd
import glob
import re
import control

from plotly import tools
import plotly.graph_objs as go

import scipy.signal as sp

from webplot import WebPlot

path = r'../6dof_ctrl'
all_files = glob.glob(path + "/*.csv")

di = {re.split("[/.]", filename)[-2]: pd.read_csv(filename, index_col=None, header=0) for filename in all_files}

Ts = 0.002


def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = sp.butter(order, normal_cutoff, btype='low', analog=False)
    return b, a


def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    return sp.lfilter(b, a, data)


def detrend(din):
    return {key: {
        "time (s)": din[key]["time (s)"],
        "input (us)": sp.detrend(din[key]["input (us)"], type="constant"),
        "force (lb)": sp.detrend(din[key]["force (lb)"], type="constant")
    } for key, value in din.items()
    }


def filt(din):
    return {key: {
        "time (s)": din[key]["time (s)"],
        "input (us)": sp.detrend(din[key]["input (us)"], type="constant"),
        "force (lb)": butter_lowpass_filter(din[key]["force (lb)"], 20, 1 / Ts, 6)
    } for key, value in din.items()
    }


def preprocess(din):
    dout = detrend(din)
    dout = filt(dout)

    return dout


data_raw = {
    key: value for key, value in di.items() if key in [
                                                   "T200_Sine_0-10_Hz_1600-1900_us",
                                                   "T200_Square_0-10_Hz_1600-1900_us"
                                               ]
}

data_prep = preprocess(data_raw)

plt = WebPlot()

for k, v in data_prep.items():
    fig = tools.make_subplots(rows=3, cols=1, print_grid=False)

    fig["layout"]["title"]["text"] = k

    fig.append_trace(go.Scatter(x=v["time (s)"], y=v["input (us)"], name="input"), 1, 1)
    fig.append_trace(go.Scatter(x=v["time (s)"], y=v["force (lb)"], name="force"), 2, 1)

    fig.append_trace(go.Scatter(x=v["time (s)"], y=v["input (us)"], name="input"), 3, 1)
    fig.append_trace(go.Scatter(x=v["time (s)"], y=v["force (lb)"] * 30, name="force"), 3, 1)

    fig["layout"]["paper_bgcolor"] = "rgb(0,0,0)"
    fig["layout"]["plot_bgcolor"] = "rgb(0,0,0)"
    fig["layout"]["legend"] = dict(orientation="h")

    plt.plot(fig)

plt.show()

# %%
def ident_sippy(u, y, dim):
    import SIPPY
    sys_id = SIPPY.system_identification(y, u, 'N4SID', SS_fixed_order=dim, SS_A_stability=True)
    #   sys_id=SIPPY.system_identification(y, u, 'MOESP', SS_fixed_order=dim, SS_A_stability=True)
    #   sys_id=SIPPY.system_identification(y, u, 'CVA', SS_fixed_order=dim, SS_A_stability=True)

    return sys_id.A, sys_id.B, sys_id.C, sys_id.D, sys_id.x0


def ident_interface(fun, u, y, dim):
    AID, BID, CID, DID, *rest = fun(u, y, dim)
    USysID = (AID, BID, CID, DID)
    print(dim)
    return USysID


# id_data = data_prep["T200_Square_0-10_Hz_1600-1900_us"]
id_data = data_prep["T200_Sine_0-10_Hz_1600-1900_us"]
input_u = id_data["input (us)"][np.newaxis, :]
output_y = id_data["force (lb)"][np.newaxis, :]

dims = range(4, 10)

USysIDs = [ident_interface(ident_sippy, input_u, output_y, dim) for dim in dims]

# %%

plt = WebPlot()

for USysID in USysIDs:
    print("\nOrder = " + str(USysID[0].shape[0]) + "\n")
    SS = control.StateSpace(*USysID, Ts)

    mag, phase, w = control.bode_plot(SS, Plot=False)
    poles, zeros = control.pzmap(SS, Plot=False)
    rlist, klist = control.root_locus(SS, kvect=np.arange(0, 50), Plot=False)

    traces = [
        go.Scatter(x=poles.real, y=poles.imag, mode='markers', marker={'color': 'blue', 'symbol': 'x'}, name="poles",
                   xaxis="x", yaxis="y"),
        go.Scatter(x=zeros.real, y=zeros.imag, mode='markers', marker={'color': 'blue', 'symbol': 'circle'},
                   name="zeros",
                   xaxis="x", yaxis="y"),
        go.Scatter(x=w * 2 * np.pi, y=np.squeeze(mag), name="mag", xaxis="x2", yaxis="y2"),
        go.Scatter(x=w * 2 * np.pi, y=np.squeeze(phase*180/np.pi), name="phase", xaxis="x3", yaxis="y3")
    ]

    traces += [
        go.Scatter(x=rlist[:, i].real, y=rlist[:, i].imag, line=go.scatter.Line(color="orange"),
                   name="gain_path " + str(i),
                   xaxis="x", yaxis="y") for i in range(0, USysID[0].shape[0])]

    layout = {
        'shapes': [dict(type='circle', xref='x', yref='y', x0=-1, y0=-1, x1=1, y1=1, line=dict(color="rgb(0,0,255)"))],
        "xaxis": dict(domain=[0, 0.2], scaleanchor="y", gridcolor='rgb(50,50,50)'),
        "yaxis": dict(domain=[0, 1], scaleanchor="x", gridcolor='rgb(50,50,50)'),
        "xaxis2": dict(domain=[0.25, 1], type='log', autorange=True, anchor='y2', gridcolor='rgb(50,50,50)'),
        "yaxis2": dict(domain=[0.55, 1], type='log', autorange=True, anchor='x2', gridcolor='rgb(50,50,50)'),
        "xaxis3": dict(domain=[0.25, 1], type='log', autorange=True, anchor='y3', gridcolor='rgb(50,50,50)'),
        "yaxis3": dict(domain=[0, 0.45], anchor='x3'),
        "paper_bgcolor": "rgb(0,0,0)",
        "plot_bgcolor": "rgb(0,0,0)",
        "title": "Order = " + str(USysID[0].shape[0]),
        "legend": dict(orientation="h")
    }

    fig = go.Figure(data=traces, layout=layout)

    plt.plot(fig)

    for k, v_data in data_prep.items():
        tout, yout, xout = sp.dlsim((*USysID, Ts), v_data["input (us)"])

        fig1 = tools.make_subplots(rows=1, cols=1, print_grid=False)
        fig1["layout"]["paper_bgcolor"] = "rgb(0,0,0)"
        fig1["layout"]["plot_bgcolor"] = "rgb(0,0,0)"
        fig1["layout"]["height"] = 400
        fig1["layout"]["title"] = k
        fig1["layout"]["legend"] = dict(orientation="h")

        fig1.append_trace(go.Scatter(x=tout, y=np.squeeze(yout), name="yout"), 1, 1)
        fig1.append_trace(go.Scatter(x=v_data["time (s)"], y=v_data["force (lb)"], name="force (lb)"), 1, 1)
        fig1.append_trace(
            go.Scatter(x=v_data["time (s)"], y=np.sqrt(np.square(v_data["force (lb)"] - np.squeeze(yout))),
                       name="error"),
            1, 1)
        fig1.append_trace(go.Scatter(x=v_data["time (s)"], y=v_data["input (us)"] / 30, name="u"), 1, 1)

        plt.plot(fig1)

plt.show()
