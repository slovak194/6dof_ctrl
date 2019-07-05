import os
import webbrowser

import plotly


class WebPlot:

    def __init__(self, file_name="plotly_plot.html"):
        self.file_name = file_name
        self.figs = []

    def plot(self, fig):
        self.figs += [fig]

    def show(self):
        with open(self.file_name, 'w') as of:
            of.write('<html><head>' +
                     '<script src="plotly-latest.min.js"></script>' +
                     '</head><body style="background-color:powderblue;">' + "\n")
            for f in self.figs:
                of.write(plotly.offline.plot(f, include_plotlyjs=False, output_type='div') + "\n")
            of.write("</body></html>")
        webbrowser.open(os.path.relpath(self.file_name))


def get_ts_scatter(ts, lmsg_name, ltrace_name, scaler=1.0):
    if "timestamp" in ts[lmsg_name].keys():
        return plotly.graph_objs.Scatter(
            x=(ts[lmsg_name]["timestamp"]),
            y=ts[lmsg_name][ltrace_name] * scaler,
            name=lmsg_name + ":" + ltrace_name,
            mode='lines+markers'
        )


def make_subplots(title, rows=1, cols=1):
    f = plotly.tools.make_subplots(rows=rows, cols=cols, print_grid=False)
    f["layout"]["title"]["text"] = title
    f["layout"]["paper_bgcolor"] = "rgb(0,0,0)"
    f["layout"]["plot_bgcolor"] = "rgb(0,0,0)"
    f["layout"]["legend"] = dict(orientation="h")
    f["layout"]["xaxis"] = dict(autorange=True)
    f["layout"]["xaxis"] = dict(tickformat=".3f")
    return f


def plot(x, y=''):
    plt = WebPlot("fast_plot.html")
    fig = make_subplots("fast_plot")

    if len(y) == 0:
        fig.append_trace(plotly.graph_objs.Scatter(
            y=x,
            name="fast_plot",
            mode='lines+markers'
        ), 1, 1)

    else:
        fig.append_trace(plotly.graph_objs.Scatter(
            x=x,
            y=y,
            name="fast_plot",
            mode='lines+markers'
        ), 1, 1)

    plt.plot(fig)
    plt.show()
