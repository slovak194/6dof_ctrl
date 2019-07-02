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
