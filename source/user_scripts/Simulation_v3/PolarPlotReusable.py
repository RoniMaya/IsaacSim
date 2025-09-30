from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
import numpy as np

class PolarPlotReusable:
    def __init__(self, size=(220,220), r_max=200, color ='r'):
        w, h = size
        dpi = 100
        self.fig = Figure(figsize=(w/dpi, h/dpi), dpi=dpi)
        self.fig.patch.set_alpha(0)  # transparent figure background
        self.canvas = FigureCanvas(self.fig)

        self.ax = self.fig.add_subplot(111, projection='polar')
        self.ax.set_facecolor('none')       # transparent polar background
        self.ax.grid(True, alpha=1.0, color='white', linestyle='--', linewidth=0.8)

        # hide labels and polar spine
        self.ax.set_xticklabels([])
        self.ax.set_yticklabels([])
        self.ax.spines['polar'].set_visible(False)

        # fix radial limits
        self.ax.set_rlim(0, r_max)

        # Create an empty scatter, store reference
        self.scat = self.ax.scatter([], [], s=12, c=color, alpha=0.95)

        # First draw: ensures grid is rasterized
        self.canvas.draw()
        self.background = self.canvas.copy_from_bbox(self.ax.bbox)

    def update(self, az_deg, r_m, snr=None):
        theta = np.deg2rad(az_deg)
        r = np.asarray(r_m)

        # Update the scatter’s offsets (and colors if needed)
        offsets = np.column_stack([theta, r])
        self.scat.set_offsets(offsets)

        if snr is not None:
            self.scat.set_array(np.asarray(snr))  # colormap based on snr
        else:
            self.scat.set_array(None)

        # Efficient redraw: restore background (with white grid) and draw scatter
        self.canvas.restore_region(self.background)
        self.ax.draw_artist(self.scat)
        self.canvas.blit(self.ax.bbox)

        return np.asarray(self.canvas.buffer_rgba())
