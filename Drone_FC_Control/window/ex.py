import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Initial parameters
amplitude_init = 1.0
frequency = 1.0
x = np.linspace(0, 10, 1000)
y = amplitude_init * np.sin(2 * np.pi * frequency * x)

# Create the plot
fig, ax = plt.subplots()
plt.subplots_adjust(left=0.1, bottom=0.25)
line, = ax.plot(x, y)
ax.set_title("Adjustable Amplitude using Slider")

# Add a slider axis below the plot
amp_ax = plt.axes([0.1, 0.1, 0.8, 0.05])  # [left, bottom, width, height]
amp_slider = Slider(amp_ax, 'Amplitude', 0.1, 2.0, valinit=amplitude_init)

# Define the update function
def update(val):
    new_amp = amp_slider.val
    line.set_ydata(new_amp * np.sin(2 * np.pi * frequency * x))
    fig.canvas.draw_idle()

# Connect the slider to the update function
amp_slider.on_changed(update)

plt.show()
