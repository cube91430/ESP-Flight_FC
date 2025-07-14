import matplotlib.pyplot as plt
import matplotlib.transforms as mtransforms
import matplotlib
from matplotlib.widgets import Slider
import numpy as np

#initial parameters
plotx = 0
ploty = 5000

adc_min = 0
adc_max = 4095

ppm_min = 1080
ppm_max = 1720
ppm_center = 1488

amplitude_init = 1.0

def analog_to_ppm(value):
    return 0.1563 * value + ppm_min

an_val = np.linspace(adc_min, adc_max, 500)
ppm_val = np.linspace(ppm_min, ppm_max, 500)

mein_axis = np.linspace(plotx, ploty, 500)

#amp_ax = plt.axes([0.1, 0.1, 0.8, 0.05])
#amp_slider = Slider(amp_ax, 'Amplitude', 0.1, 2.0, valinit=amplitude_init)

plt.title("Analog to PPM value")


plt.plot(an_val, ppm_val)
#plt.plot(an_val, c="red", linewidth=2, label="ADC")
#plt.plot(ppm_val, c="blue", linewidth=2, label="ppm")

plt.annotate("full throttle (4095, 1720)", 
            xy=(4095, 1720), 
            xytext=(2700, 1720),
            arrowprops= dict(facecolor = 'green',
                             shrink= 0.01)
            )

plt.annotate("full throttle (4095, 1488)", 
            xy=(4095, 1488), 
            xytext=(2700, 1720),
            arrowprops= dict(facecolor = 'green',
                             shrink= 0.01)
            )

plt.xlabel("ppm")
plt.ylabel("adc")
plt.grid()
plt.legend()

plt.show()
