from battmodel import BattModel

from visual import *
from visual.graph import *

model = BattModel()

volt_plot = gcurve(color=color.red)
curr_plot = gcurve(color=color.green)

dt = 1.0/60.0
t = 0
while(True):
    rate(60)
    volt_plot.plot(pos=(t,model.current_meas(t)))
    curr_plot.plot(pos=(t,model.voltage_meas(t)))
    t+=dt
