
import numpy as np
from scipy.integrate import odeint
import math
import pandas as pd



# Constants
a1, a2, a3, a4= 0.071, 0.057, 0.071,0.057
A1, A2, A3, A4 =28, 32, 28 ,32
g=981
g1=0.7
g2=0.6
k1 , k2=3.33 , 3.35
kc=0.5

# Define the function representing the system of differential equations
def tank_system(h, t):
    u1 = 3.0  #  input flow rates
    u2 = 3.0

    dh1_dt = ((-1)*a1/A1) * (math.sqrt(2*g) )*(np.sqrt(h[0]))+((a3/A1) * (math.sqrt(2*g) )*(np.sqrt(h[2]))+(g1*k1*u1/A1))
    dh2_dt = ((-1)*a2/A2) * (math.sqrt(2*g) )*(np.sqrt(h[1]))+((a4/A2) * (math.sqrt(2*g) )*(np.sqrt(h[3]))+(g2*k2*u2/A2))
    dh3_dt = ((-1)*a3/A3) * (math.sqrt(2*g) )*(np.sqrt(h[2]))+((1-g2)*k2*u2/A3)
    dh4_dt =((-1)*a4/A4) * (math.sqrt(2*g) )*(np.sqrt(h[3]))+((1-g1)*k1*u1/A4)

    return [dh1_dt, dh2_dt, dh3_dt, dh4_dt]

# Initial conditions and time points
h0 = [12.4, 12.7, 1.8, 1.4]  # Initial heights
t = np.linspace(0, 10000, 10000)  # Time points

# Solve the ODE
sol = odeint(tank_system, h0, t)

# Extract the data
h1_data, h2_data, h3_data, h4_data = sol[:, 0], sol[:, 1], sol[:, 2], sol[:, 3]

# Store the data in a pandas DataFrame
data = {'Time': t, 'Height_1': h1_data, 'Height_2': h2_data, 'Height_3': h3_data, 'Height_4': h4_data}
df = pd.DataFrame(data)

# Export the data to an Excel file
df.to_excel("tank_data.xlsx", index=False)