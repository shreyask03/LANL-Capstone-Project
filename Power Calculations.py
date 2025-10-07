import pandas as pd
import matplotlib.pyplot as plt
import glob
import os
import numpy as np

def load_csv_files(folder_path):

    # Find all CSV files in the folder
    csv_files = glob.glob(os.path.join(folder_path, "*.csv"))

    # Store each DataFrame in a dictionary with filename as key
    dataframes = {os.path.basename(file): pd.read_csv(file,encoding = "unicode_escape") for file in csv_files}
    return dataframes


if __name__ == "__main__":
    folder = "Thruster Data"
    result = load_csv_files(folder)
    
    for name, df in result.items():

        df = df.drop(df.index[0:100])
        result[name] = df

for name,df in result.items():
    if " PWM (µs)" in df.columns:
        df[" PWM (µs)"] = (df[" PWM (µs)"] - 1500) / 4

        
def ThrustPlots(column):
    
    for name,df in result.items():
        if " PWM (µs)" in df.columns:
            plt.plot(df[" PWM (µs)"],df[column],label = name[-8:-4])
        plt.xlabel("Thrust %")
        plt.ylabel(column)
        plt.title(f"{column} vs Thrust Percentage")
    plt.legend()
    plt.show()

def plotter():
    ThrustPlots(" Power (W)")
    ThrustPlots(" RPM")
    ThrustPlots(" Efficiency (g/W)")
    ThrustPlots(" Force (Kg f)")


regimes = pd.read_csv("Operating Modes.csv")

def battery_Calcs():
    current_drawn = np.zeros(6)

    for i in range(len(regimes.columns)):
      thrusters = regimes["Thrusters In Use"][i]
        percentage = regimes["Thrust Percentage"][i]
        time = regimes["Time used (h)"][i]
        j = 0
        for name,df in result.items():
            current = df[" Current (A)"][percentage+100]
            current_drawn[j] += current * thrusters * time
            j += 1
        
    current_drawn /= .7
    V = 10
    for i in current_drawn:
        print(f"The Amperage required for a {V} V battery is {i:.1f} Ah")
        V += 2








