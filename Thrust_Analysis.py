import math
import pandas as pd
import matplotlib.pyplot as plt
import glob
import os
import numpy as np

# File reading from Power Calculations.py
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
        df[" PWM (µs)"] = (df[" PWM (µs)"] - 1500) / 4


def compute_max_velocities(length, width, height, thrust_per_motor,
                           Cd_forward, Cd_lateral, Cd_vertical,
                           vertical_cant_deg, outward_cant_deg,
                           rho=1025.0):
    # convert to rads
    v_ang = math.radians(vertical_cant_deg)
    o_ang = math.radians(outward_cant_deg)
    
    # Singular motor thrust by cant in newtons
    forward_comp = thrust_per_motor * 9.807 * math.cos(v_ang) * math.cos(o_ang)
    lateral_comp = thrust_per_motor * 9.807 * math.cos(v_ang) * math.sin(o_ang)
    vertical_comp = thrust_per_motor * 9.807 * math.sin(v_ang)
    

    total_forward = 4 * forward_comp
    total_lateral = 4 * lateral_comp
    total_vertical = 4 * vertical_comp
    
    # Area in each direction
    A_forward = height * width
    A_lateral = height * length
    A_vertical = length * width
    
    # Calculate maximum speed
    def max_speed_from_force(F, A, Cd):
        if F <= 0:
            return 0.0
        return math.sqrt(2.0 * F / (rho * Cd * A))
    
    v_forward = max_speed_from_force(total_forward, A_forward, Cd_forward)
    v_lateral = max_speed_from_force(total_lateral, A_lateral, Cd_lateral)
    v_vertical = max_speed_from_force(total_vertical, A_vertical, Cd_vertical)
    
    return {
        "forces_N": {
            "forward": total_forward,
            "lateral": total_lateral,
            "vertical": total_vertical
        },
        "max_speeds_m_per_s": {
            "forward": v_forward,
            "lateral": v_lateral,
            "vertical": v_vertical
        }
    }

while True:
    print("Inputs:")
    length = float(input("Enter submarine length (m): "))
    width = float(input("Enter submarine width (m): "))
    height = float(input("Enter submarine height (m): "))
    throttle = int(input("Enter throttle percentage: "))
    Cd_forward = float(input("Enter drag coefficient (forward): "))
    Cd_lateral = float(input("Enter drag coefficient (lateral): "))
    Cd_vertical = float(input("Enter drag coefficient (vertical): "))
    vertical_cant = float(input("Enter vertical cant angle (deg): "))
    outward_cant = float(input("Enter outward cant angle (deg): "))
    
    thrust = result["Thrust Data 14 V.csv"][" Force (Kg f)"][(100+throttle)]

    results = compute_max_velocities(length, width, height, thrust,
                                     Cd_forward, Cd_lateral, Cd_vertical,
                                     vertical_cant, outward_cant)
    
    pitch_angles = np.arange(0,91,5)
    bank_angles = np.arange(0,91,5)

    angle_thrusts = pd.DataFrame(0, index = pitch_angles, columns = bank_angles, dtype = object)

    for i in pitch_angles:
        for j in bank_angles:
            angle_thrusts.at[i, j] = [
                        float(results['forces_N']['forward']),
                        float(results['forces_N']['lateral']),
                        float(results['forces_N']['vertical']),
                        float(results['max_speeds_m_per_s']['forward']),
                        float(results['max_speeds_m_per_s']['lateral']),
                        float(results['max_speeds_m_per_s']['vertical'])
                    ]    
    print(angle_thrusts)

