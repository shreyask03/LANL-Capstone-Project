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

regimes = pd.read_csv("Operating Modes.csv")



def compute_max_velocities(length, width, height, thrust_per_motor,
                           Cd_forward, Cd_lateral, Cd_vertical,
                           vertical_cant_deg, outward_cant_deg,
                           rho=997.0):
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
    #length = float(input("Enter submarine length (m): "))
    #width = float(input("Enter submarine width (m): "))
    #height = float(input("Enter submarine height (m): "))
    length = .75
    width = .5
    height = .25
    throttle = int(input("Enter throttle percentage: "))
    #Cd_forward = float(input("Enter drag coefficient (forward): "))
    #Cd_lateral = float(input("Enter drag coefficient (lateral): "))
    #Cd_vertical = float(input("Enter drag coefficient (vertical): "))
    Cd_forward = .08
    Cd_lateral = .095
    Cd_vertical = .115
    
    thrust = result["Thrust Data 14 V.csv"][" Force (Kg f)"][(100+throttle)]

    
    pitch_angles = np.arange(0,91,1)
    bank_angles = np.arange(0,91,1)

    angle_thrusts = pd.DataFrame(0, index = pitch_angles, columns = bank_angles, dtype = object)

    for i in pitch_angles:
        for j in bank_angles:
            results = compute_max_velocities(length, width, height, thrust,
                                     Cd_forward, Cd_lateral, Cd_vertical,
                                     i, j)
            angle_thrusts.at[i, j] = [
                        round(float(results['forces_N']['forward']),3),
                        round(float(results['forces_N']['lateral']),3),
                        round(float(results['forces_N']['vertical']),3),
                        round(float(results['max_speeds_m_per_s']['forward']),3),
                        round(float(results['max_speeds_m_per_s']['lateral']),3),
                        round(float(results['max_speeds_m_per_s']['vertical']),3)
                    ]   
 
    distances = np.array([100,45])
    total_distance = np.linalg.norm(distances) / 0.4
    travel_time = total_distance / (3.281 * angle_thrusts[45][45][3] * .8 * 3600)

    current = result["Thrust Data 14 V.csv"][" Current (A)"][throttle + 100]

    current_drawn = travel_time * current * 4 / .7
    
    print(f"The Amperage required for this travel distance is {current_drawn:.5f} Ah")
