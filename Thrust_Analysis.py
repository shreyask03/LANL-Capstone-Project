import math

def compute_max_velocities(length, width, height, thrust_per_motor,
                           Cd_forward, Cd_lateral, Cd_vertical,
                           vertical_cant_deg, outward_cant_deg,
                           rho=1025.0,):
    # convert to rads
    v_ang = math.radians(vertical_cant_deg)
    o_ang = math.radians(outward_cant_deg)
    
    # Singular motor thrust by cant
    forward_comp = thrust_per_motor * math.cos(v_ang) * math.cos(o_ang)
    lateral_comp = thrust_per_motor * math.cos(v_ang) * math.sin(o_ang)
    vertical_comp = thrust_per_motor * math.sin(v_ang)
    
    # total force for each direction using 4 motors
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
    print("=== 4-Motor Submarine Calculator ===")
    length = float(input("Enter submarine length (m): "))
    width = float(input("Enter submarine width (m): "))
    height = float(input("Enter submarine height (m): "))
    thrust = float(input("Enter thrust per motor (N): "))
    Cd_forward = float(input("Enter drag coefficient (forward): "))
    Cd_lateral = float(input("Enter drag coefficient (lateral): "))
    Cd_vertical = float(input("Enter drag coefficient (vertical): "))
    vertical_cant = float(input("Enter vertical cant angle (deg): "))
    outward_cant = float(input("Enter outward cant angle (deg): "))
    
    results = compute_max_velocities(length, width, height, mass, thrust,
                                     Cd_forward, Cd_lateral, Cd_vertical,
                                     vertical_cant, outward_cant)
    
    print("\n=== Results ===")
    print(f"Forward thrust (N): {results['forces_N']['forward']:.2f}")
    print(f"Lateral thrust (N): {results['forces_N']['lateral']:.2f}")
    print(f"Vertical thrust (N): {results['forces_N']['vertical']:.2f}")
    print("\n--- Max Speeds ---")
    print(f"Forward (m/s): {results['max_speeds_m_per_s']['forward']:.2f}")
    print(f"Lateral (m/s): {results['max_speeds_m_per_s']['lateral']:.2f}")
    print(f"Vertical (m/s): {results['max_speeds_m_per_s']['vertical']:.2f}")