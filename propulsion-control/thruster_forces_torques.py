import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import pandas as pd
from config_validation import get_config

config = get_config()

print(f"The current craft dimensions are: {config.get_uuv_dims()}")
print(f"The current positions and orientations of the thrusters are as follows:")
for i in range(len(config.motors)):
    print(f"Thruster {i}:\n position: {config.get_pos(i)}\n orientation: {config.get_orient(i)}\n")

config_ok = input("Would you like to keep the current configuration? (enter 'y' or 'n'): ")

# set position and orientation of thrusters if requested
if config_ok != "y":
    config.set_uuv_dims()

    for i in range(len(config.motors)):
        config.set_orient(i)
    
# create thruster matrix of positions and orients.  

thrusters = []
for i in range(len(config.motors)):
    pos = config.get_pos(i)
    thrust_vect = config.get_orient(i)
    # print(f"thrust vector: {thrust_vect}, dims: {np.ndim(thrust_vect)}")
    thruster = np.concatenate([pos,thrust_vect])
    thrusters.append(thruster)

thrusters = np.vstack(thrusters)
# print(thrusters)

# geometry
L,W,H = config.uuv_dims


# # === Geometry ===
# L, W, H = 0.3, 0.2, 0.15

# # cant angle
# theta = np.radians(45)  # cant angle from horizontal
# thetav = np.radians(45) # vertical cant angle
# cv  = np.cos((np.pi/2) - thetav)
# v = np.cos(thetav)        # horizontal projection magnitude
# h = np.sin(theta)        # vertical magnitude


# # Top canted down and in, bottom canted up and in
# thrusters = np.array([
#     [ L,  W,  H,  h, -h, -v],   # 1 front-top-right → down & in
#     [ L, -W,  H,  h,  h, -v],   # 2 front-top-left  → down & in
#     [ L,  W, -H,  h, -h,  v],   # 3 front-bottom-right → up & in
#     [ L, -W, -H,  h,  h,  v],   # 4 front-bottom-left  → up & in
#     [-L,  W,  H, -h, -h, -v],   # 5 rear-top-right → down & in
#     [-L, -W,  H, -h,  h, -v],   # 6 rear-top-left  → down & in
#     [-L,  W, -H, -h, -h,  v],   # 7 rear-bottom-right → up & in
#     [-L, -W, -H, -h,  h,  v],   # 8 rear-bottom-left  → up & in
# ])

# === T200 constants ===
rho = 1025.0
D = 0.076
K_T = 0.12
K_Q = 0.017
RPM_max = 3250
n_max = RPM_max / 60

# === Thrust and spin torque magnitudes per thruster ===
T = K_T * rho * (n_max**2) * (D**4)   # thrust [N]
Q = K_Q * rho * (n_max**2) * (D**5)   # torque [N·m]

spin_dirs = np.array([+1, -1, -1, +1, -1, +1, +1, -1])


# === B matrix ===
B = np.zeros((6, 8))
for i in range(8):
    x, y, z, tx, ty, tz = thrusters[i]
    roll_moment  = (y * tz - z * ty) / W
    pitch_moment = (z * tx - x * tz) / L
    yaw_moment   = (x * ty - y * tx) / H
    B[:, i] = [tx, ty, tz, roll_moment, pitch_moment, yaw_moment]

# === DOFs ===
targets = {
    "Surge (+X)": np.array([1, 0, 0, 0, 0, 0]),
    "Sway (+Y)" : np.array([0, 1, 0, 0, 0, 0]),
    "Heave (+Z)": np.array([0, 0, 1.41, 0, 0, 0]),
    "Roll"      : np.array([0, 0, 0, 1, 0, 0]),
    "Pitch"     : np.array([0, 0, 0, 0, 1, 0]),
    "Yaw"       : np.array([0, 0, 0, 0, 0, 1]),
}

# === solver ===
def forward_only_solve(B, tau_des, iters=6):
    f = np.zeros(B.shape[1])
    residual = tau_des.copy()
    for _ in range(iters):
        delta, _, _, _ = np.linalg.lstsq(B, residual, rcond=None)
        f += delta
        f = np.clip(f, 0, 1) # forward thrusters only
        residual = tau_des - B @ f
    return f, B @ f

# === solve all DOFs ===
results = {}
for name, tau_des in targets.items():
    f, tau_actual = forward_only_solve(B, tau_des)
    # f /= np.max(f) if np.max(f) > 0 else 1
    results[name] = (f, tau_actual)




# === Plots ===
fig = plt.figure(figsize=(14,10))
names = list(results.keys())

# spin directions
spin_dirs = np.array([+1, -1, -1, +1, -1, +1, +1, -1])

for i, name in enumerate(names):
    ax = fig.add_subplot(2, 3, i+1, projection='3d')
    f, _ = results[name]
    ax.set_title(name)
    ax.set_xlim([-1.5*L, 1.5*L])
    ax.set_ylim([-1.5*W, 1.5*W])
    ax.set_zlim([-1.5*H, 1.5*H])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.view_init(elev=20, azim=35)

    # hull box
    corners = np.array([
        [ L,  W,  H], [ L, -W,  H], [-L, -W,  H], [-L,  W,  H],
        [ L,  W, -H], [ L, -W, -H], [-L, -W, -H], [-L,  W, -H]
    ])
    faces = [
        [corners[0], corners[1], corners[2], corners[3]],
        [corners[4], corners[5], corners[6], corners[7]],
        [corners[0], corners[1], corners[5], corners[4]],
        [corners[2], corners[3], corners[7], corners[6]],
        [corners[1], corners[2], corners[6], corners[5]],
        [corners[0], corners[3], corners[7], corners[4]]
    ]
    ax.add_collection3d(Poly3DCollection(faces, alpha=0.1, facecolor='gray', edgecolor='black'))

    # arrows
    for j in range(8):
        x, y, z, tx, ty, tz = thrusters[j]
        thrust_dir = np.array([tx, ty, tz])
        n = f[j] * n_max
        F_mag = K_T * rho * (n**2) * (D**4)
        Q_mag = K_Q * rho * (n**2) * (D**5)
        F_vec = F_mag * thrust_dir
        tau_spin = spin_dirs[j] * Q_mag * thrust_dir

        if np.linalg.norm(F_vec) > 0:
            thrust_dir_norm = F_vec / np.linalg.norm(F_vec)
            spin_dir_norm = tau_spin / (np.linalg.norm(tau_spin) + 1e-9)

            arrow_scale = 0.25
            spin_scale = 0.15

            ax.quiver(x, y, z, *thrust_dir_norm,
                      length=arrow_scale, color='royalblue', lw=2.0)
            ax.quiver(x, y, z, *spin_dir_norm,
                      length=spin_scale, color='red', lw=1.6)

        ax.scatter(x, y, z, color='black', s=20)
        ax.text(x+0.02, y+0.02, z+0.02, f"T{j+1}", fontsize=7)

# legend
legend_elements = [
    Line2D([0], [0], color='royalblue', lw=2, label='Thrust Vector (T200)'),
    Line2D([0], [0], color='red', lw=2, label='Spin Reaction Torque'),
    Line2D([0], [0], marker='o', color='w', markerfacecolor='black', markersize=6, label='Thruster Position')
]
fig.legend(handles=legend_elements, loc='upper right', fontsize=9)

plt.suptitle("3D Thruster Directions & Moment Vectors (All Inward Cant, T200)", fontsize=15, y=0.985)
plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.subplots_adjust(hspace=0.3)
plt.show()





# === Resulting Force & Moment Table (Correct Projection Physics) ===
print("\n=== Resulting Force & Moment Table (Including T200 Spin Torques) ===\n")

# Use empirical max thrust
T = 65.8  # [N] from Blue Robotics datasheet
Q = 0.017 * rho * (n_max**2) * (D**5)  # keep same torque formula

theta = np.radians(45)
thetav = np.radians(45)
ch = np.cos(theta)  # 0.7071
chv = np.cos(thetav)
cvv  = np.cos((np.pi/2) - thetav)

# Active thrusters per DOF
active_thrusters = {
    "Surge (+X)": [1, 2, 3, 4],
    "Sway (+Y)" : [2, 4, 6, 8],
    "Heave (+Z)": [3, 4, 7, 8],
    "Roll"      : [2, 3, 6, 7],
    "Pitch"     : [1, 2, 7, 8],
    "Yaw"       : [2, 4, 5, 7]
}

data_full = []

for name, actives in active_thrusters.items():
    total_force = np.zeros(3)
    total_tau_force = np.zeros(3)
    total_tau_spin = np.zeros(3)

    for tnum in actives:
        j = tnum - 1
        x, y, z, tx, ty, tz = thrusters[j]
        r_vec = np.array([x, y, z])

        # --- Proper projection definitions ---
        Fx = np.sign(tx) * T * (ch * chv)   # cos45 * cos45
        Fy =  np.sign(ty) * T * (ch * chv)
        Fz =  np.sign(tz) * T * cvv         # single cos45 for vertical

        F_vec = np.array([Fx, Fy, Fz])
        tau_force = np.cross(r_vec, F_vec)
        
    
        tau_spin_x = spin_dirs[j] * Q * np.sign(tx) * (ch*chv)
        tau_spin_y = spin_dirs[j] * Q * np.sign(ty) * (ch*chv)
        tau_spin_z = spin_dirs[j] * Q * np.sign(tz) * (cvv)
        tau_spin = np.array([tau_spin_x,tau_spin_y,tau_spin_z])

        total_force += F_vec
        total_tau_force += tau_force
        total_tau_spin += tau_spin

    total_tau = total_tau_force + total_tau_spin

    data_full.append([
        name,
        *np.round(total_force, 3),
        *np.round(total_tau_force, 6),
        *np.round(total_tau_spin, 6),
        *np.round(total_tau, 6)
    ])

df_full = pd.DataFrame(data_full, columns=[
    "Case", "Fx [N]", "Fy [N]", "Fz [N]",
    "Mx_force [N·m]", "My_force [N·m]", "Mz_force [N·m]",
    "Mx_spin [N·m]", "My_spin [N·m]", "Mz_spin [N·m]",
    "Mx_total [N·m]", "My_total [N·m]", "Mz_total [N·m]"
])

print(df_full.to_string(index=False))
