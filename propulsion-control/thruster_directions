import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# -----------------------------
# 1. Geometry and Thruster Setup
# -----------------------------
L, W, H = 0.3, 0.2, 0.15   # half dimensions (meters)
c = 1 / np.sqrt(3)          # 45° components in x, y, z

# Thruster positions and directions (45° in all axes)
thrusters = np.array([
    [ L,  W,  H,  c,  c,  c],
    [ L,  W, -H,  c,  c, -c],
    [ L, -W,  H,  c, -c,  c],
    [ L, -W, -H,  c, -c, -c],
    [-L,  W,  H, -c,  c,  c],
    [-L,  W, -H, -c,  c, -c],
    [-L, -W,  H, -c, -c,  c],
    [-L, -W, -H, -c, -c, -c],
])

# -----------------------------
# 2. Build Control Allocation Matrix B (6x8)
# -----------------------------
B = np.zeros((6, 8))
for i in range(8):
    x, y, z, tx, ty, tz = thrusters[i]
    B[:, i] = [tx, ty, tz,
               y*tz - z*ty,   # roll τx
               z*tx - x*tz,   # pitch τy
               x*ty - y*tx]   # yaw τz

print("=== Control Allocation Matrix B ===")
print(np.round(B, 3))
print("Rank(B):", np.linalg.matrix_rank(B))
print()

# -----------------------------
# 3. Input Thruster RPMs
# -----------------------------
# Positive = forward, Negative = reverse


f_surge = [ 0.577, 0.577, 0.577, 0.577, 0.577, 0.577, 0.577, 0.577 ]
f_sway = [ 0.577,  0.577, -0.577, -0.577,
           0.577,  0.577, -0.577, -0.577 ]
f_heave = [ 0.577, -0.577, 0.577, -0.577,
            0.577, -0.577, 0.577, -0.577 ]
f_roll = [  0.707, -0.707,  0.707, -0.707,
           -0.707,  0.707, -0.707,  0.707 ]
f_pitch = [  0.707,  0.707,  0.707,  0.707,
            -0.707, -0.707, -0.707, -0.707 ]
f_yaw = [  0.707, -0.707, -0.707,  0.707,
           0.707, -0.707, -0.707,  0.707 ]


f = np.array(f_sway)


# -----------------------------
# 4. Compute Resultant Forces/Torques
# -----------------------------
tau = B @ f
F = tau[:3]   # [Fx, Fy, Fz]
M = tau[3:]   # [τx, τy, τz]

print("=== Thruster Commands (RPMs) ===")
for i in range(8):
    print(f"T{i+1}: {f[i]:.2f}")
print()

print("=== Resultant Forces ===")
print(f"Fx = {F[0]:.3f}, Fy = {F[1]:.3f}, Fz = {F[2]:.3f}")
print("=== Resultant Torques ===")
print(f"Roll  (τx) = {M[0]:.3f}")
print(f"Pitch (τy) = {M[1]:.3f}")
print(f"Yaw   (τz) = {M[2]:.3f}")
print()

# -----------------------------
# 5. Interpret Motion
# -----------------------------
tol = 1e-6
motions = []
if abs(F[0]) > tol: motions.append("Surge (+x)")
if abs(F[1]) > tol: motions.append("Sway (+y)")
if abs(F[2]) > tol: motions.append("Heave (+z)")
if abs(M[0]) > tol: motions.append("Roll (about x)")
if abs(M[1]) > tol: motions.append("Pitch (about y)")
if abs(M[2]) > tol: motions.append("Yaw (about z)")

if len(motions) == 0:
    print("✅ No net motion (balanced thrusts)")
else:
    print("🧭 Motion components present:")
    for m in motions:
        print(" •", m)

# -----------------------------
# 6. Plot Thruster RPMs
# -----------------------------
plt.figure(figsize=(7,3))
thruster_ids = np.arange(1, 9)
colors = ['blue' if rpm >= 0 else 'red' for rpm in f]
plt.bar(thruster_ids, f, color=colors)
plt.axhline(0, color='black', linewidth=1)
plt.title("Thruster RPM Inputs (Blue = Forward, Red = Reverse)")
plt.xlabel("Thruster Number")
plt.ylabel("RPM (Relative Units)")
plt.xticks(thruster_ids)
plt.grid(True, linestyle='--', alpha=0.5)
plt.tight_layout()

# -----------------------------
# 7. Plot 3D ROV Body + Thrusters
# -----------------------------
fig = plt.figure(figsize=(7,6))
ax = fig.add_subplot(111, projection='3d')
ax.set_title("ROV Thruster Configuration and Directions")
ax.set_xlabel("X (Forward)")
ax.set_ylabel("Y (Right)")
ax.set_zlabel("Z (Down)")
ax.set_xlim([-0.4, 0.4])
ax.set_ylim([-0.4, 0.4])
ax.set_zlim([-0.4, 0.4])
ax.view_init(elev=20, azim=35)

# Draw box (rectangular prism)
corners = np.array([
    [ L,  W,  H],
    [ L, -W,  H],
    [-L, -W,  H],
    [-L,  W,  H],
    [ L,  W, -H],
    [ L, -W, -H],
    [-L, -W, -H],
    [-L,  W, -H]
])
faces = [
    [corners[0], corners[1], corners[2], corners[3]],
    [corners[4], corners[5], corners[6], corners[7]],
    [corners[0], corners[1], corners[5], corners[4]],
    [corners[2], corners[3], corners[7], corners[6]],
    [corners[1], corners[2], corners[6], corners[5]],
    [corners[0], corners[3], corners[7], corners[4]]
]
ax.add_collection3d(Poly3DCollection(faces, alpha=0.1, facecolor='gray', linewidths=1, edgecolor='black'))

# Draw thrusters
for i in range(8):
    x, y, z, tx, ty, tz = thrusters[i]
    sign = np.sign(f[i])
    color = 'blue' if sign >= 0 else 'red'
    length = 0.15 * abs(f[i])
    ax.scatter(x, y, z, color='black')
    ax.text(x+0.02, y+0.02, z+0.02, f"T{i+1}", fontsize=8)
    ax.quiver(x, y, z, tx*sign, ty*sign, tz*sign,
              length=length, color=color, normalize=True)

plt.tight_layout()
plt.show()

