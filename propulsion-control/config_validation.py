import numpy as np

l = 2
w = 1
R = np.sqrt(l**2 + w**2) # unit placeholder

# for circular shape
def pos_on_circle(R,phi,theta=0):
  return np.array([R*np.cos(phi),R*np.sin(phi),R*np.sin(theta)])

# for rectangular shape (convert to cartesian)
def pos_on_rect(l=2,w=1,z=0):
    return np.array([l,w,z])

# thrust direction in XY
def dir_in_plane(phi):
  return np.array([np.cos(phi),np.sin(phi),0])

# thrust direction in Z
def dir_vertical(up=True):
  if up:
    return np.array([0,0,1])
  else:
    return np.array([0,0,-1])

# 3D thruster direction based on azimuthal and elevation angles
def thruster_direction(theta, phi):
    """
    Compute thruster unit direction vector in body frame (relative to CG/CoM)
    given azimuth (theta) and elevation (phi).
    
    theta = azimuth (rad), rotation about z-axis
    phi   = elevation (rad), rotation about y-axis

    if phi = 0 you get in-plane vector,
    if phi = +- pi/2 you get pure vertical vector (if theta = 0)
    """
    # Force direction in body frame
    x = np.cos(theta) * np.cos(phi)
    y = np.sin(theta) * np.cos(phi)
    z = -np.sin(phi)
    return np.array([x, y, z]) / np.linalg.norm([x, y, z])


class Motor:
  def __init__(self,pos: np.ndarray,theta=0,phi=0):
    
    self.pos = pos
    self.thrust_unit_vect = thruster_direction(theta,phi)

    # T200 motor specs

    # thrust values at diff voltages (N)
    self.t_fwd_12v = 36.383
    self.t_fwd_16v = 51.485
    self.t_fwd_20v = 65.705

    self.t_rev_12v = 28.635
    self.t_rev_16v = 40.207
    self.t_rev_20v = 49.524

    # current draw at diff voltages (A)
    self.i_12v = 17
    self.i_16v = 24
    self.i_20v = 32

    # prop diameter (mm)
    self.prop_dia = 76

    # rotor disk area (m^2)
    self.A = np.pi * np.pow((self.prop_dia/2e3),2) # convert mm to m


class Configuration:
  def __init__(self,motors: list =[],):
    self.motors = motors

    



def DOF_Analysis(config: Configuration):
  '''
  Returns number of controllable DOFs the configuration has
  Goal for project is 6 DOFs so control matrix needs atleast rank = 6
  Will also provide which dofs are uncontrollable via nullspace of control matrix

  '''
  cols = []
  for m in config.motors:
      torque = np.cross(m.pos, m.thrust_unit_vect)
      col = np.hstack([m.thrust_unit_vect, torque])
      cols.append(col)

  B = np.array(cols).T   # shape 6×n
  rank = np.linalg.matrix_rank(B)

  # compute nullspace to determine which dofs are uncontrollable
  u, s, vh = np.linalg.svd(B)
  null_mask = (s <= 1e-10)
  nullspace = vh[null_mask].T if np.any(null_mask) else None

  print("B:\n", B)
  print("Rank:", rank)
  if nullspace is not None:
    print("Nullspace (uncontrollable directions):\n", nullspace)
  return B, rank, nullspace






if __name__ == "__main__":
  six_motor_1 = Configuration(
    motors = [
        Motor(pos_on_circle(R,np.pi/4),   theta = 7*np.pi/4, phi = 0),
        Motor(pos_on_circle(R,7*np.pi/4), theta = np.pi/4, phi = 0),
        Motor(pos_on_circle(R,5*np.pi/4), theta = 3*np.pi/4, phi = 0),
        Motor(pos_on_circle(R,3*np.pi/4), theta = 5*np.pi/4,phi = 0),
        Motor(pos_on_circle(R,0),         theta = 0,phi = np.pi/2),   # vertical
        Motor(pos_on_circle(R,np.pi),     theta = 0, phi = -np.pi/2),   # vertical, opposite side
    ]
  )
  nine_motor_1 = Configuration(
    motors = [
      Motor(pos_on_circle(R, np.pi/4,np.pi/4), theta = 7*np.pi/4, phi = 0),
      Motor(pos_on_circle(R, 7*np.pi/4, np.pi/4), theta = np.pi/4, phi = 0),
      Motor(pos_on_circle(R, 5*np.pi/4, np.pi/4), theta = 3*np.pi/4, phi = 0),
      Motor(pos_on_circle(R, 3*np.pi/4, np.pi/4), theta = 5*np.pi/4, phi=0),
      Motor(pos_on_circle(R, np.pi/4, 7*np.pi/4), theta = 7*np.pi/4, phi = 0),
      Motor(pos_on_circle(R, 7*np.pi/4, 7*np.pi/4), theta = np.pi/4, phi = 0),
      Motor(pos_on_circle(R, 5*np.pi/4, 7*np.pi/4), theta = 3*np.pi/4, phi = 0),
      Motor(pos_on_circle(R, 3*np.pi/4, 7*np.pi/4), theta = 7*np.pi/4, phi = 0),
      Motor(pos_on_circle(0, 0, 0), theta = 0, phi = np.pi/2)
    ]
  )
  m2_pro_config = Configuration(
    motors = [
      # top
      Motor(pos_on_rect(l=0.5,w=-0.5,z=0.5), theta = np.pi/4, phi = np.pi/4),
      Motor(pos_on_rect(l=0.5,w=0.5,z=0.5), theta = 3*np.pi/4, phi = np.pi/4 ),
      Motor(pos_on_rect(l=-0.5,w=0.5,z=0.5), theta = 7*np.pi/4, phi = np.pi/4),
      Motor(pos_on_rect(l=-0.5,w=-0.5,z=0.5), theta = 5*np.pi/4, phi = np.pi/4),
      # bottom
      Motor(pos_on_rect(l=0.5,w=-0.5,z=-0.5), theta = np.pi/4, phi = -np.pi/4),
      Motor(pos_on_rect(l=0.5,w=0.5,z=-0.5), theta = 3*np.pi/4, phi = -np.pi/4 ),
      Motor(pos_on_rect(l=-0.5,w=0.5,z=-0.5), theta = 7*np.pi/4, phi = -np.pi/4),
      Motor(pos_on_rect(l=-0.5,w=-0.5,z=-0.5), theta = 5*np.pi/4, phi = -np.pi/4)
    ]
  )

  # DOF_Analysis(six_motor_1)
  # DOF_Analysis(nine_motor_1)
  DOF_Analysis(m2_pro_config)