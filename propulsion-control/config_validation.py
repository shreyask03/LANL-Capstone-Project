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
    self.theta = theta
    self.phi = phi
    self.thrust_unit_vect = thruster_direction(np.deg2rad(theta),np.deg2rad(phi))
    
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
  def __init__(self,motors: list =[]):
    self.motors = motors

    pos_all = np.array([m.pos for m in motors])
    max_bounds = np.max(np.abs(pos_all), axis=0)
    self.uuv_dims = max_bounds  # [L, W, H] half-dimensions
    
  
  def get_pos(self, motor_id: int):
    m = self.motors[motor_id]
    return m.pos
  
  def get_orient(self, motor_id: int):
    m = self.motors[motor_id]
    return (m.thrust_unit_vect)
  
  def get_uuv_dims(self):
    return self.uuv_dims

  def set_uuv_dims(self):
    l = float(input("Please enter the length of the UUV in meters: "))
    w = float(input("Please enter the width of the UUV in meters: "))
    h = float(input("Please enter the height of the UUV in meters: "))
    self.uuv_dims = np.array([l,w,h])

    # distances from center based on box dimensions
    l_c = self.uuv_dims[0] / 2
    w_c = self.uuv_dims[1] / 2
    h_c = self.uuv_dims[2] / 2

    for i in range(len(self.motors)):
      if i == 0:
        self.motors[i].pos = np.array([l_c, w_c, h_c])
      elif i == 1:
        self.motors[i].pos = np.array([l_c, w_c, -h_c])
      elif i == 2:
        self.motors[i].pos = np.array([l_c,-w_c,h_c])
      elif i == 3:
        self.motors[i].pos = np.array([l_c,-w_c,-h_c])
      elif i == 4:
        self.motors[i].pos = np.array([-l_c,w_c,h_c])
      elif i == 5:
        self.motors[i].pos = np.array([-l_c,w_c,-h_c])
      elif i == 6:
        self.motors[i].pos = np.array([-l_c,-w_c,h_c])
      elif i == 7:
        self.motors[i].pos = np.array([-l_c,-w_c,-h_c])

    # if(not polar):
    #   l = float(input("Please enter the longitudinal distance from center in meters: "))
    #   w = float(input("Please enter the lateral distance from center in meters: "))
    #   z = float(input("Please enter the vertical distance from center in meters: "))

    #   m.pos = np.array([l,w,z])

    # elif(polar):
    #   r = float(input("Please enter the radial distance from center in meters: "))
    #   theta = float(input("Please enter the angle from +x (from nose) in degrees: "))
    #   z = float(input("Please enter the vertical distance from center in meters: "))

    #   m.pos = polar_pos(r,theta,z)

      
  
  def set_orient(self, motor_id: int):
    m = self.motors[motor_id]
    print(
      "\nThese angles will be used to rotate a body-fixed frame about a stationary coordinate system where both share an origin at rotor center."
      "\nThe thruster wil start off facing forward along the UUV's +x axis.\n"
      "The UUV-fixed coordinate frame has an origin at its CG and is configured is as follows\n +X comes out front of UUV\n +Y comes out right side of UUV\n +Z comes out bottom of UUV\n"
    )

    theta = float(input(f"Please enter the desired angle of horizontal rotation from +x (forward) in degrees for thruster {motor_id}: "))
    phi = float(input(f"Please enter the desired angle of vertical rotation in degrees for thruster {motor_id}: "))

    
    
    m.thrust_unit_vect = thruster_direction(np.deg2rad(theta),np.deg2rad(phi))

def polar_pos(r,theta,z):
  angle = np.deg2rad(theta)
  return np.array([r*np.cos(angle), r*np.sin(angle),z])
  
    
def get_config():
  return Configuration(
    motors = [
      Motor(np.array([0.362,0.2075,0.1015]), theta = 315, phi = 45), # top front right
      Motor(np.array([0.362,-0.2075,0.1015]), theta = 45, phi = 45), # top front left
      Motor(np.array([0.362,0.2075,-0.1015]), theta = 315, phi = -45 ), # bottom front right
      Motor(np.array([0.362,-0.2075,-0.1015]), theta = 45, phi = -45), # bottom front left
      Motor(np.array([-0.362,0.2075,0.1015]), theta = 225, phi = 45), # top back right
      Motor(np.array([-0.362,-0.2075,0.1015]), theta = 135, phi = 45), # top back left
      Motor(np.array([-0.362,0.2075,-0.1015]), theta = 225, phi = -45 ), # bottom back right 
      Motor(np.array([-0.362,-0.2075,-0.1015]), theta = 135, phi = -45) # bottom back left
    ]
  )


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


# m2_pro_config = Configuration(
#     motors = [
#       # top
#       Motor(pos_on_rect(l=0.5,w=-0.5,z=0.5), theta = np.pi/4, phi = np.pi/4),
#       Motor(pos_on_rect(l=0.5,w=0.5,z=0.5), theta = 3*np.pi/4, phi = np.pi/4 ),
#       Motor(pos_on_rect(l=-0.5,w=0.5,z=0.5), theta = 7*np.pi/4, phi = np.pi/4),
#       Motor(pos_on_rect(l=-0.5,w=-0.5,z=0.5), theta = 5*np.pi/4, phi = np.pi/4),
#       # bottom
#       Motor(pos_on_rect(l=0.5,w=-0.5,z=-0.5), theta = np.pi/4, phi = -np.pi/4),
#       Motor(pos_on_rect(l=0.5,w=0.5,z=-0.5), theta = 3*np.pi/4, phi = -np.pi/4 ),
#       Motor(pos_on_rect(l=-0.5,w=0.5,z=-0.5), theta = 7*np.pi/4, phi = -np.pi/4),
#       Motor(pos_on_rect(l=-0.5,w=-0.5,z=-0.5), theta = 5*np.pi/4, phi = -np.pi/4)
#     ]
#   )


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
  
  # m2_pro_config = Configuration(
  #   motors = [
  #     # top
  #     Motor(pos_on_rect(l=0.5,w=-0.5,z=0.5), theta = np.pi/4, phi = np.pi/4),
  #     Motor(pos_on_rect(l=0.5,w=0.5,z=0.5), theta = 3*np.pi/4, phi = np.pi/4 ),
  #     Motor(pos_on_rect(l=-0.5,w=0.5,z=0.5), theta = 7*np.pi/4, phi = np.pi/4),
  #     Motor(pos_on_rect(l=-0.5,w=-0.5,z=0.5), theta = 5*np.pi/4, phi = np.pi/4),
  #     # bottom
  #     Motor(pos_on_rect(l=0.5,w=-0.5,z=-0.5), theta = np.pi/4, phi = -np.pi/4),
  #     Motor(pos_on_rect(l=0.5,w=0.5,z=-0.5), theta = 3*np.pi/4, phi = -np.pi/4 ),
  #     Motor(pos_on_rect(l=-0.5,w=0.5,z=-0.5), theta = 7*np.pi/4, phi = -np.pi/4),
  #     Motor(pos_on_rect(l=-0.5,w=-0.5,z=-0.5), theta = 5*np.pi/4, phi = -np.pi/4)
  #   ]
  # )

  
  # DOF_Analysis(six_motor_1)
  # DOF_Analysis(nine_motor_1)
  # DOF_Analysis(m2_pro_config)