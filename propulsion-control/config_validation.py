import numpy as np

l = 2
w = 1
R = np.sqrt(l**2 + w**2) # unit placeholder

# for circular shape
def pos_on_circle(R,phi,z=0):
  return np.array([R*np.cos(phi),R*np.sin(phi),z])

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



class Motor:
  def __init__(self,pos,dir):
    
    self.pos = pos
    self.thrust_unit_vect = dir / np.linalg.norm(dir)

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

    


# determines how many dofs the configuration has (goal is rank=6)
def DOF_counter(config: Configuration):

    cols = []
    for m in config.motors:
        torque = np.cross(m.pos, m.thrust_unit_vect)
        col = np.hstack([m.thrust_unit_vect, torque])
        cols.append(col)

    B = np.array(cols).T   # shape 6×n

    rank = np.linalg.matrix_rank(B)
    print("B:\n", B)
    print("Rank:", rank)






if __name__ == "__main__":
  six_motor_1 = Configuration(
    motors = [
        Motor(pos_on_circle(R,np.pi/4),   dir_in_plane(7*np.pi/4)),
        Motor(pos_on_circle(R,7*np.pi/4), dir_in_plane(np.pi/4)),
        Motor(pos_on_circle(R,5*np.pi/4), dir_in_plane(3*np.pi/4)),
        Motor(pos_on_circle(R,3*np.pi/4), dir_in_plane(5*np.pi/4)),
        Motor(pos_on_circle(R,0),         dir_vertical()),   # vertical
        Motor(pos_on_circle(R,np.pi),     dir_vertical(up=False)),   # vertical, opposite side
    ]
  )
  six_motor_2 = Configuration(
    motors=[
      Motor(pos_on_circle(R,np.pi/4),   dir_in_plane(7*np.pi/4)),
      Motor(pos_on_circle(R,7*np.pi/4), dir_in_plane(np.pi/4)),
      Motor(pos_on_circle(R,5*np.pi/4), dir_in_plane(3*np.pi/4)),
      Motor(pos_on_circle(R,3*np.pi/4), dir_in_plane(5*np.pi/4)),
      Motor(pos_on_circle(2*R/3,0),         dir_vertical()),   # vertical, offset y = +0.5
      Motor(pos_on_circle(2*R/3,np.pi),     dir_vertical(up=False))   # vertical, opposite side, offset y = -0.5
    ]
  )
  
  DOF_counter(six_motor_2)