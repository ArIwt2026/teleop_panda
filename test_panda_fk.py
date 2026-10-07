import numpy as np

def rpy_to_rot(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    # R = Rz(y) * Ry(p) * Rx(r)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx

def make_transform(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_to_rot(rpy[0], rpy[1], rpy[2])
    T[:3, 3] = xyz
    return T

class PandaKinematics:
    def __init__(self, tcp_offset=np.array([0.0, 0.0, 0.1034])):
        # Joint definitions from panda_arm.xacro: (xyz, rpy)
        self.joint_transforms = [
            make_transform([0.0, 0.0, 0.333], [0.0, 0.0, 0.0]),
            make_transform([0.0, 0.0, 0.0], [-np.pi/2, 0.0, 0.0]),
            make_transform([0.0, -0.316, 0.0], [np.pi/2, 0.0, 0.0]),
            make_transform([0.0825, 0.0, 0.0], [np.pi/2, 0.0, 0.0]),
            make_transform([-0.0825, 0.384, 0.0], [-np.pi/2, 0.0, 0.0]),
            make_transform([0.0, 0.0, 0.0], [np.pi/2, 0.0, 0.0]),
            make_transform([0.088, 0.0, 0.0], [np.pi/2, 0.0, 0.0]),
        ]
        # Link 7 to Link 8 (Flange)
        self.T_7_8 = make_transform([0.0, 0.0, 0.107], [0.0, 0.0, 0.0])
        # Flange to TCP
        self.T_8_tcp = np.eye(4)
        self.T_8_tcp[:3, 3] = tcp_offset

    def forward_kinematics(self, q):
        T = np.eye(4)
        joint_origins = []
        joint_axes = []
        for i in range(7):
            T_fixed = self.joint_transforms[i]
            # Rotation about z_i
            c, s = np.cos(q[i]), np.sin(q[i])
            R_z = np.array([
                [c, -s, 0.0, 0.0],
                [s,  c, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0]
            ])
            T = T @ T_fixed
            joint_origins.append(T[:3, 3].copy())
            joint_axes.append(T[:3, 2].copy())
            T = T @ R_z
        
        T_tcp = T @ self.T_7_8 @ self.T_8_tcp
        p_tcp = T_tcp[:3, 3]
        
        # Compute geometric Jacobian in base frame
        J = np.zeros((6, 7))
        for i in range(7):
            z_i = joint_axes[i]
            p_i = joint_origins[i]
            J[:3, i] = np.cross(z_i, p_tcp - p_i)
            J[3:, i] = z_i
            
        return T_tcp, J

pk = PandaKinematics()
q_test = np.array([0.0, -np.pi/4, 0.0, -3*np.pi/4, 0.0, np.pi/2, np.pi/4])
T_tcp, J = pk.forward_kinematics(q_test)
print("TCP Position:", np.round(T_tcp[:3, 3], 4))
print("Jacobian shape:", J.shape)
print("Jacobian condition number:", np.linalg.cond(J))
