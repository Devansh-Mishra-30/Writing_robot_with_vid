import numpy as np
from numpy.linalg import norm, solve
import pinocchio as pin
from scipy.spatial.transform import Rotation as R


class PandaMechanics():
    def __init__(self):
        self.model =  pin.buildModelFromUrdf('urdfs/panda_arm_no_hand.urdf')
        self.data = self.model.createData()


    def solve_ik(self, q:np.array, x:float, y:float, z:float) -> np.array:
        """
        Given the current joint positions (q in radians) and the desired 
        cartesian positions, returns the end effector position.
        The orientation of the end effector is downward.
        """

        JOINT_ID = self.model.njoints - 1
        oMdes = pin.SE3( np.array([ # Rotation matrix (facing down)
                            [1, 0, 0],
                            [0, -1, 0],
                            [0, 0, -1]
                        ]), 
                        np.array([x, y,  z]) # EE position
                        )

        eps    = 1e-4
        IT_MAX = 10000
        DT     = 1e-1
        damp   = 1e-12

        i=0
        while True:
            pin.forwardKinematics(self.model, self.data,q)
            dMi = oMdes.actInv(self.data.oMi[JOINT_ID])
            err = pin.log(dMi).vector
            if norm(err) < eps:
                success = True
                break
            if i >= IT_MAX:
                success = False
                break
            J = pin.computeJointJacobian(self.model,self.data,q,JOINT_ID)
            v = - J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
            q = pin.integrate(self.model,q,v*DT)
            # if not i % 10:
            #     print('%d: error = %s' % (i, err.T))
            i += 1

        if not success:
            print("\nWarning: the iterative algorithm has not reached convergence to the desired precision")

        # print('\nresult: %s' % q.flatten())
        # print('\nfinal error: %s' % err.T)

        return np.array(q.flatten())


    def solve_fk(self, q:np.array) -> np.array:
        """
        Given the current joint positions, returns
        the end effector position x/y/z (in meters) and roll/pitch/yaw (in radians)
        """

        pin.forwardKinematics(self.model, self.data, q)

        oMi = self.data.oMi[-1]  # Transformation of the last link
        position = oMi.translation.T.flat
        rotation_matrix = oMi.rotation

        # Convert the rotation matrix to roll, pitch, yaw (RPY)
        rpy = R.from_matrix(rotation_matrix).as_euler('xyz', degrees=False)
        
        return np.hstack((position, rpy))
    
    def solve_fk_velocity(self, q: np.array, dq: np.array) -> np.array:
        """
        Given the current joint positions and velocities,
        returns the end-effector velocity (linear and angular).
        """
        J = self.get_Jacobian(q)
        velocity = J @ dq

        return velocity  

    def get_M(self, q:np.array) -> np.array:
        """
        Returns M(q), the joint space inertia matrix.
        """

        M = pin.crba(self.model, self.data, q)
        return M
    

    def get_C(self, q:np.array, dq:np.array) -> np.array:
        """
        Returns C(q, qdot), the Coriolis matrix.
        """

        C = pin.computeCoriolisMatrix(self.model, self.data, q, dq)
        return C
    

    def get_G(self, q:np.array) -> np.array:
        """
        Returns G(q), the generalized gravity torque vector.
        """
        G = pin.computeGeneralizedGravity(self.model, self.data, q)
        return G


    def get_Jacobian(self, q: np.array) -> np.array:
        """
        Returns the Jacobian of the end effector.
        """
        JOINT_ID = self.model.njoints - 1
        J = pin.computeJointJacobian(self.model, self.data, q, JOINT_ID)
        return J
    




if __name__ == "__main__":
    # Example usage of This class

    q = np.array([0, -0.785, 0, -2.355, 0, 1.57, 0.785])
    dq = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

    panda_mechanics = PandaMechanics()

    print("FK:", panda_mechanics.solve_fk(q))
    print("IK:", panda_mechanics.solve_ik(q=q, x=0.31, y=0.00, z=0.5))

    print("M:", panda_mechanics.get_M(q))
    print("C:", panda_mechanics.get_C(q, dq))
    print("G:", panda_mechanics.get_G(q))

