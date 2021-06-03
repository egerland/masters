from vrepAPI import vrep
import random


class Robot():
    def __init__(self, vrep_addr='127.0.0.1', vrep_port=19997):
        vrep.simxFinish(-1)
        self.default_opmode = vrep.simx_opmode_blocking
        self.clientID = vrep.simxStart(vrep_addr, vrep_port, True, True, 5000, 5)
        if self.clientID == -1:
            raise ValueError("Connection failed!")
        vrep.simxSynchronous(self.clientID, True)
        self.gyro = {
            'x': self._start_float_signal('gyroX'),
            'y': self._start_float_signal('gyroY'),
            'z': self._start_float_signal('gyroZ')
        }
        self.acc = {
            'x': self._start_float_signal('accX'),
            'y': self._start_float_signal('accY'),
            'z': self._start_float_signal('accZ')
        }
        self.wheel_handle = self._get_object_handle('JuntaRoda_MT')
        self.weight_handle = self._get_object_handle('JuntaGiros_MT')
        self.body_handle = self._get_object_handle('RoboGiros')
        self.target_handle = self._get_object_handle('Target')
        self.floor_handle = self._get_object_handle('Floor')
        self.get_robot_position()
        self.set_target([self.robot_position[0], self.robot_position[1], 0])
        self.started = False
        self.wheel_vel = 0
        self.update_wheel()
        self.weight_pos = 0
        self.update_weight()

    def _start_float_signal(self, name):
        returnCode, data = vrep.simxGetFloatSignal(self.clientID, name,
                                                   vrep.simx_opmode_streaming)
        return data

    def _get_float_signal(self, name):
        returnCode, data = vrep.simxGetFloatSignal(self.clientID, name,
                                                   self.default_opmode)
        return data

    def _get_object_handle(self, name):
        returnCode, handle = vrep.simxGetObjectHandle(self.clientID, name,
                                                      self.default_opmode)
        return handle

    def _set_joint_velocity(self, handle, val):
        returnCode = vrep.simxSetJointTargetVelocity(self.clientID, handle, val,
                                                     self.default_opmode)
        return returnCode

    def _set_joint_position(self, handle, val):
        returnCode = vrep.simxSetJointTargetPosition(self.clientID, handle, val,
                                                     self.default_opmode)
        return returnCode

    def _get_object_position(self, handle, relativeTo=-1):
        returnCode, position = vrep.simxGetObjectPosition(self.clientID, handle, relativeTo,
                                                          self.default_opmode)
        return position

    def _set_object_position(self, handle, position, relativeTo=-1):
        returnCode = vrep.simxSetObjectPosition(self.clientID, handle, relativeTo,
                                                position, self.default_opmode)
        return returnCode

    def _check_collision(self, handle1, handle2):
        returnCode, collisionState = vrep.simxCheckCollision(self.clientID, handle1, handle2,
                                                             self.default_opmode)
        return collisionState

    def update_wheel(self):
        self._set_joint_velocity(self.wheel_handle, self.wheel_vel)

    def update_weight(self):
        self._set_joint_position(self.weight_handle, self.weight_pos)

    def get_robot_position(self):
        self.robot_position = self._get_object_position(self.body_handle)

    def set_target(self, position):
        self.target_position = position
        self._set_object_position(self.target_handle, position)

    def get_gyro_data(self):
        self.gyro = {
            'x': self._get_float_signal('gyroX'),
            'y': self._get_float_signal('gyroY'),
            'z': self._get_float_signal('gyroZ')
        }
        return self.gyro

    def get_acc_data(self):
        self.acc = {
            'x': self._get_float_signal('accX'),
            'y': self._get_float_signal('accY'),
            'z': self._get_float_signal('accZ')
        }
        return self.acc

    def set_random_target(self):
        new_position = [
            self.robot_position[0],
            self.robot_position[1] + 2 * random.random() - 1,
            0
        ]
        self.set_target(new_position)

    def step(self, action):
        if not self.started:
            self.started = True
            vrep.simxSynchronous(self.clientID, True)
            vrep.simxStartSimulation(self.clientID, self.default_opmode)
        self.wheel_vel = action[0]
        self.weight_pos = action[1]
        self.update_weight()
        self.update_wheel()
        vrep.simxSynchronousTrigger(self.clientID)
        self.get_acc_data()
        self.get_gyro_data()
        self.get_robot_position()

    def stop(self):
        self.started = False
        vrep.simxStopSimulation(self.clientID, self.default_opmode)


if __name__ == '__main__':
    robot = Robot()
    print('done')