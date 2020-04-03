import numpy as np
import math

def body_to_world_all_rotation_mat(reference_point, euler, p):
    
    yaw, pitch, roll = euler

    roll = roll * (math.pi/180)
    pitch = pitch * (math.pi/180)
    yaw = yaw * (math.pi/180)
    sin_roll, cos_roll = math.sin(roll), math.cos(roll)
    sin_pitch, cos_pitch = math.sin(pitch), math.cos(pitch)
    sin_yaw, cos_yaw = math.sin(yaw), math.cos(yaw)
    
    rot_z = np.array([[cos_yaw, -1 * sin_yaw, 0],
                    [sin_yaw, cos_yaw, 0],
                    [0, 0, 1]])
    rot_y = np.array([[cos_pitch, 0, sin_pitch],
                    [0, 1, 0],
                    [-1 * sin_pitch, 0, cos_pitch]])
    rot_x = np.array([[1, 0, 0],
                    [0, cos_roll, -1 * sin_roll],
                    [0, sin_roll, cos_roll]])
    
    composed_rotation_zyx = np.matmul(np.matmul(rot_z, rot_y), rot_x)
    print("before composed_rotation_zyx", composed_rotation_zyx.shape, composed_rotation_zyx)

    # Translation
    p = np.asarray(p)
    p.resize(3, 1)
    composed_rotation_zyx = np.column_stack((composed_rotation_zyx, p)) # stack point p against matrix
    perspective = np.array([0,0,0,1])
    composed_rotation_zyx = np.vstack((composed_rotation_zyx, perspective)) # stack perspective matrix on bottom of matrix
    reference_point.append(1)
    print("after composed_rotation_zyx", composed_rotation_zyx.shape, composed_rotation_zyx) 
    


    # calculate v' = R * v
    reference_prime = np.matmul(composed_rotation_zyx, reference_point)

    # round values below epsilon to 0
    eps = 1e-15
    reference_prime[np.abs(reference_prime) < eps] = 0
    
    return reference_prime

def body_to_world_all_quaternion(reference_point, euler, p):

    px, py, pz = p
    yaw, pitch, roll = euler

    # translation of reference point to desired point p
    reference_point.append(1) # add 1 to reference point
    reference_point = body_to_world_3d_translation(reference_point, px, py, pz)
    
    # the following implementation is implemented for rotations in order: ZYX [Trait-Bryan Angles]
    # convert euler angles to corresponding quaternion
    # TODO: enforce conditions: must be unit quaternions
    quaternion_yaw = euler_angle_to_quaternion(yaw, [0,0,1])
    quaternion_pitch = euler_angle_to_quaternion(pitch, [0,1,0])
    quaternion_roll = euler_angle_to_quaternion(roll, [1,0,0])
    
    # compose yaw, pitch, roll quaternions
    composed_rotation_zyx = quaternion_multiply(quaternion_multiply(quaternion_yaw, quaternion_pitch), quaternion_roll)
    
    # perform vector rotation with formula v' = q * v * q_star
    reference_prime = body_to_world_3d_rotation(reference_point, composed_rotation_zyx)
    
    return reference_prime

def is_unit_length(vector):
    sum = 0
    for val in vector:
        sum += val ** 2

    return sum == 1

def body_to_world_3d_translation(reference_point, px, py, pz):
    # implement extension of reference to be a 4 x 1 vector
    translation_matrix = np.array([[1, 0, 0, px],
                                    [0, 1, 0, py], 
                                    [0, 0, 1, pz],
                                    [0, 0, 0, 1]])
    print("translation_matrix:", translation_matrix)
    reference_point = np.matmul(translation_matrix, reference_point)
    return reference_point

def body_to_world_3d_rotation(reference_point, quaternion): # float * rotational_axis, float theta, float * current_vector

    '''quaternion_star = [quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3]]
    quaternion_reference = quaternion_multiply(quaternion, reference)
    reference_prime = quaternion_multiply(quaternion_reference, quaternion_star)'''


    quaternion_star, vector = [None]*4, [None]*4

    quaternion_star[0] = quaternion[0]
    quaternion_star[1] = -1 * quaternion[1]
    quaternion_star[2] = -1 * quaternion[2]
    quaternion_star[3] = -1 * quaternion[3]

    vector[0] = 0
    vector[1] = reference_point[0]
    vector[2] = reference_point[1]
    vector[3] = reference_point[2]

    # calculate q * vector * q_star ( quarternion multiplication )

    

    q_vector = quaternion_multiply(quaternion, vector)
    reference_prime = quaternion_multiply(q_vector, quaternion_star)
    
    
    return reference_prime
    
def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

def euler_angle_to_quaternion(euler_angle, rotational_axis):
    
    theta = euler_angle * (math.pi/180)

    '''sin_roll, cos_roll = math.sin(roll/2), math.cos(roll/2)
    sin_pitch, cos_pitch = math.sin(pitch/2), math.cos(pitch/2)
    sin_yaw, cos_yaw = math.sin(yaw/2), math.cos(yaw/2)'''

    quaternion = [None]*4
    quaternion[0] = math.cos(theta/2)
    quaternion[1] = math.sin(theta/2)*rotational_axis[0]
    quaternion[2] = math.sin(theta/2)*rotational_axis[1]
    quaternion[3] = math.sin(theta/2)*rotational_axis[2]
    
    '''quaternion = np.array([[cos_yaw * cos_pitch * cos_roll + sin_yaw * sin_pitch * sin_roll], 
                            [cos_yaw * cos_pitch * sin_roll - sin_yaw * sin_pitch * cos_roll], 
                            [sin_yaw * cos_pitch * sin_roll + cos_yaw * sin_pitch * cos_roll], 
                            [sin_yaw * cos_pitch * cos_roll - cos_yaw * sin_pitch * sin_roll]])'''
    return quaternion

'''print(body_to_world_all_quaternion([1, 1, 1], [90, 90, 90]))
print(body_to_world_all_rotation_mat([1, 1, 1], [90, 90, 90]))
print(body_to_world_all_quaternion([2, 1, 1], [90, 90, 90]))
print(body_to_world_all_rotation_mat([2, 1, 1], [90, 90, 90]))
print(body_to_world_all_quaternion([2, 1, 1], [45, 90, 90]))
print(body_to_world_all_rotation_mat([2, 1, 1], [45, 90, 90]))'''
print(body_to_world_all_quaternion([1, 1, 1], [20, 30, 90], [3, 5, 7]))
print(body_to_world_all_rotation_mat([1, 1, 1], [20, 30, 90], [3, 5, 7]))
'''print(body_to_world_all_quaternion([1, 1, 1], [40, 70, 110]))
print(body_to_world_all_rotation_mat([1, 1, 1], [40, 70, 110]))'''
#body_to_world_all_quaternion([1, 1, 1], [80, 10, 120])


#print(body_to_world_all_test1([1, 1, 1], [90, 90, 90]) == body_to_world_all_test2([1, 1, 1], [90, 90, 90]))
'''print(quaternion_multiply([1, 2, 3, 4], [5, 6, 7, 8]))
print(quaternion_multiply([5, 6, 7, 8], [1, 2, 3, 4]))'''  