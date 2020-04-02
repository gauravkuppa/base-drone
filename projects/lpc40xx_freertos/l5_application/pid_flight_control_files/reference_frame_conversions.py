import numpy as np
import math

def body_to_world_translation(theta, reference_point, px, py):
    reference_point = np.append(reference_point, 0)
    radians = theta * (math.pi/180)
    rotation_matrix = np.array([
        [math.cos(radians), math.sin(radians), px], 
        [-math.sin(radians), math.cos(radians), py],
        [0,0,1]
    ])
    print(rotation_matrix)
    body = np.matmul(rotation_matrix,reference_point)
    print(body)
    return body

def body_to_world_2d_rotation(theta, reference_point):
    radians = theta * (math.pi/180)
    rotation_matrix = np.array([
        [math.cos(radians), math.sin(radians)], 
        [-math.sin(radians), math.cos(radians)]
    ])
    print(rotation_matrix)
    body = np.matmul(rotation_matrix,reference_point)
    print(body)
    return body



def world_to_body(theta, reference_point):
    radians = theta * (math.pi/180)
    rotation_matrix = np.array([
        [math.cos(radians), -math.sin(radians)], 
        [math.sin(radians), math.cos(radians)]
    ])
    #print("rot mat:", rotation_matrix)
    body = np.matmul(rotation_matrix,reference_point)
    #print(body.shape)
    return body


'''def body_to_world_all_test1(reference, euler, px, py, pz):
    # pose = position + rotation

    # translation to desired position
    reference = body_to_world_3d_translation(reference, px, py, pz)

    # conversion between angles?
    quaternion = euler_to_quaternion(euler)
    # rotation to desired angles
    reference_prime = body_to_world_3d_rotation(reference, quaternion) # quaternion multipication

    return reference_prime '''

def body_to_world_all_quaternion(reference_point, euler):

    yaw, pitch, roll = euler
    
    # TODO: figure out what order of rotations really mean? the following implementation is ZYX
    quaternion_yaw = euler_angle_to_quaternion(yaw, [0,0,1])
    quaternion_pitch = euler_angle_to_quaternion(pitch, [0,1,0])
    quaternion_roll = euler_angle_to_quaternion(roll, [1,0,0])

    '''if(is_unit_length(quaternion_yaw)):
        print("yaw good")
    
    if(is_unit_length(quaternion_pitch)):
        print("pitch good")

    if(is_unit_length(quaternion_roll)):
        print("roll good")'''
    '''
    TODO: enforce conditions: must be unit quaternions
    '''
    composed_rotation_zyx = quaternion_multiply(quaternion_multiply(quaternion_yaw, quaternion_pitch), quaternion_roll)
    # is it just the regular rotation with composed_rotation
    # print("composed_quaternion zyx:", composed_rotation_zyx)
    
    reference_point.append(1) # convert vector to quaternion

    reference_prime = body_to_world_3d_rotation(reference_point, composed_rotation_zyx)
    

    return reference_prime

def is_unit_length(vector):
    sum = 0
    for val in vector:
        sum += val ** 2

    return sum == 1

'''def body_to_world_all_test2(reference, euler, px, py, pz):
    roll, pitch, yaw = euler
    roll = roll * (math.pi/180)
    pitch = pitch * (math.pi/180)
    yaw = yaw * (math.pi/180)

    sin_roll, cos_roll = math.sin(roll), math.cos(roll)
    sin_pitch, cos_pitch = math.sin(pitch), math.cos(pitch)
    sin_yaw, cos_yaw = math.sin(yaw), math.cos(yaw)
    
    rotation_matrix = np.array([[cos_yaw*cos_pitch, cos_yaw*sin_pitch*sin_roll-sin_yaw*cos_roll, cos_yaw*sin_pitch*cos_roll+sin_yaw*sin_roll, px],
                                [sin_yaw*cos_pitch, sin_yaw*sin_pitch*sin_roll+cos_yaw*cos_roll, sin_yaw*sin_pitch*cos_roll-cos_yaw*sin_roll, py],
                                [-sin_pitch, cos_pitch*sin_roll, cos_pitch*cos_roll, pz],
                                [0, 0, 0, 1]])

    print(rotation_matrix.shape)

    reference_prime = np.matmul(rotation_matrix, reference)
    return reference_prime'''

def body_to_world_all_rotation_mat(reference_point, euler):
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
    rotation_matrix = np.array([[cos_yaw*cos_pitch, cos_yaw*sin_pitch*sin_roll-sin_yaw*cos_roll, cos_yaw*sin_pitch*cos_roll+sin_yaw*sin_roll],
                                [sin_yaw*cos_pitch, sin_yaw*sin_pitch*sin_roll+cos_yaw*cos_roll, sin_yaw*sin_pitch*cos_roll-cos_yaw*sin_roll],
                                [-sin_pitch, cos_pitch*sin_roll, cos_pitch*cos_roll]])

    # round values below epsilon to 0
    eps = 1e-15
    composed_rotation_zyx[np.abs(composed_rotation_zyx) < eps] = 0

    rotation_matrix[np.abs(rotation_matrix) < eps] = 0


    # print(rotation_matrix)
    # print("composed rotation_mat zyx:", composed_rotation_zyx)
    # print(rotation_matrix == composed_rotation)

    reference_prime = np.matmul(composed_rotation_zyx, reference_point)

     # round values below epsilon to 0
    '''eps = 1e-15
    reference_prime[np.abs(reference_prime) < eps] = 0'''
    
    return reference_prime

def body_to_world_3d_translation(reference_point, px, py, pz):
    # implement extension of reference to be a 4 x 1 vector
    translation_matrix = np.array([[1, 0, 0, px],
                                    [0, 1, 0, py], 
                                    [0, 0, 1, pz],
                                    [0, 0, 0, 1]])
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

    '''
    (Q1 * Q2).a = (a1a2 - b1b2 - c1c2 - d1d2)
    (Q1 * Q2).b = (a1b2 + b1a2 + c1d2 - d1c2)
    (Q1 * Q2).c = (a1c2 - b1d2 + c1a2 + d1b2)
    (Q1 * Q2).d = (a1d2 + b1c2 - c1b2 + d1a2)
    '''

    q_vector = quaternion_multiply(quaternion, vector)
    reference_prime = quaternion_multiply(q_vector, quaternion_star)
    '''q_vector->a = q->a*vector->a - q->b*vector->b - q->c*vector->c -
    q->d*vector->d; q_vector->b = q->a*vector->b + q->b*vector->a + q->c*vector->d -
    q->d*vector->c; q_vector->c = q->a*vector->c - q->b*vector->d + q->c*vector->a +
    q->d*vector->b; q_vector->d = q->a*vector->d + q->b*vector->c - q->c*vector->b +
    q->d*vector->a;


    q_vector_q_star->a = q_vector->a*vector->a - q_vector->b*vector->b -
    q_vector->c*vector->c - q_vector->d*vector->d; q_vector_q_star->b =
    q_vector->a*vector->b + q_vector->b*vector->a + q_vector->c*vector->d -
    q_vector->d*vector->c; q_vector_q_star->c = q_vector->a*vector->c -
    q_vector->b*vector->d + q_vector->c*vector->a + q_vector->d*vector->b;
    q_vector_q_star->d = q_vector->a*vector->d + q_vector->b*vector->c -
    q_vector->c*vector->b + q_vector->d*vector->a;'''
    
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



def main():
    theta = 45
    '''reference = np.array([[3], [4]])
    reference1 = np.array([[4], [3]])'''
    reference2 = np.array([[4], [4]])
    reference3 = np.array([[2],[0]])
    px, py = 0,0
    '''print("reference point: (3,4)", world_to_body(theta, reference))
    print("reference point: (4,3)", world_to_body(theta, reference1))'''
    print("reference point: (4,4)", world_to_body(theta, reference2))
    print("reference point: (2,0)", world_to_body(theta, reference3))
    print("reference point: (4,4)", body_to_world_2d_rotation(theta, reference2))

'''print(body_to_world_all_quaternion([1, 1, 1], [90, 90, 90]))
print(body_to_world_all_rotation_mat([1, 1, 1], [90, 90, 90]))
print(body_to_world_all_quaternion([2, 1, 1], [90, 90, 90]))
print(body_to_world_all_rotation_mat([2, 1, 1], [90, 90, 90]))
print(body_to_world_all_quaternion([2, 1, 1], [45, 90, 90]))
print(body_to_world_all_rotation_mat([2, 1, 1], [45, 90, 90]))'''
print(body_to_world_all_quaternion([1, 1, 1], [20, 30, 90]))
print(body_to_world_all_rotation_mat([1, 1, 1], [20, 30, 90]))
print(body_to_world_all_quaternion([1, 1, 1], [40, 70, 110]))
print(body_to_world_all_rotation_mat([1, 1, 1], [40, 70, 110]))
#body_to_world_all_quaternion([1, 1, 1], [80, 10, 120])


#print(body_to_world_all_test1([1, 1, 1], [90, 90, 90]) == body_to_world_all_test2([1, 1, 1], [90, 90, 90]))
'''print(quaternion_multiply([1, 2, 3, 4], [5, 6, 7, 8]))
print(quaternion_multiply([5, 6, 7, 8], [1, 2, 3, 4]))'''  