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

def body_to_world(theta, reference_point):
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
print("reference point: (4,4)", body_to_world(theta, reference2))