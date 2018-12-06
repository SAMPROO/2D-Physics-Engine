from matplotlib import pyplot as plt
import matplotlib.animation as animation
import numpy as np
import random

#Set coordinates and aspect ratio
fig = plt.figure()
fig.set_dpi(100)
axis_max = 100
axis_min = 0
axis = plt.axes(xlim=(axis_min, axis_max), ylim=(axis_min, axis_max))
plt.gca().set_aspect(1)

#Gravity and time step
G = 9.80665
TSTEP = 0.0009
t = TSTEP

#Array axis indexes 
x_axis = 0
y_axis = 1
z_axis = 2

#Initial velocity and angle
v0 = 70
v1 = 30 
alpha_deg = 0

#Degrees to radians
alpha_rad = alpha_deg/180 * np.pi

#Velocity to x and y components
velocity_0 = [
    v0 * np.cos(alpha_rad),
    v0 * np.sin(alpha_rad),
    0.0]

velocity_1 = [
    v1 * np.cos(alpha_rad),
    v1 * np.sin(alpha_rad),
    0.0]

velocities = [velocity_0, velocity_1]

angle_0 = [0.0]
angle_1 = [0.0]

angles = [angle_0, angle_1]

angular_velocity_0 = [0.0, 0.0, -5.0]
angular_velocity_1 = [0.0, 0.0, 2.0]

angular_velocities = [angular_velocity_0, angular_velocity_1]


#
e = 0.7
mass = 0.1
momentum = 0.9

#Initial 
#angle = 0



""" #Initial velocity and angle
v0 = random.randint(0, 100)
alpha_deg = random.randint(0, 360)

#Degrees to radians
alpha_rad = alpha_deg/180 * np.pi

#Velocity to x and y components
velocity = [
    v0 * np.cos(alpha_rad),
    v0 * np.sin(alpha_rad),
    0.0]

#
e = random.random()
mass = random.uniform(1.0, 20.0)
momentum = mass * np.power(5, 2)

#Initial 
angle = 0
angular_velocity = [0.0, 0.0, random.uniform(0.0, 5.0)] """

#Edge normal vectors
yNorm_floor = [0, 1, 0]
yNorm_roof = [0, -1, 0]
xNorm_rWall = [-1, 0, 0]
xNorm_lWall = [1, 0, 0]

l = 10.0
#Giving polygon points and center of mass
polygon_vertex = np.array([[10.0, 10.0], [10.0, 20.0], [20.0, 20.0], [20.0, 10.0]])

#Giving polygon points and center of mass
polygon_vertex_2 = np.array([[80.0, 80.0], [80.0, 90.0], [90.0, 90.0], [90.0, 80.0]])

polygons = [polygon_vertex, polygon_vertex_2]
polygons_cm = []

#Polygon center of mass set to 0,0 coordinates
polygon_vertex_cm = np.array([[-5.0, -5.0], [-5.0, 5.0], [5.0, 5.0], [5.0, -5.0]])

''' #Initialize lists
velocities = []
angles = []
angular_velocities = [] '''

#Initialize random parameters for each polygon in polygons list
for i in polygons:

    x_value = 0.0
    y_value = 0.0
    m = 0.0

    '''  #Random angle for each polygon
    alpha_deg = random.randint(0, 360)

    #Random velocity for each polygon
    velocity = [
        random.randint(0, 100) * np.cos(alpha_deg/180 * np.pi),
        random.randint(0, 100) * np.sin(alpha_deg/180 * np.pi),
        0.0]

    #Add unique velocity for each polygon in polygon list
    velocities.append(velocity)

    #Initial angle is always 0
    angle = [0.0]

    #Add beginning angle for each polygon in polygon list
    angles.append(angle)

    #Random angular velocity between (x, y)
    angular_velocity = [0.0, 0.0, random.uniform(0, 10)]

    #Add unique angular velocity for each polygon in polygon list
    angular_velocities.append(angular_velocity) '''

    #Calculation for each polygon center of mass in polygon list
    for polygon in i:
        x_value += polygon[x_axis] * mass
        y_value += polygon[y_axis] * mass
        m += mass

    polygon_cm = [x_value/m, y_value/m]
    
    #Add polygon cm to polygonS cm list
    polygons_cm.append(polygon_cm)

objects = []

for i in polygons:
    objects.append(plt.Polygon(i, closed=False, color='r'))

for i in objects:
    axis.add_patch(i)

def init():
    return objects

def collision_calculation(
    coordinate, 
    norm, 
    vel, 
    dot_hitVelocity_norm,
    crossP_hitPoint_norm,
    angular_v, 
    ang):

    bounce = -1 + e
    mass_divided = 1/mass
    kolmas = np.power(crossP_hitPoint_norm[z_axis], 2) / momentum


    #Impact impulse
    impulse = (bounce * dot_hitVelocity_norm) / (mass_divided + kolmas)

    #New velocities after impact
    vel[x_axis] += impulse/mass * norm[x_axis]
    vel[y_axis] -= impulse/mass * norm[y_axis]

    #Change speed to opposite
    vel[coordinate] *= -1

    #New angular velocity and angle
    angular_v[z_axis] = angular_v[z_axis] + impulse/momentum * crossP_hitPoint_norm[z_axis]

    #New angle
    ang = ang + angular_v[z_axis] * t
    
    return [vel, angular_v, ang]

# Toista niin kauan kuin pallo maanpinnan yläpuolella
def animate(i):

    global t
    global velocities
    global angles
    global angular_velocities
    global momentum

    t = TSTEP
    collision = False

    index = 0

    print(collision)

    #Polygons new center of mass location
    for cm, velocity in zip(polygons_cm, velocities) :
        cm[x_axis] += velocity[x_axis] * t
        cm[y_axis] += velocity[y_axis] * t #tähän vy initial ja vy keskiarvo

    for polygon, cm, ang in zip(polygons, polygons_cm, angles):

        for i,j in zip(polygon, polygon_vertex_cm):
            
            #Polygon point new location
            i[x_axis] = j[x_axis] * np.cos(ang) - j[y_axis] * np.sin(ang) + cm[x_axis]
            i[y_axis] = j[x_axis] * np.sin(ang) + j[y_axis] * np.cos(ang) + cm[y_axis]

    for polygon, cm, velocity, angular_vel, ang in zip(polygons, polygons_cm, velocities, angular_velocities, angles):

        for i in polygon:
        
            #Calculation for impact vector from the center of mass
            hit_point_vector = [
            i[x_axis] - cm[x_axis], 
            i[y_axis] - cm[y_axis],
            0.0]

            #Cross product between angle velocity and hitpoint
            crossP_angularV_hitVector = np.cross(angular_vel, hit_point_vector)

            #Hitpoint velocity
            hit_point_velocity = velocity + crossP_angularV_hitVector
            
            #Dot product between impact velocity and impact norm
            dot_hit_vel_yNorm_floor = np.dot(hit_point_velocity, yNorm_floor)
            dot_hit_vel_yNorm_roof = np.dot(hit_point_velocity, yNorm_roof)
            dot_hit_vel_xNorm_rWall = np.dot(hit_point_velocity, xNorm_rWall)
            dot_hit_vel_xNorm_lWall = np.dot(hit_point_velocity, xNorm_lWall)

            #Cross product between impact vector and impact norm
            crossP_hit_vector_floor = np.cross(hit_point_vector, yNorm_floor)
            crossP_hit_vector_roof = np.cross(hit_point_vector, yNorm_roof)
            crossP_hit_vector_rWall = np.cross(hit_point_vector, xNorm_rWall)
            crossP_hit_vector_lWall = np.cross(hit_point_vector, xNorm_lWall)

            #Collision with RIGHT WALL
            if(i[x_axis] > axis_max and dot_hit_vel_xNorm_rWall < 0):
                print("Collision with RIGHT WALL")

                result = collision_calculation(
                    coordinate=x_axis, 
                    norm=xNorm_rWall, 
                    vel=velocity,
                    dot_hitVelocity_norm=dot_hit_vel_xNorm_rWall,
                    crossP_hitPoint_norm=crossP_hit_vector_rWall,
                    angular_v=angular_vel,
                    ang=ang)

                collision = True
                print(result)

            #Collision with LEFT WALL
            if i[x_axis] < axis_min and dot_hit_vel_xNorm_lWall < 0:
                print("Collision with LEFT WALL")

                result = collision_calculation(
                    coordinate=x_axis, 
                    norm=xNorm_lWall, 
                    vel=velocity,
                    dot_hitVelocity_norm=dot_hit_vel_xNorm_lWall,
                    crossP_hitPoint_norm=crossP_hit_vector_lWall,
                    angular_v=angular_vel,
                    ang=ang)
                    
                collision = True

            #Collision with ROOF
            if(i[y_axis] > axis_max and dot_hit_vel_yNorm_roof < 0):
                print("Collision with ROOF")

                result = collision_calculation(
                    coordinate=y_axis, 
                    norm=yNorm_floor, 
                    vel=velocity,
                    dot_hitVelocity_norm=dot_hit_vel_yNorm_roof,
                    crossP_hitPoint_norm=crossP_hit_vector_roof,
                    angular_v=angular_vel,
                    ang=ang)

                collision = True

            #Collision with FLOOR
            if(i[y_axis] < axis_min and dot_hit_vel_yNorm_floor < 0): 
                print("Collision with FLOOR")
                
                result = collision_calculation(
                    coordinate=y_axis,  
                    norm=yNorm_floor, 
                    vel=velocity,
                    dot_hitVelocity_norm=dot_hit_vel_yNorm_floor,
                    crossP_hitPoint_norm=crossP_hit_vector_floor,
                    angular_v=angular_vel,
                    ang=ang)
                
                collision = True

        #When collision has happened the results determine the new parameters
        if  collision is True:
        
            velocity = result[0]
            angular_vel = result[1]
            ang = result[2]
        #When collision has NOT happened the results are normally calculated
        else:

            #angular_vel[z_axis] = angular_vel[z_axis]
            velocity[x_axis] = velocity[x_axis]
            velocity[y_axis] -= G*t
            ang[0] += angular_vel[z_axis] * t

    objects[index].set_xy = polygon
    index += 1

    return objects
        

# Add labels to coordinates
plt.xlabel("x (m)")
plt.ylabel("y (m)")

animation = animation.FuncAnimation(fig, animate, init_func=init, frames=200, interval=TSTEP * 1000, repeat=True, blit=True)

plt.show()

