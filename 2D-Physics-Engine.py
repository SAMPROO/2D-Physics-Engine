from matplotlib import pyplot as plt
import matplotlib.animation as animation
import numpy as np
import random

#Set coordinates and aspect ratio
fig = plt.figure()
fig.set_dpi(150)
axis_max = 10
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

velocities = []

angles = []

angular_velocities = []

bounces = []

masses = []

momentums_of_inertia = []

polygons = []

#Edge normal vectors
yNorm_floor = [0, 1, 0]
yNorm_roof = [0, -1, 0]
xNorm_rWall = [-1, 0, 0]
xNorm_lWall = [1, 0, 0]

bounce = 0.5
mass = 2
momentum_of_intertia = np.power(1.0, 4)/12

########################################################

#Giving polygon points and center of mass
polygon_vertex_0 = np.array([[1.0, 1.0], [1.0, 2.0], [2.0, 2.0], [2.0, 1.0]])

alpha_deg_0 = 180

#Degrees to radians
alpha_rad_0 = alpha_deg_0/180 * np.pi

#Initial velocity and angle
v0 = 40.0

#Velocity to x and y components
velocity_0 = [
    v0 * np.cos(alpha_rad_0),
    v0 * np.sin(alpha_rad_0),
    0.0]

velocities.append(velocity_0)

angle_0 = [0.0]
angles.append(angle_0)

angular_velocity_0 = [0.0, 0.0, 9.0]
angular_velocities.append(angular_velocity_0)




#####################################################

#Giving polygon points and center of mass
polygon_vertex_1 = np.array([[3.0, 1.0], [3.0, 2.0], [4.0, 2.0], [4.0, 1.0]])

alpha_deg_1 = 45

#Degrees to radians
alpha_rad_1 = alpha_deg_1/180 * np.pi

#Initial velocity and angle
v1 = 80.0

#Velocity to x and y components
velocity_1 = [
    v1 * np.cos(alpha_deg_1),
    v1 * np.sin(alpha_deg_1),
    0.0]

velocities.append(velocity_1)

angle_1 = [0.0]
angles.append(angle_1)

angular_velocity_1 = [0.0, 0.0, -9.0]
angular_velocities.append(angular_velocity_1)


########################################

polygons = [polygon_vertex_0, polygon_vertex_1]

polygons_cm = []

#Polygon center of mass set to 0,0 coordinates
polygon_vertex_cm = np.array([[-0.5, -0.5], [-0.5, 0.5], [0.5, 0.5], [0.5, -0.5]])

#Initialize random parameters for each polygon in polygons list
for i in polygons:

    x_value = 0.0
    y_value = 0.0
    m = 0.0

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
    color = tuple(random.uniform(0.1, 0.5) for i in range(3))

    objects.append(plt.Polygon(i, closed=False, color=color))

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

    e = -(1 + bounce)
    mass_divided = 1/mass
    kolmas = np.power(crossP_hitPoint_norm[z_axis], 2) / momentum_of_intertia


    #Impact impulse
    impulse = (e * dot_hitVelocity_norm) / (mass_divided + kolmas)

    #New velocities after impact
    vel[x_axis] += impulse/mass * norm[x_axis]
    vel[y_axis] += impulse/mass * norm[y_axis]

    #New angular velocity and angle
    angular_v[z_axis] += impulse/momentum_of_intertia * crossP_hitPoint_norm[z_axis]

    #New angle
    ang += angular_v[z_axis] * t
    
    return [vel, angular_v, ang]

def animate(i):

    global t
    global velocities
    global angles
    global angular_velocities
    global momentum_of_intertia

    t = TSTEP
    collision = False

    index = 0

    #Polygons new center of mass location
    for cm, velocity in zip(polygons_cm, velocities) :
        cm[x_axis] += velocity[x_axis] * t
        cm[y_axis] += velocity[y_axis] * t

    for polygon, cm, angle in zip(polygons, polygons_cm, angles):

        for i,j in zip(polygon, polygon_vertex_cm):
            
            #Polygon point new location
            i[x_axis] = j[x_axis] * np.cos(angle) - j[y_axis] * np.sin(angle) + cm[x_axis]
            i[y_axis] = j[x_axis] * np.sin(angle) + j[y_axis] * np.cos(angle) + cm[y_axis]

    for polygon, cm, velocity, angular_vel, angle in zip(polygons, polygons_cm, velocities, angular_velocities, angles):

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

                #Move impact point back to axis
                move_by = i[x_axis] - axis_max

                for j in polygon:
                    j[x_axis] -= move_by
                
                cm[x_axis] -= move_by

                result = collision_calculation(
                    coordinate=x_axis, 
                    norm=xNorm_rWall, 
                    vel=velocity,
                    dot_hitVelocity_norm=dot_hit_vel_xNorm_rWall,
                    crossP_hitPoint_norm=crossP_hit_vector_rWall,
                    angular_v=angular_vel,
                    ang=angle)

                collision = True

            #Collision with LEFT WALL
            if i[x_axis] < axis_min and dot_hit_vel_xNorm_lWall < 0:
                print("Collision with LEFT WALL")
                
                #Move impact point back to axis
                move_by = axis_min - i[x_axis]

                for j in polygon:
                    j[x_axis] += move_by
                
                cm[x_axis] += move_by

                result = collision_calculation(
                    coordinate=x_axis, 
                    norm=xNorm_lWall, 
                    vel=velocity,
                    dot_hitVelocity_norm=dot_hit_vel_xNorm_lWall,
                    crossP_hitPoint_norm=crossP_hit_vector_lWall,
                    angular_v=angular_vel,
                    ang=angle)
                    
                collision = True

            #Collision with ROOF
            if(i[y_axis] > axis_max and dot_hit_vel_yNorm_roof < 0):
                print("Collision with ROOF")

                #Move impact point back to axis
                move_by = i[y_axis] - axis_max

                for j in polygon:
                    j[y_axis] -= move_by
                
                cm[y_axis] -= move_by

                result = collision_calculation(
                    coordinate=y_axis, 
                    norm=yNorm_roof, 
                    vel=velocity,
                    dot_hitVelocity_norm=dot_hit_vel_yNorm_roof,
                    crossP_hitPoint_norm=crossP_hit_vector_roof,
                    angular_v=angular_vel,
                    ang=angle)

                collision = True

            #Collision with FLOOR
            if(i[y_axis] < axis_min and dot_hit_vel_yNorm_floor < 0): 
                print("Collision with FLOOR")
                
                #Move impact point back to axis
                move_by = axis_min - i[y_axis]

                for j in polygon:
                    j[y_axis] += move_by
                
                cm[y_axis] += move_by

                result = collision_calculation(
                    coordinate=y_axis,  
                    norm=yNorm_floor, 
                    vel=velocity,
                    dot_hitVelocity_norm=dot_hit_vel_yNorm_floor,
                    crossP_hitPoint_norm=crossP_hit_vector_floor,
                    angular_v=angular_vel,
                    ang=angle)
            
                collision = True

            
            #When collision has happened the results determine the new parameters
        if  collision is True:
        
            velocity = result[0]
            velocity[y_axis] -= G*t
            angular_vel = result[1]
            angle = result[2]

            collision = False
        #When collision has NOT happened the results are normally calculated
        else:

            #angular_vel[z_axis] = angular_vel[z_axis]
            velocity[x_axis] = velocity[x_axis]
            velocity[y_axis] -= G*t
            angle[0] += angular_vel[z_axis] * t

    #Initialize lists
    distance = []
    vectors = []
    collision_vector = []

    #Collision check with other polygons
    for p in range(len(polygons)):
        
        for k in range(len(polygons)):

            for h in range(len(polygons[k])):

                collision_check_list = []
                collision = False
                print("THIS ", p)
                for j in range(len(polygons[p])):

                    if j < len(polygons[p]) - 1:

                        vector = [
                            polygons[p][j + 1][x_axis] - polygons[p][j][x_axis],
                            polygons[p][j + 1][y_axis] - polygons[p][j][y_axis]]

                        vector_point = [
                            polygons[k][h][x_axis] - polygons[p][j][x_axis],
                            polygons[k][h][y_axis] - polygons[p][j][y_axis]]

                        collision_point = [
                            polygons[k][h][x_axis],
                            polygons[k][h][y_axis],
                            0.0]

                    else:
                        vector = [
                            polygons[p][0][x_axis] - polygons[p][len(polygons[p]) - 1][x_axis],
                            polygons[p][0][y_axis] - polygons[p][len(polygons[p]) - 1][y_axis]]

                        vector_point = [
                            polygons[k][h][x_axis] - polygons[p][len(polygons[p]) - 1][x_axis],
                            polygons[k][h][y_axis] - polygons[p][len(polygons[p]) - 1][y_axis]]

                        collision_point = [
                            polygons[k][h][x_axis],
                            polygons[k][h][y_axis], 0.0]

                    smth = [
                        polygons[p][j][x_axis] - collision_point[x_axis],
                        polygons[p][j][y_axis] - collision_point[y_axis]]

                    collision_vector.append(smth)

                    d = np.linalg.norm(np.cross(vector, smth))/np.linalg.norm(vector)

                    vectors.append(vector)
                    distance.append(d)
                    
                    ###################################################

                    crossP_vector_vectorPoint = np.cross(vector, vector_point)

                    #Add result to collision check list
                    if crossP_vector_vectorPoint < 0.0:

                        collision_check_list.append(1)
                    else:
                        collision_check_list.append(0)
                    
                    #Break if cross products are not either all >0 or <0
                    if(len(set(collision_check_list)) != 1):
                        collision = False

                        normal_unit_vector_list = []
                        distance = []
                        vectors = []
                        collision_vector = []
                        break
                
                #If all values in collisipon check list are equal then collision has occured
                if(len(set(collision_check_list)) == 1):
                    collision = True
                else:
                    collisiton = False

                #Collision calculation
                if collision is True:

                    #Index for which polygon side is closest to colliding point
                    min_value_index = distance.index(min(distance))

                    #Polygon side vector that is getting collided with
                    vector = vectors[min_value_index]

                    #Magnitude of vector
                    magnitude = np.sqrt(np.power(vector[x_axis], 2) + np.power(vector[y_axis], 2))

                    #Unit vector of vector
                    unit_vector = [vector[x_axis]/magnitude, vector[y_axis]/magnitude, 0.0/magnitude]

                    #Normal unit vector of the vector
                    normal_unit_vector = np.cross(unit_vector, [0.0, 0.0, -1.0])

                    #Center of mass point for polygon that is getting collided with
                    cm_p = polygons_cm[p]
                    #Center of mass point for polygon that is colliding
                    cm_k = polygons_cm[k]

                    #Vector between center of mass and hitpoint for polygon 0
                    vector_cm_hitpoint_p = [

                        collision_point[x_axis] - cm_p[x_axis],
                        collision_point[y_axis] - cm_p[y_axis],
                        0.0
                    ]
                    #Vector between center of mass and hitpoint for polygon 1
                    vector_cm_hitpoint_k = [

                        collision_point[x_axis] - cm_k[x_axis],
                        collision_point[y_axis] - cm_k[y_axis],
                        0.0
                    ]

                    crossP_hit_norm_p = np.cross(vector_cm_hitpoint_p, normal_unit_vector)
                    crossP_hit_norm_k = np.cross(vector_cm_hitpoint_k, normal_unit_vector)
                    
                    #Cross product between vector from cm to collision point and normal unit vector
                    crossP_cm_hitpoint_norm_p = np.cross(vector_cm_hitpoint_p, normal_unit_vector)
                    crossP_cm_hitpoint_norm_k = np.cross(vector_cm_hitpoint_k, normal_unit_vector)

                    #Hitpoint velocities for polygon hitpoints
                    point_velocity_p = velocities[p] + np.cross(angular_velocities[p], vector_cm_hitpoint_p)
                    point_velocity_k = velocities[k] + np.cross(angular_velocities[k], vector_cm_hitpoint_k)

                    #Relative velocity
                    relative_velocity = point_velocity_p - point_velocity_k

                    dot_relativeVelocity_norm = np.dot(relative_velocity, normal_unit_vector)

                    bouncyness = -(1.0 + bounce)
                    mass_divided = 1.0/mass*2.0
                    kolmas = np.power(crossP_cm_hitpoint_norm_p[z_axis], 2) / momentum_of_intertia + np.power(crossP_cm_hitpoint_norm_k[z_axis], 2) / momentum_of_intertia


                    #Impact impulse
                    impulse = (bouncyness * dot_relativeVelocity_norm) / (mass_divided + kolmas)

                    #New velocities after impact
                    velocities[p][x_axis] += impulse/mass * normal_unit_vector[x_axis]
                    velocities[p][y_axis] += impulse/mass * normal_unit_vector[y_axis]
                    
                    velocities[k][x_axis] -= impulse/mass * normal_unit_vector[x_axis]
                    velocities[k][y_axis] -= impulse/mass * normal_unit_vector[y_axis]

                    #New angular velocities after impact
                    angular_velocities[p][z_axis] += impulse/momentum_of_intertia * crossP_hit_norm_p[z_axis]
                    angular_velocities[k][z_axis] -= impulse/momentum_of_intertia * crossP_hit_norm_k[z_axis]

                    #Move collision corner back to collision point
                    offset = normal_unit_vector * distance[min_value_index]

                    for f in polygons[k]:
                        f[x_axis] += offset[x_axis]
                        f[y_axis] += offset[y_axis]

                    #Move colliding polygon cm back by offset
                    polygons_cm[k][x_axis] += offset[x_axis]
                    polygons_cm[k][y_axis] += offset[y_axis]
                    
                    print("COLLISION")
                    collision = False
                    break

    objects[index].set_xy = polygon
    index += 1

    return objects
        
# Add labels to coordinates
plt.xlabel("x (m)")
plt.ylabel("y (m)")

animation = animation.FuncAnimation(fig, animate, init_func=init, frames=200, interval=TSTEP * 1000, repeat=True, blit=True)

plt.show()