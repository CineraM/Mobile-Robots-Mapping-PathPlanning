# assume angle is in radians
# def rotationInPlace(direction, degree):
#     # Determines Rotation and sets proper speeds
#     if direction == "left":
#         degree *= -1

#     if degree < 0 :
#         sign = -1
#     else:
#         sign = 1
        
#     X_rad = math.radians(degree)
#     phi = sign*2 # defualt 2

#     # Calculates time need for rotation
#     omega = 2*abs(phi)*w_r / distBtwWhe
#     T = abs(X_rad / omega)
#     # end_heading = (predicted_pose[3] - degree)%360

#     t_start = robot.getTime()

#     setSpeedIPS(phi*w_r, -phi*w_r)

#     starting_theta = round(imuCleaner(imu.getRollPitchYaw()[2]))
#     end_heading = round((starting_theta - degree)%360,2)

#     marg_error = .01

#     while robot.step(timestep) != -1:
#         # current_heading = imuCleaner(imu.getRollPitchYaw()[2])
#         # east_flag = True if end_heading <= 4 or end_heading >= 356 else False
#         if (robot.getTime() - t_start) >= T:
#                 setSpeedIPS(0,0)
#                 ROBOT_POSE.updatePose(MAZE)
#                 break
#             # if east_flag:
#             #     current_heading = current_heading - 360 if current_heading > 355 else current_heading
#             # if current_heading > (end_heading+marg_error):
#             #     setSpeedIPS(.01, -.01)
#             # elif current_heading < (end_heading-marg_error):
#             #     setSpeedIPS(-.01, .01)
#             # else:
#         ROBOT_POSE.updatePose(MAZE)