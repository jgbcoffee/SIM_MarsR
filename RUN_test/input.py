exec(open("Modified_data/testdr.dr").read())
#exec(open("Modified_data/realtime.py").read())


# Environment and parameters
# dyn.marsr.env.gravity         = [0.0, 0.0, -1.62]
dyn.marsr.env.inertia         = 200    
dyn.marsr.env.cd              = 0.2          
dyn.marsr.env.maxThrust       = 845000 
dyn.marsr.env.maxSteer        = 20    
dyn.marsr.env.steerDamp       = 20   


# Motor and steering
# dyn.marsr.motor.sample_time_array    = [0  ,     12,   24,     36,  48,  60,  72,  84,  96,  108,  120]
dyn.marsr.motor.thrust_sample_values = [1,      1,  1,      1,   1,   1,   1,   1,   1,    1,    1]
dyn.marsr.motor.steer_sample_values  = [0  ,  0.001,    0, -0.001,   0,   0,   0,   0,   0,    0,    0]


# Position and velocity
dyn.marsr.position[0] = 0.0
dyn.marsr.position[1] = 0.0
dyn.marsr.position[2] = 0.0
dyn.marsr.velocity[0] = 0.0
dyn.marsr.velocity[1] = 0.0
dyn.marsr.velocity[2] = 0.001
dyn.marsr.phi         = 1.57079633
dyn.marsr.omega       = 0.0

# Time
dyn.marsr.mission_time      = 0.0
dyn.marsr.mission_time_rate = 0.01






trick.exec_set_terminate_time(500.0)
