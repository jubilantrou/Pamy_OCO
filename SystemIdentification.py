'''
This script is used to do the system identification
'''
import numpy as np
import o80
import os
from get_handle import get_handle
import o80_pam
# %% constant: anchor lists, dof, amp_list,
anchor_ago_list        = [20000,19000,17000,17000] #np.array([17500, 18500, 16000, 15000])
anchor_ant_list        = [20000,20000,17000,17000] #np.array([17500, 18500, 16000, 15000])
dof                    = 3 # which dof to excite
fs                     = 100
frequency              = 500.0  # frequency of the backend 
period                 = 1.0/frequency  # period of backend
duration_per_command   = 1 / fs  # period of frontend
iterations_per_command = int(duration_per_command/period)  # sychronize the frontend and the backend
anchor_ago             = anchor_ago_list[dof]
anchor_ant             = anchor_ant_list[dof]
amp_list               = [2.0] #[3.5,2.0,3.5,2.0]  # amp_u * 1000
# %% connect to the simulation and initilize the posture
# handle           = get_handle(mujoco_id='SI',mode='pressure',generation='second')
# frontend         = handle.frontends["robot"]
segment_id = "real_robot"
frontend = o80_pam.FrontEnd(segment_id)
duration         = o80.Duration_us.seconds(2)
frontend.add_command(anchor_ago_list, anchor_ant_list,
                     duration,
                     o80.Mode.QUEUE)
frontend.pulse_and_wait()
# %%
p         = 10  # how many periods
file_path = '/home/mtian/Pamy_OCO/excitation signals/'
f_input   = file_path + 'excitation_2hz_0.csv' # two set signals
u         = np.loadtxt(f_input, delimiter=',')
[m, N]    = u.shape
u         = np.tile(u, p).flatten()
t_stamp_u = np.arange(0, (m * p * N)/fs, 1/fs)
t         = 1 / fs
T         = m * p * N / fs
# %% do the identification
for amp in amp_list:

    frontend.add_command(anchor_ago_list, anchor_ant_list,
                     duration,
                     o80.Mode.QUEUE)
    frontend.pulse_and_wait()

    u_temp = u * amp  #change the amplitude of the signal
    f_output = file_path + "mirroring_real_dof_" + str(dof) + ".txt"
    # f_output = file_path + "response_pamy2_real_dof_" + str(dof) + '_amp_' + str(amp) + 'excitation_2_hz_' + ".txt"
    # initilization
    position = np.array([])
    velocity = np.array([])
    obs_pressure_ant = np.array([])
    obs_pressure_ago = np.array([])
    des_pressure_ant = np.array([])
    des_pressure_ago = np.array([])
    t_stamp = np.array([])
    i_stamp = np.array([]) # for mirroring

    ref_iteration   = frontend.latest().get_iteration()  # read the current iteration
    iteration_begin = ref_iteration + 1000  # set the iteration when beginning
    iteration       = iteration_begin
    
    print("begin to excite the mujoco...")
    print(len(u_temp))
    for i in range(90000, len(u_temp)):
        diff = int( u_temp[i] )
        
        # hardware paper as ref
        pressure_ago = int( anchor_ago + diff )  # red line --> agonist
        pressure_ant = int( anchor_ant - diff ) # green line --> antagonist

        frontend.add_command(dof, pressure_ago, pressure_ant,
                            o80.Iteration(iteration),
                            o80.Mode.QUEUE)

        frontend.add_command(dof, pressure_ago, pressure_ant,
                            o80.Iteration(iteration + iterations_per_command - 1),
                            o80.Mode.QUEUE)

        frontend.pulse()
        iteration += iterations_per_command

    iteration_end  = iteration
    iteration = iteration_begin

    while iteration < iteration_end:
        observation = frontend.read(iteration)

        obs_pressure = observation.get_observed_pressures()
        des_pressure = observation.get_desired_pressures()
        obs_position = observation.get_positions()
        obs_velocity = observation.get_velocities()

        obs_pressure_ago = np.append(obs_pressure_ago, obs_pressure[dof][0])
        obs_pressure_ant = np.append(obs_pressure_ant, obs_pressure[dof][1])

        des_pressure_ago = np.append(des_pressure_ago, des_pressure[dof][0])
        des_pressure_ant = np.append(des_pressure_ant, des_pressure[dof][1])

        position = np.append(position, obs_position[dof])
        velocity = np.append(velocity, obs_velocity[dof])

        t_stamp = np.append(t_stamp, observation.get_time_stamp() * 1e-9)
        i_stamp = np.append(i_stamp, observation.get_iteration())

        iteration += iterations_per_command

    print("...completed")
    
    initial_time = t_stamp[0]

    for i in range(0, len(t_stamp)):
        t_stamp[i] = t_stamp[i] - initial_time

    print("total duration:",t_stamp[-1]-t_stamp[0])
    print("expected:",t_stamp_u[-1])
    print("number of simulation:",len(t_stamp))
    print("desired number:",len(t_stamp_u))

    # print("begin to write data to the file...")
    # f = open(f_output, 'w')
    # f.write(str(p))
    # f.write("\n")
    # f.write(str(m))
    # f.write("\n")
    # f.write(str(N))
    # f.write("\n")
    # np.savetxt(f, t_stamp,          fmt='%.5f')
    # np.savetxt(f, u_temp,           fmt='%.8f')
    # np.savetxt(f, des_pressure_ago, fmt='%.5f')
    # np.savetxt(f, des_pressure_ant, fmt='%.5f')
    # np.savetxt(f, obs_pressure_ago, fmt='%.5f')
    # np.savetxt(f, obs_pressure_ant, fmt='%.5f')
    # np.savetxt(f, position,         fmt='%.8f')
    # f.close()
    # print('...completed')

    print("begin to write data to the file...")
    f = open(f_output, 'w')
    np.savetxt(f, i_stamp,          fmt='%.1f')
    np.savetxt(f, t_stamp,          fmt='%.5f')
    np.savetxt(f, position,         fmt='%.8f')
    f.close()
    print('...completed')