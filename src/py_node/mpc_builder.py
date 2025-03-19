#!/usr/bin/python3
import numpy as np # Numerical computation library
import casadi as ca # Symbolic variable library
import opengen as og # Opengen framework
import sys


# Function to build the optimizer
def build_mpc(n_obs):
    '''
    This function builds the optimizer for MPC.
    The state variable x has current position, reference position and state limits.
    The input variable u is augmented with the future values. 
    '''

    N = 20 # Prediction horizon
    dt = 0.05 # Sampling time

    x_size = 

    # Define symbolic variables for states and inputs
    x = ca.MX.sym('x', 7) 
    u = ca.MX.sym('u', 3*N)
    
    X = ca.cumsum(u.reshape((-1, N)), 1)*dt + x[:2]


    cost = ca.sumsqr(x[:2]-x[2:4]) + 4*ca.sumsqr(X-x[2:4]) + ca.sumsqr(u)
    c = ca.veccat(ca.fmin(ca.MX.zeros(1,N), ca.sqrt(ca.sum1((X-x[4:6])**2)) - x[6]))

    umin = [-0.6, -0.6]*N
    umax = [0.6, 0.6]*N

    bounds = og.constraints.Rectangle(umin, umax)

    problem = og.builder.Problem(u, x, cost).with_constraints(bounds).with_penalty_constraints(c)
    tcp_config = og.config.TcpServerConfiguration(bind_port=3320)

    build_config = og.config.BuildConfiguration().with_build_directory("../../../").with_build_mode("release").with_tcp_interface_config(tcp_config)
    meta = og.config.OptimizerMeta().with_optimizer_name("mpc_test")


    solver_config = og.config.SolverConfiguration() \
                    .with_tolerance(1e-5) \
                    .with_initial_tolerance(1e-5) \
                    .with_max_duration_micros(40000) \
                    .with_max_outer_iterations(5) \
                    .with_penalty_weight_update_factor(5) \
                    .with_initial_penalty(1000.0)


    builder = og.builder.OpEnOptimizerBuilder(problem, meta,
                                          build_config, solver_config).with_verbosity_level(4)

    builder.build()


def mpc_test():
    mng = og.tcp.OptimizerTcpManager('../../../mpc_build')
    mng.start()

    x = np.array([2.0, 2.1, -1.0, 0.0, 1.0, 1.0, 1.0])
    mng.ping()
    sol = mng.call(x, initial_guess=[-1.0, -1.0]*20, buffer_len=8*4096)
    uM = np.array(sol['solution']).reshape((20, -1))
    X = np.cumsum(uM, axis=1)*0.1 + x[:2]


    print(f"Solution: {sol['solution']}")
    print(f"Progression: {X}")


    mng.kill()




if __name__ == '__main__':
    n = len(sys.argv)
    if n < 2:
        print("Not enough arguments. Exiting the MPC builder")
        sys.exit()
    else:
        build_mpc(sys.argv[1])

        mpc_test()
