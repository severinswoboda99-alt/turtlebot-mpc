import matplotlib.pyplot as plt

# Plotting Execution
plt.rcParams['font.size'] = '12'

if __name__ == "__main__":
    # Plot summarized horizon size data
    
    # # Path 0
    # # Horizon sizes
    # N = [10, 15, 20, 25, 30, 35, 40, 45, 50]
    # # Average solve times (ms)
    # solve_time = [1.670, 4.689, 9.956, 19.131, 31.304, 48.803, 72.782, 104.327, 140.941]
    # # Maximum solve times (ms)
    # max_time = [2.353, 5.965, 12.319, 28.153, 36.450, 59.900, 85.530, 124.498, 163.455]
    # # Standard deviation (ms)
    # std_dev = [0.504, 0.204, 0.938, 1.980, 1.826, 2.328, 3.786, 4.650, 7.432]
    
    # Path 3
    # Horizon sizes
    N = [10, 15, 20, 25, 30, 35, 40, 45, 50]
    # Average solve times (ms)
    solve_time = [1.723, 4.646, 10.321, 19.372, 32.650, 51.253, 74.916, 107.479, 145.765]
    # Maximum solve times (ms)
    max_time = [2.369, 6.019, 14.666, 25.243, 40.376, 63.210, 86.121, 119.494, 175.035]
    # Standard deviation (ms)
    std_dev_time = [0.211, 0.465, 1.138, 1.620, 2.028, 2.886, 3.076, 3.774, 6.119]
    # Average positional error (m)
    pos_error = [0.193, 0.175, 0.178, 0.162, 0.147, 0.146, 0.142, 0.154, 0.153]
    # Standard Deviation positional error (m)
    std_dev_pos = [0.089, 0.076, 0.093, 0.079, 0.064, 0.068, 0.063, 0.082, 0.074]
    
    # Plot Solve Time Summary
    plt.figure()
    plt.errorbar(N, solve_time, yerr=std_dev_time, fmt='o-', capsize=4, label='Mean Solve Time ±1σ', color='blue')
    plt.plot(N, max_time, marker='o', label='Maximum Time', color='goldenrod')
    plt.xlabel('MPC Horizon (N)')
    plt.ylabel('Time (ms)')
    plt.legend()
    plt.title('Effect of MPC Horizon on Solve Time, Path 3')
    plt.grid()
    plt.savefig("horizon_solve_time_summary_P3.pdf")
    plt.show()
    
    # Plot Positional Error Summary
    plt.figure()
    plt.errorbar(N, pos_error, yerr=std_dev_pos, fmt='o-', capsize=4, label='Mean Error ±1σ', color='indigo')
    plt.xlabel('MPC Horizon (N)')
    plt.ylabel('Deviation (m)')
    plt.legend()
    plt.title('Effect of MPC Horizon on Position Error, Path 3')
    plt.grid()
    plt.savefig("horizon_position_error_summary_P3.pdf")
    plt.show()