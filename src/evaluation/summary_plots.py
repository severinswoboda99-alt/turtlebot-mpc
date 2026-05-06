import matplotlib.pyplot as plt
import numpy as np

# Plotting Execution
plt.rcParams['font.size'] = '12'

if __name__ == "__main__":
    # Horizon sizes
    N = [10, 15, 20, 25, 30, 35, 40]

    # -------------------
    # Path 0 data
    # -------------------
    avg_p0 = [1.578, 4.370, 9.685, 18.572, 29.889, 46.031, 69.935]
    max_p0 = [2.509, 5.462, 14.771, 23.236, 37.575, 52.688, 82.175]
    std_p0 = [0.226, 0.469, 1.312, 1.762, 2.439, 2.156, 4.467]

    # -------------------
    # Path 3 data
    # -------------------
    avg_p3 = [1.645, 4.472, 9.680, 17.988, 30.032, 47.237, 70.029]
    max_p3 = [2.694, 5.917, 12.797, 22.563, 39.123, 57.021, 89.338]
    std_p3 = [0.257, 0.534, 1.017, 1.453, 2.150, 3.320, 4.867]

    # Convert x-axis to numeric positions
    x_pos = np.arange(len(N))

    # -------------------
    # Plot Path 0
    # -------------------
    plt.figure()
    plt.errorbar(x_pos, avg_p0, yerr=std_p0, fmt='o-', capsize=4, label='Average ±1σ')
    plt.plot(x_pos, max_p0, marker='o', label='Maximum')
    plt.xlabel('Horizon size N')
    plt.ylabel('Solve Time (ms)')
    plt.title('Solve Time vs Horizon Size (Path 0)')
    plt.xticks(x_pos, N)
    plt.grid(linewidth=0.5, axis='y')
    plt.legend()
    plt.savefig("solve_time_path0.pdf")

    # -------------------
    # Plot Path 3
    # -------------------
    plt.figure()
    plt.errorbar(x_pos, avg_p3, yerr=std_p3, fmt='o-', capsize=4, label='Average ±1σ')
    plt.plot(x_pos, max_p3, marker='o', label='Maximum')
    plt.xlabel('Horizon size N')
    plt.ylabel('Solve time (ms)')
    plt.title('Solve Time vs Horizon Size (Path 3)')
    plt.xticks(x_pos, N)
    plt.grid(linewidth=0.5, axis='y')
    plt.legend()
    plt.savefig("solve_time_path3.pdf")

    plt.show()