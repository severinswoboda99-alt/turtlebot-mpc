import matplotlib.pyplot as plt
import numpy as np

# Plotting Execution
plt.rcParams['font.size'] = '12'

if __name__ == "__main__":
    # Plot summarized data
    
    gen = [1, 2, 4]
    
    avg_pos_error = [0.179, 0.027, 0.024]
    max_pos_error = [0.417, 0.045, 0.038]
    std_pos_error = [0.094, 0.009, 0.008]
    
    avg_theta_error = [15.913, 2.168, 1.506]
    max_theta_error = [70.683, 30.978, 19.745]
    std_theta_error = [15.617, 2.727, 1.812]
    
    avg_v_error = [0.011, 0.028, 0.021]
    max_v_error = [0.174, 0.211, 0.174]
    std_v_error = [0.032, 0.054, 0.041]
    
    avg_w_error = [0.292, 0.265, 0.277]
    max_w_error = [1.185, 2.852, 2.852]
    std_w_error = [0.263, 0.358, 0.398]
    
    avg_dv = [0.006, 0.040, 0.036]
    max_dv = [0.174, 0.211, 0.174]
    std_dv = [0.015, 0.055, 0.046]
    
    avg_dw = [0.079, 0.175, 0.310]
    max_dw = [1.823, 1.421, 2.852]
    std_dw = [0.164, 0.212, 0.517]

    # Plot Solve Time Summary
    x_pos = np.arange(len(gen)) 
    plt.figure()
    plt.errorbar(x_pos, avg_dw, yerr=std_dw, fmt='o-', capsize=4, label='Mean Effort ±1σ')
    plt.plot(x_pos, max_dw, marker='o', label='Maximum Effort')
    plt.xlabel('Generation')
    plt.ylabel('Effort (rad/s)')
    plt.legend()
    plt.title('Weight matrices effect on angular control effort, Path 3')
    plt.xticks(x_pos, gen)  
    plt.grid(linewidth = 0.5, axis='y')
    plt.savefig("WM_ac_summary_P3.pdf")
    plt.show()