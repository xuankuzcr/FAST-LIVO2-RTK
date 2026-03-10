import matplotlib.pyplot as plt
import numpy as np
import os

def plot_velocity_data(livo_filename="opt_vel.txt", gps_filename="gps_vel.txt"):
    # 动态获取当前 Python 脚本所在的绝对目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 拼接出同级目录下文件的完整路径
    livo_file = os.path.join(script_dir, livo_filename)
    gps_file = os.path.join(script_dir, gps_filename)

    if not os.path.exists(livo_file):
        print(f"Error: LIVO Odometry data file '{livo_file}' not found.")
        return
    if not os.path.exists(gps_file):
        print(f"Error: GPS data file '{gps_file}' not found.")
        return

    # --- 加载 LIVO Odometry 数据 ---
    livo_timestamps = []
    livo_velocities = []
    with open(livo_file, 'r') as f:
        # 跳过表头
        next(f) 
        for line in f:
            if line.strip() and not line.startswith('#'):
                try:
                    ts, vel = map(float, line.split())
                    livo_timestamps.append(ts)
                    livo_velocities.append(vel)
                except ValueError:
                    print(f"Skipping malformed line in {livo_file}: {line.strip()}")
    
    livo_timestamps = np.array(livo_timestamps)
    livo_velocities = np.array(livo_velocities)

    # --- 加载 GPS 数据 ---
    gps_timestamps = []
    gps_velocities = []
    with open(gps_file, 'r') as f:
        # 跳过表头
        next(f)
        for line in f:
            if line.strip() and not line.startswith('#'):
                try:
                    ts, vel = map(float, line.split())
                    gps_timestamps.append(ts)
                    gps_velocities.append(vel)
                except ValueError:
                    print(f"Skipping malformed line in {gps_file}: {line.strip()}")

    gps_timestamps = np.array(gps_timestamps)
    gps_velocities = np.array(gps_velocities)

    if len(livo_timestamps) == 0:
        print(f"Warning: No valid data found in {livo_file}.")
        return
    if len(gps_timestamps) == 0:
        print(f"Warning: No valid data found in {gps_file}.")
        return

    # --- 计算相对时间（以第一个 LIVO Odometry 时间戳为基准） ---
    start_time_livo = livo_timestamps[0]
    livo_relative_times = livo_timestamps - start_time_livo
    gps_relative_times = gps_timestamps - start_time_livo

    # --- 绘制数据 ---
    plt.figure(figsize=(15, 7))
    plt.plot(livo_relative_times, livo_velocities, label='LIVO Odometry Velocity Norm', color='blue', alpha=0.7)
    plt.plot(gps_relative_times, gps_velocities, label='GPS Velocity Norm', color='red', alpha=0.7)

    plt.title('LIVO Odometry and GPS Velocity Norm Comparison')
    plt.xlabel('Relative Time (seconds, starting from LIVO_0)')
    plt.ylabel('Velocity Norm (m/s)')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # 如果你的文件名不是默认的，可以在这里传参，例如：
    # plot_velocity_data(livo_filename="slam_vel.txt")
    plot_velocity_data()