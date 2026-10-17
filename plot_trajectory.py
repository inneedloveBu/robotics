import csv
import math
import matplotlib.pyplot as plt

# 读取数据
times, gt_x, gt_y, est_x, est_y = [], [], [], [], []
with open('data/trajectory_data.csv', 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        times.append(float(row['time']))
        gt_x.append(float(row['gt_x']))
        gt_y.append(float(row['gt_y']))
        est_x.append(float(row['est_x']))
        est_y.append(float(row['est_y']))

# 计算最终误差
final_error = math.sqrt((gt_x[-1] - est_x[-1])**2 + (gt_y[-1] - est_y[-1])**2)

# 绘图
plt.figure(figsize=(8, 6))
plt.plot(gt_x, gt_y, 'b-', linewidth=2, label='Ground Truth')
plt.plot(est_x, est_y, 'r--', linewidth=2, label='Estimated (Noisy Odometry)')
plt.scatter(gt_x[0], gt_y[0], color='green', s=80, marker='o', label='Start', zorder=5)
plt.scatter(gt_x[-1], gt_y[-1], color='blue', s=80, marker='s', label='End (GT)', zorder=5)
plt.scatter(est_x[-1], est_y[-1], color='red', s=80, marker='x', label='End (Est)', zorder=5)

plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title(f'Robot Localization: Estimated vs Ground Truth\nFinal Position Error = {final_error:.3f} m')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.tight_layout()
plt.savefig('results/trajectory_plot.png', dpi=150)
plt.savefig('results/trajectory_plot.pdf')
print(f'Plot saved. Final error: {final_error:.3f} m')
