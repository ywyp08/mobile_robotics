import pandas as pd
import matplotlib.pyplot as plt

# Load data
df = pd.read_csv('path_log.csv')
df.columns = df.columns.str.strip()

# Plot path
plt.figure(figsize=(8, 6))
plt.plot(df['x_mm'], df['y_mm'], '-', linewidth=2, color='green')

plt.xlabel('X position (mm)')
plt.ylabel('Y position (mm)')
plt.title('Robot Path')
plt.axis('equal')
plt.grid(True)
plt.gca().invert_yaxis()
plt.show()
