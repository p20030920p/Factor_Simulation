import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Configure canvas
fig, ax = plt.subplots(figsize=(14, 10))
ax.set_xlim(0, 14)
ax.set_ylim(0, 10)
ax.set_aspect('equal')
ax.axis('off')

# Define box drawing function
def draw_box(x, y, width, height, text, color='lightblue'):
    rect = patches.Rectangle((x, y), width, height,
                            linewidth=1, edgecolor='black', facecolor=color)
    ax.add_patch(rect)
    ax.text(x + width/2, y + height/2, text,
           ha='center', va='center', fontsize=10)

# Define arrow drawing function
def draw_arrow(start, end, color='black'):
    ax.annotate('', xy=end, xytext=start,
               arrowprops=dict(arrowstyle='->', color=color))

# 1. Main flow
draw_box(6, 9, 2, 1, 'Start Simulation', 'lightgreen')

# 2. Task management module
draw_box(2, 7, 2, 1, 'Task Management\n(Generation, Prioritization, Assignment)')
draw_arrow((7, 9), (3, 8))  # From Start to Task Management

# 3. Path planning module
draw_box(6, 7, 2, 1, 'Path Planning\n(A* Algorithm, Safety Distance)')
draw_arrow((3, 7), (7, 8))  # From Task Management to Path Planning

# 4. Collision detection & CBS module
draw_box(10, 7, 2, 1, 'Collision Detection\n(CBS Implementation)')
draw_arrow((7, 7), (11, 8))  # From Path Planning to Collision Detection

# 5. Deadlock management module
draw_box(2, 5, 2, 1, 'Deadlock Management\n(Priority Sorting, Waiting Mechanism)')
draw_arrow((11, 7), (3, 6))  # From Collision Detection to Deadlock Management

# 6. Position update module
draw_box(6, 5, 2, 1, 'Robot Position Update')
draw_arrow((3, 5), (7, 6))  # From Deadlock Management to Position Update
draw_arrow((7, 7), (7, 6))  # From Path Planning to Position Update

# 7. Production monitoring module
draw_box(10, 5, 2, 1, 'Production Monitoring\n(Completion Rate, Utilization)')
draw_arrow((7, 5), (11, 6))  # From Position Update to Production Monitoring

# 8. Visualization module
draw_box(6, 3, 2, 1, 'Visualization\n(Matplotlib Display)')
draw_arrow((11, 5), (7, 4))  # From Production Monitoring to Visualization

# 9. Termination check
draw_box(6, 1, 2, 1, 'Simulation End?\n(Red Bucket Tasks=15 or Time Limit)')
draw_arrow((7, 3), (7, 2))  # From Visualization to Termination Check

# 10. Loop & Termination
draw_box(10, 1, 2, 1, 'Terminate', 'lightcoral')
draw_arrow((7, 1), (11, 2), 'red')  # Yes: Terminate
draw_arrow((7, 1), (3, 6), 'blue')  # No: Return to Deadlock Management

# Add loop annotation
ax.text(9, 2.5, 'NO: Loop Back', color='blue', fontsize=10)

# Add title
plt.title('Factory Simulation System Flowchart\n(With Deadlock Management and CBS)',
         fontsize=14, pad=20)

# Display
plt.show()