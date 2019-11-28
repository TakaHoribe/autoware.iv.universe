import csv

with open('/home/kosuke/Downloads/us_sample_waypoint.csv') as f:
    reader = csv.reader(f)
    l = [row for row in reader]
    l.pop(0)
with open('/home/kosuke/data/us_sample/path.csv', 'w') as f:
    writer = csv.writer(f)
    writer.writerow(["x", "y", "z", "roll[rad]", "pitch[rad]", "yaw[rad]", "linear_velocity[m/s]", "angular_velocity[rad/s]", "linear_acceleration[m/s^2]", "angular_acceleration[rad/s^2]"])
    for row in l:
      writer.writerow([row[0],row[1], row[2], 0, 0, row[3], row[4], 0, 0, 0])