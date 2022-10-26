import csv

csvfile = open('task.csv', newline='')
WAYPOINTS = csv.DictReader(csvfile)

print(len(list(WAYPOINTS)))

csvfile = open('task.csv', newline='')
WAYPOINTS = csv.DictReader(csvfile)
for p in WAYPOINTS:
  print(p)