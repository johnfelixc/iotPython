import numpy as np

sensorDataObject = np.array([[1472020505.148035, 44.3], [1472020506.1499994, 44.3], [1472020507.1515243, 44.3], [1472020508.153034, 44.3], [1472020509.154541, 44.3], [1472020510.1560533, 44.3], [1472020511.15798, 43.9], [1472020512.1599216, 44.3], [1472020513.1618438, 44.3], [1472020514.16341, 44.3]])
sensorDataHuman = np.array([[1472020505.161385, False], [1472020506.1624656, False], [1472020507.1635149, False], [1472020508.164555, False], [1472020509.1655958, False], [1472020510.1666353, False], [1472020511.1676989, False], [1472020512.16877, False], [1472020513.1698575, False], [1472020514.170934, False]])

print(np.concatenate((sensorDataObject, sensorDataHuman), axis=1))

