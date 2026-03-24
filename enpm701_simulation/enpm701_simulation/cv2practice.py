import cv2
import numpy as np

img = np.zeros((500, 500, 3), dtype=np.uint8) * 255
robot_pos = [-1.2192, -1.2192, 3.1515/4]

def xy_to_pixel(x, y):
    pixel_x = int((x + (3.048/2)) / 3.048 * 500)
    pixel_y = 500 - int((y + (3.048/2)) / 3.048 * 500)
    return pixel_x, pixel_y
const_x, const_y = xy_to_pixel(-0.3084, 0.3084)
print(const_x, const_y)
land_x, land_y = xy_to_pixel(-0.9144, -0.9144)
print(land_x, land_y)
robot_x, robot_y = xy_to_pixel(robot_pos[0], robot_pos[1])
cv2.rectangle(img, (0, 0), (const_x, const_y), (100, 100, 100), -1)
cv2.rectangle(img, (0,500), (land_x, land_y), (100, 100, 100), -1)
cv2.circle(img, (robot_x, robot_y), 10, (255, 255, 255), -1)
cv2.line(img, (robot_x, robot_y), (robot_x + int(10 * np.cos(robot_pos[2])), robot_y - int(10 * np.sin(robot_pos[2]))), (0, 0, 0), 2) 
cv2.imshow('map', img)
cv2.waitKey(0)
cv2.destroyAllWindows()