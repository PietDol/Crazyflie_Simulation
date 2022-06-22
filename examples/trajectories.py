import cv2
import numpy as np

base = cv2.imread("../Crazyflie_Simulation/solid/Rendering/final_image.png", cv2.IMREAD_UNCHANGED)


# base = np.zeros((880, 880, 3), np.uint8)
# base[:, :] = (255, 255, 255)


#  image = cv2.circle(image, center_coordinates, radius, color, thickness)
# im = cv2.circle(im, (440, 440), 2, (255, 0, 0), 2)
# radius = 1 meter = 133 pixels
# radius_px = 133
# im = cv2.circle(im, (440 - radius_px, 440), radius_px, (255, 0, 0), 2)
# im = cv2.circle(im, (440 + radius_px, 440), radius_px, (255, 0, 0), 2)

green = (0, 255, 0)

def legenda(im):
    ### Rechtsonderin
    # rectangle_start = [940-190-70, 940-105-70]
    ### Rechtsbovenin
    rectangle_start = [940-250-70, 70]
    ### Linksbovenin
    # rectangle_start = [60, 60]
    ### Linksonderin
    # rectangle_start = [60, 750]

    rectangle_size = [250, 145]

    # rectangle_pos = [(630, 750), (820, 820)]
    rectangle_pos = [(rectangle_start[0], rectangle_start[1]), (rectangle_start[0] + rectangle_size[0], rectangle_start[1] + rectangle_size[1])]
    length_line = 40
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 0.8

    im = cv2.rectangle(im, rectangle_pos[0], rectangle_pos[1], (0, 0, 0), 1)
    red = [np.linspace(255, 155, length_line), np.zeros(length_line), np.linspace(155, 255, length_line)]
    black = [np.linspace(200, 50, length_line), np.linspace(200, 50, length_line), np.linspace(200, 50, length_line)]
    for i in range(length_line):
        im = cv2.line(im, (int(rectangle_pos[0][0] + 10 + i*1.5), rectangle_pos[0][1] + 60),
                      (rectangle_pos[0][0] + 10 + i, rectangle_pos[0][1] + 60), (red[0][i], red[1][i], red[2][i]), 10)
        im = cv2.line(im, (int(rectangle_pos[0][0] + 10 + i*1.5), rectangle_pos[0][1] + 30),
                      (rectangle_pos[0][0] + 10 + i, rectangle_pos[0][1] + 30), (black[0][i], black[1][i], black[2][i]), 10)
        im = cv2.line(im, (int(rectangle_pos[0][0] + 10 + i*1.5), rectangle_pos[0][1] + 90),
                      (rectangle_pos[0][0] + 10 + i, rectangle_pos[0][1] + 90), green, 10)
    im = cv2.putText(im, "1-4", (rectangle_pos[0][0] + 10, rectangle_pos[0][1] + 125), font, fontScale,
                     green, 2)
    im = cv2.putText(im, "= simulation", (rectangle_pos[0][0] + 80, rectangle_pos[0][1] + 35), font, fontScale, (0, 0, 0), 2)
    im = cv2.putText(im, "= ODE model", (rectangle_pos[0][0] + 80, rectangle_pos[0][1] + 65), font, fontScale, (0, 0, 0), 2)
    im = cv2.putText(im, "= trajectory", (rectangle_pos[0][0] + 80, rectangle_pos[0][1] + 95), font, fontScale, (0, 0, 0), 2)
    im = cv2.putText(im, "= points", (rectangle_pos[0][0] + 80, rectangle_pos[0][1] + 125), font, fontScale,
                     (0, 0, 0), 2)


    return im


def eight(im):
    radius_px_x = 160
    x_mid = 470
    radius_px_y = 348
    y_mid = 488
    # im = cv2.circle(im, (470 - radius_px, 470), radius_px, green, 2)
    im = cv2.ellipse(im, ((x_mid - radius_px_x), y_mid), (radius_px_x, radius_px_y),
                0, 0, 360, green, 2)
    # im = cv2.circle(im, (470 + radius_px, 470), radius_px, green, 2)
    im = cv2.ellipse(im, ((x_mid + radius_px_x), y_mid), (radius_px_x, radius_px_y),
                     0, 0, 360, green, 2)

    return im

def line(im):
    # points 2 0, -2.5 1, 2.5 1, -2.5 1
    points = [(470,137),(137,803),(803,803),(137,803)]

    im = cv2.putText(im, str("1"), points[0], cv2.FONT_HERSHEY_PLAIN, 3, green, 5)
    im = cv2.putText(im, str("2,4"), points[1], cv2.FONT_HERSHEY_PLAIN, 3, green, 5)
    im = cv2.putText(im, str("3"), points[2], cv2.FONT_HERSHEY_PLAIN, 3, green, 5)

    return im

def rectangle(im):
    # points -1 1, 1 1, 1 3, -1 3, -1 1
    # y1 = 803
    # y2 = 137
    # x-1 = 203
    # x1 = 737

    im = cv2.rectangle(im, (203,803), (737,137), green, 2)

    return im

def triangle(im):
    # points -1 1, 1 1, 0 2.73, -1 1
    # y1 = 797
    # y2.73 = 168

    # x-1 = 203
    # x1 = 737

    im = cv2.line(im, (203, 797), (737, 797), green, 2)
    im = cv2.line(im, (737, 797), (470, 168), green, 2)
    im = cv2.line(im, (203, 797), (470, 168), green, 2)

    return im


# trajectory = eight(base) # xrange = [-2.5, 2.5] yrange = [0.9, 3.2] 940x940px 70px offset
trajectory = line(base) # xrange = [-3, 3] yrange = [0.9, 2.1]
# trajectory = rectangle(base) # xrange = [-1.5, 1.5] yrange = [0.8, 3.2]
# trajectory = triangle(base) # xrange = [-1.5, 1.5] yrange = [0.8, 3]
trajectory = legenda(trajectory)
cv2.imshow('displaymywindows', trajectory)
cv2.waitKey(0)
cv2.destroyAllWindows()
