import cv2
import numpy as np

base = cv2.imread("eight.png", cv2.IMREAD_UNCHANGED)


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
    # rectangle_start = [630, 750]
    ### Rechtsbovenin
    rectangle_start = [630, 60]
    ### Linksbovenin
    # rectangle_start = [60, 60]
    ### Linksonderin
    # rectangle_start = [60, 750]

    rectangle_size = [190, 105]

    # rectangle_pos = [(630, 750), (820, 820)]
    rectangle_pos = [(rectangle_start[0], rectangle_start[1]), (rectangle_start[0] + rectangle_size[0], rectangle_start[1] + rectangle_size[1])]
    length_line = 40
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 0.6

    im = cv2.rectangle(im, rectangle_pos[0], rectangle_pos[1], (0, 0, 0), 1)
    red = [np.linspace(255, 155, length_line), np.zeros(length_line), np.linspace(155, 255, length_line)]
    black = [np.linspace(200, 50, length_line), np.linspace(200, 50, length_line), np.linspace(200, 50, length_line)]
    for i in range(length_line):
        im = cv2.line(im, (rectangle_pos[0][0] + 10 + i, rectangle_pos[0][1] + 20),
                      (rectangle_pos[0][0] + 10 + i, rectangle_pos[0][1] + 20), (red[0][i], red[1][i], red[2][i]), 5)
        im = cv2.line(im, (rectangle_pos[0][0] + 10 + i, rectangle_pos[0][1] + 50),
                      (rectangle_pos[0][0] + 10 + i, rectangle_pos[0][1] + 50), (black[0][i], black[1][i], black[2][i]), 5)
        im = cv2.line(im, (rectangle_pos[0][0] + 10 + i, rectangle_pos[0][1] + 80),
                      (rectangle_pos[0][0] + 10 + i, rectangle_pos[0][1] + 80), green, 5)
    im = cv2.putText(im, "= simulation", (rectangle_pos[0][0] + 60, rectangle_pos[0][1] + 25), font, fontScale, (0, 0, 0))
    im = cv2.putText(im, "= ODE model", (rectangle_pos[0][0] + 60, rectangle_pos[0][1] + 55), font, fontScale, (0, 0, 0))
    im = cv2.putText(im, "= trajectory", (rectangle_pos[0][0] + 60, rectangle_pos[0][1] + 85), font, fontScale, (0, 0, 0))


    return im


def eight(im):
    radius_px = 133
    im = cv2.circle(im, (440 - radius_px, 440), radius_px, green, 2)
    im = cv2.circle(im, (440 + radius_px, 440), radius_px, green, 2)

    return im


trajectory = eight(base)
trajectory = legenda(trajectory)
cv2.imshow('displaymywindows', trajectory)
cv2.waitKey(0)g
cv2.destroyAllWindows()
