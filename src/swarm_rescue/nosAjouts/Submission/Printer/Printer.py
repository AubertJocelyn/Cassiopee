import copy

import cv2
import numpy as np


class Printer:
    def __init__(self):
        self.compteur = 0

    def show(self, grid_to_display, listPoints):
        grid_customisable = copy.deepcopy(grid_to_display)
        grid_customisable.reset_grid()
        if grid_customisable is not None:
            if (listPoints is not None) and (listPoints != [None]):
                for point in listPoints:
                    grid_customisable[point[0], point[1]] = 1

        self.display(grid_to_display.zoomed(), grid_customisable.zoomed(),
                     title="untitled")

    def update(self, grid, point):
        self.compteur += 1
        if self.compteur % 30 == 0:
            self.show(grid, point)

    @staticmethod
    def display(grid_to_display: np.ndarray, grid_customisable: np.ndarray, title: str = "grid"):


        img = grid_to_display.T
        img = img - img.min()
        img = img / img.max() * 255
        img = np.uint8(img)
        img_color = cv2.applyColorMap(src=img, colormap=cv2.COLORMAP_JET)

        ###ajout chat GPT pour afficher le drone de manière consistante
        mask = (grid_customisable != 0).T
        img_color[mask] = [255, 255, 255]

        cv2.imshow(title, img_color)
        cv2.waitKey(1)