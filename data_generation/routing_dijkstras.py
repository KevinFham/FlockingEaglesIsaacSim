import numpy as np
import matplotlib.pyplot as plt
from os import listdir
from PIL import Image

def pixel_recode(px):
    if list(px) == [68, 1, 84]:
        return 0
    elif list(px) == [32, 144, 140]:
        return 1
    else:
        return 2

def dijkstras():
    pass



if __name__ == '__main__':
    for img_name in listdir("./data"):
        img = np.asarray(Image.open("./data/" + img_name).convert('RGB'))

        bit_map = np.array([list(map(pixel_recode, x)) for x in img], dtype=np.uint8)

        # plt.imshow(bit_map)
        # plt.show()