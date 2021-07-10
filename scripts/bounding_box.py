import os
from os.path import join
import numpy as np
import cv2 as cv

def load_images(root_dir, inputs_dir, masks_dir,
                input_prefix = 'Tr', mask_prefix = 'Tes'):
    input_images = []
    mask_images = []
    input_path = join(root_dir, inputs_dir)
    mask_path = join(root_dir, masks_dir)
    for input_filename in os.listdir(input_path):
        mask_filename = mask_prefix + input_filename[len(input_prefix):]
        input_images.append(read_image(join(input_path, input_filename)))
        mask_images.append(read_image(join(mask_path, mask_filename)))
    return input_images, mask_images

def read_image(filepath):
    return cv.imread(filepath, cv.IMREAD_GRAYSCALE)

def get_h_bounding_box_center(image):
    contours, _ = cv.findContours(image, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    rects = []
    for contour in contours[1]:
        polygonal_contour = cv.approxPolyDP(contour, 5, True)
        rect = cv.boundingRect(polygonal_contour)
        cv.rectangle(image,
                     (int(rect[0]), int(rect[1])),
                     (int(rect[0]+rect[2]), int(rect[1]+rect[3])),
                     (255, 255, 255), 2)
    cv.imshow('h', image)
    cv.waitKey(0)

    # return x, y

def cut_image_pair(original, mask):

    return cut_original, cut_mask

def save_cut_images(root_directory):
    pass

if __name__ == '__main__':
    inputs, masks = load_images('D:\\CodigosVS\\sky\\H\\Images\\training',
                                'inputs\\data',
                                'masks\\data')
    get_h_bounding_box_center(masks[0])