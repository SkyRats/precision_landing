import os
import  numpy as np
from PIL import UnidentifiedImageError
from PIL import Image

INPUT_SHAPE = (64,64)
BASE_DIR = 'faces'
INPUT_DIR = 'inputs'
MASK_DIR = 'masks'
WRITTEN_IMAGE_EXTENSION = '.png'

counter = 0

def parse_images_from_directory(dir):
    global counter
    for filename in os.listdir(dir):
        if filename.endswith('.pgm'):
            filepath = os.path.join(dir, filename)
            try:
                image = Image.open(filepath)
                input_ = image.resize(INPUT_SHAPE)
                mask = create_empty_image()
                save_images(os.path.splitext(filename)[0],
                            input_,
                            mask)
            except UnidentifiedImageError:
                pass

def create_empty_image():
    mask = np.zeros(INPUT_SHAPE)
    return Image.fromarray(mask)

def save_images(filename, input_image, mask_image):
    input_path = os.path.join(BASE_DIR, INPUT_DIR, 'Tr-'+filename+WRITTEN_IMAGE_EXTENSION)
    mask_path = os.path.join(BASE_DIR, MASK_DIR, 'Tes-'+filename+WRITTEN_IMAGE_EXTENSION)
    input_image.convert('L').save(input_path)
    mask_image.convert('L').save(mask_path)

if __name__ == '__main__':
    parse_images_from_directory('faces')