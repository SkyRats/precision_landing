import os

from PIL import Image
import numpy as np

def rotate_images_from_folder(path, limit=None):
    input_path = os.path.join(path, 'inputs', 'data')
    mask_path = os.path.join(path, 'masks', 'data')
    files = [file[3:] for file in os.listdir(input_path) 
             if os.path.splitext(file)[1] == '.png']
    inputs = []
    masks = []

    if limit is None:
        limit = len(files)
    for i in range(limit):
        file = files[i]
        input_ = Image.open(os.path.join(input_path, 'Tr-'+file)).convert('L')
        mask = Image.open(os.path.join(mask_path, 'Tes-'+file)).convert('L')

        input_np = np.asarray(input_)
        input_np = np.transpose(input_np)
        mask_np = np.asarray(mask)
        mask_np = np.transpose(mask_np)

        new_input_filename = os.path.join(input_path, 'Tr-'+os.path.splitext(file)[0]+'-90.png')
        new_mask_filename = os.path.join(mask_path, 'Tes-'+os.path.splitext(file)[0]+'-90.png')
        Image.fromarray(input_np).convert('L').save(new_input_filename)
        Image.fromarray(mask_np).convert('L').save(new_mask_filename)

if __name__ == '__main__':
    TRAINING_PATH = os.path.join('Images', 'training')
    TESTING_PATH = os.path.join('Images', 'testing')

    rotate_images_from_folder(TRAINING_PATH)
    rotate_images_from_folder(TESTING_PATH)
