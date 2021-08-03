import os
from datetime import datetime
from os.path import join

import numpy as np
import pandas as pd
from PIL import Image
from matplotlib import pyplot as plt

import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras.metrics import Accuracy, Precision, Recall

MAX_GPU_MEM_IN_GB = 4

gpus = tf.config.list_physical_devices('GPU')
if gpus:
  # Restrict TensorFlow to only allocate 4GB of memory on the first GPU
  try:
    tf.config.experimental.set_virtual_device_configuration(
        gpus[0],
        [tf.config.experimental.VirtualDeviceConfiguration(memory_limit=1024*MAX_GPU_MEM_IN_GB)])
    logical_gpus = tf.config.experimental.list_logical_devices('GPU')
  except RuntimeError as e:
    # Virtual devices must be set before GPUs have been initialized
    print(e)

class HMaskModel:
    def __init__(self, input_shape):    
        self.output_channels = 1
        self.input_shape = input_shape
        self.model = None
        self.history = None
        self.test_performance = None

    # TODO testar mais combinacoes de parametros
    ## Função de ativação
    ## Padding
    ## Função de perda
    ## Otimizador
    def define_model(self, filters, kernel_sizes, strides, activation, use_batch_norm):

        def downsample():
            down_model = keras.Sequential()
            for i in range(len(filters)):
                down_model.add(layers.Conv2D(
                    filters[i],
                    kernel_sizes[i],
                    strides=strides[i],
                    padding='same'))
                down_model.add(layers.PReLU())
                if use_batch_norm:
                    down_model.add(layers.BatchNormalization())
            return down_model

        def upsample():
            up_model = keras.Sequential()
            if len(filters) == 2:
                filter_indices = [2]
            else:
                filter_indices = list(range(1, len(filters)))
                
            for i in filter_indices:
                reverse_index = len(filters)-i
                up_model.add(layers.Conv2DTranspose(
                    filters[reverse_index],
                    kernel_sizes[reverse_index],
                    strides=strides[reverse_index],
                    padding='same'))
                up_model.add(layers.PReLU())
                if use_batch_norm:
                    up_model.add(layers.BatchNormalization())
            return up_model

        last_layer = layers.Conv2DTranspose(self.output_channels,
                                            kernel_sizes[0],
                                            strides=strides[0],
                                            padding='same',
                                            activation='sigmoid')

        inputs = layers.Input(shape=self.input_shape)
        model = downsample()(inputs)
        model = upsample()(model)
        model = last_layer(model)

        self.model = keras.Model(inputs=inputs, outputs=model)

    def train(self, 
              train_inputs, train_masks, 
              batch_size, epochs, validation_split,
              callbacks=None):
        
        self.model.compile(
            optimizer='adam', 
            loss='binary_crossentropy',
            metrics=[Accuracy(), Precision(), Recall(), ],
        )
        
        self.history = self.model.fit(
            train_inputs, train_masks,
            epochs=epochs,
            callbacks=callbacks,
            batch_size=batch_size,
            validation_split=validation_split,
        ).history

    def test(self, test_inputs, test_masks, callbacks=None):
        self.test_performance = self.model.evaluate(
            test_inputs, test_masks,
            callbacks=callbacks,
            return_dict=True,
        )

    def save(self, save_path):
        self.model.save(save_path)

    def load(self, save_path):
        self.model = keras.models.load_model(save_path)

    def show_mask_from_image(self, image):
        mask = self.model(image, training=False)
        fig, ax = plt.subplots(nrows=1, ncols=2)
        self.plot_image_and_mask(image, ax[0], mask, ax[1])
        fig.savefig('image_mask')

    def show_masks_from_images(self, images, batch_size):
        masks = self.model.predict(
            images, 
            batch_size=batch_size
        )
        self.plot_images_and_masks(images, masks)
        plt.show()

    def plot_images_and_masks(self, images, masks):
        num_images = len(images)
        _, axs = plt.subplots(num_images/2, 2)
        num_axs = len(axs)
        for i in range(0, num_axs-1, 2):
            j = axs//2
            self.plot_image_and_mask(images[j], axs[i], masks[j], axs[i+1])
        
    def plot_image_and_mask(self, image, ax0, mask, ax1):
        image = np.reshape(image, (self.input_shape[0], self.input_shape[1]))
        mask = np.reshape(mask, (self.input_shape[0], self.input_shape[1]))
        ax0.imshow(image)
        ax1.imshow(mask)


def load_images_for_model(path, train_limit=None, test_limit=None):
    train = load_images_from_folder(join(path, 'training'), train_limit)
    test = load_images_from_folder(join(path, 'testing'), test_limit)
    return train, test

def load_images_from_folder(path, limit):
    input_path = join(path, 'inputs', 'data')
    mask_path = join(path, 'masks', 'data')
    files = [file[3:] for file in os.listdir(input_path) 
             if os.path.splitext(file)[1] == '.png']
    inputs = []
    masks = []

    if limit is None:
        limit = len(files)
    for i in range(limit):
        file = files[i]
        input_ = Image.open(join(input_path, 'Tr-'+file)).convert('L')
        mask = Image.open(join(mask_path, 'Tes-'+file)).convert('L')

        input_np = np.asarray(input_)/255.0
        mask_np = np.asarray(mask)/255.0
        
        inputs.append(input_np.T)
        masks.append(mask_np.T)


    inputs = np.stack(inputs)
    masks = np.stack(masks)

    return inputs, masks

if __name__ == '__main__':
    current_datetime = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

    LOAD_LIMIT = 60
    TRAINING_BATCH_SIZE = 4
    TRAINING_EPOCHS = 20
    FILTERS = [32, 64]
    KERNEL_SIZES = [13, 13]
    INPUT_SHAPE = (64, 64, 1)
    VALIDATION_SPLIT = 0.2
    ACTIVATION_FUNCTION = None
    USE_BATCH_NORM = False
    STRIDES = [(2, 2),  (2, 2)]

    TEST_IMAGE_FILENAME = 'Tr-137.png'

    EXPERIMENT_DIR = 'initial'
    EXPERIMENT_SUFFIX = '60_images'

    MODEL_SAVE_PATH = 'models'
    TRAINING_HISTORY_SAVE_DIR = (
        join('results', EXPERIMENT_DIR, 'training'))
    TESTING_HISTORY_SAVE_DIR  = (
        join('results', EXPERIMENT_DIR, 'testing'))

    (train_inputs, train_masks), (test_inputs, test_masks) = load_images_for_model('Images', LOAD_LIMIT)
    
    h_mask_model = HMaskModel(INPUT_SHAPE)
    h_mask_model.define_model(FILTERS, KERNEL_SIZES, STRIDES, ACTIVATION_FUNCTION, USE_BATCH_NORM)
    h_mask_model.model.summary()

    train_masks = np.where(train_masks > 0.5, 1, 0)
    test_masks = np.where(test_masks > 0.5, 1, 0)

    h_mask_model.train(
        train_inputs, train_masks, 
        TRAINING_BATCH_SIZE, TRAINING_EPOCHS,
        VALIDATION_SPLIT)

    h_mask_model.test(
        test_inputs, test_masks)

    # Save a test images
    test_image_path = join('Images', 'testing', 'inputs', 'data', TEST_IMAGE_FILENAME)
    pgm_test_image_path = 'an2i_left_angry_sunglasses_4.pgm'
    image = Image.open(pgm_test_image_path).resize((INPUT_SHAPE[0], INPUT_SHAPE[1])).convert('L')
    image_np = np.asarray(image).T
    image_np = np.reshape(image_np, (-1, INPUT_SHAPE[0], INPUT_SHAPE[1], 1))
    h_mask_model.show_mask_from_image(image_np)

    # Save model
    h_mask_model.save(join(MODEL_SAVE_PATH, current_datetime))

    # Save model results
    os.makedirs(TRAINING_HISTORY_SAVE_DIR,
                exist_ok=True)
    os.makedirs(TESTING_HISTORY_SAVE_DIR,
                exist_ok=True)

    training_history_df = pd.DataFrame(h_mask_model.history)
    with open(join(TRAINING_HISTORY_SAVE_DIR, current_datetime+'-'+EXPERIMENT_SUFFIX+'.csv'),
              mode='w') as f:
        training_history_df.to_csv(f)

    testing_history_df = pd.DataFrame(h_mask_model.test_performance, index=[0])
    with open(join(TESTING_HISTORY_SAVE_DIR, current_datetime+'-'+EXPERIMENT_SUFFIX+'.csv'),
              mode='w') as f:
        testing_history_df.to_csv(f)
