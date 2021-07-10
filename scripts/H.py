import os

import numpy as np
from PIL import Image
from matplotlib import pyplot as plt

import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras.preprocessing.image import ImageDataGenerator
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
    def __init__(self, input_shape, filters, kernel_sizes, strides, activation, use_batch_norm=False):    
        self.output_channels = 1
        self.input_shape = input_shape
        self.model = self.define_model(input_shape, filters, kernel_sizes, strides, activation, use_batch_norm)
        self.history = None
        self.test_performance = None

    # TODO testar mais combinacoes de parametros
    ## Função de ativação
    ## Padding
    ## Função de perda
    ## Otimizador
    def define_model(self, input_shape, filters, kernel_sizes, strides, activation, use_batch_norm):

        def downsample():
            down_model = keras.Sequential()
            for i in range(len(filters)):
                down_model.add(layers.Conv2D(
                    filters[i],
                    kernel_sizes[i],
                    strides=strides[i],
                    activation=activation,
                    padding='same'))
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
                    activation=activation,
                    padding='same'))
                if use_batch_norm:
                    up_model.add(layers.BatchNormalization())
            return up_model

        last_layer = layers.Conv2DTranspose(self.output_channels,
                                            kernel_sizes[0],
                                            strides=strides[0],
                                            padding='same',
                                            activation='sigmoid')

        inputs = layers.Input(shape=input_shape)
        model = downsample()(inputs)
        model = upsample()(model)
        model = last_layer(model)

        return keras.Model(inputs=inputs, outputs=model)

    def train(self, 
              train_inputs, train_masks, 
              test_inputs, test_masks,
              batch_size, epochs, callbacks=None):
        
        self.model.compile(
            optimizer='adam', 
            loss='binary_crossentropy',
            metrics=[Accuracy(), Precision(), Recall(), ],
        )
        
        self.history = self.model.fit(
            train_inputs, train_masks,
            epochs=epochs,
            callbacks=callbacks,
            batch_size=batch_size
        ).history

    def test(self, test_inputs, test_masks, callbacks=None):
        self.test_performance = self.model.evaluate(
            test_inputs, test_masks,
            callbacks=callbacks,
        )

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
    train = load_images_from_folder(os.path.join(path, 'training'), train_limit)
    test = load_images_from_folder(os.path.join(path, 'testing'), test_limit)
    return train, test

def load_images_from_folder(path, limit):
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

        input_np = np.asarray(input_)/255.0
        mask_np = np.asarray(mask)/255.0

        # entity gambiarra is
        ## input_np = input_np[:, 1:]
        ## mask_np = mask_np[:, 1:]
        ## two_zero_rows = np.zeros((2,52))
        ## input_np = np.concatenate((input_np, two_zero_rows), axis=0)
        ## mask_np = np.concatenate((mask_np, two_zero_rows), axis=0)
        # end gambiarra
        
        inputs.append(input_np.T)
        masks.append(mask_np.T)


    inputs = np.stack(inputs)
    masks = np.stack(masks)

    return inputs, masks

if __name__ == '__main__':

    LOAD_LIMIT = None
    TRAINING_BATCH_SIZE = 4
    TRAINING_EPOCHS = 20
    FILTERS = [32, 64]
    KERNEL_SIZES = [13, 13]
    INPUT_SHAPE = (64, 64, 1)
    VALIDATION_SPLIT = 0.2 # TODO
    ACTIVATION_FUNCTION = 'sigmoid'
    USE_BATCH_NORM = False
    STRIDES = [(2, 2),  (2, 2)]
    TEST_IMAGE_FILENAME = 'Tr-137.png'

    (train_inputs, train_masks), (test_inputs, test_masks) = load_images_for_model('Images', LOAD_LIMIT)
    
    h_mask_model = HMaskModel(INPUT_SHAPE, FILTERS, KERNEL_SIZES, STRIDES, ACTIVATION_FUNCTION, USE_BATCH_NORM)
    h_mask_model.model.summary()

    train_masks = np.where(train_masks > 0.5, 1, 0)
    test_masks = np.where(test_masks > 0.5, 1, 0)

    h_mask_model.train(
        train_inputs, train_masks, 
        test_inputs, test_masks,
        TRAINING_BATCH_SIZE, TRAINING_EPOCHS)

    test_image_path = os.path.join('Images', 'testing', 'inputs', 'data', TEST_IMAGE_FILENAME)
    pgm_test_image_path = 'an2i_left_angry_sunglasses_4.pgm'
    image = Image.open(pgm_test_image_path).resize((INPUT_SHAPE[0], INPUT_SHAPE[1])).convert('L')
    image_np = np.asarray(image).T
    # entity gambiarra is
    ## image_np = image_np[1:, :]
    ## two_zero_rows = np.zeros((52,2))
    ## image_np = np.concatenate((image_np, two_zero_rows), axis=1)
    # end gambiarra
    image_np = np.reshape(image_np, (-1, INPUT_SHAPE[0], INPUT_SHAPE[1], 1))
    h_mask_model.show_mask_from_image(image_np)