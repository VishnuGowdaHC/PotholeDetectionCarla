import os
import numpy as np
import tensorflow as tf
from tensorflow.keras.applications import MobileNetV2
from tensorflow.keras import layers, Model
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.callbacks import EarlyStopping, ModelCheckpoint, ReduceLROnPlateau
import matplotlib.pyplot as plt

DATASET_DIR   = "dataset"       
IMG_SIZE      = (224, 224)
BATCH_SIZE    = 32
EPOCHS_FROZEN = 10            
EPOCHS_FINE   = 20             
LEARNING_RATE = 1e-4
MODEL_SAVE    = "pothole_model.h5"
TFLITE_SAVE   = "pothole_model.tflite"
TFLITE_INT8   = "pothole_model_int8.tflite"

def preprocess(image):
    """Convert to grayscale then stack to 3 channels — matches OV7251 output."""
    # image arrives as RGB float32 [0,1]
    gray = tf.image.rgb_to_grayscale(image)        # (224,224,1)
    gray = tf.concat([gray, gray, gray], axis=-1)  # (224,224,3)
    return gray


def build_generators():
    datagen = ImageDataGenerator(
        preprocessing_function=preprocess,
        validation_split=0.2,
        rotation_range=10,
        width_shift_range=0.1,
        height_shift_range=0.1,
        brightness_range=[0.7, 1.3],
        horizontal_flip=True,
        zoom_range=0.1,
        fill_mode='nearest'
    )

    train_gen = datagen.flow_from_directory(
        DATASET_DIR,
        target_size=IMG_SIZE,
        batch_size=BATCH_SIZE,
        class_mode='binary',
        subset='training',
        shuffle=True,
        seed=42
    )

    val_gen = datagen.flow_from_directory(
        DATASET_DIR,
        target_size=IMG_SIZE,
        batch_size=BATCH_SIZE,
        class_mode='binary',
        subset='validation',
        shuffle=False,
        seed=42
    )

    return train_gen, val_gen

def build_model():
    base = MobileNetV2(
        input_shape=(*IMG_SIZE, 3),
        include_top=False,
        weights='imagenet'
    )
    base.trainable = False  # freeze base 

    inputs = tf.keras.Input(shape=(*IMG_SIZE, 3))
    x = base(inputs, training=False)
    x = layers.GlobalAveragePooling2D()(x)
    x = layers.Dropout(0.3)(x)
    x = layers.Dense(128, activation='relu')(x)
    x = layers.Dropout(0.2)(x)
    outputs = layers.Dense(1, activation='sigmoid')(x)  # binary

    model = Model(inputs, outputs)
    return model, base


def train():
    print("\n=== Building generators ===")
    train_gen, val_gen = build_generators()
    print(f"Classes: {train_gen.class_indices}")
    print(f"Train samples: {train_gen.samples} | Val samples: {val_gen.samples}")

    print("\n=== Building model ===")
    model, base = build_model()
    model.compile(
        optimizer=tf.keras.optimizers.Adam(LEARNING_RATE),
        loss='binary_crossentropy',
        metrics=['accuracy']
    )
    model.summary()

    callbacks = [
        EarlyStopping(patience=5, restore_best_weights=True, monitor='val_accuracy'),
        ModelCheckpoint(MODEL_SAVE, save_best_only=True, monitor='val_accuracy'),
        ReduceLROnPlateau(factor=0.5, patience=3, min_lr=1e-7, monitor='val_loss')
    ]

    # Phase 1 — frozen base, train top layers only
    print(f"\n=== Phase 1: Training top layers ({EPOCHS_FROZEN} epochs) ===")
    history1 = model.fit(
        train_gen,
        epochs=EPOCHS_FROZEN,
        validation_data=val_gen,
        callbacks=callbacks
    )

    # Phase 2 — unfreeze last 30 layers, fine tune
    print(f"\n=== Phase 2: Fine tuning last 30 layers ({EPOCHS_FINE} epochs) ===")
    base.trainable = True
    for layer in base.layers[:-30]:
        layer.trainable = False

    model.compile(
        optimizer=tf.keras.optimizers.Adam(LEARNING_RATE / 10),  
        loss='binary_crossentropy',
        metrics=['accuracy']
    )

    history2 = model.fit(
        train_gen,
        epochs=EPOCHS_FINE,
        validation_data=val_gen,
        callbacks=callbacks
    )

    print(f"\n=== Model saved to {MODEL_SAVE} ===")
    return model, train_gen, history1, history2


def convert_tflite(model):
    # Float32 TFLite
    print("\n=== Converting to TFLite float32 ===")
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    tflite_model = converter.convert()
    with open(TFLITE_SAVE, 'wb') as f:
        f.write(tflite_model)
    print(f"Saved: {TFLITE_SAVE} ({os.path.getsize(TFLITE_SAVE) / 1024 / 1024:.1f} MB)")


def convert_tflite_int8(model, train_gen):
    print("\n=== Converting to TFLite INT8 ===")

    def representative_dataset():
        for images, _ in train_gen:
            for img in images:
                yield [np.expand_dims(img.astype(np.float32), axis=0)]
            if train_gen.batch_index == 0:
                break

    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    converter.representative_dataset = representative_dataset
    converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    converter.inference_input_type = tf.uint8
    converter.inference_output_type = tf.uint8

    tflite_int8 = converter.convert()
    with open(TFLITE_INT8, 'wb') as f:
        f.write(tflite_int8)
    print(f"Saved: {TFLITE_INT8} ({os.path.getsize(TFLITE_INT8) / 1024 / 1024:.1f} MB)")


def plot_history(h1, h2):
    acc  = h1.history['accuracy']  + h2.history['accuracy']
    val  = h1.history['val_accuracy'] + h2.history['val_accuracy']
    loss = h1.history['loss'] + h2.history['loss']
    vloss= h1.history['val_loss'] + h2.history['val_loss']

    epochs = range(1, len(acc) + 1)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))

    ax1.plot(epochs, acc, label='Train acc')
    ax1.plot(epochs, val, label='Val acc')
    ax1.axvline(x=EPOCHS_FROZEN, color='r', linestyle='--', label='Unfreeze')
    ax1.set_title('Accuracy')
    ax1.legend()

    ax2.plot(epochs, loss, label='Train loss')
    ax2.plot(epochs, vloss, label='Val loss')
    ax2.axvline(x=EPOCHS_FROZEN, color='r', linestyle='--', label='Unfreeze')
    ax2.set_title('Loss')
    ax2.legend()

    plt.tight_layout()
    plt.savefig('training_plot.png')
    print("Training plot saved to training_plot.png")


def test_tflite_int8(image_path):
    """
    Test the INT8 model on a single image.
    Copy this function to your Pi 4 inference script.
    """
    import cv2

    interpreter = tf.lite.Interpreter(model_path=TFLITE_INT8)
    interpreter.allocate_tensors()

    input_details  = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    img = cv2.resize(img, IMG_SIZE)
    img = np.stack([img, img, img], axis=-1) 
    img = np.expand_dims(img, axis=0).astype(np.uint8)

    interpreter.set_tensor(input_details[0]['index'], img)
    interpreter.invoke()

    output = interpreter.get_tensor(output_details[0]['index'])
    score  = output[0][0] / 255.0  

    label = "POTHOLE" if score > 0.5 else "NORMAL"
    print(f"Result: {label} (confidence: {score:.2f})")
    return label, score


if __name__ == '__main__':
    gpus = tf.config.list_physical_devices('GPU')
    print(f"GPUs available: {gpus}")
    if not gpus:
        print("WARNING: No GPU detected — training will be slow")

    model, train_gen, h1, h2 = train()

    convert_tflite(model)
    convert_tflite_int8(model, train_gen)

    plot_history(h1, h2)

    print("\n=== Done ===")
    print(f"Float32 model : {TFLITE_SAVE}")
    print(f"INT8 model    : {TFLITE_INT8}")
    print(f"Training plot : training_plot.png")