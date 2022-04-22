import tensorflow as tf
import numpy as np
import os
import shutil

from parser import ParseArgs
from sampler import UniformSampler
from reader import PNGReader
from model import CNNModel
from model2 import CNNModel2

class Train:
    def __init__(self):
        self.settings = ParseArgs()
        self.dataset = PNGReader(self.settings.train_file, self.settings.train_root, self.settings.val_file, self.settings.val_root)
        self.sampler = UniformSampler(self.dataset)
        self.model = CNNModel2(32, 32, 3, self.settings.num_class, self.dataset.mean, self.dataset.std, self.settings.dropout)
        # self.optimizer = tf.keras.optimizers.Adam(learning_rate=self.settings.lr)
        self.optimizer = tf.keras.optimizers.Adam(learning_rate=self.settings.lr, amsgrad=True)
        self.train_loss = tf.keras.losses.SparseCategoricalCrossentropy()
        self.train_loss_metric = tf.keras.metrics.Mean('train_loss',dtype=tf.float32)
        self.train_accuracy_metric = tf.keras.metrics.SparseCategoricalAccuracy('train_accuracy',dtype=tf.float32)
        self.val_loss = tf.keras.losses.SparseCategoricalCrossentropy()
        self.val_loss_metric = tf.keras.metrics.Mean('val_loss',dtype=tf.float32)
        self.val_accuracy_metric = tf.keras.metrics.SparseCategoricalAccuracy('val_accuracy',dtype=tf.float32)

        self.model(np.zeros([1,32,32,3]))
        self.model.summary()
        
        if os.path.isdir(self.settings.output_dir):
            shutil.rmtree(self.settings.output_dir)  # Remove existing output folder
            print("Existing output folder removed.")
    
    @tf.function
    def train_step(self, x, y):
        with tf.GradientTape() as tape:
            # pred = self.model(x)
            pred = self.model(x, training=True)  # This line is changed
            # loss = self.train_loss(y, pred)
            loss = self.train_loss(y, pred) + tf.add_n(self.model.losses)  # The second term is regularization loss
        grads = tape.gradient(loss, self.model.trainable_variables)
        self.optimizer.apply_gradients(zip(grads, self.model.trainable_variables))
        self.train_loss_metric.update_state(loss)
        self.train_accuracy_metric.update_state(y, pred)
        
    @tf.function
    def val_step(self, x, y):
        pred = self.model(x, training=False)
        loss = self.val_loss(y, pred)
        self.val_loss_metric.update_state(loss)
        self.val_accuracy_metric.update_state(y, pred)

    def train(self):
        best_acc = 0
        self.train_writer = tf.summary.create_file_writer(self.settings.tensorboard_dir+'/train')
        self.val_writer = tf.summary.create_file_writer(self.settings.tensorboard_dir+'/val')
        with self.train_writer.as_default():
            tf.summary.graph(self.train_step.get_concrete_function(np.ones([1,32,32,3]),np.ones(1)).graph)

        step = 0
        while (step < self.settings.max_iter):
        #for step in range(self.settings.max_iter):
            x,y = self.sampler.sample_train_batch(self.settings.bs)
            #print(y.shape, x.shape)
            self.train_step(x,y)
            step += 1
            if step%25==0:
                # x,y = self.sampler.sample_val_batch(self.settings.bs)
                x,y = self.sampler.sample_val_batch(1200)
                self.val_step(x,y) 
            if step%100==0:
                print('Step       : ', step)
                print('Train loss : ', float(self.train_loss_metric.result()), 'train accuracy : ', float(self.train_accuracy_metric.result())*100,'%')
                print('Val loss   : ', float(self.val_loss_metric.result()),  'val accuracy   : ', float(self.val_accuracy_metric.result())*100,'%')
                # Early stopping
                if float(self.val_accuracy_metric.result()) > best_acc:
                    best_acc = float(self.val_accuracy_metric.result())
                    print(f"New best val accuracy = {best_acc}, saving model")
                    self.model.save(os.path.join(self.settings.model_dir,'best_model'))
                with self.train_writer.as_default():
                    tf.summary.scalar('loss', self.train_loss_metric.result(), step=step)
                    tf.summary.scalar('accuracy', self.train_accuracy_metric.result(), step=step)
                with self.val_writer.as_default():
                    tf.summary.scalar('loss', self.val_loss_metric.result(), step=step)
                    tf.summary.scalar('accuracy', self.val_accuracy_metric.result(), step=step)
                self.train_loss_metric.reset_state()
                self.train_accuracy_metric.reset_state()
                self.val_loss_metric.reset_state()
                self.val_accuracy_metric.reset_state()
                print(f'Best val acc up to now : {best_acc}')
            if step%1000 == 0:
            	layer1_wts = np.array(self.model.layers[0].get_weights(), dtype=object)
            	layer1_wts = np.concatenate([i.flatten() for i in layer1_wts])
            	layer2_wts = np.array(self.model.layers[1].get_weights(), dtype=object)
            	layer2_wts = np.concatenate([i.flatten() for i in layer2_wts])
            	layer3_wts = np.array(self.model.layers[2].get_weights(), dtype=object)
            	layer3_wts = np.concatenate([i.flatten() for i in layer3_wts])
            	layer4_wts = np.array(self.model.layers[3].get_weights(), dtype=object)
            	layer4_wts = np.concatenate([i.flatten() for i in layer4_wts])
            	layer5_wts = np.array(self.model.layers[4].get_weights(), dtype=object)
            	layer5_wts = np.concatenate([i.flatten() for i in layer5_wts])
            	layer7_wts = np.array(self.model.layers[6].get_weights(), dtype=object)
            	layer7_wts = np.concatenate([i.flatten() for i in layer7_wts])
            	layer9_wts = np.array(self.model.layers[8].get_weights(), dtype=object)
            	layer9_wts = np.concatenate([i.flatten() for i in layer9_wts])
            	print('Conv1 abs mean : ', np.mean(np.abs(layer1_wts)), 'std : ', np.std(layer1_wts))
            	print('Norm1 abs mean : ', np.mean(np.abs(layer2_wts)), 'std : ', np.std(layer2_wts))
            	print('Conv2 abs mean : ', np.mean(np.abs(layer3_wts)), 'std : ', np.std(layer3_wts))
            	print('Norm2 abs mean : ', np.mean(np.abs(layer4_wts)), 'std : ', np.std(layer4_wts))
            	print('Conv3 abs mean : ', np.mean(np.abs(layer5_wts)), 'std : ', np.std(layer5_wts))
            	print('Dense1 abs mean: ', np.mean(np.abs(layer7_wts)), 'std : ', np.std(layer7_wts))
            	print('Dense2 abs mean: ', np.mean(np.abs(layer9_wts)), 'std : ', np.std(layer9_wts))
            # if step%250==0:
                # self.model.save(os.path.join(self.settings.model_dir,'step_'+str(step)))
        print('Training done ! ^_^')

if __name__ == '__main__':
    tf.keras.utils.set_random_seed(42)  # Sets all random seeds for the program (Python, NumPy, and TF)
    T = Train()
    T.train()
