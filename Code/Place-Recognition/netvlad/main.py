import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import tensorflow as tf

from torch.autograd import Variable
from torchvision.models import resnet18

from tensorflow import keras
from keras.preprocessing import image

import PIL
import torch
import torchvision

import matplotlib.pyplot as plt
from sklearn import svm, datasets, metrics

import netvlad
import hard_triplet_loss
import model

torch.manual_seed(777)


###################################### Define Model ##############################################

vlad = model.NetVLADModel()

###################################### Train ##############################################

bef_train_imagenet_data = torchvision.datasets.ImageFolder('/home/jun/Data/Sejong/train', transform=vlad.transforms)
bef_train_data_loader = torch.utils.data.DataLoader(bef_train_imagenet_data,
                                                    batch_size=776,
                                                    shuffle=False,
                                                    num_workers=0)

for bef_train_image, bef_train_label in bef_train_data_loader:
    bef_train_image = bef_train_image
    bef_train_label = bef_train_label

train_imagenet_data = torchvision.datasets.ImageFolder('/home/jun/Data/Sejong/train', transform=vlad.transforms)
train_data_loader = torch.utils.data.DataLoader(train_imagenet_data,
                                                batch_size=8,
                                                shuffle=True,
                                                num_workers=0)
test_imagenet_data = torchvision.datasets.ImageFolder('/home/jun/Data/Sejong/test', transform=vlad.transforms)
test_data_loader = torch.utils.data.DataLoader(test_imagenet_data,
                                               shuffle=False,
                                               batch_size=100,
                                               num_workers=0)

globaliter = 0

for epoch in range(vlad.epochs):
    for batch_idx, (train_image, train_label) in enumerate(train_data_loader):
        output_train = vlad.model(train_image.cuda())
        triplet_loss = vlad.criterion(output_train, train_label.cuda())
        vlad.optimizer.zero_grad()
        triplet_loss.backward(retain_graph=True)
        vlad.optimizer.step()
        # This is where I'm recording to Tensorboard
        # tb.save_value('Train Loss', 'train_loss', globaliter, triplet_loss.item())
        print('epoch : {}, globaliter : {}, batch_idx  : {}, triplet_loss : {}'.format(epoch, globaliter, batch_idx,
                                                                                       triplet_loss.item()))
        globaliter += 1
    model_save_name = 'model_{:02d}.pt'.format(epoch)
    path = F"./{model_save_name}"
    torch.save(vlad.model.state_dict(), path)

########################### Test #####################################

state_dict = torch.load('./model_46.pt')
vlad.model.load_state_dict(state_dict)

out_train_image = vlad.model(bef_train_image.cuda())
X_train = out_train_image
Y_train = bef_train_label

for test_image, test_label in test_data_loader:
    output_test = vlad.model(test_image.cuda())
    X_test = output_test
    Y_test = test_label

from PIL import Image
from sklearn import (manifold, datasets, decomposition, ensemble,
                     discriminant_analysis, random_projection)
import torchvision.transforms.functional as Function
from IPython.display import display
from time import time
from matplotlib import offsetbox
from sklearn.neighbors import DistanceMetric


def plot_embedding(X, y_t, title=None):
    y = y_t.numpy()

    x_min, x_max = np.min(X, 0), np.max(X, 0)
    X = (X - x_min) / (x_max - x_min)

    plt.figure()
    for i in range(X.shape[0]):
        if i == 776:
            plt.text(X[i, 0], X[i, 1], str(y[i]),
                     color='black',
                     fontdict={'weight': 'bold', 'size': 30})
        else:
            plt.text(X[i, 0], X[i, 1], str(y[i]),
                     color=plt.cm.Set1(y[i] / 10.),
                     fontdict={'weight': 'bold', 'size': 9})

    plt.xticks([]), plt.yticks([])
    if title is not None:
        plt.title(title)


check_index = 10

# test_image.shape
# test_label.shape

image_query = X_test[check_index].view(1, -1)
label_query = Y_test[check_index].view(1)
X_total = torch.cat([X_train, image_query], dim=0)
Y_total = torch.cat([Y_train, label_query], dim=0)

test_img = Function.to_pil_image(test_image[check_index])
print('Test image : {}'.format(test_label[check_index].item()))
display(test_img)

tsne = manifold.TSNE(n_components=2, init='pca', random_state=0)
X_tsne = tsne.fit_transform(X_total.cpu().detach().numpy())

pairwise_dist_t = hard_triplet_loss._pairwise_distance(X_total)
pairwise_dist_n = pairwise_dist_t.cpu().detach().numpy()

pairwise_dist_sort = np.sort(pairwise_dist_n[-1][:-1])

test_img = Function.to_pil_image(test_image[check_index])
print('Query image : {}\n'.format(test_label[check_index].item()))
display(test_img)
print("\n")

for ii in range(5):
    idx = np.where(pairwise_dist_n[-1] == pairwise_dist_sort[ii])
    print('{} second similar {} second image : '.format(ii + 1, idx[0][0]))
    img = Function.to_pil_image(bef_train_image[idx[0][0]])
    #     img.save('test{}_{}.png'.format(Y_total[ii],ii))
    display(img)
    print("\n")

plot_embedding(X_tsne, Y_total, "Sejong Landmark data set")
plt.show()
