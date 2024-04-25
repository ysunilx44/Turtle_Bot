#!/usr/bin/env python3
import cv2
import sys
import csv
import time
import math
import numpy as np
import random
import os

### Load training images and labels

# imageDirectory = 'C:\Users\yadum\GT\Spring 2024\ME 7785\Lab 6\2023Fimgs\2023Fimgs'
#os.listdir(imageDirectory)
imageDirectory = os.path.join(os.getcwd(), '2023Fimgs', '2023Fimgs')
labelsPath = os.path.join(imageDirectory, 'labels.txt')
print(imageDirectory)

with open(labelsPath, 'r') as f:
    reader = csv.reader(f)
    lines = list(reader)
print(len(lines))
print(lines)
#Randomly choose train and test data (50/50 split).
random.shuffle(lines)
train_lines = lines[:math.floor(len(lines)/2)][:]
# print(train_lines)
test_lines = lines[math.floor(len(lines)/2):][:]

# this line reads in all images listed in the file in color, and resizes them to 25x33 pixels
train = np.array([np.array(cv2.resize(cv2.imread(imageDirectory+"\\"+train_lines[i][0]+".jpg"),(25,33))) for i in range(len(train_lines))])

# here we reshape each image into a long vector and ensure the data type is a float (which is what KNN wants), note the *3 is due to 3 channels of color.
train_data = train.flatten().reshape(len(train_lines), 33*25*3)
train_data = train_data.astype(np.float32)

# read in training labels
train_labels = np.array([np.int32(train_lines[i][1]) for i in range(len(train_lines))])


### Train classifier
knn = cv2.ml.KNearest_create()
knn.train(train_data, cv2.ml.ROW_SAMPLE, train_labels)

# if(__debug__):
# 	Title_images = 'Original Image'
# 	Title_resized = 'Image Resized'
# 	cv2.namedWindow( Title_images, cv2.WINDOW_AUTOSIZE )

correct = 0.0
confusion_matrix = np.zeros((6,6))

k = 7

for i in range(len(test_lines)):
    original_img = cv2.imread(imageDirectory+"\\"+test_lines[i][0]+".jpg")
    test_img = np.array(cv2.resize(cv2.imread(imageDirectory+"\\"+test_lines[i][0]+".jpg"),(25,33)))
    # if(__debug__):
    #     cv2.imshow(Title_images, original_img)
    #     cv2.imshow(Title_resized, test_img)
    #     key = cv2.waitKey()
    #     if key==27:    # Esc key to stop
    #         break
    test_img = test_img.flatten().reshape(1, 33*25*3)
    test_img = test_img.astype(np.float32)

    test_label = np.int32(test_lines[i][1])

    ret, results, neighbours, dist = knn.findNearest(test_img, k)

    if test_label == ret:
        print(str(lines[i][0]) + " Correct, " + str(ret))
        correct += 1
        confusion_matrix[np.int32(ret)][np.int32(ret)] += 1
    else:
        confusion_matrix[test_label][np.int32(ret)] += 1
        
        print(str(test_lines[i][0]) + " Wrong, " + str(test_label) + " classified as " + str(ret))
        print("\tneighbours: " + str(neighbours))
        print("\tdistances: " + str(dist))



print("\n\nTotal accuracy: " + str(correct/len(test_lines)))
print(confusion_matrix)
