#!/usr/bin/env python
import numpy as np
import rosbag
import rospy
import sys, pygame, math, heapq
from pygame.locals import *

cell_size =  8
num_cells = 101

def initBoard(board):
    background = pygame.Surface(board.get_size())
    background = background.convert()
    background.fill (white)
    
    for i in range(0,(cell_size*num_cells)+1)[::cell_size]:
        pygame.draw.line(background, black, (i, 0), (i, cell_size*num_cells), 2)
        pygame.draw.line(background, black, (0, i), (cell_size*num_cells, i), 2)
    return background

black = (0,0,0)             
bright_green = (0, 204, 102) 
red = (255, 44, 0)          
orange = (255, 175, 0)     
blue = (0, 124, 204)        
white = (250,250,250)       

pygame.init()
size = width, height = (cell_size*num_cells)+2, (cell_size*num_cells)+2
screen = pygame.display.set_mode(size)
pygame.display.set_caption = ('Pathfinder')


board = initBoard(screen)

counta = 0
while 1:
    for event in pygame.event.get():
        if event.type == pygame.QUIT: sys.exit()
        key=pygame.key.get_pressed()
        left_click, middle_click, right_click = pygame.mouse.get_pressed()
        x, y = pygame.mouse.get_pos()
        left = ((x/cell_size)*cell_size)+2
        top = ((y/cell_size)*cell_size)+2
        x_index = (left-2)/cell_size
        y_index = (top-2)/cell_size

        if left_click:
            board = initBoard(screen)
            fileobj = open ( "matrix" + str(counta) + ".txt" , 'r')
            counta+=1
            lst = [ map(int,line.split()) for line in fileobj ]
            for obx,lstobx in enumerate(lst):
            	for oby,lstob in enumerate(lstobx):
            		if(lstob >= 50):
            			x_index = oby
            			y_index = obx 
            			left = (x_index*cell_size)+2
            			top = (y_index*cell_size)+2
            			r = pygame.Rect(left, top, cell_size-2, cell_size-2)
            			pygame.draw.rect(board, black, r, 0)
            fileobj.close()
        screen.blit(board, (0,0))
        pygame.display.flip()