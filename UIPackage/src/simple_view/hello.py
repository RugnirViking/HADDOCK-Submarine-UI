#!/usr/bin/env python

import rospy
import math
import pygame, sys
import pygame.camera
import os 
import datetime
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int16

white = (255, 255, 255) 
darkblue = (10, 0, 60) 
black = (0, 0, 0)


navBallSizeN = 360

camSize = (320,240)

screenXSize = 1200
screenYSize = 800

roll = 0
pitch = 0
heading = 360
time = 0
depth = 10


def IMU_message_recieved(data):
	global roll
	global pitch
	global heading
	roll = data.x
	pitch = data.y
	heading = data.z
	rospy.logdebug("IMU Message recieved. Values: "+str(roll)+" : "+str(pitch)+" : "+str(heading))
def Depth_message_recieved(data):
	global depth
	depth = data.data
	rospy.logdebug("Depth Message recieved. Value: "+str(data.data))

def rot_center(image, angle):
	#"""rotate an image while keeping its center and size"""
	orig_rect = image.get_rect()
	rot_image = pygame.transform.rotate(image, angle)
	rot_rect = orig_rect.copy()
	rot_rect.center = rot_image.get_rect().center
	rot_image = rot_image.subsurface(rot_rect).copy()
	return rot_image
def NavBallImg(navImgZ, screen, X, Y, roll, pitch, heading):
        navballSize = (navBallSizeN,navBallSizeN)
        #navballSize = (450,450)

        nsX, nsY = navballSize
        radius = nsX/2
        navball = pygame.Surface(navballSize)
        niX, niY = navImgZ.get_size()
        #print "Pitch: " + str(pitch) + " Roll: " + str(roll)
        #Roll
        navImg = pygame.Surface((niX,niY))
        text = str(int(round(heading)))+"'"
        color = (255,255,255)
        font = pygame.font.Font(None, 28)
        tx, ty = font.size(text)
        navImg.blit(navImgZ, (0, (navBallSizeN/180)*pitch))
        navball.blit(navImg, ( -niX/2 + nsX/2, -niY/2 + nsY/2  ))
        navball.blit(rot_center(navball, roll), (0,0))
        navball.blit(font.render(text, 1, color), (nsX/2-ty/2+100, nsY/2- ty/2))
        screen.blit(navball, (X - nsX/2,Y - nsY/2 -2))

def say(name):
	global roll
	global pitch
	global heading
	global time
	global depth


	# basic setup
	rospy.init_node('python_sub_controller', anonymous=True) # node is called 'python_sub_controller'
	rospy.Subscriber("imu_values_topic", Vector3, IMU_message_recieved)
	rospy.Subscriber("depth_topic", Int16, Depth_message_recieved)
	rospy.loginfo("Loading Control Screen...")
	pygame.init()
	pygame.camera.init()
	windowSurfaceObj= pygame.display.set_mode((screenXSize,screenYSize))

	# UI setup

	pygame.display.set_caption('Submarine Prototype')
	title_font = pygame.font.Font('freesansbold.ttf', 32) 
	info_font = pygame.font.Font('freesansbold.ttf', 12) 
	camera_overlay_font = pygame.font.Font('freesansbold.ttf', 12) 

	carImg = pygame.image.load('resources/aaulogo.png') # AAU Logo
	#navImg = pygame.image.load('resources/navball.png') # Navball

	# Cameras Setup

	clist = pygame.camera.list_cameras()
	if not clist:
		raise ValueError("Sorry, no cameras detected.")
	cam = pygame.camera.Camera(clist[0], camSize,"RGB")
	cam.start()
	print(cam.get_size())

    # create a surface to capture to.  for performance purposes
    # bit depth is the same as that of the display surface.
	snapshot = pygame.surface.Surface(camSize, 0, windowSurfaceObj)
	snapshot2 = pygame.surface.Surface(camSize, 0, windowSurfaceObj)
	snapshot3 = pygame.surface.Surface(camSize, 0, windowSurfaceObj)
	snapshot4 = pygame.surface.Surface(camSize, 0, windowSurfaceObj)

	# setup navball
	navImg = pygame.Surface((1400,1400))
	pygame.draw.rect(navImg, (128,128,255), (0, 0,1400, 700), 0)
	pygame.draw.rect(navImg, (128,64,0), (0, 700,1400, 1400), 0)

	navImg.convert()
	navImg = pygame.transform.scale(navImg, (1400,1400))

	# Define update rate, which includes max fps. Having both ros 
	# messaging and graphics in the same thread is probably bad theoretically, but 
	# hopefully we can get away with it by not being too taxing on the gpu
	# that it starts to fall below 50 frames per second.

	rate = rospy.Rate(50) # 50hz
	time=0
	rospy.loginfo("Control Screen Finished Loading!")
	while not rospy.is_shutdown(): 
		## Each of the stuff below here is done on every frame
		time+=1
		###
		### GRAPHICS GOES BELOW HERE \/  \/  \/
		###
		hello_str = "HADDOCK Submarine Simulation"
		windowSurfaceObj.fill(white)
		text = title_font.render(hello_str, True, darkblue, white) 
		textRect = text.get_rect()  
		textRect.topleft = (screenXSize*0.15, 32) 
		windowSurfaceObj.blit(text, textRect) 

		currentPyTime = datetime.datetime.fromtimestamp(rospy.get_time())
		time_text = title_font.render("T = "+currentPyTime.isoformat(' '), True, darkblue, white) 
		time_textRect = time_text.get_rect()  
		time_textRect.topleft = (screenXSize*0.6, 32) 
		windowSurfaceObj.blit(time_text, time_textRect) 


		x =  (screenXSize * 0.025)
		y = (screenYSize * 0.3)
		imgRect = carImg.get_rect()
		
		windowSurfaceObj.blit(pygame.transform.scale(carImg,(134,122)), (36,36))

		# DRAW ROLL PITCH YAW
		roll_text = info_font.render("Roll (deg): "+str(int(round(roll,4))), True, darkblue, white) 
		roll_textRect = roll_text.get_rect()  
		roll_textRect.topleft = (x+328, 200) 
		windowSurfaceObj.blit(roll_text, roll_textRect) 

		pitch_text = info_font.render("Pitch (deg): "+str(int(round(pitch,4))), True, darkblue, white) 
		pitch_textRect = pitch_text.get_rect()  
		pitch_textRect.topleft = (x+328, 216) 
		windowSurfaceObj.blit(pitch_text, pitch_textRect) 

		heading_text = info_font.render("Heading (deg): "+str(int(round(heading,4))), True, darkblue, white) 
		heading_textRect = heading_text.get_rect()  
		heading_textRect.topleft = (x+328, 232) 
		windowSurfaceObj.blit(heading_text, heading_textRect) 

		depth_text = info_font.render("Depth (m): "+str(int(round(depth,4))), True, darkblue, white) 
		depth_textRect = depth_text.get_rect()  
		depth_textRect.topleft = (x+328, 264) 
		windowSurfaceObj.blit(depth_text, depth_textRect) 

		if cam.query_image():
			snapshot = cam.get_image(snapshot)

		windowSurfaceObj.blit(snapshot, (x,200))
		#pygame.camera.colorspace(snapshot, "HSV", snapshot2)
		#windowSurfaceObj.blit(snapshot2, (x+400,100))
		fCamText = camera_overlay_font.render("CAMERA FEED [001] - FRONT", True, darkblue, white) 
		fCamtextRect = fCamText.get_rect()  
		fCamtextRect.topleft = (x+4, 204) 
		windowSurfaceObj.blit(fCamText, fCamtextRect) 
		
		pygame.camera.colorspace(snapshot, "HSV", snapshot3)
		windowSurfaceObj.blit(snapshot3, (x,200+250))

		rCamText = camera_overlay_font.render("CAMERA FEED [002] - REAR", True, darkblue, white) 
		fCamtextRect = rCamText.get_rect()  
		fCamtextRect.topleft = (x+4, 454) 
		windowSurfaceObj.blit(rCamText, fCamtextRect) 

		
		navBallPos=(600-navBallSizeN/2,508-navBallSizeN/2)
		NavBallImg(navImg,windowSurfaceObj,600,510,roll,pitch,heading)


 		boxX, boxY = (navBallSizeN,navBallSizeN)
		navCentreX, navCentreY = navBallPos
		#boxX, boxY = (525,525)
		sidecut = 5.0 #this is a fraction
		PFDbackground = pygame.Surface((boxX,boxY))
		PFDbackground.set_colorkey((255,255,254))
		PFDbackground.fill((255, 255, 255))
		pygame.draw.circle(PFDbackground, (255,255,254), (boxX/2, boxY/2), (boxY)/2, 0)
		#pygame.draw.rect(PFDbackground, (0,0,0), (0, 0,boxX/sidecut, boxY), 0)
		windowSurfaceObj.blit(PFDbackground,navBallPos)

		pygame.draw.rect(windowSurfaceObj, (0,200,0), (navCentreX+100+boxX-boxX/sidecut, navCentreY,20, boxY), 0)
		pygame.draw.rect(windowSurfaceObj, (0,200,0), (navCentreX, navCentreY-40,boxX, 20), 0)
		pygame.draw.rect(windowSurfaceObj, (0,140,0), (navCentreX+100+boxX-boxX/sidecut, navCentreY+boxY*0.4,19, boxY*0.2), 2)
		pygame.draw.rect(windowSurfaceObj, (0,140,0), (navCentreX+boxX*0.4, navCentreY-40,boxX*0.2, 19), 2)

		spiritLevel_padding = 4
		pitchSpiritMinY=navCentreY+spiritLevel_padding
		pitchSpiritY=boxY*0.8
		pitchSpiritBubbleY=pitchSpiritMinY
		if pitch>5:
			pitchSpiritBubbleY = pitchSpiritBubbleY+pitchSpiritY
		elif pitch<-5:
			pitchSpiritBubbleY=pitchSpiritMinY
		else: 
			pitchSpiritBubbleY = pitchSpiritMinY+(0.5+pitch*0.1)*pitchSpiritY
		pygame.draw.rect(windowSurfaceObj, (0,140,0), (navCentreX+100+boxX-boxX/sidecut+spiritLevel_padding,pitchSpiritBubbleY ,20-spiritLevel_padding*2, 1+boxY*0.2-spiritLevel_padding*2), 0)

		rollSpiritMinX=navCentreX+spiritLevel_padding
		rollSpiritX=boxX*0.8
		rollSpiritBubbleX=rollSpiritMinX
		if roll>5:
			rollSpiritBubbleX = rollSpiritBubbleX+rollSpiritX
		elif roll<-5:
			rollSpiritBubbleX=rollSpiritMinX
		else: 
			rollSpiritBubbleX = rollSpiritMinX+(0.5+roll*0.1)*rollSpiritX
		pygame.draw.rect(windowSurfaceObj, (0,140,0), (rollSpiritBubbleX, navCentreY-37,boxX*0.2-spiritLevel_padding*2, 18-spiritLevel_padding), 0)

		pygame.draw.rect(windowSurfaceObj, white, (navCentreX+boxX/2-boxX/5/2,navCentreY+boxY/2,boxX/5,5), 2)
		pygame.draw.line(windowSurfaceObj, white, (navCentreX+boxX/2-boxX/5, navCentreY+boxY/2), (navCentreX+boxX/2+boxX/5, navCentreY+boxY/2), 3)

		###
		### GRAPHICS GOES ABOVE HERE /\   /\   /\
		###

		pygame.display.update()
		# if the user presses the cross button on the window, close it.
		for event in pygame.event.get() : 
			if event.type == pygame.QUIT : 
				cam.stop()
				pygame.quit() 
				quit() 
		rate.sleep()
