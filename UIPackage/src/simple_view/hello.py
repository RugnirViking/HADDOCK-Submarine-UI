#!/usr/bin/env python

import cmd
import subprocess
import rospy
import math
import pygame, sys
import pygame.camera
import pygame.locals 
import os 
import datetime
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import String
from sensor_msgs.msg import Image

white = (255, 255, 255) 
darkblue = (10, 0, 60) 
black = (0, 0, 0)
top_speed = 2.5

navBallSizeN = 360

camSize = (320,240)

screenXSize = 1350
screenYSize = 800

roll = 0
pitch = 0
heading = 360
time = 0
depth = 10

pitch_input_val = 0
roll_input_val = 0
heading_input_val = 0
altitude_input_val = 0

cmd_response_text = ""


mode1_enabled = True
mode2_enabled = False
mode3_enabled = False
control_enabled = True
brush_enabled=False
emergencyStop_enabled=False
emergencyStop_reallyEnabled=False
logging_enabled=False
verbose_enabled=False
current_ros_image = None
current_ros_rear_image = None
currentLogfile = None
img_in=False
rimg_in=False

xSpeed=0
ySpeed=0
zSpeed=0
bleh = 0

class Shell_Class(cmd.Cmd):
    """Simple command processor example."""
    
    # Disable rawinput module use
    use_rawinput = False
    
    last_output = ''

    def do_shell(self, line):
		global cmd_response_text
        #"Run a shell command"
		print "running shell command:", line
		try:
			cmd = subprocess.Popen(line,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
			output, error = cmd.communicate()
			#print cmd_response_text
			if(error):
				print(error)
			else:
				self.last_output = output
				cmd_response_text = output

		except OSError as e: 
			print("inside exception", e)
			self.last_output = str(e)
			cmd_response_text = str(e)
		except AttributeError as e: 
			print("inside exception", e)
			self.last_output = str(e)
			cmd_response_text = str(e)
		except:
			cmd_response_text = sys.exc_info()[0]
		#print output
    
    def do_echo(self, line):
        "Print the input, replacing '$out' with the output of the last shell command"
        # Obviously not robust
        print line.replace('$out', self.last_output)
    
    def do_greet(self, line):
        print "hello,", line
    
    def do_EOF(self, line):
        return True

def IMU_message_received(data):
	global roll
	global pitch
	global heading
	roll = data.x
	pitch = data.y
	heading = data.z
	rospy.logdebug("IMU Message recieved. Values: "+str(roll)+" : "+str(pitch)+" : "+str(heading))

def Depth_message_received(data):
	global depth
	depth = data.data
	rospy.logdebug("Depth Message recieved. Value: "+str(data.data))

def log_message_received(data):
	global logging_enabled
	global verbose_enabled

	if logging_enabled and verbose_enabled:
		currentLogfile.write(data.data+"\n")
	elif logging_enabled:
		currentLogfile.write(data.data.split("||")[0]+"\n")

def image_received(data):
	global current_ros_image
	global img_in
	img_in = True
	current_ros_image = data.data[::-1]

def rear_image_received(data):
	global current_ros_rear_image
	global rimg_in
	rimg_in = True
	current_ros_rear_image = data.data[::-1]

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
	# draw rectangle
    navImg.blit(navImgZ, (0, (navBallSizeN/180)*pitch))
    navball.blit(navImg, ( -niX/2 + nsX/2, -niY/2 + nsY/2  ))
    navball.blit(rot_center(navball, roll), (0,0))

	# draw heading indicator
    navball.blit(font.render(text, 1, color), (nsX/2-ty/2+100, nsY/2- ty/2))
    screen.blit(navball, (X - nsX/2,Y - nsY/2 -2))


def Speed_message_received(data):
	global xSpeed
	global ySpeed
	global zSpeed
	xSpeed = data.x
	ySpeed = data.y
	zSpeed = data.z
	rospy.logdebug("Speed Message recieved. Values: "+str(xSpeed)+" : "+str(ySpeed)+" : "+str(zSpeed))

def heading_speed_recieved(data):
	global bleh
	bleh = data.data
	rospy.logdebug("bleh recieved. Value: "+str(bleh))


def say(name):
	global roll
	global pitch
	global heading
	global time
	global depth
	global cmd_response_text
	global pitch_input_val
	global roll_input_val
	global heading_input_val
	global altitude_input_val
	global mode1_enabled 
	global mode2_enabled 
	global mode3_enabled 
	global control_enabled 
	global brush_enabled
	global emergencyStop_enabled
	global logging_enabled
	global verbose_enabled
	global emergencyStop_reallyEnabled
	global currentLogfile
	global img_in
	global rimg_in
	global xSpeed
	global ySpeed
	global zSpeed
	global bleh

	console_active=False


	# basic setup
	rospy.init_node('python_sub_controller', anonymous=True) # node is called 'python_sub_controller'
	rospy.Subscriber("imu_values_topic", Vector3, IMU_message_received)
	rospy.Subscriber("python_submarine_speeds", Vector3, Speed_message_received)
	
	rospy.Subscriber("depth_topic", Float32, Depth_message_received)
	rospy.Subscriber("python_submarine_logger", String, log_message_received)
	rospy.Subscriber("python_submarine_camera_images", Image, image_received)
	rospy.Subscriber("python_submarine_rear_camera_images", Image, rear_image_received)
	rospy.Subscriber("python_submarine_heading_speed", Float32, heading_speed_recieved)

	pitch_pub = rospy.Publisher('pitch_control_input', Int16, queue_size=10)
	roll_pub = rospy.Publisher('roll_control_input', Int16, queue_size=10)
	heading_pub = rospy.Publisher('heading_control_input', Int16, queue_size=10)
	altitude_pub = rospy.Publisher('altitude_control_input', Int16, queue_size=10)

	mode_pub = rospy.Publisher('control_mode', Int16, queue_size=10)
	brush_pub = rospy.Publisher('brush_toggle', Int16, queue_size=10)
	emergencyStopPub = rospy.Publisher('r', Int16, queue_size=10)

	rospy.loginfo("Loading Control Screen...")
	pygame.init()
	pygame.camera.init()
	windowSurfaceObj= pygame.display.set_mode((screenXSize,screenYSize))

	# UI setup

	pygame.display.set_caption('Submarine Prototype')
	title_font = pygame.font.Font('freesansbold.ttf', 32) 
	info_font = pygame.font.SysFont('Sawasdee', 12,True) 
	console_font = pygame.font.SysFont('Lucida Console', 14,True) 
	panel_font = pygame.font.SysFont('Sawasdee', 25,True) 
	camera_overlay_font = pygame.font.SysFont('Lucida Console', 12,True) 

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

	console_text = ""
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

		# DRAW SPEEDS

		xSpeed_text = info_font.render("Forwards Speed (m/s): "+str(round(xSpeed,4)), True, darkblue, white) 
		xSpeed_textRect = xSpeed_text.get_rect()  
		xSpeed_textRect.topleft = (x+478, 200) 
		windowSurfaceObj.blit(xSpeed_text, xSpeed_textRect) 

		ySpeed_text = info_font.render("Vertical Speed (m/s): "+str(round(ySpeed,4)), True, darkblue, white) 
		ySpeed_textRect = ySpeed_text.get_rect()  
		ySpeed_textRect.topleft = (x+478, 216) 
		windowSurfaceObj.blit(ySpeed_text, ySpeed_textRect) 

		zSpeed_text = info_font.render("Lateral Speed (m/s): "+str(round(zSpeed,4)), True, darkblue, white) 
		zSpeed_textRect = zSpeed_text.get_rect()  
		zSpeed_textRect.topleft = (x+478, 232) 
		windowSurfaceObj.blit(zSpeed_text, zSpeed_textRect) 

		depth_text = info_font.render("Depth (m): "+str(round(depth,4)), True, darkblue, white) 
		depth_textRect = depth_text.get_rect()  
		depth_textRect.topleft = (x+328, 264) 
		windowSurfaceObj.blit(depth_text, depth_textRect) 

		bleh_text = info_font.render("Heading Speed (m): "+str(round(bleh,4)), True, darkblue, white) 
		bleh_textRect = bleh_text.get_rect()  
		bleh_textRect.topleft = (x+478, 264) 
		windowSurfaceObj.blit(bleh_text, bleh_textRect) 

		if img_in and rimg_in:
			## DRAW CAMERAS
			snapshot = pygame.transform.flip(pygame.image.fromstring(current_ros_image,(320,240),"ARGB"),True,True)
			snapshot3 = pygame.transform.flip(pygame.image.fromstring(current_ros_rear_image,(320,240),"ARGB"), False, True)
				
	
			windowSurfaceObj.blit(snapshot, (x,200+250))
			#pygame.camera.colorspace(snapshot, "HSV", snapshot2)
			#windowSurfaceObj.blit(snapshot2, (x+400,100))
			fCamText = camera_overlay_font.render("CAMERA FEED [001] - FRONT", True, darkblue, white) 
			fCamtextRect = fCamText.get_rect()  
			fCamtextRect.topleft = (x+4, 454) 
			windowSurfaceObj.blit(fCamText, fCamtextRect) 
			
			#pygame.camera.colorspace(snapshot, "HSV", snapshot3)
			windowSurfaceObj.blit(snapshot3, (x,200))
	
			rCamText = camera_overlay_font.render("CAMERA FEED [002] - REAR", True, darkblue, white) 
			fCamtextRect = rCamText.get_rect()  
			fCamtextRect.topleft = (x+4, 204) 
			windowSurfaceObj.blit(rCamText, fCamtextRect) 

		## DRAW NAVBALL
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

		# draw level indicator
		pygame.draw.rect(windowSurfaceObj, (0,200,0), (navCentreX+100+boxX-boxX/sidecut, navCentreY,20, boxY), 0)
		pygame.draw.rect(windowSurfaceObj, (0,200,0), (navCentreX, navCentreY-40,boxX, 20), 0)
		pygame.draw.rect(windowSurfaceObj, (0,140,0), (navCentreX+100+boxX-boxX/sidecut, navCentreY+boxY*0.4,19, boxY*0.2), 2)
		pygame.draw.rect(windowSurfaceObj, (0,140,0), (navCentreX+boxX*0.4, navCentreY-40,boxX*0.2, 19), 2)

		pygame.draw.rect(windowSurfaceObj, (240,240,240), (navCentreX+boxX*0.5, navCentreY+boxY*0.45,2, 40), 2)

		surface2 = pygame.Surface((screenXSize,screenYSize))
		surface2.set_colorkey((0,0,0))
		magnitude = math.sqrt(zSpeed*zSpeed+xSpeed*xSpeed)
		print(magnitude)
		if magnitude>top_speed:
			magnitude = top_speed
		magnitude = magnitude/top_speed
		magnitude = magnitude**(1/float(3))
		surface2.set_alpha(magnitude*255)

		# draw speed vector indicator
		curAngle = math.atan2(zSpeed, xSpeed)
		if not abs(curAngle)>math.pi/2:
			pos = (int(navCentreX+curAngle*(navBallSizeN)/math.pi)+navBallSizeN/2,navCentreY+navBallSizeN/2)
			pygame.draw.circle(surface2, (250,250,0), pos, 8)

		# draw anti-speed vector indicator
		curAngle = math.atan2(-zSpeed, -xSpeed)
		if not abs(curAngle)>math.pi/2:
			print(curAngle)
			if curAngle>math.pi/2:
				curAngle=curAngle-math.pi/2
			if curAngle<-math.pi/2:
				curAngle=curAngle+math.pi/2
			pos = (int(navCentreX+navBallSizeN/2+curAngle*(navBallSizeN)/math.pi),navCentreY+navBallSizeN/2)
			pygame.draw.circle(surface2, (250,0,250), pos, 8)

		windowSurfaceObj.blit(surface2, (0,0))


		# draw spirit levels
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
		pygame.draw.rect(windowSurfaceObj, (0,140,0), (rollSpiritBubbleX, navCentreY-37,boxX*0.2-spiritLevel_padding*2+1, 18-spiritLevel_padding), 0)

		pygame.draw.rect(windowSurfaceObj, white, (navCentreX+boxX/2-boxX/5/2,navCentreY+boxY/2,boxX/5,5), 2)
		pygame.draw.line(windowSurfaceObj, white, (navCentreX+boxX/2-boxX/5, navCentreY+boxY/2), (navCentreX+boxX/2+boxX/5, navCentreY+boxY/2), 3)

		## DRAW TERMINAL
		terminal_bg_color=(0,0,0)
		if console_active:
			terminal_bg_color = (100,100,100)
		else:
			terminal_bg_color = (50,50,50)
		

		text = info_font.render("INFO CONSOLE", True, (0,0,0), white) 
		textRect = text.get_rect()  
		textRect.topleft = (screenXSize*0.65+8,screenYSize*0.32+4) 
		windowSurfaceObj.blit(text, textRect)

		

		# setup terminal
		input_box = pygame.Rect(screenXSize*0.65,screenYSize*0.83,400,24)

		pygame.draw.rect(windowSurfaceObj, terminal_bg_color, input_box, 0)
		pygame.draw.rect(windowSurfaceObj, (0,0,0), (screenXSize*0.65,screenYSize*0.63,400,screenYSize*0.2), 0)
		pygame.draw.rect(windowSurfaceObj, (0,0,0), (screenXSize*0.65,screenYSize*0.35,400,screenYSize*0.2), 0)

		text = console_font.render(console_text, True, white, terminal_bg_color) 
		textRect = text.get_rect()  
		textRect.topleft = (screenXSize*0.65+8,screenYSize*0.83+4) 
		windowSurfaceObj.blit(text, textRect) 


		pygame.draw.rect(windowSurfaceObj, (160,160,160), (screenXSize*0.15,screenYSize*0.10,screenXSize*0.80-16,96), 0)

		text = info_font.render("COMMAND CONSOLE", True, (0,0,0), white) 
		textRect = text.get_rect()  
		textRect.topleft = (screenXSize*0.65+8,screenYSize*0.6+4) 
		windowSurfaceObj.blit(text, textRect)
		

		cmd_response_lines = cmd_response_text.split("\n")
		i=0
		for line in cmd_response_lines:
			if len(line)>0:
				text = console_font.render(line, True, white, (0,0,0)) 
				textRect = text.get_rect()  
				textRect.topleft = (screenXSize*0.65+8,screenYSize*0.63+4+i*14) 
				windowSurfaceObj.blit(text, textRect) 
			i+=1

		## DRAWING STATUS PANEL

		for x in range(0,5):
			for y in range(0,2):
				button_enabled=False
				button_color = white
				current_text = "EGG"
				if x==0 and y==0:
					if mode1_enabled:
						button_enabled = True
					current_text = " MODE1 "
					button_color = (255,255,0)
				if x==0 and y==1:
					if mode2_enabled:
						button_enabled = True
					current_text = " MODE2 "
					button_color = (0,255,0)
				if x==1 and y==0:
					if mode3_enabled:
						button_enabled = True
					current_text = "MODE3"
					button_color = (0,255,255)
				if x==1 and y==1:
					if control_enabled:
						button_enabled = True
					current_text = "CONTROL"
					button_color = (255,170,0)
				if x==2 and y==0:
					if brush_enabled:
						button_enabled = True
					current_text = "BRUSH"
					button_color = (170,0,255)
				if x==2 and y==1:
					current_text = "ESTOP"
					if emergencyStop_enabled:
						button_enabled = True
						current_text = "ESTOP ARMED"
					button_color = (140,140,140)
					if emergencyStop_reallyEnabled:
						current_text = "ESTOP ACTIVE!!"
						button_color = (255,0,0)
				if x==3 and y==0:
					if logging_enabled:
						button_enabled = True
					current_text = "LOGGING"
					button_color = (0,255,170)
				if x==3 and y==1:
					if verbose_enabled:
						button_enabled = True
					current_text = "VERBOSE"
					button_color = (0,255,220)
				if button_enabled:
					text = panel_font.render(current_text, True, button_color, (130,130,130)) 
					pygame.draw.rect(windowSurfaceObj, (130,130,130), (screenXSize*0.15+4+(screenXSize*0.16-4)*x,screenYSize*0.10+4+46*y,screenXSize*0.16-8,50-8), 0)
				else:
					text = panel_font.render(current_text, True, (140,140,140), (110,110,110)) 
					pygame.draw.rect(windowSurfaceObj, (110,110,110), (screenXSize*0.15+4+(screenXSize*0.16-4)*x,screenYSize*0.10+4+46*y,screenXSize*0.16-8,50-8), 0)

				textRect = text.get_rect()  
				textRect.center = (screenXSize*0.15+4+(screenXSize*0.16-4)*x+(screenXSize*0.16-8)/2,screenYSize*0.10+4+46*y+(50-8)/2) 
				windowSurfaceObj.blit(text, textRect) 
		###
		### GRAPHICS GOES ABOVE HERE /\   /\   /\
		###

		pygame.display.update()


		pitch_pub.publish(pitch_input_val)
		roll_pub.publish(roll_input_val)
		heading_pub.publish(heading_input_val)
		altitude_pub.publish(altitude_input_val)
		
		pitch_input_val = 0
		roll_input_val = 0
		heading_input_val = 0
		altitude_input_val = 0

		keys = pygame.key.get_pressed()
		if keys[pygame.K_w]:
			pitch_input_val=1
		elif keys[pygame.K_s]:
			pitch_input_val=-1
		elif keys[pygame.K_a]:
			roll_input_val=1
		elif keys[pygame.K_d]:
			roll_input_val=-1
		elif keys[pygame.K_q]:
			heading_input_val=1
		elif keys[pygame.K_e]:
			heading_input_val=-1
		elif keys[pygame.K_SPACE]:
			altitude_input_val=1
		elif keys[pygame.K_LSHIFT]:
			altitude_input_val=-1
		# if the user presses the cross button on the window, close it.
		for event in pygame.event.get() : 
			if event.type == pygame.QUIT : 
				cam.stop()
				currentLogfile.close()
				pygame.quit() 
				quit() 
			if event.type == pygame.MOUSEBUTTONDOWN:
                # If the user clicked on the input_box rect.
				if input_box.collidepoint(event.pos):
					# Toggle the active variable.
					console_active = not console_active
					control_enabled = not control_enabled
				else:
					console_active = False
			if event.type == pygame.KEYDOWN:
				if console_active:
					if event.key == pygame.K_RETURN:
						print("Operator entered command: "+console_text)
						Shell_Class().onecmd(console_text)
						console_text = ''
					elif event.key == pygame.K_BACKSPACE:
						console_text = console_text[:-1]
					else:
						console_text += event.unicode
				else:
					if event.key == pygame.K_1:
						mode1_enabled = True
						mode2_enabled = False
						mode3_enabled = False
						brush_enabled = False
						mode_pub.publish(1)
						brush_pub.publish(0)
					elif event.key == pygame.K_2:
						mode1_enabled = False
						mode2_enabled = True
						mode3_enabled = False
						brush_enabled = False
						brush_pub.publish(0)
					elif event.key == pygame.K_3:
						mode1_enabled = False
						mode2_enabled = False
						mode3_enabled = True
						mode_pub.publish(3)
					elif event.key == pygame.K_HASH:
						if emergencyStop_enabled:
							emergencyStop_reallyEnabled = True
						emergencyStop_enabled = True
					elif event.key == pygame.K_RIGHTBRACKET:
						if emergencyStop_enabled and not emergencyStop_reallyEnabled:
							emergencyStop_enabled=False
							emergencyStopPub.publish(1)
					elif event.key == pygame.K_TAB:
						if mode3_enabled and not brush_enabled:
							brush_enabled = True
							brush_pub.publish(1)
						else:
							brush_enabled=False
							brush_pub.publish(0)

					elif event.key == pygame.K_l:
						
						logging_enabled = not logging_enabled
						verbose_enabled = False
						if logging_enabled:
							currentLogfile = open("logs_"+str(rospy.get_time()),"w")
						else:
							currentLogfile.close()
					elif event.key == pygame.K_SEMICOLON:
						if logging_enabled:
							verbose_enabled = not verbose_enabled
				### Control Schema
				
		rate.sleep()
