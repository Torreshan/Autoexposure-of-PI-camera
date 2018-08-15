#!/usr/bin/env python

"""
Author: Fz
Data: 2018.06.17
"""
#import roslib
import math
import numpy as np
import cv2
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import dynamic_reconfigure.client
import argparse
import os
import signal
import pickle



err_i = 0
err_d = 0
n = 0 #stable or not
m = 0 # if the fixed iso mode cannot improve, this will terminate the code
result = []
record = False
optimize = True
key_frame = 1
isoLocked  = False
i_lock_iso = 0

def str2bool(v):
	if v.lower() in ('yes', 'true', 't','y', '1'):
		return True
	elif v.lower() in ('no', 'false', 'f', 'n', '0'):
		return False


def sigint_handler(signum, frame):
	if record == True:
		with open("/home/fengze/Autoexposure-of-PI-camera-v3/auto_exposure_control/log/log.pickle","wb") as logx:
			pickle.dump(result, logx)
	print('system terminate')


signal.signal(signal.SIGINT, sigint_handler)



def get_exposure(dyn_client):
	global result
	values = dyn_client.get_configuration()
	print('Iso value is:{}'.format(values['ISO']))
	print('Shutter time is:{}'.format(values['shutterSpeed']))
	print('\n*************************************************************\n')
	result.append('ShutterTime:' + str(values['shutterSpeed']) + '#############' + 'ISO:' + str(values['ISO'])) 
	return values['shutterSpeed'], values['ISO']

def set_exposure(dyn_client, exposure, iso, iso_mode = False):
	if iso_mode == False:
		params = {'shutterSpeed' : exposure,'exposure_mode' : 'off'}
	else:
		params = {'shutterSpeed' : exposure, 'exposure_mode' : 'auto','ISO' : iso}
	config = dyn_client.update_configuration(params)
	rospy.sleep(0.2) #2000ms   get the fps of the image now to calculate the sleep tim

#def shutter_time_cal(fps):
#	shutter = 1/ fps 
#	return shutter  


def image_callback(image, args):
	
	global err_i, err_d,n,m,result,optimize,key_frame,isoLocked,i_lock_iso 
	#if key_frame % 2 != 0:
		#print('test is:{}'.format(key_frame))
	#	key_frame = 1
	#else:
	key_frame += 1
	if key_frame % 7 == 0:
		key_frame = 1
		bridge = args['cv_bridge']
		dyn_client = args['dyn_client']
		max_shutter = args['maxshutter']
		min_shutter = args['minshutter']
		fixed_iso = args['iso']
		live = args['livemode']
		optimizer = args['optimizemode']
		record = args['record']

		cv_image = bridge.imgmsg_to_cv2(image, desired_encoding = "bgr8")	
		(rows, cols, channels) = cv_image.shape
		if (channels == 3):
			brightness_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)[:,:,2]
		else:
			brightness_image = cv_image
		hist = cv2.calcHist([brightness_image], [0], None, [5], [0, 256])
		mean_sample_value = 0
		for i in range(len(hist)):
			mean_sample_value += hist[i] * (i + 1)
			
		mean_sample_value /= (rows * cols)
		print('mean_sample_value is:{}'.format(mean_sample_value))

		# Middle value MSV is 2.5, range is 0-5
		# Note: You may need to retune the PI gains if you change this
		desired_msv = 2.5
		# Gains
		k_p = 200#100  
		k_i = 20 #30
		# Maximum integral value
		max_i = 3.0 #3
		err_p = desired_msv-mean_sample_value
		if abs(err_p) < 0.5:
			n += 1
			if n >= 20:
				print('The scene is relative stable now\n')
				
				if live is True: 
					if fixed_iso == 0:
						params = {'exposure_mode' : 'auto'}
						config = dyn_client.update_configuration(params)
						print('Code terminated and exposure_mode: auto\n')
					else:
						params = {'exposure_mode' : 'off'}
						config = dyn_client.update_configuration(params)
						print('Code terminated and exposure_mode: off\n')	
					try:
						_,_ = get_exposure(dyn_client)
						if record:
							with open("/home/fengze/Autoexposure-of-PI-camera-v3/auto_exposure_control/log/log.pickle","wb") as logx:
								pickle.dump(result, logx)
						os._exit(0)
					except:
						print('Program is dead')  
				else:

					if fixed_iso == 0:
						params = {'exposure_mode' : 'auto'}
						config = dyn_client.update_configuration(params)
					else:
						params = {'exposure_mode' : 'off'}
						config = dyn_client.update_configuration(params)
		
			

		if abs(err_p) >= 0.5 and optimizer == True:
			n = 0	
			err_i += err_p
			if abs(err_i) > max_i:
				err_i = np.sign(err_i) * max_i
		  
			
	#****************************************Non fixed ISO mode ********************************************************#		
			
			if fixed_iso == 0:
				shutter, iso = get_exposure(dyn_client) 
				shutter_ =  shutter + k_p * err_p + k_i * err_i
				print('defined shutter_ is{}'.format(shutter_))
				
				#if isoLocked == True:
				#	i_lock_iso += 1
				#	print('i_lock_iso = {}'.format(i_lock_iso))
					
				#if i_lock_iso >= 5:
				#	isoLocked = False
				#	i_lock_iso = 0
				
				if shutter_ <= min_shutter:
					shutter_ = min_shutter
					

				#if shutter_ <= (max_shutter + min_shutter)/2.0:	
					if iso > 100:
						iso -= 300
						#if isoLocked == False:
						#	iso -= 100
						#	isoLocked = True
					
					else:
						iso = 100
						m += 1

				if shutter_ >= max_shutter: #100ms   0.1s   10fps
		 			shutter_ = max_shutter
		 			if iso < 1600:
		 				iso += 100						
						#if isoLocked == False:
		 				#	iso += 100	 	
						#	isoLocked = True							
		 			else:
		 				iso = 1600 
		 				m += 1
				
				if m == 20 and live == True:
					m = 0
					print('The scene is relative stable now\n')
					os._exit(0)		
				set_exposure(dyn_client, shutter_, iso, True)		
	#************************************** Fixed ISO mode********************************************************#				
			
			else:	

				shutter, _ = get_exposure(dyn_client) 
				shutter_ =  shutter + k_p * err_p + k_i * err_i
				if fixed_iso < 100:
					fixed_iso = 100
					print('ISO cannot be smaller than 100, already automatically modified')
				if shutter_ <= min_shutter:
					shutter_ = min_shutter
					m +=1   					

				if shutter_ >= max_shutter:   #100ms   0.1s   10fps
					shutter_ = max_shutter
					m += 1
				if m == 20 and live == True:
					m = 0
					print('The scene is relative stable now\n')
					os._exit(0)
				set_exposure(dyn_client, shutter_, fixed_iso, True)


		
def main(args):
#*******************************************User Parameter*******************************************************#	
	parser = argparse.ArgumentParser(description='Set the maximum framerate')
	parser.add_argument('--maxshutter',type=float, default=20000, help='Please give the maximum exposure time')
	parser.add_argument('--minshutter',type=float, default=10, help='Please give the minimum exposure time')   #fixed iso value give ; two flags about enable or disable two modes
	parser.add_argument('--iso', type=float, default=0, help='Here you can input a fixed iso value, if 0 iso will be automated adjusted')
	parser.add_argument('--livemode',type=str2bool, default=True, help='If true, the code will terminate itself automatically in stable scenes')	
	parser.add_argument('--optimizemode',type=str2bool, default=True,  help='If true, the parameters will be optimized by PI controller when the code terminated; else it will be optimized by auto-exposure')
	parser.add_argument('--fps',type=float,default=1,help='please input the fps of camera')
	parser.add_argument('--record',type=str2bool, default=False, help='If true, the shutter time and ISO will be recorded at same time')
	arg=parser.parse_args()


#********************************************Initialization******************************************************# different mode pre set initialization greedy search 
	global record, optimize 
	rospy.init_node('auto_exposure_control')
	bridge = CvBridge()
	fps = arg.fps
	maxst = min((1.0/fps) * 1000, arg.maxshutter) 
	minst = arg.minshutter
	iso = arg.iso
	livemode = arg.livemode
	optimize = arg.optimizemode
	dyn_client = dynamic_reconfigure.client.Client("/cam0", timeout=None, config_callback=None, description_callback=None)
	record = arg.record

	params = {'shutterSpeed' : 100,'exposure_mode' : 'off','ISO':100}
	config = dyn_client.update_configuration(params)
	
	values = dyn_client.get_configuration()
	print('intial_configuration:\n shutter time:{}ms \n exposure_mode:auto \n fixed_iso:{} \n livemode:{} \n'.format(values['shutterSpeed']/1000, iso, livemode))
	if optimize is True:
		print('\n**************************Optimizer Starting**************************\n')
	if optimize is False:
		print('\n**************************Without Optimization Starting***************\n')

	if iso is not 0: 	
		params = {'exposure_mode' : 'auto','ISO' : iso}
		config = dyn_client.update_configuration(params)
		params = {'exposure_mode' : 'off'}
		config = dyn_client.update_configuration(params)
		print('The iso you set is {}'.format(iso))

	args = {}
	args['cv_bridge'] = bridge
	args['dyn_client'] = dyn_client
	args['maxshutter'] = maxst
	args['minshutter'] = minst 
	args['iso'] = iso
	args['livemode'] = livemode
	args['optimizemode'] = optimize
	args['fps'] = fps
	args['record'] = record


#******************************************Callback***************************************************************#	
	img_sub=rospy.Subscriber('/cam0/image_raw', Image, image_callback, args)

	rospy.spin()
	
if __name__ == "__main__":
	try:
		main(sys.argv)
	except KeyboardInterrupt: 
		pass
