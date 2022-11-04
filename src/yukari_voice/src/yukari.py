#!/usr/bin/env python3

from std_msgs.msg import UInt8
import simpleaudio as sa
import rospy

is_callback=bool()
msg=UInt8()
def callback(msg_rx):
	global is_callback
	is_callback = True
	msg.data = msg_rx.data 

def main():
	rospy.init_node('yukari', anonymous=True)

	sub = rospy.Subscriber('yukari_voice', UInt8, queue_size=1, callback=callback)
	rate = rospy.Rate(5)
	wav_obj = sa.WaveObject.from_wave_file('/home/robo-a/ai_sound/yukari-voice/yukari-start.wav')
	play_obj = wav_obj.play()
	global is_callback
	counter = int(0)
	target_file = ['/home/robo-a/ai_sound/yukari-voice/yukari-emergency.wav', '/home/robo-a/ai_sound/yukari-voice/yukari-move.wav', '/home/robo-a/ai_sound/yukari-voice/yukari-slow.wav', '/home/robo-a/ai_sound/yukari-voice/yukari-hispeed.wav', '/home/robo-a/ai_sound/yukari-voice/yukari-no-plane.wav', '/home/robo-a/ai_sound/yukari-voice/yukari-spin.wav', '/home/robo-a/ai_sound/yukari-voice/yukari-logic-start.wav', '/home/robo-a/ai_sound/yukari-voice/yukari-restoration.wav', '/home/robo-a/ai_sound/yukari-voice/yukari-start.wav', '/home/robo-a/ai_sound/yukari-voice/yukari-middle.wav', '/home/robo-a/ai_sound/yukari-voice/yukari-shot.wav', '/home/robo-a/ai_sound/yukari-voice/yukari-stop.wav']
	while not rospy.is_shutdown():
		if (play_obj.is_playing() is False) and is_callback:
			counter+=1
			rospy.loginfo('==')
			wav_obj = sa.WaveObject.from_wave_file(target_file[msg.data])
			play_obj = wav_obj.play()
		
		rate.sleep()

		if play_obj.is_playing() is False and counter >= 1:
			counter = 0
			rospy.loginfo('--')
			is_callback = False

main()
