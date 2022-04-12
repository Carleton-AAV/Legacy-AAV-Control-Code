#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import pygame

pygame.init()
display = pygame.display.set_mode((100,100))

def talker():
  pub = rospy.Publisher('chatter', String, queue_size=10)
  motorPub = rospy.Publisher('motor', String, queue_size=10)
  rospy.init_node('talker', anonymous=True) 
  rate = rospy.Rate(10) # 10hz
 
  while not rospy.is_shutdown():
    pygame.event.pump()
    keys = pygame.key.get_pressed()

    if keys[pygame.K_w]:
      hello_str = "w"
      rospy.loginfo(hello_str)
      pub.publish(hello_str)
      rate.sleep()
     
    if keys[pygame.K_a]:
      hello_str = "a"
      rospy.loginfo(hello_str)
      motorPub.publish(hello_str)
      rate.sleep()
     
    if keys[pygame.K_s]:
      hello_str = "s"
      rospy.loginfo(hello_str)
      motorPub.publish(hello_str)
      pub.publish(hello_str)
      rate.sleep()
       
    if keys[pygame.K_d]:
      hello_str = "d"
      rospy.loginfo(hello_str)
      motorPub.publish(hello_str)
      rate.sleep()

    if keys[pygame.K_o]:
      exit()
    
if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass
