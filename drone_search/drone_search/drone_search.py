import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge

from ultralytics import YOLO
import cv2 

from drone_search import Recognition
from drone_search import Detection
from collections import defaultdict
import face_recognition
import logging
import os


#Node:
class SimplePubSub(Node):

    def __init__(self):
        super().__init__('simple_pub_sub')

        #Define the codec and create VideoWriter object
        fourcc=cv2.VideoWriter_fourcc(*'mp4v')
        self.out=cv2.VideoWriter("/path_to_the_ros_Package/drone_search/drone_flight01.mp4", fourcc,33, (960,720))
        #self.out=cv2.VideoWriter("/home/istrazivac6/tello_ros2_ws/src/luka_dron/drone_flight01.mp4", fourcc,33, (1280,720))

        #Define used variables
        self.TRACK_ID=None            #The ID of the object to track
        self.FACE_DETECTED=False    #Bool value if the object is detected
        self.FACE_STORED=False      #Bool value if the stored face template exists
        self.prev_time=self.get_clock().now().nanoseconds/ 1e9  #Get starting time
        self.distance_to_person=[]  #List to store the distance to person for filtering
        self.PERSON_FOLLOW_STOPPED=False #The flag that raises when the following stopped
        self.COUNTER_TRACK_LOST=0   #The counter that counts the number of frames after drone lost following
        self.distance_exceeded_counter=0 #The counter for number of frames that exceed the distance
        

        #Previous errors values (for the derivative effect)
        self.prev_error1,self.prev_error2,self.prev_error3,self.prev_error4=0,0,0,0
        self.prev_errors=[self.prev_error1, self.prev_error2, self.prev_error3, self.prev_error4]

        #Integral values (for the integral effect)
        self.integral1,self.integral2,self.integral3,self.integral4=0,0,0,0
        self.integrals=[self.integral1, self.integral2, self.integral3, self.integral4]

        template_image_path="path_to_the_template_image.jpg"

        #Store the name
        file_name = os.path.splitext(template_image_path)[0]
        self.name=file_name.split('/')[-1]
        print(self.name)

        #Template encoding
        template=Recognition.Recognition()
        self.template_encoding=template.encode_template(template_image_path=template_image_path)


        #Track history
        self.track_history=defaultdict(lambda:[])

        #Define matcher
        self.matcher=Recognition.Matcher()
        
        #Turn off logger (yolo prints the detection log by default)
        logging.getLogger('ultralytics').setLevel(logging.WARNING)

        #Tracker setup 
        self.tracker=Detection.Detection(matcher=self.matcher)
        self.model=self.tracker.setup_detector('yolov8s.pt', ["person"])

        #YOLO Pose model
        self.pose_model = YOLO("yolov8s-pose.pt")

        #Setup the ROS image publisher
        self.publisher_ = self.create_publisher(Image, "obradjena_slika" , 10)

        #Setup the ROS velocity1 publisher
        self.cmd_vel_pub2 = self.create_publisher(Twist, "/drone_cmd_vel", 10)
        self.br = CvBridge()
        self.cmd_msg = Twist()

        #Setup subscriber
        self.subscription = self.create_subscription(Image, "image_raw", self.img_callback, 10)
        #self.subscription = self.create_subscription(Image, "/camera/realsense2_camera/color/image_raw", self.img_callback, 10)
        self.subscription 

        #Services for joy buttons
        self.start_follow_srv = self.create_service(Trigger, 'x', self.start_follow_callback)
        self.stop_follow_srv = self.create_service(Trigger, 'circle', self.stop_follow_callback)
        self.start_search_srv = self.create_service(Trigger, 'rectangle', self.start_search_callback)
        self.stop_search_srv = self.create_service(Trigger, 'triangle', self.start_search_template_callback)

        self.start_left_circle_srv = self.create_service(Trigger, 'b_left', self.start_left_circle_callback)
        self.start_right_circle_srv = self.create_service(Trigger, 'b_right', self.start_right_circle_callback)
        self.increase_radius_srv = self.create_service(Trigger, 'b_up', self.start_increase_radius_callback)
        self.decrease_radius_srv = self.create_service(Trigger, 'b_down', self.start_decrease_radius_callback)

        #Initial drone state
        self.state = "free"         #State of flight
        self.previous_state="free"  #Previous state of flight (used to monitor the commands flow)
        self.radius_state=250       #180 cm is the starting distance between drone and person


    def match_check():
        return True
    
    #Callback functions for setting desired state:
    def start_follow_callback(self, request, response):
        self.get_logger().info('Received start_follow request')
        if self.state != 'follow':
            self.previous_state=self.state
            self.state='follow'
            self.radius_state=250
        response.success = True
        response.message = 'Follow started successfully'
        return response

    def stop_follow_callback(self, request, response):
        self.get_logger().info('Received stop_follow request')
        if self.state != 'free':
            self.previous_state=self.state
            self.state='free'
        response.success = True
        response.message = 'Follow stopped successfully'
        return response

    def start_search_callback(self, request, response):
        self.get_logger().info('Received start_search request')
        if self.state != 'search':
            self.previous_state=self.state
            self.state='search'
            self.FACE_STORED=False
            #If the template image is stored before, run the search by template
            if self.template_encoding is not None:
                print("search_template")
                self.state='search_template'
                #self.FACE_DETECTED=True     #The face is detected
                self.FACE_STORED=True       #The face is found and stored
                
        response.success = True
        response.message = 'Search started successfully'
        return response

    def start_left_circle_callback(self, request, response):
        self.get_logger().info('Received start_left_circle request')
        if self.state != 'left_circle':
            self.previous_state=self.state
            self.state='left_circle'
        response.success = True
        response.message = 'Left circle started successfully'
        return response

    def start_right_circle_callback(self, request, response):
        self.get_logger().info('Received start_right_circle request')
        if self.state != 'right_circle':
            self.previous_state=self.state
            self.state='right_circle'
        response.success = True
        response.message = 'Right circle started successfully'
        return response
    
    def start_increase_radius_callback(self, request, response):
        self.get_logger().info('Received increase radius request')
        self.radius_state += 1
        response.success = True
        response.message = 'Radius increased successfully'
        return response
    
    def start_decrease_radius_callback(self, request, response):
        self.get_logger().info('Received decrease radius request')
        self.radius_state -= 1
        response.success = True
        response.message = 'Radius decreased successfully'
        return response
    
    def start_search_template_callback(self, request, response):
        self.get_logger().info('Received start_search_template request')
        if self.state != 'search_template':
            self.previous_state=self.state
            self.state='search_template'
            self.get_logger().info(f"Face stored: {self.FACE_STORED}")   
        response.success = True
        response.message = 'Search template started successfully'
        return response
    
    #Image callback
    def img_callback(self, data):
        frame = self.br.imgmsg_to_cv2(data) #Store the data in cv2 format   
        #Get the tracking results
        results=self.tracker.track(frame)

        cv2.putText(frame, "TELLO DRONE TRACKING ",(300,30), 2, 1.0, (255,255,255), 2)  #Title 
        cv2.line(frame, (int(frame.shape[1]/2)-20,int(frame.shape[0]/2)), (int(frame.shape[1]/2)+20,int(frame.shape[0]/2)),(255,255,255),2)
        cv2.line(frame, (int(frame.shape[1]/2),int(frame.shape[0]/2)-20), (int(frame.shape[1]/2),int(frame.shape[0]/2)+20),(255,255,255),2)

        #Get the boxes and track IDs of detected individuals
        if results.boxes.id!= None:
            #Parse the boxes and track_id-s
            boxes=results.boxes.xywh.cpu()
            track_ids=results.boxes.id.int().cpu().tolist()
            #print(f"track_ids: {track_ids}")
            if self.state=='free':
                #Free drive
                self.FACE_DETECTED=False        #Reset the face search - there is no detected faces
                self.TRACK_ID=None              #Reset the face search - there is no detected ids
                #Reset the speeds in case of remaining speeds
                self.cmd_msg.linear.x=0.0
                self.cmd_msg.linear.z=0.0 
                self.cmd_msg.angular.z = 0.0
                self.cmd_msg.linear.y=0.0
                self.PERSON_FOLLOW_STOPPED=False
                cv2.putText(frame, "MODE:", (20,360),2,1.0, (200,200,200),2)
                cv2.putText(frame, "Free drive",(20,400), 2, 1.0, (255,255,255), 2)
                cv2.putText(frame, "People detected",(20,700), 2, 1.0, (255,255,255), 2)

            elif self.state=='search':
                #Search mode without face template
                box=None
                track_id=None
                #If the face is aimed than we know the TRACK_ID and can exclude searching repeatedly
                if self.FACE_DETECTED==True:
                    for box, track_id in zip(boxes, track_ids):
                        #print(f"track_id: {track_id}")
                        if track_id==self.TRACK_ID:
                            #print("PASS")
                            self.cmd_msg.angular.z = 0.0
                            self.cmd_msg.linear.x=0.0
                            self.cmd_msg.linear.z=0.0
                            self.cmd_msg.linear.y=0.0
                            # Calculate time
                            current_time = self.get_clock().now().nanoseconds / 1e9  # Get current time
                            dt = current_time - self.prev_time  # Calculate time difference
                            self.prev_time = current_time  # Update prev_time for next iteration
                            cv2.putText(frame, "MODE:", (20,360),2,1.0, (200,200,200),2)
                            cv2.putText(frame, "Search mode",(20,400), 2, 1.0, (255,255,255), 2)
                            #cv2.putText(frame, "1. LOOP",(20,460), 2, 1.0, (255,255,255), 2)
                            #cv2.putText(frame, f"track_id: {track_id}",(20,600), 2, 1.0, (255,255,255), 2)
                            #Center the body
                            frame, self.cmd_msg.linear.x, self.cmd_msg.angular.z, self.cmd_msg.linear.z,self.prev_errors,self.integrals, self.distance_to_person,self.distance_exceeded_counter=self.tracker.center_detected_body(box,track_id,self.pose_model,self.prev_errors,self.integrals,dt,self.name, self.distance_to_person, self.FIRST_LOOP,self.distance_exceeded_counter)                       
                            print(self.cmd_msg.angular.z)
                else:                
                    #Check if the drone aims to the person - to start centering
                    for box0, track_id0 in zip(boxes, track_ids):
                        if box0[0]-box0[2]/2< frame.shape[1]/2 and box0[0]+box0[2]/2> frame.shape[1]/2 and box0[1]-box0[3]/2< frame.shape[0]/2 and box0[1]+box0[3]/2>frame.shape[0]/2:
                            box=box0
                            track_id=track_id0
                            self.TRACK_ID=track_id
                            #print("AIM")
                            #cv2.putText(frame, "2. LOOP STARTED",(20,460), 2, 1.0, (255,255,255), 2)
                    self.name="Unknown" #The found person is unknown (there is not template image)

                    if box is not None:
                        #In case no faces are found and stored (usually only first step)
                        if self.FACE_STORED==False:
                            #Save the image of the found person as template
                            cv2.imwrite("/home/istrazivac6/tello_ros2_ws/src/drone_search/drone_search/template_image.jpg",frame)
                            cv2.putText(frame, "Face stored",(20,500), 2, 1.0, (255,255,255), 2)

                            #Calculate the embedding of the template
                            template_image_path="/home/istrazivac6/tello_ros2_ws/src/drone_search/drone_search/template_image.jpg"
                            template=Recognition.Recognition()
                            self.template_encoding=template.encode_template(template_image_path=template_image_path)

                            self.FACE_DETECTED=True     #The face is detected - flag that enables tracking
                            self.FACE_STORED=True       #The face is found and stored - flag that enables search by template
                            self.FIRST_LOOP=True        #Flag for setting the self.distance_to_person values

                        # Calculate time
                        current_time = self.get_clock().now().nanoseconds / 1e9     # Get current time
                        dt = current_time - self.prev_time          # Calculate time difference
                        self.prev_time = current_time       # Update prev_time for next iteration
                        cv2.putText(frame, "MODE:", (20,360),2,1.0, (200,200,200),2)
                        cv2.putText(frame, "Search mode",(20,400), 2, 1.0, (255,255,255), 2)
                        #print(f"Stored : {self.FACE_STORED}, detected: { self.FACE_DETECTED}, box: {box}, track_id: {track_id}")

                        self.cmd_msg.linear.y=0.0 #Reset the speed in y direction 

                        #Center the body in the image
                        frame, self.cmd_msg.linear.x, self.cmd_msg.angular.z, self.cmd_msg.linear.z,self.prev_errors,self.integrals, self.distance_to_person, self.distance_exceeded_counter=self.tracker.center_detected_body(box,track_id,self.pose_model,self.prev_errors,self.integrals,dt,self.name, self.distance_to_person, self.FIRST_LOOP,self.distance_exceeded_counter)                       
                        self.FIRST_LOOP=False   #Disable setting the self.distance_to_person values

            elif self.state=="follow" and (self.previous_state=="search" or self.previous_state=="search_template"):
                #Follow mode (activated only if face is found and centered before)
                #Loop through all detections and extact only the right one
                for box, track_id in zip(boxes, track_ids):
                    cv2.putText(frame, "MODE:", (20,360),2,1.0, (200,200,200),2)
                    cv2.putText(frame, "Follow mode",(20,400), 2, 1.0, (255,255,255), 2)

                    if self.FACE_DETECTED==True and track_id==self.TRACK_ID:
                        # Calculate time
                        current_time = self.get_clock().now().nanoseconds / 1e9  # Get current time
                        dt = current_time - self.prev_time  # Calculate time difference
                        self.prev_time = current_time  # Update prev_time for next iteration

                        self.cmd_msg.linear.y=0.0

                        #Track the body
                        frame, self.cmd_msg.linear.x, self.cmd_msg.angular.z, self.cmd_msg.linear.z, self.prev_errors, self.integrals, self.distance_to_person,self.distance_exceeded_counter=self.tracker.track_detected_body(box,track_id,self.pose_model,self.prev_errors, self.integrals,dt,self.name, self.radius_state, self.distance_to_person,self.FIRST_LOOP,self.distance_exceeded_counter)
                        
                        self.PERSON_FOLLOW_STOPPED=False  #If the person follow did not stopped

            elif self.state=="left_circle" and (self.previous_state=="search" or self.previous_state=="search_template"):
                #Perform left circle aronud the specified individual
                #Loop through all detections and extact only the right one
                for box, track_id in zip(boxes, track_ids):
                    cv2.putText(frame, "MODE:", (20,360),2,1.0, (200,200,200),2)
                    cv2.putText(frame, "Orbit mode (left)",(20,400), 2, 1.0, (255,255,255), 2)
                    self.circle_direction="LEFT"

                    if self.FACE_DETECTED==True:
                        # Calculate time
                        current_time = self.get_clock().now().nanoseconds / 1e9  # Get current time
                        dt = current_time - self.prev_time  # Calculate time difference
                        self.prev_time = current_time  # Update prev_time for next iteration
                
                        #Track the body
                        frame, self.cmd_msg.linear.x, self.cmd_msg.angular.z, self.cmd_msg.linear.z, self.prev_errors, self.integrals, self.distance_to_person,self.distance_exceeded_counter=self.tracker.track_detected_body(box,track_id,self.pose_model,self.prev_errors, self.integrals,dt,self.name, self.radius_state, self.distance_to_person,self.FIRST_LOOP,self.distance_exceeded_counter)
                        self.cmd_msg.linear.y=0.2
                    else:
                        self.cmd_msg.linear.y=0.0

            elif self.state=="right_circle" and (self.previous_state=="search" or self.previous_state=="search_template"):
                #Loop through all detections and extact only the right one
                for box, track_id in zip(boxes, track_ids):
                    cv2.putText(frame, "MODE:", (20,360),2,1.0, (200,200,200),2)
                    cv2.putText(frame, "Orbit mode (right)",(20,400), 2, 1.0, (255,255,255), 2)
                    self.circle_direction="RIGHT"

                    if self.FACE_DETECTED==True:
                        # Calculate time
                        current_time = self.get_clock().now().nanoseconds / 1e9  # Get current time
                        dt = current_time - self.prev_time  # Calculate time difference
                        self.prev_time = current_time  # Update prev_time for next iteration
                    
                        #Track the body
                        frame, self.cmd_msg.linear.x, self.cmd_msg.angular.z, self.cmd_msg.linear.z, self.prev_errors, self.integrals, self.distance_to_person,self.distance_exceeded_counter=self.tracker.track_detected_body(box,track_id,self.pose_model,self.prev_errors, self.integrals,dt,self.name, self.radius_state, self.distance_to_person,self.FIRST_LOOP,self.distance_exceeded_counter)
                        self.cmd_msg.linear.y=-0.2
                    else:
                        self.cmd_msg.linear.y=0.0
            
            elif self.state=='search_template' and self.FACE_STORED==True:
                cv2.putText(frame, "MODE:", (20,360),2,1.0, (200,200,200),2)
                cv2.putText(frame, "Search Template",(20,400), 2, 1.0, (255,255,255), 2)
                
                #Loop through all detections and extact only the right one
                for box, track_id in zip(boxes, track_ids):
                    #Proceed only for specified ID that represents recognized person
                    if self.FACE_DETECTED==True:
                        if track_id==self.TRACK_ID:
                            self.cmd_msg.angular.z = 0.0
                            self.cmd_msg.linear.x=0.0
                            self.cmd_msg.linear.z=0.0
                            self.cmd_msg.linear.y=0.0
                            #cv2.putText(frame, f"{track_id}",(20,600), 2, 1.0, (0,0,255), 2)
                            # Calculate time
                            current_time = self.get_clock().now().nanoseconds / 1e9  # Get current time
                            dt = current_time - self.prev_time  # Calculate time difference
                            self.prev_time = current_time  # Update prev_time for next iteration
                            self.FIRST_LOOP=True
                            #Center the body
                            frame, self.cmd_msg.linear.x, self.cmd_msg.angular.z, self.cmd_msg.linear.z,self.prev_errors,self.integrals, self.distance_to_person,self.distance_exceeded_counter=self.tracker.center_detected_body(box,track_id,self.pose_model,self.prev_errors,self.integrals,dt,self.name, self.distance_to_person, self.FIRST_LOOP,self.distance_exceeded_counter)                       
                                

                    elif self.FACE_DETECTED==False:
                        cv2.putText(frame, "People detected",(20,700), 2, 1.0, (255,255,255), 2)
                        #Set the rotatiton speeds
                        self.cmd_msg.angular.z = 0.5
                        self.cmd_msg.linear.x=0.0
                        self.cmd_msg.linear.z=0.0
                        self.cmd_msg.linear.y=0.0
                        
                        #Try to match the template to the detected faces
                        self.TRACK_ID,self.FACE_DETECTED,frame, self.cmd_msg.angular.z=self.tracker.recognize_template(frame,self.matcher,box,track_id,self.template_encoding)
                        #cv2.putText(frame, f"{self.FACE_DETECTED}",(20,600), 2, 1.0, (0,0,255), 2)
                        #If the person was followed and the person is detected in this step then start follow again
                        if self.PERSON_FOLLOW_STOPPED ==True and self.FACE_DETECTED==True:
                            self.state = "follow"
                        # else:
                        #     self.FACE_DETECTED=False 
                        #     cv2.putText(frame, "Error",(20,600), 2, 1.0, (0,0,255), 2)

            #Reset the conter track lost
            self.COUNTER_TRACK_LOST=0
            
        else:
            
            self.FACE_DETECTED=False    #Reset the flag
            self.TRACK_ID=None          #Reset the flag
            self.cmd_msg.linear.x=0.0   #Reset the speed
            self.cmd_msg.linear.z=0.0   #Reset the speed
            #If the search_template state is started set the rotation speed
            if self.state == "search_template" and self.COUNTER_TRACK_LOST <=100:
                self.COUNTER_TRACK_LOST+=1
                cv2.putText(frame, "MODE:", (20,360),2,1.0, (200,200,200),2)
                cv2.putText(frame, "Search Template",(20,400), 2, 1.0, (255,255,255), 2)
                self.cmd_msg.angular.z = 0.5
            elif self.state == "search_template" and self.COUNTER_TRACK_LOST >100:
                self.cmd_msg.angular.z = 0.5    #Reset the speed
                self.COUNTER_TRACK_LOST=0
            else:
                self.cmd_msg.angular.z = 0.0   #Reset the speed
                self.COUNTER_TRACK_LOST=0
            #If the person was followed and then lost, start the search template process
            if self.state == "follow":
                self.PERSON_FOLLOW_STOPPED=True     #Follow stopped
                self.state = "search_template"      #Start search template
                self.FIRST_LOOP=True                #Reset the first loop procedure
            if self.state == "free":
                cv2.putText(frame, "MODE:", (20,360),2,1.0, (200,200,200),2)
                cv2.putText(frame, "Free drive",(20,400), 2, 1.0, (255,255,255), 2)
                
                
        #Publish the data    
        #print(self.cmd_msg.angular.z)       
        self.cmd_vel_pub2.publish(self.cmd_msg) 
        self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
        self.out.write(frame)
        cv2.imshow("Tello Drone Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            

def main(args=None):
    #ROS initialization
    rclpy.init(args=args)
    #Creation of a node
    simple_pub_sub = SimplePubSub()
    rclpy.spin(simple_pub_sub)
    #Destory the node
    simple_pub_sub.destroy_node()
    #ROS shutdown
    rclpy.shutdown()

  
if __name__ == '__main__':
  main()
