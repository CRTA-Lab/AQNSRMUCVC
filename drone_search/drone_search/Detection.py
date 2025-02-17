from ultralytics import YOLO
from collections import defaultdict
import cv2
from drone_search import Recognition
import rclpy
from rclpy.node import Node
import supervision as sv
import math




class Detection:
    def __init__(self, matcher):
        self.matcher=matcher
        #Track history
        self.track_history=defaultdict(lambda:[])
        self.MAX_TRACK_LENGTH=300
        self.FRAME_TO_REFRESH=100
        

    def setup_detector(self, model_path, classes_to_detect):
        """
        Method to setup the detection model and detection classes
        
        Args: model_path - path to the YOLO model
            classes_to_detect - list of a classes to detect

        Returns:
        """
        self.model_path=model_path
        self.classes_to_detect=classes_to_detect
        self.model=YOLO(self.model_path)
        self.classes=[]

        for id, name in self.model.names.items():
            if name in classes_to_detect:
                self.classes.append(id)

        return self.model

    def track(self,frame):
        """
        Method to track using YOLO
        
        Args: frame - frame from the camera

        Returns: results - tracking results
        """
        self.frame=frame
        self.results=self.model.track(self.frame,persist=True, tracker="bytetrack.yaml", classes=self.classes)
        return self.results[0]
    

    def center_detected_body(self, box, track_id, pose_model,prev_errors, integrals,dt,name,distance_to_person, FIRST_LOOP,distance_exceeded_counter):
        """
        Method to center the detected person body in the frame if it is recognized.

        Args:box - bounding box of detected object
            track_id - track ID of detected object
            pose_model - CNN for estimating body keypoints
            prev_errors - previous errors stored for PID regulator
            integrals - integrals stored for PID regulator
            dt - time interval

        Returns:frame - annotated frame
            cmd_linear_x - drone cmd_vel linear command in x direction
            cmd_angular_z - drone cmd_vel angular command in z direction
            cmd_linear_z - drone cmd_vel linear command in z direction
            prev_errors - stored errors in this loop for the next iteration
            integrals - stored integrals in this loop for the next iteration
        """

        self.box=box
        self.track_id=track_id
        self.pose_model=pose_model  
        self.prev_errors=prev_errors
        self.dt=dt
        self.integrals=integrals
        self.face_lost=False
        self.name=name
        self.distance_to_person=distance_to_person
        self.FIRST_LOOP=FIRST_LOOP
        self.DISTANCE_EXCEEDED_COUNTER = distance_exceeded_counter
        #YOLOv8 person bounding box
        x,y,w,h=self.box     #x,y (box center), width, height
        yolo_box=int(x-w/2),int(y-h/2),int(w),int(h)    #x,y (top left), width, height

        track=self.track_history[self.track_id]
        track.append((float(x), float(y)))
        if len(track)>self.MAX_TRACK_LENGTH:
                    track.pop(0)
        
        #Detect pose in the sepcified bounding box
        self.pose_results=self.pose_model(self.frame)

        #Check if the results are obtrained
        if self.pose_results and len(self.pose_results) > 0:
            #Extract the keypoints from the first result
            self.pose_keypoints=sv.KeyPoints.from_ultralytics(self.pose_results[0])
            for i in range(len(self.pose_keypoints)):
                #Check if any keypoints are obtained
                if len(self.pose_keypoints) > 0:
                    self.keypoint_names=StoreKeypoints(keypoint_detections=self.pose_keypoints,i=i)

                #Top of the chest keypoint (center between )
                self.top_center_body_keypoint=((self.keypoint_names.LEFT_SHOULDER[0]+self.keypoint_names.RIGHT_SHOULDER[0])/2,(self.keypoint_names.LEFT_SHOULDER[1]+self.keypoint_names.RIGHT_SHOULDER[1])/2)
                #Center between hips
                self.bottom_center_body_keypoint=((self.keypoint_names.LEFT_HIP[0]+self.keypoint_names.RIGHT_HIP[0])/2,(self.keypoint_names.LEFT_HIP[1]+self.keypoint_names.RIGHT_HIP[1])/2)
                
                if self.top_center_body_keypoint[0] > yolo_box[0] and self.top_center_body_keypoint[0] < yolo_box[0]+yolo_box[2] and self.top_center_body_keypoint[1] > yolo_box[1] and self.top_center_body_keypoint[1] < yolo_box[1]+yolo_box[3]:
                
                    #The situation where both top and bottom body keypoints are visible
                    if self.top_center_body_keypoint != (0.0,0.0) and self.bottom_center_body_keypoint != (0.0,0.0) and self.keypoint_names.LEFT_SHOULDER[0] != 0.0 and self.keypoint_names.LEFT_SHOULDER[1] != 0.0 and self.keypoint_names.RIGHT_SHOULDER[0] != 0.0 and self.keypoint_names.RIGHT_SHOULDER[1] != 0.0 and self.keypoint_names.LEFT_HIP[0] != 0.0 and self.keypoint_names.LEFT_HIP[1] != 0.0 and self.keypoint_names.RIGHT_HIP[0] != 0.0 and self.keypoint_names.RIGHT_HIP[1] != 0.0:
                        
                        #Calculate distance between two points: sqrt((x2 -x1)**2 + (y2 -y1)**2)
                        self.body_keypoints_distance=math.sqrt(((self.bottom_center_body_keypoint[0]-self.top_center_body_keypoint[0])**2)+((self.bottom_center_body_keypoint[1]-self.top_center_body_keypoint[1])**2))

                        #Calculate the distance to the person:
                        calculated_distance=int((-0.6*self.body_keypoints_distance) + 310)
                        #Fill the list in the first loop
                        if self.FIRST_LOOP==True:
                            self.distance_to_person=[calculated_distance]*25
                        #Average distance to person
                        self.avg_distance_to_person=sum(self.distance_to_person)/len(self.distance_to_person)

                        if calculated_distance < 1.25*self.avg_distance_to_person and calculated_distance > 0.85*self.avg_distance_to_person:
                            self.distance_to_person.pop(0)
                            self.distance_to_person.append(calculated_distance)
                        
                        
                        cv2.rectangle(self.frame, (int(x)-int(float(w)/2), int(y)-int(float(h)/2)-30),(int(x)-int(float(w)/2)+180, int(y)-int(float(h)/2)), (255,207,137),-1)
                        cv2.putText(self.frame,self.name, (int(x)-int(float(w)/2)+2, int(y)-int(float(h)/2)-4),2,1.0,(255,255,255),2)
                        #cv2.putText(self.frame,"Luka", (int(x)-int(float(w)/2)+2, int(y)-int(float(h)/2)-4),2,1.0,(255,255,255),2)

                        cv2.rectangle(self.frame, (int(x)-int(float(w)/2), int(y)-int(float(h)/2)), (int(x)+int(float(w)/2), int(y)+int(float(h)/2)), (255,207,137), 2)
                        cv2.line(self.frame,(int(self.top_center_body_keypoint[0]),int(self.top_center_body_keypoint[1])),(int(self.bottom_center_body_keypoint[0]), int(self.bottom_center_body_keypoint[1])),(255,255,255),2,2)

                        cv2.circle(self.frame, (int(self.top_center_body_keypoint[0]), int(self.top_center_body_keypoint[1])), 6,(255,0,0),-1)
                        cv2.putText(self.frame, "Distance: " + str(int(0.00978*self.body_keypoints_distance**2-5.472*self.body_keypoints_distance+949.9)) + "cm",(20,700), 2, 1.0, (255,255,255), 2)
                        cv2.putText(self.frame, "FACE DETECTED:",(20,440), 2, 1.0, (200,200,200), 2)
                        cv2.putText(self.frame, self.name,(20,480), 2, 1.0, (255,255,255), 2)
                        


                        if calculated_distance > 1.25*self.avg_distance_to_person or calculated_distance < 0.85*self.avg_distance_to_person:
                            cv2.putText(self.frame,"Distance exceeded!",(280,700), 2, 1.0, (255,0,0), 2)
                            self.DISTANCE_EXCEEDED_COUNTER+=1
                            if self.DISTANCE_EXCEEDED_COUNTER > 50:
                                self.DISTANCE_EXCEEDED_COUNTER=0

                        cv2.circle(self.frame, (int(self.bottom_center_body_keypoint[0]), int(self.bottom_center_body_keypoint[1])), 6,(255,0,0),-1)
                        #cv2.line(self.frame,(int(self.top_center_body_keypoint[0]),int(self.top_center_body_keypoint[1])),(int(self.bottom_center_body_keypoint[0]), int(self.bottom_center_body_keypoint[1])),(255,255,255),6,4)

                        #PID regulator(x movement):
                        #The center: (480,360)
                        #Left to right
                        if not(self.top_center_body_keypoint[0] > 460 and self.top_center_body_keypoint[0] < 500):
                            #PID gains
                            Kp1=0.002 #0.0018
                            Ki1=0.0
                            Kd1=0.0004

                        
                            error1=(480-self.top_center_body_keypoint[0])
                            #P
                            p1=Kp1*error1
                            #I
                            self.integrals[0]+=Ki1*error1*self.dt
                            #D
                            d1=Kd1*(error1-self.prev_errors[0])/self.dt

                            #self.cmd_msg_angular_z=p1
                            self.cmd_msg_angular_z=p1+self.integrals[0]
                            self.cmd_msg_angular_z=p1+d1 

                            #Restrict the angular z:
                            if self.cmd_msg_angular_z > 0.9:
                                self.cmd_msg_angular_z = 0.9
                            if self.cmd_msg_angular_z < -0.9:
                                self.cmd_msg_angular_z = -0.9

                            #Store the error
                            self.prev_errors[0]=error1

                        else:
                            self.cmd_msg_angular_z=0.0
                        

                        #Store the error
                        self.prev_errors[1]=0.0
                        self.cmd_msg_linear_z=0.0

                    
                        #Store the error
                        self.prev_errors[2]=0.0
                        
                    
                        self.cmd_msg_linear_x=0.0
                    else:
                        self.cmd_msg_linear_x=0.0
                        self.cmd_msg_linear_z=0.0
                        self.cmd_msg_angular_z=0.0
      
        else:
             self.cmd_msg_linear_x=0.0
             self.cmd_msg_linear_z=0.0
             self.cmd_msg_angular_z=0.0

        return self.frame,self.cmd_msg_linear_x, self.cmd_msg_angular_z, self.cmd_msg_linear_z, self.prev_errors, self.integrals, self.distance_to_person,self.DISTANCE_EXCEEDED_COUNTER


    def track_detected_body(self, box, track_id, pose_model,prev_errors, integrals, dt,name, radius_state, distance_to_person, FIRST_LOOP,distance_exceeded_counter):
        """
        Method to track the detected person body in the frame if it is recognized.

        Args:box - bounding box of detected object
            track_id - track IDs of detected object
            face_detection - results of face detections
            prev_errors - previous errors stored for PID regulator
            integrals - integrals stored for PID regulator
            dt - time interval

        Returns:frame - annotated frame
            cmd_linear_x - drone cmd_vel linear command in x direction
            cmd_angular_z - drone cmd_vel angular command in z direction
            cmd_linear_z - drone cmd_vel linear command in z direction
            prev_errors - stored errors in this loop for the next iteration
            integrals - stored integrals in this loop for the next iteration
        """

        self.box=box
        self.track_id=track_id
        self.pose_model=pose_model  
        self.prev_errors=prev_errors
        self.integrals=integrals
        self.dt=dt
        self.face_lost=False
        self.name=name
        self.radius_state=radius_state
        self.distance_to_person=distance_to_person
        self.FIRST_LOOP=FIRST_LOOP
        self.DISTANCE_EXCEEDED_COUNTER = distance_exceeded_counter

        #YOLOv8 person bounding box
        x,y,w,h=self.box     #x,y (box center), width, height
        yolo_box=int(x-w/2),int(y-h/2),int(w),int(h)    #x,y (top left), width, height
        print(yolo_box)
        track=self.track_history[self.track_id]
        track.append((float(x), float(y)))
        if len(track)>self.MAX_TRACK_LENGTH:
                    track.pop(0)
        
        #Detect pose in the sepcified bounding box
        self.pose_results=self.pose_model(self.frame)

        #Check if the results are obtrained
        if self.pose_results and len(self.pose_results) > 0:
            #Extract the keypoints from the first result
            self.pose_keypoints=sv.KeyPoints.from_ultralytics(self.pose_results[0])
            for i in range(len(self.pose_keypoints)):
                #Check if any keypoints are obtained
                if len(self.pose_keypoints) > 0:
                    self.keypoint_names=StoreKeypoints(keypoint_detections=self.pose_keypoints,i=i)

                #Top of the chest keypoint (center between)
                self.top_center_body_keypoint=((self.keypoint_names.LEFT_SHOULDER[0]+self.keypoint_names.RIGHT_SHOULDER[0])/2,(self.keypoint_names.LEFT_SHOULDER[1]+self.keypoint_names.RIGHT_SHOULDER[1])/2)
                #Center between hips
                self.bottom_center_body_keypoint=((self.keypoint_names.LEFT_HIP[0]+self.keypoint_names.RIGHT_HIP[0])/2,(self.keypoint_names.LEFT_HIP[1]+self.keypoint_names.RIGHT_HIP[1])/2)
                if self.top_center_body_keypoint[0] > yolo_box[0] and self.top_center_body_keypoint[0] < yolo_box[0]+yolo_box[2] and self.top_center_body_keypoint[1] > yolo_box[1] and self.top_center_body_keypoint[1] < yolo_box[1]+yolo_box[3]:
                   
                    #The situation where both top and bottom body keypoints are visible
                    if self.top_center_body_keypoint != (0.0,0.0) and self.bottom_center_body_keypoint != (0.0,0.0) and self.keypoint_names.LEFT_SHOULDER[0] != 0.0 and self.keypoint_names.LEFT_SHOULDER[1] != 0.0 and self.keypoint_names.RIGHT_SHOULDER[0] != 0.0 and self.keypoint_names.RIGHT_SHOULDER[1] != 0.0 and self.keypoint_names.LEFT_HIP[0] != 0.0 and self.keypoint_names.LEFT_HIP[1] != 0.0 and self.keypoint_names.RIGHT_HIP[0] != 0.0 and self.keypoint_names.RIGHT_HIP[1] != 0.0:
                        
                        #Calculate distance between two points: sqrt((x2 -x1)**2 + (y2 -y1)**2)
                        self.body_keypoints_distance=math.sqrt(((self.bottom_center_body_keypoint[0]-self.top_center_body_keypoint[0])**2)+((self.bottom_center_body_keypoint[1]-self.top_center_body_keypoint[1])**2))
                        
                        #Calculate the distance to the person:
                        calculated_distance=int(0.00978*self.body_keypoints_distance**2-5.472*self.body_keypoints_distance+949.9)
                        if self.FIRST_LOOP==True:
                            self.distance_to_person=[calculated_distance]*25
                        self.avg_distance_to_person=sum(self.distance_to_person)/len(self.distance_to_person)
                        if calculated_distance < 1.25*self.avg_distance_to_person and calculated_distance > 0.85*self.avg_distance_to_person:
                            self.distance_to_person.pop(0)
                            self.distance_to_person.append(calculated_distance)

                        cv2.rectangle(self.frame, (int(x)-int(float(w)/2), int(y)-int(float(h)/2)-30),(int(x)-int(float(w)/2)+180, int(y)-int(float(h)/2)), (255,207,137),-1)
                        cv2.putText(self.frame,self.name, (int(x)-int(float(w)/2)+2, int(y)-int(float(h)/2)-4),2,1.0,(255,255,255),2)
                        #cv2.putText(self.frame,"Luka", (int(x)-int(float(w)/2)+2, int(y)-int(float(h)/2)-4),2,1.0,(255,255,255),2)
                        cv2.rectangle(self.frame, (int(x)-int(float(w)/2), int(y)-int(float(h)/2)), (int(x)+int(float(w)/2), int(y)+int(float(h)/2)), (255,207,137), 2)
                        cv2.line(self.frame,(int(self.top_center_body_keypoint[0]),int(self.top_center_body_keypoint[1])),(int(self.bottom_center_body_keypoint[0]), int(self.bottom_center_body_keypoint[1])),(255,255,255),2,2)

                        cv2.circle(self.frame, (int(self.top_center_body_keypoint[0]), int(self.top_center_body_keypoint[1])), 6,(255,0,0),-1)
                        cv2.putText(self.frame, "Distance: " + str(int(0.00978*self.body_keypoints_distance**2-5.472*self.body_keypoints_distance+949.9)) + " cm",(20,700), 2, 1.0, (255,255,255), 2)
                        cv2.putText(self.frame, "FACE DETECTED:",(20,440), 2, 1.0, (200,200,200), 2)
                        cv2.putText(self.frame, self.name,(20,480), 2, 1.0, (255,255,255), 2)
                        #cv2.putText(self.frame, "Luka",(20,520), 2, 1.0, (255,255,255), 2)


                        #cv2.putText(self.frame, "Avg distance:"+ str(self.avg_distance_to_person)+"cm", (20,700), 2, 1.0, (0,255,0), 2)

                        
                        self.DISTANCE_EXCEEDED_COUNTER+=1
                        if self.DISTANCE_EXCEEDED_COUNTER > 50:
                                self.DISTANCE_EXCEEDED_COUNTER=0
                        if calculated_distance > 1.25*self.avg_distance_to_person or calculated_distance < 0.85*self.avg_distance_to_person:
                            cv2.putText(self.frame,"Distance exceeded!",(280,700), 2, 1.0, (255,0,0), 2)
                            self.cmd_msg_linear_x=0.0
                            self.cmd_msg_angular_z=0.0
                            self.cmd_msg_linear_z=0.0
                            
                        else:          

                            cv2.circle(self.frame, (int(self.bottom_center_body_keypoint[0]), int(self.bottom_center_body_keypoint[1])), 6,(255,0,0),-1)
                            cv2.line(self.frame,(int(self.top_center_body_keypoint[0]),int(self.top_center_body_keypoint[1])),(int(self.bottom_center_body_keypoint[0]), int(self.bottom_center_body_keypoint[1])),(255,255,255),6,4)

                            #PID regulator(x movement):
                            #The center: (480,360)
                            #Left to right
                            if not(self.top_center_body_keypoint[0] > 460 and self.top_center_body_keypoint[0] < 500):
                                #PID gains
                                Kp1=0.002 #0.0018
                                Ki1=0.0001
                                Kd1=0.0004

                            
                                error1=(480-self.top_center_body_keypoint[0])
                                #P
                                p1=Kp1*error1
                                #I
                                self.integrals[0]+=Ki1*error1*self.dt
                                #D
                                d1=Kd1*(error1-self.prev_errors[0])/self.dt

                                #self.cmd_msg_angular_z=p1
                                #self.cmd_msg_angular_z=p1+self.integrals[0]
                                self.cmd_msg_angular_z=p1+self.integrals[0]+d1 

                                #Restrict the angular z:
                                if self.cmd_msg_angular_z > 0.9:
                                    self.cmd_msg_angular_z = 0.9
                                if self.cmd_msg_angular_z < -0.9:
                                    self.cmd_msg_angular_z = -0.9

                                #Store the error
                                self.prev_errors[0]=error1

                            else:
                                self.cmd_msg_angular_z=0.0
                                self.integrals[0]=0.0
                            #Top to bottom
                            if not(self.top_center_body_keypoint[1] >= 340 and self.top_center_body_keypoint[1] <=380):
                                #PID gains
                                Kp2=0.8498
                                Ki2=0.0
                                Kd2=(0.336*9.446)/(1+9.446)

                                error2=(360-self.top_center_body_keypoint[1])/600
                                #P
                                p2=Kp2*error2
                                #I
                                self.integrals[1]+=Ki2*error2*self.dt
                                #D
                                d2=Kd2*(error2-self.prev_errors[1])/self.dt

                                #self.cmd_msg_linear_z=p2
                                #self.cmd_msg_linear_z=p2+self.integrals[1]+d2
                                self.cmd_msg_linear_z=Kp2*error2+Ki2*self.integrals[1]

                                #Store the error
                                self.prev_errors[1]=error2
                                
                            else:
                                self.cmd_msg_linear_z=0.0
                                self.integrals[1]=0.0

                            #X axis
                            if not(self.body_keypoints_distance >= 185 and self.body_keypoints_distance <=200):
                                #PID gains
                                Kp3=0.6004 #1.3004
                                Ki3=0.0
                                Kd3=(0.0746*3393.8)/(1+3393.8) #0.0746
                                #print(f"Radius state: {self.radius_state}")

                                error3=(calculated_distance-self.radius_state)/130
                                #P
                                p3=Kp3*error3
                                #print(f"Error3 : {error3}")
                                #I
                                self.integrals[2]+=Ki3*error3*self.dt
                                #D
                                d3=Kd3*(error3-self.prev_errors[2])/self.dt

                                #self.cmd_msg_linear_z=p2
                                #self.cmd_msg_linear_z=p2+self.integrals[1]+d2
                                self.cmd_msg_linear_x=Kp3*error3+Ki3*self.integrals[2]+Kd3*d3
                                #print(f"Command: {self.cmd_msg_linear_x}")
                                if self.cmd_msg_linear_x > 0.99:
                                    self.cmd_msg_linear_x = 0.99
                                if self.cmd_msg_linear_x < -0.99:
                                    self.cmd_msg_linear_x = -0.99
                                #Store the error
                                self.prev_errors[2]=error3
                                cv2.putText(self.frame, f"Vel X: {int(self.cmd_msg_linear_x)}",(20,600), 2, 1.0, (255,255,255), 2)
                            else:
                                self.cmd_msg_linear_x=0.0
                                self.integrals[2]=0.0


                        #Watch out: if the point is not visible in the frame, the system by default sets the point values to 0
                    elif self.keypoint_names.LEFT_SHOULDER[0] != 0.0 and self.keypoint_names.LEFT_SHOULDER[1] != 0.0 and self.keypoint_names.RIGHT_SHOULDER[0] != 0.0 and self.keypoint_names.RIGHT_SHOULDER[1] != 0.0:
                        cv2.circle(self.frame, (int(self.top_center_body_keypoint[0]), int(self.top_center_body_keypoint[1])), 8,(255,255,255),-1)
                        cv2.rectangle(self.frame, (int(x)-int(float(w)/2), int(y)-int(float(h)/2)-30),(int(x)-int(float(w)/2)+180, int(y)-int(float(h)/2)), (255,207,137),-1)
                        cv2.putText(self.frame,self.name, (int(x)-int(float(w)/2)+2, int(y)-int(float(h)/2)-4),2,1.0,(255,255,255),2)
                        #cv2.putText(self.frame,"Luka", (int(x)-int(float(w)/2)+2, int(y)-int(float(h)/2)-4),2,1.0,(255,255,255),2)
                        cv2.rectangle(self.frame, (int(x)-int(float(w)/2), int(y)-int(float(h)/2)), (int(x)+int(float(w)/2), int(y)+int(float(h)/2)), (255,207,137), 2)
                        #PID regulator(x movement):
                        #The center: (480,360)
                        #Left to right
                        if not(self.top_center_body_keypoint[0] > 460 and self.top_center_body_keypoint[0] < 500):
                            #PID gains
                            Kp1=0.0012
                            Ki1=0.0001
                            Kd1=0.0004

                        
                            error1=(480-self.top_center_body_keypoint[0])
                            #P
                            p1=Kp1*error1
                            #I
                            self.integrals[0]+=Ki1*error1*self.dt
                            #D
                            d1=Kd1*(error1-self.prev_errors[0])/self.dt

                            #self.cmd_msg_angular_z=p1
                            #self.cmd_msg_angular_z=p1+self.integrals[0]
                            self.cmd_msg_angular_z=p1+self.integrals[0]+d1 

                            #Restrict the angular z:
                            if self.cmd_msg_angular_z > 0.9:
                                self.cmd_msg_angular_z = 0.9
                            if self.cmd_msg_angular_z < -0.9:
                                self.cmd_msg_angular_z = -0.9

                            #Store the error
                            self.prev_errors[0]=error1

                        else:
                            self.cmd_msg_angular_z=0.0
                            self.integrals[0]=0.0
                        #Top to bottom
                        if not(self.top_center_body_keypoint[1] >= 340 and self.top_center_body_keypoint[1] <=380):
                            #PID gains
                            Kp2=0.8498
                            Ki2=0.0
                            Kd2=(0.336*9.446)/(1+9.446)

                            error2=(360-self.top_center_body_keypoint[1])/300
                            #P
                            p2=Kp2*error2
                            #I
                            self.integrals[1]+=Ki2*error2*self.dt
                            #D
                            d2=Kd2*(error2-self.prev_errors[1])/self.dt

                            #self.cmd_msg_linear_z=p2
                            #self.cmd_msg_linear_z=p2+self.integrals[1]+d2
                            self.cmd_msg_linear_z=Kp2*error2+Ki2*self.integrals[1]

                            #Store the error
                            self.prev_errors[1]=error2
                            
                        else:
                            self.cmd_msg_linear_z=0.0
                            self.integrals[1]=0.0

                        #Back-off if too close
                        self.cmd_msg_linear_x = -0.6
        
        return self.frame,self.cmd_msg_linear_x, self.cmd_msg_angular_z, self.cmd_msg_linear_z, self.prev_errors, self.integrals, self.distance_to_person,self.DISTANCE_EXCEEDED_COUNTER


    def recognize_template(self,frame,matcher,box,track_id,template_encoding):
        """
        Method to track the detected person in the frame if it is recognized.
        
        Args: box - bounding box of detected object
            track_id - track IDs of detected object
            template_encoding - encoding of template image
           
        Returns: frame - annotated frame
        """
        self.matcher=matcher
        self.box=box
        self.track_id=track_id
        self.template_encoding=template_encoding
        self.frame=frame
        self.cmd_msg_angular_z = 0.5
        self.cmd_msg_linear_x=0.0
        x,y,w,h=self.box     #x,y (box center), width, height
        yolo_box=int(x-w/2),int(y-h/2),int(w),int(h)
        track=self.track_history[self.track_id]
        track.append((float(x), float(y)))
        if len(track)>self.MAX_TRACK_LENGTH:
                    track.pop(0)
        self.unknown_image=self.frame[int(y)-int(float(h)/2):int(y)+int(float(h)/2),int(x)-int(float(w)/2):int(x)+int(float(w)/2)]
        
        self.result=self.matcher.match(self.unknown_image,self.template_encoding,self.track_id)
        
        if self.result is not None:
            self.TRACK_ID, self.FACE_DETECTED, self.cmd_msg_angular_z = self.result
            return self.TRACK_ID, self.FACE_DETECTED, self.frame,self.cmd_msg_angular_z 

        else:
             self.TRACK_ID=None
             self.FACE_DETECTED=False
             return self.TRACK_ID, self.FACE_DETECTED,self.frame,self.cmd_msg_angular_z


class StoreKeypoints():
    def __init__(self,keypoint_detections,i):
        self.NOSE=keypoint_detections.xy[i][0]
        self.LEFT_EYE=keypoint_detections.xy[i][1]
        self.RIGHT_EYE=keypoint_detections.xy[i][2]
        self.LEFT_EAR=keypoint_detections.xy[i][3]
        self.RIGHT_EAR=keypoint_detections.xy[i][4]
        self.LEFT_SHOULDER=keypoint_detections.xy[i][5]
        self.RIGHT_SHOULDER=keypoint_detections.xy[i][6]
        self.LEFT_ELBOW=keypoint_detections.xy[i][7]
        self.RIGHT_ELBOW=keypoint_detections.xy[i][8]
        self.LEFT_WRIST=keypoint_detections.xy[i][9]
        self.RIGHT_WRIST=keypoint_detections.xy[i][10]
        self.LEFT_HIP=keypoint_detections.xy[i][11]
        self.RIGHT_HIP=keypoint_detections.xy[i][12]
        self.LEFT_KNEE=keypoint_detections.xy[i][13]
        self.RIGHT_KNEE=keypoint_detections.xy[i][14]
        self.LEFT_ANKLE=keypoint_detections.xy[i][15]
        self.RIGHT_ANKLE=keypoint_detections.xy[i][16]
