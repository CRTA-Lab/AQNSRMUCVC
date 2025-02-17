import face_recognition
import cv2


class Recognition: 
    def encode_template(self,template_image_path):
        """Functio to encode the single tempalte image
        
        Args: template_image_path - path to the template image
        
        Return: template_encoding - the encoding of the template image
        """
        self.template_image_path=template_image_path
        #Loading the template image
        template_image=face_recognition.load_image_file(self.template_image_path)
        #Try to get the encoding on the template image if there is a person

        try:
            template_encoding=face_recognition.face_encodings(template_image)[0]
            print("Template encoding successful!")
            return template_encoding
        except IndexError:
            print("No faces detected on template image!")
            return False

class Matcher:
    def match(self, unknown_image, template_encoding, track_id):
        self.unknown_image=unknown_image
        self.template_encoding=template_encoding
        self.track_id=track_id

        try:  
            self.unknown_image=cv2.cvtColor(self.unknown_image, cv2.COLOR_BGR2RGB)
            self.unknown_encoding=face_recognition.face_encodings(self.unknown_image)[0]
            results=face_recognition.compare_faces([self.template_encoding], self.unknown_encoding)
            if results[0]==True:
                self.TRACK_ID=self.track_id
                self.FACE_DETECTED=True
                self.cmd_msg_angular_z = 0.0

                return self.TRACK_ID, self.FACE_DETECTED,self.cmd_msg_angular_z
        except:
            print("No faces detected on the image!")
            
        
