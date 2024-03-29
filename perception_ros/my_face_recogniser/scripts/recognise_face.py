#!/usr/bin/env python
import sys
import os
import rospy
from sensor_msgs.msg import Image
import rospkg

try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except Exception as e:
    print(e)
    print("No path needed to be deleted")

print("Importing cv2")
import cv2
print("Importing cv2..Done")
import face_recognition
from cv_bridge import CvBridge, CvBridgeError

class FaceRecogniser(object):

    def __init__(self):
        rospy.loginfo("Start FaceRecogniser Init process...")
        # get an instance of RosPack with the default search paths
        self.rate = rospy.Rate(5)
        rospack = rospkg.RosPack()
        # get the file path for my_face_recogniser
        self.path_to_package = rospack.get_path('my_face_recogniser')

        self.bridge_object = CvBridge()
        rospy.loginfo("Start camera suscriber...")
        self.cam_topic = "/head_camera/rgb/image_raw"
        self._check_cam_ready()
        self.image_sub = rospy.Subscriber(self.cam_topic,Image,self.camera_callback)
        rospy.loginfo("Finished FaceRecogniser Init process...Ready")

    def _check_cam_ready(self):
      self.cam_image = None
      while self.cam_image is None and not rospy.is_shutdown():
         try:
               self.cam_image = rospy.wait_for_message(self.cam_topic, Image, timeout=1.0)
               rospy.logdebug("Current "+self.cam_topic+" READY=>" + str(self.cam_image))

         except:
               rospy.logerr("Current "+self.cam_topic+" not ready yet, retrying.")

    def camera_callback(self,data):
        self.cam_image = data

    def loop(self):

        while not rospy.is_shutdown():
            self.recognise(self.cam_image)
            self.rate.sleep()

    def recognise(self,data):

        # Get a reference to webcam #0 (the default one)
        try:
            # We select bgr8 because its the OpneCV encoding by default
            video_capture = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)


        # Load a sample picture and learn how to recognize it.
        image_path = os.path.join(self.path_to_package,"person_img/standing_person.png")

        standing_person_image = face_recognition.load_image_file(image_path)
        standing_person_face_encoding = face_recognition.face_encodings(standing_person_image)[0]


        # Initialize some variables
        face_locations = []
        face_encodings = []
        face_names = []

        # Resize frame of video to 1/2 size for faster face recognition processing
        # If this is done be aware that you will have to make the recognition nearer.
        # In this case it will work around maximum 1 meter, more it wont work properly
        small_frame = cv2.resize(video_capture, (0, 0), fx=0.5, fy=0.5)

        #cv2.imshow("SMALL Image window", small_frame)


        # Find all the faces and face encodings in the current frame of video
        face_locations = face_recognition.face_locations(small_frame)
        face_encodings = face_recognition.face_encodings(small_frame, face_locations)

        if not face_encodings:
            rospy.logwarn("No Faces found, please get closer...")

        face_names = []
        for face_encoding in face_encodings:
            # See if the face is a match for the known face(s)
            match = face_recognition.compare_faces([standing_person_face_encoding], face_encoding)
            name = "Unknown"

            if match[0]:
                rospy.loginfo("MATCH")
                name = "StandingPerson"
            else:
                rospy.logwarn("NO Match")

            face_names.append(name)


        for (top, right, bottom, left), name in zip(face_locations, face_names):

            # Scale back up face locations since the frame we detected in was scaled to 1/4 size
            top *= 2
            right *= 2
            bottom *= 2
            left *= 2

            # Draw a box around the face
            cv2.rectangle(video_capture, (left, top), (right, bottom), (0, 0, 255), 2)

            # Draw a label with a name below the face
            cv2.rectangle(video_capture, (left, bottom - 35), (right, bottom), (0, 0, 255))
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(video_capture, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

        # Display the resulting image
        cv2.imshow("Image window", video_capture)
        cv2.waitKey(1)




def main():
    rospy.init_node('face_recognising_python_node', anonymous=True, log_level=rospy.INFO)

    face_recogniser_object = FaceRecogniser()

    face_recogniser_object.loop()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
