"""
Initial code from: https://core-electronics.com.au/guides/pose-and-face-landmark-raspberry-pi/#Init and https://github.com/google/mediapipe/blob/master/docs/solutions/pose.md

Modified heavily to adapt to falls recognition based on pose and location of hands, face and legs
"""


#Import all important functionality
import cv2
import mediapipe as mp
import time
from camera_read import CameraRead


# check if fallen
def has_person_fallen(body_parts, y_threshold=0.6, visibility_threshold=0.9):
    """
    Checks if a person has fallen by measuring distance from ground certain body parts are.
    """
    valid_body_part = []
    found_legs = False
    found_arms = False
    found_face = False

    # filter valid body parts
    for idx, part in enumerate(body_parts):
        # check the part has valid coordinates
        # print(f"{idx}: y={part.y}, visibility={part.visibility}")
        if part is not None and part.y is not None:
            # only count visible body parts
            if part.visibility > visibility_threshold:
                # don't need to differentiate between left and right, only legs, arms and face
                if idx in [0, 1]:
                    found_legs = True
                elif idx in [2, 3]:
                    found_arms = True
                elif idx == 4:
                    found_face = True
                valid_body_part.append(part)

    # check if at least 1 body part found and either an arm + leg combo OR a face
    if len(valid_body_part) >= 1 and ((found_arms and found_legs) or found_face):
        for part in valid_body_part:
            # if the body part is above the threshold < not fallen. Only care about y_value. [0,0] is top left so y=1 = floor, ~0.7 is ankle height
            if part.y < y_threshold:
                return False
    else:
        return False

    # person has fallen - wait 1 second
    time.sleep(0.2)
    return True


def track_pose():
    """
    Uses camera to search for people. Checks pose of person, if hand and legs or face on the floor, they are considered to have fallen.
    """
    # Make a camera which can read the shared memory
    camera = CameraRead()

    #Initialise Media Pipe Pose features
    mp_pose=mp.solutions.pose
    mpDraw=mp.solutions.drawing_utils
    pose=mp_pose.Pose()

    #Start endless loop to create video frame by frame Add details about video size and image post-processing to better identify bodies
    while True:
        # grab latest frame from shared memory
        frame = camera.get_camera_frame()
        flipped=cv2.flip(frame,flipCode=1)
        frame_flipped = cv2.resize(flipped,(640,480))
        rgb_img=cv2.cvtColor(frame_flipped,cv2.COLOR_BGR2RGB)
        result=pose.process(rgb_img)
        
        try:
            if result.pose_landmarks is not None:
                # must look for body parts in the following order: legs > arms > face
                left_ankle = result.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ANKLE]
                right_ankle = result.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ANKLE]
                left_wrist = result.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST]
                right_wrist = result.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST]
                nose = result.pose_landmarks.landmark[mp_pose.PoseLandmark.NOSE]
                has_fallen = has_person_fallen([left_ankle, right_ankle, left_wrist, right_wrist, nose])
                print(has_fallen)
        except:
            pass



        #Draw the framework of body onto the processed image and then show it in the preview window
        mpDraw.draw_landmarks(frame_flipped,result.pose_landmarks,mp_pose.POSE_CONNECTIONS)
        cv2.imshow("Detect Fall",frame_flipped)
        
        #At any point if the | q | is pressed on the keyboard then the system will stop
        key = cv2.waitKey(1) & 0xFF
        if key ==ord("q"):
            break

    # Release the shared memory and destroy all windows
    camera.close()


if __name__ == "__main__":
    try:
        track_pose()
    except:
        pass