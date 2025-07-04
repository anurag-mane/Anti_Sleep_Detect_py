import cv2
import mediapipe as mp
import time
import winsound
import threading    
import os


def play_alarm():
    sound_path = os.path.abspath("digital-alarm-clock-151920.wav")
    winsound.PlaySound(sound_path, winsound.SND_FILENAME)

mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh()
cap = cv2.VideoCapture(0)

LEFT_EYE_TOP = 159
LEFT_EYE_BOTTOM = 145
RIGHT_EYE_TOP = 386
RIGHT_EYE_BOTTOM = 374

start_time = 0
eyes_closed = False
alarm_on = False

alarm_thread = None

while True:
    success, frame = cap.read()
    if not success:
        break

    frame = cv2.flip(frame, 1)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    h, w, _ = frame.shape
    results = face_mesh.process(rgb)

    if results.multi_face_landmarks:
        for face_landmarks in results.multi_face_landmarks:
            l_top = face_landmarks.landmark[LEFT_EYE_TOP]
            l_bottom = face_landmarks.landmark[LEFT_EYE_BOTTOM]
            r_top = face_landmarks.landmark[RIGHT_EYE_TOP]
            r_bottom = face_landmarks.landmark[RIGHT_EYE_BOTTOM]

            left_eye_height = abs(l_top.y - l_bottom.y) * h
            right_eye_height = abs(r_top.y - r_bottom.y) * h

            print(f"Left Eye: {left_eye_height:.2f}, Right Eye: {right_eye_height:.2f}")

            if left_eye_height < 10 and right_eye_height < 10:
                if not eyes_closed:
                    eyes_closed = True
                    start_time = time.time()
                else:
                    if time.time() - start_time > 3 and not alarm_on:
                        print("🔔 Alarm condition met! Playing sound...")
                        alarm_on = True
                        alarm_thread = threading.Thread(target=play_alarm, daemon=True)
                        alarm_thread.start()

            else:
                eyes_closed = False
                if alarm_on:
                    alarm_on = False
                    winsound.PlaySound(None, winsound.SND_PURGE)  

           
            for point in [l_top, l_bottom, r_top, r_bottom]:
                cx, cy = int(point.x * w), int(point.y * h)
                cv2.circle(frame, (cx, cy), 3, (255, 0, 0), -1)

    
    if alarm_thread and alarm_thread.is_alive():
        cv2.putText(frame, "😴 SLEEP DETECTED!", (30, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

    cv2.imshow("Anti-Sleep Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
