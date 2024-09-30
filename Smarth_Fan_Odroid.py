import cv2 as cv
import threading
import time
import serial
import requests

# Load the face detection model
face_cascade = cv.CascadeClassifier(cv.data.haarcascades + 'haarcascade_frontalface_default.xml')

class VideoStream:
    def __init__(self, src=0):
        self.capture = cv.VideoCapture(src)
        self.ret, self.frame = self.capture.read()
        self.stopped = False

    def start(self):
        threading.Thread(target=self.update, args=()).start()
        return self

    def update(self):
        while not self.stopped:
            self.ret, self.frame = self.capture.read()
            time.sleep(0.05)

    def read(self):
        if not self.ret:
            return None
        return self.frame

    def stop(self):
        self.stopped = True
        self.capture.release()

# Initialize serial communication
SERIAL_PORT = '/dev/ttyS1'  # UART1 Odroid C4
BAUD_RATE = 19200
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# Start video stream
stream = VideoStream().start()

# ThingSpeak settings
thingspeak_write_key = 'UFGD4J66QXX4BRK1'
thingspeak_url = 'https://api.thingspeak.com/update'

def send_to_thingspeak(payload):
    response = requests.post(thingspeak_url, params=payload)
    print(f'Sent data to ThingSpeak: {payload}')
    print(f'Response Code: {response.status_code}, Response Text: {response.text}')

# Timing and control variables
last_send_time = 0
send_interval = 15  # Send data every 15 seconds
send_temperature_next = True  # Start by sending temperature data

frame_count = 0
detect_interval = 20
faces = []

previous_x_left = 0
previous_x_right = 0
threshold = 15
temperature = None

while True:
    frame = stream.read()
    if frame is None:
        print("No frame detected")
        break

    frame_count += 1
    small_frame = cv.resize(frame, (0, 0), fx=0.75, fy=0.75)

    if frame_count % detect_interval == 0:
        gray = cv.cvtColor(small_frame, cv.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(50, 50))

    if len(faces) > 0:
        faces = sorted(faces, key=lambda face: face[0])
        leftmost_x = faces[0][0]
        rightmost_x = faces[-1][0]

        if abs(previous_x_left - leftmost_x) > threshold:
            print(f'FaceX: {leftmost_x}')
            ser.write(f"FaceX: {leftmost_x}\n".encode())
            previous_x_left = leftmost_x

        if abs(previous_x_right - rightmost_x) > threshold:
            print(f'FaceY: {rightmost_x}')
            ser.write(f"FaceY: {rightmost_x}\n".encode())
            previous_x_right = rightmost_x

        for i, (x, y, w, h) in enumerate(faces):
            x, y, w, h = [int(v / 0.75) for v in (x, y, w, h)]
            color = (0, 0, 255) if i == 0 else (255, 0, 0) if i == len(faces) - 1 else (0, 255, 0)
            cv.rectangle(small_frame, (x, y), (x + w, y + h), color, 2)

    current_time = time.time()

    # Read data from serial port
    if ser.in_waiting > 0:
        raw_data = ser.readline().decode('utf-8', errors='ignore').strip()

        if raw_data.startswith("Temperature: "):
            temperature = float(raw_data.split(":")[1].strip())

    # Send data to ThingSpeak every 15 seconds, alternating between temperature and face count
    if current_time - last_send_time >= send_interval:
        if send_temperature_next:
            if temperature is not None:
                temp_payload = {
                    'api_key': thingspeak_write_key,
                    'field1': temperature
                }
                send_to_thingspeak(temp_payload)
                print(f'Sent temperature: {temperature}')
        else:
            face_count = len(faces)
            face_payload = {
                'api_key': thingspeak_write_key,
                'field2': face_count
            }
            send_to_thingspeak(face_payload)
            print(f'Sent face count: {face_count}')

        # Alternate between sending temperature and face count data
        send_temperature_next = not send_temperature_next
        last_send_time = current_time

    cv.imshow('Video Stream', small_frame)

    if cv.waitKey(1) == ord('q'):
        break

# Clean up resources
stream.stop()
cv.destroyAllWindows()
ser.close()
