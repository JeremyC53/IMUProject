import math
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *
import serial
import time
import paho.mqtt.client as mqtt
import asyncio
import websockets
import json


# IMU Data
ax = ay = az = 0.0
last_time = time.time()
websocket = None

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    topic = "imu_data"  # Replace with your actual topic
    print(f"Subscribing to: {topic}")
    client.subscribe(topic)
    print("Successfully subscribed")

def on_message(client, userdata, msg):
    global ax, ay, az, last_time
    try:
        data = msg.payload.decode()
        values = data.split(',')
        decoded_values = list(map(float, values))
        if len(values) == 9:
            accel_x, accel_y, accel_z = decoded_values[:3]
            gyro_x, gyro_y, gyro_z = decoded_values[3:6]

            # Calculate pitch and roll
            ay = math.atan2(accel_x, math.sqrt(accel_y**2 + accel_z**2)) * 180 / math.pi
            ax = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2)) * 180 / math.pi

            # Integrate yaw from gyro_z
            current_time = time.time()
            delta_time = current_time - last_time
            az += gyro_z * delta_time
            last_time = current_time
            print(f"Gyroscope: {gyro_x:.2f}, {gyro_y:.2f}, {gyro_z:.2f}")
            print(f"Angles - Pitch: {ay:.2f}, Roll: {ax:.2f}, Yaw: {az:.2f}")
            send_websocket_data()

            
    except Exception as e:
        print(f"Error processing message: {e}")

async def connect_websocket():
    global websocket
    try:
        websocket = await websockets.connect('ws://localhost:8765')
        print("WebSocket connected")
    except Exception as e:
        print(f"WebSocket connection error: {e}")
        websocket = None

def send_websocket_data():
    global websocket
    if websocket:
        data = {"ax": ax, "ay": ay, "az": az}
        print("Yurr")
        asyncio.get_event_loop().run_until_complete(websocket.send(json.dumps(data)))

def resize(width, height):
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0 * width / height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)


def drawText(position, textString):
    font = pygame.font.SysFont("Courier", 18, True)
    textSurface = font.render(textString, True, (255, 255, 255, 255), (0, 0, 0, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)


def draw():
    global ax, ay, az

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    glTranslatef(0, 0.0, -7.0)

    # Display pitch, roll, and yaw
    osd_text = f"Pitch: {ay:.2f}, Roll: {ax:.2f}, Yaw: {az:.2f}"
    drawText((-2, -2, 2), osd_text)

    # Rotate cube
    glRotatef(az, 0.0, 1.0, 0.0)  # Yaw: Rotate around the y-axis
    glRotatef(ay, 1.0, 0.0, 0.0)  # Pitch: Rotate around the x-axis
    glRotatef(-ax, 0.0, 0.0, 1.0)  # Roll: Rotate around the z-axis

    # Cube rendering
    glBegin(GL_QUADS)
    # Top face
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(1.0, 0.2, 1.0)

    # Bottom face
    glColor3f(1.0, 0.5, 0.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(1.0, -0.2, -1.0)

    # Front face
    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)

    # Back face
    glColor3f(1.0, 1.0, 0.0)
    glVertex3f(1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, -1.0)

    # Left face
    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, 1.0)

    # Right face
    glColor3f(1.0, 0.0, 1.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, -1.0)
    glEnd()


def main():
    loop = asyncio.get_event_loop()
    loop.run_until_complete(connect_websocket())
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    
    # Connect to the MQTT broker
    mqtt_broker = "172.20.10.2"  # Replace with your broker's IP
    client.connect(mqtt_broker, 1883, 60)
    client.loop_start()
    # Run the WebSocket server
    pygame.init()
    video_flags = OPENGL | DOUBLEBUF
    screen = pygame.display.set_mode((640, 480), video_flags)
    pygame.display.set_caption("Press Esc to quit")

    resize(640, 480)
    init()

    clock = pygame.time.Clock()

    try:
        while True:
            for event in pygame.event.get():
                if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                    raise KeyboardInterrupt
            
            draw()
            pygame.display.flip()
            clock.tick(60)

    except KeyboardInterrupt:
        print("Closing application...")
    finally:
        # Cleanup
        if websocket:
            loop.run_until_complete(websocket.close())
        pygame.quit()
        client.loop_stop()
        client.disconnect()


if __name__ == '__main__':
    main()
