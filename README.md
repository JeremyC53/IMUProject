# This is my third year project on creating a body motion capture system for sports

ArduinoIMU.ino is the arduino code ran directly on the ESP32. Mosquitto need to be downloaded onto your local laptop to be able to communicate with the ESP32 through WiFi.
Then go to command prompt and write the following code. \
`net start mosquitto`\
`mosquitto_sub -h localhost -t imu_data`\
Now You should be able to see strings of numbers on the screen.\
For the demonstration using a cube animation, run the python code in wifitest.py\
Run the following code to download all dependencies in the terminal of vscode after downloading Python.\
`pip install math pygame PyOpenGL serial time paho-mqtt asyncio websockets`\
Then simply open cube.html on live server and there you go.
