#/usr/bin/sh

python3 -m venv env --system-site-packages
./env/bin/pip3 install adafruit-circuitpython-pca9685
./env/bin/pip3 install adafruit-circuitpython-servokit

echo "run 'source env/bin/activate'"
