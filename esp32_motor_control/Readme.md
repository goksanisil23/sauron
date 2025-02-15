# Setup #
- Source the ESP environment:
source $HOME/esp/esp-idf/export.sh

- Go to the project dir and run
idf.py set-target esp32

- Then build
idf.py build

- Flash and monitor
idf.py -p /dev/ttyACM0 flash monitor

## Python environment
- python -m venv --system-site-packages my_venv