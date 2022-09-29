#!/bin/bash
#sudo
sudo cp *.rules /etc/udev/rules.d/

sudo service udev reload

sudo service udev restart
