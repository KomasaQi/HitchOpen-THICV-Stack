#!/bin/bash

sudo cp frpc /usr/bin/

if [[ ! -d /etc/frp ]]; then
	sudo mkdir /etc/frp
fi

sudo cp test.toml /etc/frp/frpc.toml

sudo cp systemd/frpc.service /etc/systemd/system/

sudo systemctl daemon-reload

sudo systemctl enable frpc.service

sudo systemctl start frpc.service
