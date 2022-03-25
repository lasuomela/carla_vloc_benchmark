#!/usr/bin/env bash

apt-get update & apt-get install --no-install-recommends -y tmux

pip3 install -r ./requirements.txt
