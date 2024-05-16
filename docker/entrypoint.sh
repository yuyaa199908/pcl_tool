#!/bin/bash

USER_ID=${LOCAL_UID:-9001}
GROUP_ID=${LOCAL_GID:-9001}

echo "Starting with UID : $USER_ID, GID: $GROUP_ID"
useradd -u $USER_ID -o -m user
groupmod -g $GROUP_ID user
export HOME=/home/user

adduser --disabled-password --gecos '' user
adduser user sudo
echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

exec /usr/sbin/gosu user "$@"