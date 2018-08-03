#!/usr/bin/env bash

RED='\033[0;31m'
YELLOW='\e[33m'
NO_COLOR='\033[0m'

function info() {
  (>&2 echo -e "[\e[34m\e[1mINFO\e[0m] $*")
}

function ok() {
  (>&2 echo -e "[\e[32m\e[1m OK \e[0m] $*")
}

function warning() {
  (>&2 echo -e "${YELLOW}[WARNING] $*${NO_COLOR}")
}

info "Installing pylon camera sdk"
warning "If the provided SDK version (5.0.9) is not compatible with your system, download correct sdk form here and install:
https://www.baslerweb.com/de/support/downloads/downloads-software/"

apt-get install libceres libgflags2 libgoogle-glog0 libspqr1.3
dpkg -i ./pylon_5.0.9.10389-deb0_amd64.deb
ok "Pylon camera sdk has been installed successfully"
info "Now you can build the pylon camera model by executing> bash /apollo/apollo.sh build_pylon_cam"