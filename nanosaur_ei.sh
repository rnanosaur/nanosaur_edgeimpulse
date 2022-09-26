#!/bin/bash
# Copyright (C) 2022, Raffaello Bonghi <raffaello@rnext.it>
# All rights reserved
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright 
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its 
#    contributors may be used to endorse or promote products derived 
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

bold=`tput bold`
red=`tput setaf 1`
green=`tput setaf 2`
yellow=`tput setaf 3`
blue=`tput setaf 4`
reset=`tput sgr0`

usage()
{
    if [ "$1" != "" ]; then
        echo "${red}$1${reset}" >&2
    fi

    local name=$(basename ${0})
    echo "$name Edge impulse on nanosaur." >&2
    echo "${bold}Commands:${reset}" >&2
    echo "  $name help                     This help" >&2
    echo "  $name run                      Run nanosaur ei in debug mode" >&2
    echo "  $name build                    Build docker" >&2
}

main()
{
    # Check if run in sudo
    if [[ `id -u` -eq 0 ]] ; then 
        echo "${red}Please don't run as root${reset}" >&2
        exit 1
    fi
    local option=$1
    if [ -z "$option" ] ; then
        usage
        exit 0
    fi
    # Load all arguments except the first one
    local arguments=${@:2}

    # Options
    if [ $option = "help" ] || [ $option = "-h" ]; then
        usage
        exit 0
    elif [ $option = "run" ] ; then
        echo "${green}Run Edge Impulse nanosaur docker debug ${reset}"
        echo " - ${bold}Load volume:${reset} ${yellow}$(pwd)${reset}" >&2
        #local pwd=$(pwd)
        #echo $local_folder
        docker run -it --rm -v $(pwd):/opt/ros_ws/src/nanosaur_ei nanosaur/edge_impulse:latest bash
        exit 0
    elif [ $option = "build" ] ; then
        local CI_OPTIONS=""
        if [[ $arguments = "CI" ]] ; then
            # Set no-cache and pull before build
            # https://newbedev.com/what-s-the-purpose-of-docker-build-pull
            CI_OPTIONS="--no-cache --pull"
        fi
        echo "${green}Build Edge Impulse nanosaur docker ${reset}"
        docker build $CI_OPTIONS -t nanosaur/edge_impulse:latest .
        exit 0
    fi

    usage "[ERROR] Unknown option: $option" >&2
    exit 1
}
main $@
exit 0
# EOF
