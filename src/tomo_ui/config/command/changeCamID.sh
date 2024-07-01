#!/bin/bash
sshId='tomo@192.168.0.222'
user='tomo'
pass='tomo'
while [[ $# -gt 0 ]]; do
    case "$1" in
    -o)
        o=$2
        shift
        ;;
    -n)
        n=$2
        shift
        ;;
    *)
        echo "Invalid argument: $1"
        exit 1
    esac
    shift
done

su -c "SSHPASS=$pass sshpass -e ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no $sshId 'sudo sed -i s/$o/$n/ /home/tomo/tomo_config/imaging/imaging_config.yaml'" -s "/bin/bash" $user
