#!/bin/bash
clear

while getopts ":s:" opt; do
    case $opt in
        s)
			pkill -9 tomo_ui
            $HOME/ui_moduleinstall/lib/tomo_backup/tomo_backup $OPTARG
            if [ $? -eq 0 ]; then
                exit 0
            else
                exit 1
            fi
    esac
done

helpFunction()
{
   echo ""
   echo "Usage: -s [backup name to restore]"
   echo ""
   echo -e "\t-s Use it to restore backup file"

   exit 1 # Exit script after printing help
}

# Print helpFunction in case parameters are empty
if [ -z $OPTARG]
then
   echo -e "${BRed}[Backup Script] Some or all of the parameters are empty${NC}";
   helpFunction
fi
