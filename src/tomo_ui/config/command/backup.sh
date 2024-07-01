#!/bin/bash
BBlue='\033[1;34m'
BGreen='\033[1;32m'
BRed='\033[1;31m'
BYellow='\033[1;33m'
NC='\033[0m'

# Local backup script for Xavier devices

# Set default values
name_backup=""
remove_backup=""
extract_backup=""

xavier_1_ip="tomo@192.168.0.221"
xavier_2_ip="tomo@192.168.0.222"
xavier_3_ip="tomo@192.168.0.223"
xavier_4_ip="tomo@192.168.0.224"
# xavier_5_ip="tomo@192.168.0.225"

# pass='tomo'
# user='tomo'

pass='tomo'
user='tomo'

clear

helpFunction()
{
   echo ""
   echo "Usage:  -n [backup name to save] | -r [backup name to remove] | -s [backup name to restore]"
   echo ""
   echo -e "\t-n Use it to save new backup file (Default path is ~/tomo_backup)"
   echo -e "\t-r Use it to remove backup file"
   echo -e "\t-s Use it to restore backup file"

   exit 1 # Exit script after printing help
}

backup_and_scp() {
    xavier=$1
    folder=$2
    sshId=$3

	su -c "SSHPASS=$pass sshpass -e ssh -q -o ConnectTimeout=5 -o StrictHostKeyChecking=no $sshId exit" -s "/bin/bash" $user

    if [ $? -eq 0 ]; then
		cd $(dirname $name_backup)
		mkdir -p $(basename $name_backup) && cd $(basename $name_backup)
		mkdir -p $xavier && cd $xavier

        # RSYNC the backup file to your local computer
		while IFS=';' read -ra ADDR; do
			counter=0
			for path in "${ADDR[@]}"; do
				if [ -e "$zip_file" ]; then
					sudo rm -rf $(basename $name_backup)
				fi
				counter=$((counter+1))
				sudo mkdir folder_$counter
				SSHPASS=$pass rsync -avrq --rsh="sshpass -e ssh -l $user" $sshId:$path $name_backup/$xavier/folder_$counter

				if [ $? -eq 0 ]; then
					echo -e "${BGreen}[Backup Script] Backup file [$path] to [$name_backup/$xavier] in local computer successfully${NC}"
				else
					echo -e "${BRed}[Backup Script] Failed to backup file [$path] to [$name_backup/$xavier] in local computer${NC}"
					exit 1
				fi
			done
		done <<< "$folder"
		cd $(dirname $name_backup)
		zip -r -q $(basename $name_backup).zip $(basename $name_backup)
		rm -rf $(basename $name_backup)
    else
        echo -e "${BRed}[Backup Script] Backup failed, can not connect xavier name [$xavier] with ip [$sshId]${NC}"
        exit 1
    fi
}

remove_backup(){

	cd $(dirname $remove_backup)
	rm -rf "$(basename $remove_backup)" && rm -rf "$(basename $remove_backup).zip"
	if [ $? -eq 0 ]; then
		echo -e "${BGreen}[Backup Script] Removing backup file: $remove_backup successfully${NC}"
	else
		echo -e "${BRed}[Backup Script] Removing failed, please report it${NC}"
        exit 1
	fi
}

extract_backup(){
	xavier=$1
	folder=$2
    sshId=$3

	su -c "SSHPASS=$pass sshpass -e ssh -q -o ConnectTimeout=5 -o StrictHostKeyChecking=no $sshId exit" -s "/bin/bash" $user

	if [ $? -eq 0 ]; then
        # The extract file to your local computer
		while IFS=';' read -ra ADDR; do
			counter=0
			for path in "${ADDR[@]}"; do
				##########
				# Important - confirm before delete folder. To minimize risks, before delete we zip current folder
				# If can not extract we restore by current folder by zip file.

				su -c "SSHPASS=$pass sshpass -e ssh -q -o ConnectTimeout=5 -o StrictHostKeyChecking=no $sshId 'if [ -d $path ] ; then exit 0; else exit 1;fi'" -s "/bin/bash" $user
				if [ $? -ne 0 ]; then
					echo -e "${BYellow}[Backup Script] [$path] not exist in [$sshId]. Create empty folder${NC}"
					su -c "SSHPASS=$pass sshpass -e ssh -q -o ConnectTimeout=5 -o StrictHostKeyChecking=no $sshId 'sudo mkdir $path'" -s "/bin/bash" $user
				fi

				# Do zip
				currentEpocTime=`date +"%s"`
				su -c "SSHPASS=$pass sshpass -e ssh -q -o ConnectTimeout=5 -o StrictHostKeyChecking=no $sshId 'cd $(dirname $path) && sudo zip -rq $(basename $path).zip $(basename $path) && sudo mv -f $(basename $path).zip $(basename $path)_$currentEpocTime.zip'" -s "/bin/bash" $user
				# Do delete
				if [ $? -eq 0 ]; then
					su -c "SSHPASS=$pass sshpass -e ssh -q -o ConnectTimeout=5 -o StrictHostKeyChecking=no $sshId 'sudo rm -rf $path'" -s "/bin/bash" $user
					if [ $? -eq 0 ]; then
						su -c "SSHPASS=$pass sshpass -e ssh -q -o ConnectTimeout=5 -o StrictHostKeyChecking=no $sshId 'sudo mkdir $path'" -s "/bin/bash" $user
						if [ $? -eq 0 ]; then
							echo -e "${BGreen}[Backup Script] Clear [$(basename $path)] in [$xavier] complete.${NC}"
						else
							echo -e "${BRed}[Backup Script]	Can not create [$path] in [$$sshId]. Error occur, please contact with admin${NC}"
							# Do restore from local
							su -c "SSHPASS=$pass sshpass -e ssh -q -o ConnectTimeout=5 -o StrictHostKeyChecking=no $sshId 'cd $(dirname $path) && sudo unzip -q -o $(basename $path)_$currentEpocTime.zip'" -s "/bin/bash" $user
							exit 1
						fi
					else
						echo -e "${BRed}[Backup Script]	Can not remove [$path] in [$$sshId]. Error occur, please contact with admin${NC}"
						# Do restore from local
						su -c "SSHPASS=$pass sshpass -e ssh -q -o ConnectTimeout=5 -o StrictHostKeyChecking=no $sshId 'cd $(dirname $path) && sudo unzip -q -o $(basename $path)_$currentEpocTime.zip'" -s "/bin/bash" $user
						exit 1
					fi
				else
					echo -e "${BRed}[Backup Script] Zip file for [$path] not exist, no delete. Error occur, please contact with admin${NC}"
					# Do restore from local
					su -c "SSHPASS=$pass sshpass -e ssh -q -o ConnectTimeout=5 -o StrictHostKeyChecking=no $sshId 'cd $(dirname $path) && sudo unzip -q -o $(basename $path)_$currentEpocTime.zip'" -s "/bin/bash" $user
					exit 1
				fi
				# Do extract, before that change some permission
				su -c "SSHPASS=$pass sshpass -e ssh -q -o ConnectTimeout=5 -o StrictHostKeyChecking=no $sshId 'sudo chown -R $user $path'" -s "/bin/bash" $user
				counter=$((counter+1))
				SSHPASS=$pass rsync -avhq --rsh="sshpass -e ssh -l $user" $xavier/folder_$counter/$(basename $path)/ $sshId:$path
				###########

				if [ $? -eq 0 ]; then
					echo -e "${BGreen}[Backup Script] Extract [$(basename $path)] to [$xavier] computer successfully${NC}"
					su -c "SSHPASS=$pass sshpass -e ssh -q -o ConnectTimeout=5 -o StrictHostKeyChecking=no $sshId 'cd $(dirname $path) && sudo rm -rf $(basename $path)_$currentEpocTime.zip'" -s "/bin/bash" $user
				else
					echo -e "${BYellow}[Backup Script] Failed to extract [$path] in [$xavier] computer. Will restore with local backup${NC}"
					# Do restore from local
					su -c "SSHPASS=$pass sshpass -e ssh -q -o ConnectTimeout=5 -o StrictHostKeyChecking=no $sshId 'cd $(dirname $path) && sudo unzip -q -o $(basename $path)_$currentEpocTime.zip'" -s "/bin/bash" $user
				fi
		done
		done <<< "$folder"
    else
        echo -e "${BRed}[Backup Script] Extract failed, can not connect xavier name [$xavier] with ip [$sshId]${NC}"
        exit 1
    fi
}

while getopts ":n:r:s:" opt; do
    case $opt in
        n)
			echo -e "${BBlue}[Backup Script] Backuping ...${NC}"
            name_backup=$OPTARG

			# Example usage with custom backup name
			# backup_and_scp "Test1" "$HOME/ui_moduleinstall;$HOME/ws_tomo/install;$HOME/tomo_stats;/etc/tomo"	"localhost"

			# Backup and SCP
			backup_and_scp "XC1" 	"/home/tomo/tomo_config;/etc/tomo"    											$xavier_1_ip
			backup_and_scp "XC2" 	"/home/tomo/tomo_config/imaging"    											$xavier_2_ip
			backup_and_scp "XC3" 	"/home/tomo/tomo_config;/etc/tomo"   											$xavier_3_ip
			backup_and_scp "XC4" 	"/home/tomo/tomo_config"       													$xavier_4_ip
			# backup_and_scp "XC5" 	"/home/tomo/ws_tomo/install;/home/tomo/ui_moduleinstall;/home/tomo/tomo_stats"       	$xavier_5_ip

			echo -e "${BBlue}[Backup Script] Done${NC}"
            exit 0
            ;;
        r)
            # Capture the value of the -r option
			echo -e "${BBlue}[Backup Script] Removing ...${NC}"
            remove_backup=$OPTARG
			remove_backup
			echo -e "${BBlue}[Backup Script] Done${NC}"
            exit 0
            ;;
		s)
			# Capture the value of the -s option
			echo -e "${BBlue}[Backup Script] Extracting ...${NC}"
			extract_backup=$OPTARG

			cd $(dirname $extract_backup)
			unzip -q -o "$(basename $extract_backup).zip"
			cd $(basename $extract_backup)

			# Example usage with custom backup name
			# extract_backup "Test1" "$HOME/ui_moduleinstall;$HOME/ws_tomo/install;$HOME/tomo_stats;/etc/tomo"	"localhost"

			# Extract and SCP
			extract_backup "XC1" 	"/home/tomo/tomo_config;/etc/tomo"    											$xavier_1_ip
			extract_backup "XC2" 	"/home/tomo/tomo_config/imaging"    											$xavier_2_ip
			extract_backup "XC3" 	"/home/tomo/tomo_config;/etc/tomo"       													$xavier_3_ip
			extract_backup "XC4" 	"/home/tomo/tomo_config"       	$xavier_4_ip
			# extract_backup "XC5" 	"/home/tomo/tomo_stats"       	$xavier_5_ip

			cd $(dirname $extract_backup)
			rm -rf $(basename $extract_backup)

			echo -e "${BBlue}[Backup Script] Done${NC}"
			exit 0
			;;
        \?)
            echo -e "${BRed}[Backup Script] Invalid option: -$OPTARG ${NC}" >&2
            exit 1
            ;;
        :)
            echo -e "${BRed}[Backup Script] Option -$OPTARG requires an argument${NC}" >&2
            exit 1
            ;;
    esac
done

shift $((OPTIND - 1))

# Print helpFunction in case parameters are empty
if [ -z $name_backup ] && [ -z $remove_backup ] && [ -z $extract_backup]
then
   echo -e "${BRed}[Backup Script] Some or all of the parameters are empty${NC}";
   helpFunction
fi
  