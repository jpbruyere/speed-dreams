
# Script to scan through all robots

# Uncomment for SVN version checking
versioning="-s"
# or for Git-svn version checking
#versioning="-g"

# Process all files regardless of versioning
#versioning="-a"

drivers="`pwd`/../../../data/drivers"
cars="`pwd`/../../../data/cars/models/"

# Enable the screen shots
config="-C $HOME/.speed-dreams-2"
run="-r $HOME/Sources/speed-dreams/trunk/build/games/speed-dreams-2"

# Use alternative script to process images
#proc="-p my_script.sh"

# Check the car models (for humans)
python check_car_skins.py -c $cars $versioning $config $run $proc

# Usr
python check_robot_skins.py -d $drivers/usr_36GP -c $cars $versioning $config $run $proc usr_36GP.xml
python check_robot_skins.py -d $drivers/usr_ls1  -c $cars $versioning $config $run $proc usr_ls1.xml
python check_robot_skins.py -d $drivers/usr_ls2  -c $cars $versioning $config $run $proc usr_ls2.xml
python check_robot_skins.py -d $drivers/usr_lp1  -c $cars $versioning $config $run $proc usr_lp1.xml
python check_robot_skins.py -d $drivers/usr_mpa1 -c $cars $versioning $config $run $proc usr_mpa1.xml
python check_robot_skins.py -d $drivers/usr_sc   -c $cars $versioning $config $run $proc usr_sc.xml
python check_robot_skins.py -d $drivers/usr_trb1 -c $cars $versioning $config $run $proc usr_trb1.xml
python check_robot_skins.py -d $drivers/usr_rs   -c $cars $versioning $config $run $proc usr_rs.xml
python check_robot_skins.py -d $drivers/usr_mp10 -c $cars $versioning $config $run $proc usr_mp10.xml

# Simplix
python check_robot_skins.py -d $drivers/simplix_36GP -c $cars $versioning $config $run $proc simplix_36GP.xml
python check_robot_skins.py -d $drivers/simplix_ls1  -c $cars $versioning $config $run $proc simplix_ls1.xml
python check_robot_skins.py -d $drivers/simplix_ls2  -c $cars $versioning $config $run $proc simplix_ls2.xml
python check_robot_skins.py -d $drivers/simplix_lp1  -c $cars $versioning $config $run $proc simplix_lp1.xml
python check_robot_skins.py -d $drivers/simplix_mp5  -c $cars $versioning $config $run $proc simplix_mp5.xml
python check_robot_skins.py -d $drivers/simplix_sc   -c $cars $versioning $config $run $proc simplix_sc.xml
python check_robot_skins.py -d $drivers/simplix_trb1 -c $cars $versioning $config $run $proc simplix_trb1.xml
python check_robot_skins.py -d $drivers/simplix_mpa1 -c $cars $versioning $config $run $proc simplix_mpa1.xml
python check_robot_skins.py -d $drivers/simplix_ref  -c $cars $versioning $config $run $proc simplix_ref.xml

# Kilo2008
python check_robot_skins.py -d $drivers/kilo2008/ -c $cars $versioning $config $run $proc kilo2008.xml

# dandroid
python check_robot_skins.py -d $drivers/dandroid_36GP -c $cars $versioning $config $run $proc dandroid_36GP.xml

# shadow
python check_robot_skins.py -d $drivers/shadow_sc   -c $cars $versioning $config $run $proc shadow_sc.xml
python check_robot_skins.py -d $drivers/shadow_mpa11 -c $cars $versioning $config $run $proc shadow_mpa11.xml
python check_robot_skins.py -d $drivers/shadow_mpa12 -c $cars $versioning $config $run $proc shadow_mpa12.xml
