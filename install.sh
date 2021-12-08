DIR_SELF="anymal_with_wheels/"
DIR_ANYMAL="anymal_c_simple_description/"
DIR_CPR="cpr_gazebo/"
DIR_VELODYNE="velodyne_simulator/"

RED=`tput setaf 1`
GREEN=`tput setaf 2`
RESET=`tput sgr0`

if ! [ -d "../$DIR_ANYMAL" ]; then
  ### Install anymal c ###
  echo "${RED}Installing anymal C files in ../${DIR_ANYMAL}:${RESET}"
  git clone https://github.com/ANYbotics/anymal_c_simple_description.git ./../anymal_c_simple_description/
fi

if ! [ -d "../$DIR_CPR" ]; then
  ### Install clearpath simulator ###
  echo "${RED}Installing CPR worlds in ../${DIR_CPR}:${RESET}"
  git clone https://github.com/clearpathrobotics/cpr_gazebo.git ./../cpr_gazebo/
fi

if ! [ -d "../$DIR_VELODYNE" ]; then
  ### Install velodyne simulator ###
  echo "${RED}Installing velodyne simulator file in ../${DIR_VELODYNE}:${RESET}"
  git clone https://bitbucket.org/DataspeedInc/velodyne_simulator.git ./../velodyne_simulator/
fi

echo "${RED}Installing packages dependencies:${RESET}"
cd ../../
rosstack profile && rospack profile

echo "${GREEN}Installing Rosdep: ${RESET}"
sudo apt-get install python-rosdep

echo "${GREEN}Installing Terminator: ${RESET}"
sudo apt-get install terminator

echo "${GREEN}Installig ${DIR_SELF} dependencies :${RESET}"
rosdep install --from-paths src/"${DIR_SELF}" --ignore-src -r -y
echo "${GREEN}Installig ${DIR_ANYMAL} dependencies :${RESET}"
rosdep install --from-paths src/"${DIR_ANYMAL}" --ignore-src -r -y
echo "${GREEN}Installig ${DIR_CPR} dependencies :${RESET}"
rosdep install --from-paths src/"${DIR_CPR}" --ignore-src -r -y
echo "${GREEN}Installig ${DIR_VELODYNE} dependencies :${RESET}"
rosdep install --from-paths src/"${DIR_VELODYNE}" --ignore-src -r -y
