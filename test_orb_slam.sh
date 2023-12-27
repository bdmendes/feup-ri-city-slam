# ORB SLAM Dataset
cd lib/ORB_SLAM3/
mkdir -p Datasets/EuRoc/
cd Datasets/EuRoc/

if [ ! -d "MH01" ]; then
	wget -c http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip
	mkdir MH01
	unzip MH_01_easy.zip -d MH01/
	cd ../../ORB_SLAM3/
fi

cd ../..
./Examples/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular/EuRoC.yaml ./Datasets/EuRoc/MH01 ./Examples/Monocular/EuRoC_TimeStamps/MH01.txt dataset-MH01_mono
