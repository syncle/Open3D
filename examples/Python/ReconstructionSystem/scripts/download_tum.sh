download_tum_scene()
{
    DATA_CLASS=$1
    DATA_NAME=$2
    wget https://vision.in.tum.de/rgbd/dataset/$DATA_CLASS/rgbd_dataset_$DATA_CLASS_$DATA_NAME.tgz
    tar zxvf rgbd_dataset_$DATA_CLASS_$DATA_NAME.tgz
    python synchronize_tum.py rgbd_dataset_$DATA_CLASS_$DATA_NAME
    mv rgbd_dataset_$DATA_CLASS_$DATA_NAME ../dataset/tum/$DATA_CLASS_$DATA_NAME
    rm rgbd_dataset_$DATA_CLASS_$DATA_NAME.tgz
}

download_tum_scene "freiburg1" "room"
download_tum_scene "freiburg2" "desk"
download_tum_scene "freiburg3" "long_office_household"
