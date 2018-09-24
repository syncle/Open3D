download_sun_scene()
{
    DATA_NAME=$1
    DATA_SCAN=$2
    wget -q -x -r -np -nH http://sun3d.cs.princeton.edu/data/$DATA_NAME/$DATA_SCAN/
    mkdir ../dataset/sun/
    mkdir ../dataset/sun/$DATA_NAME
    mv data/$DATA_NAME/$DATA_SCAN/* ../dataset/sun/$DATA_NAME/
    rm -rf data
}

download_sun_scene "hotel_stb" "scan1"
download_sun_scene "hotel_sf" "scan1"
download_sun_scene "mit_32_123" "123_1"
download_sun_scene "harvard_c6" "hv_c6_2"
download_sun_scene "home_puigpunyen" "home_puigpunyent_scan2_2012_aug_23"
download_sun_scene "hotel_graz" "scan1_2012_aug_29"
download_sun_scene "mit_3_huge_office" "cl_1"
