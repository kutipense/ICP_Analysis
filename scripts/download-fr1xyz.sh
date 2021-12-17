DATASET_FOLDER=dataset/fr1xyz
TMP_FOLDER=/tmp

[[ ! -d $DATASET_FOLDER ]] && mkdir -p $DATASET_FOLDER

modelLinks="\
https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.tgz
"

for link in $modelLinks; do
    model=`basename $link`
    wget -O $TMP_FOLDER/$model $link

    tar -xf $TMP_FOLDER/$model -C $DATASET_FOLDER
    rm -rf $TMP_FOLDER/$model
done