DATASET_FOLDER=dataset/3dscanrep
TMP_FOLDER=/tmp

[[ ! -d $DATASET_FOLDER ]] && mkdir -p $DATASET_FOLDER

modelLinks="\
http://graphics.stanford.edu/pub/3Dscanrep/bunny.tar.gz \
http://graphics.stanford.edu/pub/3Dscanrep/dragon/dragon_stand.tar.gz
"

for link in $modelLinks; do
    model=`basename $link`
    wget -O $TMP_FOLDER/$model $link

    tar -xf $TMP_FOLDER/$model -C $DATASET_FOLDER
    rm -rf $TMP_FOLDER/$model
done