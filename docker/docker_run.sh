# docker run -it -e LOCAL_UID=$(id -u $USER) -e LOCAL_GID=$(id -g $USER) -v $PWD/:/home -v /mnt/bigdata/00_students/aichi_ucl/:/home/bigdata/ pointcloudlibrary/env:20.04 bash