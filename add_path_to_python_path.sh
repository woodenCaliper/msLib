#!/bin/sh

#import msLibができるように
#python_pathにmsLibのパスを覚えさす


dirPath=$(cd $(dirname $0) && pwd)
echo '\nexport PYTHONPATH='${dirPath}':$PYTHONPATH' >> ~/.bashrc
