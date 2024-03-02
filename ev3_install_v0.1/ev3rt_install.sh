#!/usr/bin/bash

#setup gcc,make,diff,unzip
sudo apt update
sudo apt install build-essential
sudo apt install unzip


#setup shell
sudo apt install ruby
sudo gem install shell

#setup ev3rt compilers
wget http://ev3rt-git.github.io/public/ev3rt-prepare-ubuntu.sh 
sudo bash ev3rt-prepare-ubuntu.sh 

#setup ev3rt source codes
wget http://www.toppers.jp/download.cgi/ev3rt-1.1-release.zip
/usr/bin/unzip ev3rt-1.1-release.zip
mv -i ev3rt-1.1-release /mnt/c/ 

cd /mnt/c/ev3rt-1.1-release
tar xvf hrp3.tar.xz

#update pass1.rb
cd ~
cp pass1.rb /mnt/c/ev3rt-1.1-release/hrp3/cfg/pass1.rb
ln -s /mnt/c/ev3rt-1.1-release/