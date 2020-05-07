#!/bin/bash

make O=build clean
make O=build -j4
make O=build modules -j4
rm -rf /home/jiedeng/work_/github/projectacrn/nuc/deploy-github
make O=build modules_install INSTALL_MOD_PATH=/home/jiedeng/work_/github/projectacrn/nuc/deploy-github
rm /home/jiedeng/work_/github/projectacrn/nuc/deploy-github/lib/modules/5.4.28-PKT-200203T060100Z-00002-ga8cd22f/build
rm /home/jiedeng/work_/github/projectacrn/nuc/deploy-github/lib/modules/5.4.28-PKT-200203T060100Z-00002-ga8cd22f/source
