toolchain=~/arm-cortex_a7-linux-gnueabihf-linaro_4.9/bin
toolchain2="arm-eabi-"
export CROSS_COMPILE=$toolchain/"$toolchain2"
export KBUILD_BUILD_USER="chijure"
export KBUILD_BUILD_HOST="team-Redhawk"
export ARCH_MTK_PLATFORM="mt6582"
export TARGET_PRODUCT="wt98360"
./mk -opt=TARGET_BUILD_VARIANT=user wt98360 new k
