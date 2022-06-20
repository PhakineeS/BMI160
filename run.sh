arm-linux-gnueabihf-g++ ./code/readData.cpp -lpthread ./bmi160.c  ./bmi160_interface.cpp -o c.out && adb push ./c.out /userdata
adb shell  /userdata/c.out