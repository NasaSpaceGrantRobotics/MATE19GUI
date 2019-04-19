#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <string>
#include <stdio.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt16MultiArray.h>

using namespace std;

int addr1 = 0x04;  //primary address //successive i2c devices
int addr2 = 0x40; //successive i2c devices
int file_i2c; //primary address
int file_i2c2; //successive addresses
int pwmVals[8];
char *filename = (char*)"/dev/i2c-1";
int i2c_slave_overflow = 32;

void arrayCallback(const std_msgs::UInt16MultiArray::ConstPtr& array);

bool writeStream(int file_i2c, string output) { //writes a single string, must be interconnected words, to the i2c file and subsequent addresses
    if (output.length() <= i2c_slave_overflow) {
        if (write(file_i2c, output.c_str(), output.length()) != output.length()) {
            printf("\tFailed to write to the i2c bus.\n");
        }
    }
    else {  
        printf("\t\tinput too long |:| overflow bounds exceeded and message voided\n");
    }
    return true;
}

bool readStream(int file_i2c, char buf[]) { //reads a string from the i2c address and writes it to buf[], n must be length of expected message, otherwise there will be a filed read transaction
    int n=32; //this should be the number of expected characters
    if (read(file_i2c, buf, n) != n) {
        printf("\ti2c read transaction |:| failed\n");
    }
    else {
        //buf[] should contains read bytes/words??
        printf("\t%s\n", buf);
    }
}

void convertTo3BitHex(int val[], char pwmValsHex[]){
	for(int i =0; i<val.length(); i++){
		pwmValsHex.append(hex<<val[i]);
		if(i<(val.length()-1)){
			pwmValsHex.append('~');
		}
	}
	
}

void arrayCallback(const std_msgs::UInt16MultiArray::ConstPtr& array){
	int i = 0;
	//iterate through values
	for(std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it){
		pwmVals[i] = *it;
		i++;
	}
	char hexVals[32];
	//add in the sending of values and the hex conversion here
	return;
}

int main() {
	//Open i2c bus
	if((file_i2c = open(filename, O_RDWR)) < 0) {
		printf("\tFailed to open the i2c bus\n");
	}
	if(ioctl(file_i2c, I2C_SLAVE, addr1) < 0) {
		printf("\tFailed to acquire bus access and/or talk to slave.\n");
	}
	string input;
	
	ros::init(argc, argv, "pwmSubscriber");
	ros::NodeHandle n;
	ros::Subscriber subPWM = n.subscribe("arrayCallback");
	ros::spinOnce();
	//send data
	while(1){
		cin >> input;
	        writeStream(file_i2c, input);
        	char buf[32];
	        readStream(file_i2c, buf);
	}
	return 0;
}