/** @file bridge-2020/main.cpp
    @brief Bridge node for AY20 Sailbot Hull 14 mod 3
    D Evangelista, 2019 

    October 2019
    To support PT1, the bridge node should receive all messages and
    pass them along to rosserial. Bridge node will eventually also deal
    with the Spektrum receiver but that's for later. 
*/

#include "mbed.h"
#include "rtos.h"
#include "nmea2k.h" // use dev branch!
#include "pgn/iso/Pgn60928.h" // ISO address claim
#include "pgn/Pgn126993.h" // heartbeat
//#include "pgn/Pgn127245.h" // later rudder
#include "hull14mod3.h"

#define BRIDGE_VERSION "14.3.0 PT1"

Serial pc(USBTX,USBRX);
nmea2k::CANLayer n2k(p30,p29); // used for sending nmea2k messages
DigitalOut txled(LED1);
DigitalOut rxled(LED2); 
unsigned char node_addr = HULL14MOD3_BRIDGE_ADDR; 
Thread heartbeat_thread;
//Thread spektrum_thread;

void heartbeat_process(void);
//void spektrum_process(void); 

int main(void){
  nmea2k::Frame f;
  nmea2k::PduHeader h;

  // startup messages
  pc.printf("0x%02x:main: Bridge node version %s\r\n",node_addr,BRIDGE_VERSION);
  pc.printf("0x%02x:main: nmea2k version %s\r\n",node_addr,NMEA2K_VERSION);

  // Assert ISO address and wait

  // start the various processes
  heartbeat_thread.start(&heartbeat_process); 

  pc.printf("0x%02x:main: listening for any pgn\r\n",node_addr);
  while (1){

    if (n2k.read(f)){
      rxled = 1; 
      h = nmea2k::PduHeader(f.id);
      pc.printf("0x%02x:main: recieved priority %d, pgn %d, sa 0x%02x, da 0x%02x: 0x",node_addr,h.p(), h.pgn(), h.sa(), h.da());
      for (int i=0; i<f.len; i++)
	pc.printf("%02x",f.data[i]);
      pc.printf("\r\n");
      // LATER - pass it to ROS via rosserial
      rxled = 0; 
    } // if (n2k.read(f))
    ThisThread::sleep_for(10); 
  } // while(1)
} // int main(void)






void heartbeat_process(void){
  nmea2k::Frame m;     // holds nmea2k data frame before sending
  nmea2k::PduHeader h; // ISO11783-3 header information 
  nmea2k::Pgn126993 d(6000,0);   // for PGN data fields
  unsigned int heartbeat_interval=60;
  unsigned char c=0;           // heartbeat sends a heartbeat counter

  pc.printf("0x%02x:heartbeat_thread: starting heartbeat_process\r\n",
	    node_addr); 

  while (1){
    h = nmea2k::PduHeader(d.p,d.pgn,node_addr,NMEA2K_BROADCAST); // form header 
    d = nmea2k::Pgn126993(heartbeat_interval*100,c++); // form PGN fields
    m = nmea2k::Frame(h.id(),d.data(),d.dlen); // assemble message
    if (n2k.write(m)){ // send it!
      txled = 1; 
      pc.printf("0x%02x:heartbeat_thread: sent %s, %0.0f s, count %d\r\n",
		node_addr,
		d.name,
		(float) d.update_rate()/100.0,
		d.heartbeat_sequence_counter());
      ThisThread::sleep_for(5); 
      txled = 0;
    }
    else
      pc.printf("0x%02x:heartbeat_thread: failed sending %s\r\n",
		node_addr,
		d.name); 
    ThisThread::sleep_for(heartbeat_interval*1000); 
  } // while(1)
} // void heartbeat_thread(void)




// TODO Later
// spektrum_process listens for radio inputs
// if mode is not manual, then be quiet
// else by default, it spits out current rudder command and mainsail command
// at given interval. 




// /**
//    rosserial Publisher example adapted for melodic
//    Prints "hello world!"
// */

// #include "mbed.h"

// #include "ros.h"
// #include "std_msgs/String.h"

// ros::NodeHandle nh;

// std_msgs::String str_msg;
// ros::Publisher chatter("chatter", &str_msg);

// char hello[13] = "hello world!";

// DigitalOut heartbeat(LED1);

// int main(void){
//   nh.initNode();
//   nh.advertise(chatter);

//   while(1) {
//     heartbeat = !heartbeat;
//     str_msg.data = hello;
//     chatter.publish(&str_msg);
//     nh.spinOnce();
//     wait_ms(1000);
//   } // while(1)
// } // main()
