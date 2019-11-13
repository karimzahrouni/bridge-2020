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
//#include "ros.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Temperature.h>
//#include "std_msgs/String.h"
#include "nmea2k.h" // use dev branch!
#include "pgn/iso/Pgn60928.h" // ISO address claim
#include "pgn/Pgn126993.h" // heartbeat
#include "pgn/Pgn127245.h" // rudder
#include "Spektrum.h"
#include "hull14mod3.h"

#define BRIDGE_VERSION "14.3.0 PT1"

// ros::NodeHandle nh;
// std_msgs::String str_msg;
// ros::Publisher chatter("chatter", &str_msg);
Serial pc(USBTX,USBRX);
nmea2k::CANLayer n2k(p30,p29); // used for sending nmea2k messages
Spektrum rx(p13,p14);
DigitalOut txled(LED1);
DigitalOut rxled(LED2);
DigitalOut lifeled(LED3);
unsigned char node_addr = HULL14MOD3_BRIDGE_ADDR;

Thread heartbeat_thread;
Thread spektrum_thread;
Thread ros_thread;


void heartbeat_process(void);
void spektrum_process(void);
void ros_process(void);


//ROS stuff
ros::NodeHandle  nh;
std_msgs::String str_msg;
sensor_msgs::Temperature temp_msg;
ros::Publisher chatter("chatter", &str_msg);
ros::Publisher temp_pub("Temperature", &temp_msg);
ros::Publisher nav_pub("Location", &lat, &lon);
//this was here before ros::Publisher temp_pub("Temperature", &temp_msg);


float RC_1 = 0.0;
float RC_2 = 0.0;
float mast_cmd = 0.0;
float rud_cmd = 0.0;

int main(void)
{
    nmea2k::Frame f;
    nmea2k::PduHeader h;

    // TODO startup ROS publisher LATER
    //nh.initNode();
    //nh.advertise(chatter);

    // startup messages
    pc.printf("0x%02x:main: Bridge node version %s\r\n",node_addr,BRIDGE_VERSION);
    pc.printf("0x%02x:main: nmea2k version %s\r\n",node_addr,NMEA2K_VERSION);
    pc.printf("0x%02x:main: Spektrum version %s\r\n",node_addr,SPEKTRUM_VERSION);

    // Assert ISO address and wait

    // start the various processes
    heartbeat_thread.start(&heartbeat_process);
    spektrum_thread.start(&spektrum_process);
    ros_thread.start(&ros_process);

    pc.printf("0x%02x:main: listening for any pgn\r\n",node_addr);
    while (1) {

        if (n2k.read(f)) {
            rxled = 1;
            h = nmea2k::PduHeader(f.id);
            pc.printf("0x%02x:main: recieved priority %d, pgn %d, sa 0x%02x, da 0x%02x: 0x",node_addr,h.p(), h.pgn(), h.sa(), h.da());
            for (int i=0; i<f.len; i++)
                pc.printf("%02x",f.data[i]);
            pc.printf("\r\n");

            // TODO pass NMEA2000 to ROS via rosserial LATER
            // heartbeat = !heartbeat; // blink ROS activity light
            // str_msg.data = hello; // form message
            // chatter.publish(&str_msg); // publish it

            rxled = 0;
        } // if (n2k.read(f))

        //nh.spinOnce();
        ThisThread::sleep_for(10);
    } // while(1)
} // int main(void)






void heartbeat_process(void)
{
    nmea2k::Frame m;     // holds nmea2k data frame before sending
    nmea2k::PduHeader h; // ISO11783-3 header information
    nmea2k::Pgn126993 d(6000,0);   // for PGN data fields
    unsigned int heartbeat_interval=60;
    unsigned char c=0;           // heartbeat sends a heartbeat counter

    pc.printf("0x%02x:heartbeat_thread: starting heartbeat_process\r\n",
              node_addr);

    while (1) {
        h = nmea2k::PduHeader(d.p,d.pgn,node_addr,NMEA2K_BROADCAST); // form header
        d = nmea2k::Pgn126993(heartbeat_interval*100,c++); // form PGN fields
        m = nmea2k::Frame(h.id(),d.data(),d.dlen); // assemble message
        if (n2k.write(m)) { // send it!
            txled = 1;
            pc.printf("0x%02x:heartbeat_thread: sent %s, %0.0f s, count %d\r\n",
                      node_addr,
                      d.name,
                      (float) d.update_rate()/100.0,
                      d.heartbeat_sequence_counter());
            ThisThread::sleep_for(5);
            txled = 0;
        } else
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
// rx.channel[SPEKTRUM_CHANNELS].
// on DX9 normally 0 is throttle, 3 is rudder. Wheel is ???, mode sw is ???
// rx.channel values 0-2048
// fades is number of fades, valid is if data is valid
void spektrum_process(void)
{
    nmea2k::Frame m;     // holds nmea2k data frame before sending
    nmea2k::PduHeader h; // ISO11783-3 header information
    nmea2k::Pgn127245 d(0,0,0,0);   // rudder PGN
    unsigned int spektrum_interval=2;
    //unsigned char c=0;           // heartbeat sends a heartbeat counter

    pc.printf("0x%02x:spektrum_thread: starting spektrum_process\r\n",
              node_addr);

    // loop
    while(1) {
        // if rx.valid
        // if manual mode
        // updated commanded rudder, mainsail

        if (rx.valid) {
            RC_1 = rx.channel[1];
            RC_2 = rx.channel[2];     //channels will likely change
        } else {
            pc.printf("RCrudder invalid\r\n");
        }

        rud_cmd = (RC_1/6.77)-5.0;    //puts RC command into degrees
        mast_cmd = (RC_2/6.77)+39.19;

        pc.printf("rudder: %.1f, mast: %.1f\n", rud_cmd, mast_cmd);
        // send rudder via nmea
        // log rudder via ros because our own transmissions are not recorded?
        // send mainsail via nmea
        // log mainsail via ros because our own transmissions are not recorded?

        h = nmea2k::PduHeader(d.p,d.pgn,node_addr,NMEA2K_BROADCAST);
        d = nmea2k::Pgn127245(0, // instance
                              round(mast_cmd*PGN_127245_ANGLE_RES), // not actually the correct place for mast order
                              round(rud_cmd*PGN_127245_ANGLE_RES), // rudder command
                              0.0
                             );
        m = nmea2k::Frame(h.id(),d.data(),d.dlen);
        if (n2k.write(m)) {
            txled = 1;
            pc.printf("0x%02x:spektrum_thread: sent %s\r\n",
                      node_addr,
                      d.name);
            ThisThread::sleep_for(5);
            txled = 0;
        } else
            pc.printf("0x%02x:spektrum_thread: failed sending %s\r\n",
                      node_addr,
                      d.name);


        ThisThread::sleep_for(spektrum_interval*1000); // wait for loop execution time
    }
} // void spektrum_process(void)


void ros_process(void)
{

    nh.initNode();
    nh.advertise(chatter);
    nh.advertise(temp_pub);

    while (1) {
        
        // str_msg.data = hello;
        // chatter.publish( &str_msg );
        temp_msg.header.stamp = nh.now();
        temp=5.5;
        temp_msg.temperature=temp;
        temp_pub.publish(&temp_msg);

        temp_msg.header.frame_id = "boat";

        temp_msg.temperature = temp;

        temp_pub.publish(&temp_msg);
        nh.spinOnce();
        ThisThread::sleep_for(100);

    }//while(1)
} // void ros_process(void)


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
