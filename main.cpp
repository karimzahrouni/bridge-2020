/** @file bridge-2020/main.cpp
    @brief Bridge node for AY20 Sailbot Hull 14 mod 3
    D Evangelista, 2019
    October 2019
    To support PT1, the bridge node should receive all messages and
    pass them along to rosserial. Bridge node will eventually also deal
    with the Spektrum receiver but that's for later.
*/

#include "mbed.h"
#include <ros_lib_kinetic/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include "nmea2k.h" // use dev branch!
#include "pgn/iso/Pgn60928.h" // ISO address claim
#include "pgn/Pgn126993.h" // heartbeat
#include "pgn/Pgn127245.h" // rudder
#include "pgn/Pgn127250.h"
#include "pgn/Pgn127508.h"
#include "pgn/Pgn129025.h"
#include "pgn/Pgn130306.h"
#include "Spektrum.h"
#include "hull14mod3.h"

#define BRIDGE_VERSION "14.3.0 PT1"


Serial pc(NC,NC);
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
sensor_msgs::NavSatFix gps_msg;
sensor_msgs::Imu Imu_msg;
std_msgs::Float32 rudder_msg;
std_msgs::Float32 mast_msg; 
ros::Publisher gps_pub("NavSatFix", &gps_msg); //put in variable names here
ros::Publisher Imu_pub("Imu", &Imu_msg);
ros::Publisher rudder_pub("Rudder", &rudder_msg);
ros::Publisher mast_pub("Mast", &mast_msg); 
ros::Publisher chatter("chatter", &str_msg); //del

char hello[13] = "hello world!"; //del

float yaw = 0.0;
float lat = 0.0;
float lon = 0.0;
float bat = 0.0;

float RC_1 = 0.0;
float RC_2 = 0.0;
float mast_cmd = 0.0;
float rud_cmd = 0.0;
float cmnd[2];
int ii;

int main(void)
{
    nmea2k::Frame f;
    nmea2k::PduHeader h;

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
    while(1){
    if (n2k.read(f)) {
        h = nmea2k::PduHeader(f.id);
        if ((h.da() == NMEA2K_BROADCAST) || (h.da() == node_addr))
            switch(h.pgn()) {
                                            //cmnd[ii]/180.0*NMEA2K_PI*PGN_127245_ANGLE_RES
                case 127250:
		  {nmea2k::Pgn127250 d(f.data);
                    yaw = (float)d.heading()/PGN_127250_ANGLE_RES;
                    pc.printf("\r\n yaw: %f \r\n",yaw);}
		  break;

                case 129025:
		  {nmea2k::Pgn129025 d(f.data);
                    lat = (float)d.latitude()/PGN_129025_RES_LATITUDE;
                    lon = (float)d.longitude()/PGN_129025_RES_LONGITUDE;
                    pc.printf("\r\n lat: %f lon: %f \r\n",lat, lon);}
		  break; 

                case 127508:
		  {nmea2k::Pgn127508 d(f.data);
                    bat = (float)d.current()/PGN_127508_CURRENT_RES;
                    pc.printf("\r\n battery: %f \r\n",bat);}
		  break;

                default:
                    pc.printf("0x%02x:main: received unhandled PGN %d\r\n",
                              node_addr,h.pgn());
            } //if(h.pgn()...

        rxled = 0;
    } // if (n2k.read(f))

    //nh.spinOnce();
    ThisThread::sleep_for(100);
    } // while(1)

} //int main void




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
    //I forgot how to get data out of pgns..
}


void spektrum_process(void)
{

    nmea2k::Frame m;     // holds nmea2k data frame before sending
    nmea2k::PduHeader h; // ISO11783-3 header information
    nmea2k::Pgn127245 d(0,0,0,0);   // for PGN data fields
    int rudder_interval = .5; // nominally at a 1 s interval

    pc.printf("0x%02x:rudder_process: starting Rudder PGN 127245 tx process\r\n",
              node_addr);
    while (1) {

        //Spektrum part
        if (rx.valid) {
            RC_1 = rx.channel[0];
            RC_2 = rx.channel[7];
            pc.printf("RC: %f ",RC_1);

        } else {
            pc.printf("RCrudder invalid\r\n");
        }
        //(RC_1/6.77)-40.0;
        cmnd[0] = (RC_1/9)-20.0;
        cmnd[1] = (RC_2/9)-20.0;     //(RC_2/9)-20.0;
        pc.printf("rudder: %f mast: %f \r\n",cmnd[0],cmnd[1]);
        //rud_cmd = RC_1;    //puts RC command into degrees
        //mast_cmd = (RC_2/6.77)+39.19;
        //end spektrum part
        for(ii = 0 ; ii < 2 ; ii++) {
            txled = 1;
            d = nmea2k::Pgn127245((uint8_t) ii, // instance
                                  (uint8_t) PGN_127245_DIRECTION_RIGHT, // direction_order?
                                  (int16_t) round(cmnd[ii]/180.0*NMEA2K_PI*PGN_127245_ANGLE_RES), // angle_order
                                  (int16_t) round(-15.0/180.0*NMEA2K_PI*PGN_127245_ANGLE_RES)); // position
            h = nmea2k::PduHeader(d.p,d.pgn,node_addr,NMEA2K_BROADCAST); // form header
            m = nmea2k::Frame(h.id(),d.data(),d.dlen); // assemble message

            //debug("0x%02x:rudder_process: sending data",node_addr);
            //for (int i=0; i<8; i++){
            //  debug("%02x",d.data()[i]);
            //}
            //debug("\r\n");
            if (n2k.write(m)) // send it!
                pc.printf("0x%02x:rudder_process: sent %s, instance %d, direction_order %d, angle_order %3.1f, position %3.1f\r\n",
                          node_addr,
                          d.name,
                          d.instance(),
                          d.direction_order(),
                          (float) d.angle_order()/PGN_127245_ANGLE_RES*180.0/NMEA2K_PI,
                          (float) d.position()/PGN_127245_ANGLE_RES*180.0/NMEA2K_PI);
            else
                pc.printf("0x%02x:rudder_process: failed sending %s\r\n",
                          node_addr,d.name);
            txled = 0;
            ThisThread::sleep_for(10);
        }//for(int ii...
        ThisThread::sleep_for(rudder_interval*100);
    } // while(1)

} // void spektrum_process(void)

   


void ros_process(void){
  nh.initNode();
  nh.advertise(chatter); //del
  nh.advertise(gps_pub);
  nh.advertise(Imu_pub);
  nh.advertise(rudder_pub);
  nh.advertise(mast_pub); 
  
        
  while (1) {
    //led = !led;
    str_msg.data = hello; //del
    chatter.publish( &str_msg ); //del
    
    //GPS
    gps_msg.header.stamp = nh.now();
    gps_msg.latitude = lat;
    gps_msg.longitude = lon;
    gps_msg.altitude = 1.5;
    gps_pub.publish(&gps_msg);
    
    gps_msg.header.frame_id = "boat";
    gps_msg.latitude = lat;
    gps_msg.longitude = lon;
    gps_msg.altitude = 1.5;

    gps_pub.publish(&gps_msg);

    //IMU
    Imu_msg.header.stamp = nh.now();
    Imu_msg.orientation.x = 5;
    Imu_msg.orientation.y = 10;
    Imu_msg.orientation.z = yaw;
    Imu_pub.publish(&Imu_msg);
    
    Imu_msg.header.frame_id = "boat";
    Imu_msg.orientation.x = 5;
    Imu_msg.orientation.y = 10;
    Imu_msg.orientation.z = yaw;
    
    Imu_pub.publish(&Imu_msg);

    //RUDDER
    rudder_msg.data = rud_cmd;
    rudder_pub.publish(&rudder_msg);
    
    //MAST
    mast_msg.data = mast_cmd;
    mast_pub.publish(&mast_msg);    
    
    nh.spinOnce();
    
    ThisThread::sleep_for(100);
  } // while(1)
} // void ros_process(void)*/
