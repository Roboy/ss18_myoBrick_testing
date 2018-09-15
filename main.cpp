/*
    BSD 3-Clause License

    Copyright (c) 2017, Roboy
            All rights reserved.

    Redistribution and use in source and binary forms, with or without
            modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

    * Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
            IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
            FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
            DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
            SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
            CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    author: Simon Trendel ( simon.trendel@tum.de ), 2018
    description: main
*/

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "hwlib.h"
#include "socal/hps.h"
#include "roboy_plexus/hps_0.h"
#include "roboy_plexus/roboyPlexus.hpp"
#include "roboy_plexus/myoControl.hpp"
#include <ctime>
#include <roboy_plexus/A1335.hpp>
#include <image_transport/image_transport.h>
    #include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <stdio.h>

//#include <cv_bridge/cv_bridge.h>
using namespace std;

#define ALT_LWFPGASLVS_OFST 0xFF200000
#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )








int main(int argc, char *argv[]) {
    void *virtual_base;
    int fd;
    int32_t *h2p_lw_led_addr;
    int32_t *h2p_lw_adc_addr;
    int32_t *h2p_lw_switches_addr;
    int32_t *h2p_lw_darkroom_addr;
    vector<int32_t*> h2p_lw_darkroom_ootx_addr;
    vector<int32_t*> h2p_lw_myo_addr;
    vector<int32_t*> h2p_lw_i2c_addr;

//     map the address space for all registers into user space so we can interact with them.
//     we'll actually map in the entire CSR span of the HPS since we want to access various registers within that span
    if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
        printf( "ERROR: could not open \"/dev/mem\"...\n" );
        return( 1 );
    }

    virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );

    if( virtual_base == MAP_FAILED ) {
        printf( "ERROR: mmap() failed...\n" );
        close( fd );
        return( 1 );
    }

    h2p_lw_led_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + LED_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
    h2p_lw_switches_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + SWITCHES_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
    h2p_lw_myo_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + MYOCONTROL_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
    h2p_lw_myo_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + MYOCONTROL_1_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#ifdef I2C_0_BASE
    h2p_lw_i2c_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#else
    h2p_lw_i2c_addr.push_back(nullptr);
#endif
#ifdef I2C_1_BASE
    h2p_lw_i2c_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_1_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#else
    h2p_lw_i2c_addr.push_back(nullptr);
#endif
#ifdef I2C_2_BASE
    h2p_lw_i2c_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_2_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#else
    h2p_lw_i2c_addr.push_back(nullptr);
#endif
#ifdef I2C_3_BASE
    h2p_lw_i2c_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_3_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#else
    h2p_lw_i2c_addr.push_back(nullptr);
#endif
#ifdef DARKROOM_0_BASE
    h2p_lw_darkroom_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + DARKROOM_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
#else
    h2p_lw_darkroom_addr = nullptr;
#endif
#ifdef DARKROOMOOTXDECODER_0_BASE
    h2p_lw_darkroom_ootx_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + DARKROOMOOTXDECODER_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#else
    h2p_lw_darkroom_ootx_addr.push_back(nullptr);
#endif
#ifdef ADC_LTC2308_0_BASE
    h2p_lw_adc_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ADC_LTC2308_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
#else
    h2p_lw_adc_addr = nullptr;
#endif

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "roboy_plexus");
        ros::start();
    }



/// JULIUS STUFF:


    ros::NodeHandlePtr nh = ros::NodeHandlePtr(new ros::NodeHandle);
    ros::Publisher motorAngle_pub = nh->advertise<roboy_communication_middleware::MotorAngle>("/roboy/middleware/MotorAngle",
                                                                                              1);
///  TEMPERATURE SENSOR INITATION

    // set measure number for ADC convert
    IOWR(h2p_lw_adc_addr, 0x01, NUMBER_OF_ADC_SAMPLES);
    // start measure
    for (uint channel = 0; channel < 8; channel++) {
        IOWR(h2p_lw_adc_addr, 0x00, (channel << 1) | 0x00);
        IOWR(h2p_lw_adc_addr, 0x00, (channel << 1) | 0x01);
        IOWR(h2p_lw_adc_addr, 0x00, (channel << 1) | 0x00);
        usleep(1);
    }


    cout << "testing my stuff" << endl;
    //has helper functions
    MyoControlPtr myoControl = MyoControlPtr(new MyoControl(h2p_lw_myo_addr,h2p_lw_adc_addr));
    const int B = 4275;               // B value of the thermistor
    const int R0 = 100000;            // R0 = 100k





///ANGLE SENSOR INITIATION

    I2C i2c(h2p_lw_i2c_addr[1]);
    vector<uint8_t> active_devices;
    i2c.checkAddressSpace(0,127,active_devices);
    ROS_INFO("found %ld active devices", active_devices.size());
//    for(auto device:active_devices)
//        printf("%x\t",device);
    cout << endl;
    vector<uint8_t> deviceIDs = {0xE,0xC,0xD};
    vector<uint8_t> motorIDs = {0,1,2};
    A1335 motorAngle(h2p_lw_i2c_addr[0], motorIDs,deviceIDs);


    clock_t begin = clock();
    cout <<"begin: "<< begin << endl;
    ros::Rate rate(5);
    while(ros::ok()){
        vector<A1335State> state;
        motorAngle.readAngleData(state);
        roboy_communication_middleware::MotorAngle msg;
        cout << "debug4"
             << endl;

        for (auto s:state) {


            if(s.address == 12){
                cout << "Motor Angle Sensor on i2C address " << (int) s.address << " is " << (s.isOK ? "ok" : "not ok")
                     << endl;
                cout << "angle:         " << s.angle << endl;

//            str << "angle_flags:   " << motorAngle.decodeFlag(s.angle_flags, ANGLES_FLAGS) << endl;
//            str << "err_flags:     " << motorAngle.decodeFlag(s.err_flags, ERROR_FLAGS) << endl;
//            str << "fieldStrength: " << s.fieldStrength << endl;
//            str << "status_flags:  " << motorAngle.decodeFlag(s.status_flags, STATUS_FLAGS) << endl;
//            str << "xerr_flags:    " << motorAngle.decodeFlag(s.xerr_flags, XERROR_FLAGS) << endl;
                msg.angles.push_back(s.angle);
                msg.magneticFieldStrength.push_back(s.fieldStrength);
            }
        }

        // ROS_INFO_STREAM_THROTTLE(1,str.str());









        uint32_t r = myoControl->readADC(0);
        double R = 4096.0/(r) - 1.0;
        //R = R0*R;
        cout <<"Temperature Raw: "  <<  r << "      Temp: "<< 1.0/ (log(R) / B + 1/298.15 ) - 273.15 << endl;
        msg.id = 5;

        msg.temperature.push_back( 1.0/ (log(R) / B + 1/298.15 ) - 273.15);
        cout << "TEMP SIZE: "<< msg.temperature.size() << endl;
        motorAngle_pub.publish(msg);
    }
    cout <<"End: "<<  clock() << endl;








    //ros::Rate rate(5);
//    vector<uint8_t> deviceIDs = {0xE,0xC,0xD};
//    vector<uint8_t> motorIDs = {0,1,2};
//    A1335 motorAngle(h2p_lw_i2c_addr[0], motorIDs,deviceIDs);

//    while(ros::ok()) {
//        vector<A1335State> state;
//        motorAngle.readAngleData(state);
//        stringstream str;
//        str<<endl;
//
//
//
//        for (auto s:state) {
//
//
//
//
//
//            str << "Motor Angle Sensor on i2C address " << (int) s.address << " is " << (s.isOK ? "ok" : "not ok")
//                << endl;
//            str << "angle:         " << s.angle << endl;
//            str << "angle_flags:   " << motorAngle.decodeFlag(s.angle_flags, ANGLES_FLAGS) << endl;
//            str << "err_flags:     " << motorAngle.decodeFlag(s.err_flags, ERROR_FLAGS) << endl;
//            str << "fieldStrength: " << s.fieldStrength << endl;
//            str << "status_flags:  " << motorAngle.decodeFlag(s.status_flags, STATUS_FLAGS) << endl;
//            str << "xerr_flags:    " << motorAngle.decodeFlag(s.xerr_flags, XERROR_FLAGS) << endl;
////            msg.temperature.push_back(s.temp);
//        }
//        ROS_INFO_STREAM_THROTTLE(1,str.str());
//    }
    //start the main program!

/*
 * Copied from http://wiki.seeedstudio.com/Grove-Temperature_Sensor_V1.2/

    const int B = 4275;               // B value of the thermistor
    const int R0 = 100000;            // R0 = 100k
    const int pinTempSensor = A0;     // Grove - Temperature Sensor connect to A0
    int a = analogRead(pinTempSensor);

    float R = 1023.0/a-1.0;
    R = R0*R;

    float temperature = 1.0/(log(R/R0)/B+1/298.15)-273.15;

 */



    // clean up our memory mapping and exit
    if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
        printf( "ERROR: munmap() failed...\n" );
        close( fd );
        return( 1 );
    }

    close( fd );

    return( 0 );
}