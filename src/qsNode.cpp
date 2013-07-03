/**
 *
 *  File: qsNode.cc
 *  Desc: ROS QuickSilver motor driver
 *
 *  Copyright (c) 2013, UDM Advanced Mobile Robotics Lab 
 *  All Rights Reserved
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 *
 */

// ROS
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "tf/tf.h"
#include "LinearMath/btMatrix3x3.h"

// use thread for ros::spin() 
#include <boost/thread.hpp>

// For port control?
#include <termios.h>
#include <fcntl.h>
#include <aio.h>

// Standard libraries
#include <math.h>
#include <string>

// QuickSilver libraries/specifics
#include "silverlode_commands.h"
#include "silverlode_registers.h"
//#define READ_TIMEOUT 100000
#define READ_TIMEOUT 10000

// Assuming NORMALIZE is usually defined in playercore.h
// Normalize angle to domain -pi, pi
#ifndef NORMALIZE
    #define NORMALIZE(z) atan2(sin(z), cos(z))
#endif

// use thread for ros::spin() 
void spinThread();

/*
 * Drivetrain class def'n
 */


        void qsDrivetrain(ros::NodeHandle n);
        int Setup();
        int Shutdown();

        /* ROS-specific members and functions */
        ros::Publisher odom_pub;
        ros::Publisher stall_pub;
        ros::Subscriber twist_sub;
        void twistCallback(const geometry_msgs::Twist&);

        /* This stuff used to be private, moved out so it's accessible from main() */
        bool stall;
        /* Just used by software watchdog current */
        /* Both in milliseconds */
        double trans_vel_last_sent, rot_vel_last_sent;
        int watchdog_hardware_time, watchdog_software_time;
        int64_t last_command_time;
				double cmd_UpdateRate;
        int PetWatchDog();
        int64_t GetTime();

        int SetSpeed(double trans_vel, double yaw_rad_per_sec);
        int UpdateOdom(double pyaw = NAN);
        int UpdateStall();


    #ifdef DEBUGLOG   
        int fd_debuglog;
    #endif

         /* Store local heading here*/
        double pa=0;

        /* wheelcircumference is calculated (once) from wheeldiam */
        double wheelbase, wheeldiam, wheelcircumference;

        /* File descriptors for both serial ports */
        int fd_left, fd_right;
        /* Motor IDs */
        int qs_left_id, qs_right_id;
        /* Should be the same for both motors. Encoder tics per rev */
        int tics_per_rev;
        int motor_gearbox_ratio;
        /* Max RPM matters for scaling */
        int motor_max_rpm;
        double motor_rpm_scale;

        /* Baud rate, note this is the BXXX define, not the actual number */
        int serial_port_baud;
        /* Path to serial ports for both devices */
        //const char *serial_port_left, *serial_port_right;
        
        //utayba
        char serial_port_left[100], serial_port_right[100];
        
        /* tristate mode for single serial port */
        /* This mode is not recommended (Latency!) */
        int serial_single_port;

        /* Safety maximums, both in meters per second */
        double max_trans_vel, max_bias_vel,min_trans_vel;
        double  max_turn_rate; //  max turn rate , rad/s
        /* Player-specific? If this is to be converted, could get broadcast as another topic or
        pushed into the parameter server
        IntProperty LeftTics;
        IntProperty RightTics;
        */

        /* For encoder calculations */
        int32_t ltics_last, rtics_last;
        double tics_time_last;

        int OpenTerm(const char *);
        int OpenTerms();
        int ReadLine(int, char *, ssize_t, int);
        struct aiocb * AioWriteBuf(int , char *, int);

        /* Get and publish PX and PY from encoders */
        void RefreshData();

        /*
        Not used in current config?
        void ProcessCompass(player_position3d_data_t &data);
        void ShutdownCompass();
        int SetupCompass();
        */

        int GetTics(int32_t &, int32_t &, double &);

        double TicsToMeters(int tics, double wc, double tpr);
        int32_t MPSToSVU(double, double);
        /* SilverLode Velocity Units. See the SilverLode user manual */

       int qsQuery(char *, int ,  char *, char *, int, char *, int);
        inline int CheckResponseCode(int, char *);
        double limits(double Max,double Min, double data);

        /* Drivetrain state */
        nav_msgs::Odometry odom_msg;


/*
 * Drivetrain class constructor
 */
void qsDrivetrain(ros::NodeHandle n) {
    std::string serial_port_left_str, serial_port_right_str;
    // Parameters?
    n.getParam("wheelbase", wheelbase);
    n.param<double>("wheeldiam", wheeldiam, 0.5);
    //n.getParam("sl",sl);
    //n.getParam("sw",sw);
    n.param<int>("qs_left_id",  qs_left_id,  0);
    n.param<int>("qs_right_id", qs_right_id, 1);
    
//    printf("[Get parameters] qs_left_id: %d qs_right_id:%d\n",qs_left_id , qs_right_id);

    n.getParam("port_left", serial_port_left_str);
    //serial_port_left = serial_port_left_str.c_str();
    //utayba
    strcpy(serial_port_left,serial_port_left_str.c_str());
   
//    printf("[Get parameters]left serial port string %s\n",serial_port_left);
    n.getParam("port_right", serial_port_right_str);
    //utayba 
    //serial_port_right = serial_port_right_str.c_str();
    strcpy(serial_port_right,serial_port_right_str.c_str());

//    printf("[Get parameters]right serial port string %s\n",serial_port_right);

    n.param<int>("port_baud", serial_port_baud, 57600);
    n.param<int>("motor_max_rpm", motor_max_rpm, 4000);
    n.param<double>("max_trans_vel", max_trans_vel,  1.0);
    n.param<double>("min_trans_vel", min_trans_vel, -0.5);
    n.param<double>("max_bias_vel", max_bias_vel, max_trans_vel*2);
    n.param<double>("max_turn_rate", max_turn_rate, 1.57);  // set default to 90 degree /sec
    n.param<int>("watchdog_hardware_time", watchdog_hardware_time, 300);
    n.param<int>("watchdog_software_time", watchdog_software_time, 2000);
    n.param<int>("tics_per_rev", tics_per_rev, 16000);
    n.param<int>("motor_gearbox_ratio", motor_gearbox_ratio, 10);
	n.param<double>("UpdateRate"  ,cmd_UpdateRate         , 10            );

    motor_rpm_scale = 4000 / motor_max_rpm;

    // Serial port baud to standard format
    switch (serial_port_baud) {
        case 9600:
            serial_port_baud = B9600;
            break;
        case 38400:
            serial_port_baud = B38400;
            break;
        case 57600:
            serial_port_baud = B57600;
            break;
        case 115200:
            serial_port_baud = B115200;
            break;
        case 230400:
            serial_port_baud = B230400;
            break;
        default:
            ROS_INFO("Unknown baud rate [%d] defaulting to B57600\n",
                   serial_port_baud);
            serial_port_baud = B57600;
            break;
    }

    wheelcircumference  = M_PI*wheeldiam;
    serial_single_port = 0;
/* the checking is not working , I just disable it
    if (serial_port_right == 0)
        serial_single_port = 1;
    if (serial_port_right == 0)
        printf("[ set it to one serial port]\n");
*/
    return;
}


/*
 * Driver setup
 */ 
int Setup() {
    ROS_INFO("qsDrivetrain driver initializing...");

    // initlize the odom heading to 0
    odom_msg.pose.pose.orientation.x=0;
    odom_msg.pose.pose.orientation.y=0;
    odom_msg.pose.pose.orientation.z=0;
    odom_msg.pose.pose.orientation.w=1;
    pa=0;


    // Here you do whatever is necessary to setup the device, like open and
    // configure a serial port.
    if ( OpenTerms() < 0) {
        ROS_ERROR("failed to initialize qsDrivetrain");
        return(-1);
    }

#ifdef DEBUGLOG
    fd_debuglog = open("debuglog.txt", O_WRONLY | O_APPEND);
#endif

    SetSpeed(0, 0);
    GetTics(ltics_last, rtics_last, tics_time_last);
    /* 
    Assumed not in use in this config
    if (use_compass) {
        if (!SetupCompass()) {
            PLAYER_ERROR("failed to subscribe to compass");
            return -1;
        }
    }
    */

    ROS_INFO("qsDrivetrain driver ready");

    // Start the device thread; spawns a new thread and executes
    // Main(), which contains the main loop for the driver.
    // StartThread();
    /* No longer here, pulled into main node loop */

    return(0);
}

/*
 * Open all terminals
 */
int OpenTerms() {
    //printf("left serial port string %s\n",serial_port_left);
    if (serial_single_port) {
    printf("[Mode]one serial port mode %s\n",serial_port_left);
        fd_left = OpenTerm(serial_port_left);
       // serial_port_left_str.c_str()
        fd_right = fd_left;
    } else {
    printf("[Mode] two serial port %s ; %s\n",serial_port_left,serial_port_right);
        fd_left = OpenTerm(serial_port_left);
        fd_right = OpenTerm(serial_port_right);
    }
    printf("[Mode] serial port status :%i ; %i\n",fd_left,fd_right );

    if (fd_left == -1 || fd_right == -1)
        return -1;
    else
        return 0;
}

/*
 * Open one terminal
 */
int OpenTerm(const char *serial_port) {
    struct termios term;
    int fd;
    if ((fd = open(serial_port, O_RDWR | O_SYNC, S_IRUSR | S_IWUSR )) < 0 ) {
        printf("error tag for serial port is %d\n", fd);
        ROS_ERROR("open() of %s failed: %s", serial_port, strerror(errno));
        return(-1);
    }

    if (tcgetattr(fd, &term) < 0 ) {
        ROS_ERROR("tcgetattr() failed: %s", strerror(errno));
        close(fd);
        return(-1);
    }


    cfmakeraw(&term);
    // serial_port_baud = B115200; // for debug
    cfsetispeed(&term, serial_port_baud);
    cfsetospeed(&term, serial_port_baud);
    term.c_cflag |= CSTOPB;

    if (tcsetattr(fd, TCSAFLUSH, &term) < 0 ) {
        ROS_ERROR("tcsetattr() failed: %s", strerror(errno));
        close(fd);
        return(-1);
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

/*
 * Write command to output? (I think)
 */
struct aiocb *AioWriteBuf(int fd, char *buf, int len) {
    struct aiocb *aiocbp;
    aiocbp = (struct aiocb *) malloc(sizeof(struct aiocb));

    memset(aiocbp, 0, sizeof(aiocb));
    aiocbp->aio_nbytes = len;
    aiocbp->aio_offset = 0;
    aiocbp->aio_buf = buf;
    aiocbp->aio_fildes = fd;

#ifdef DEBUGLOG
    write(fd_debuglog, buf, len);
#endif

    if (-1 == aio_write(aiocbp)) {
        ROS_ERROR("error writing to device [%s].", strerror(errno));
        usleep(500000); /* Wait half a second, then try to reopen motors */

        if ( OpenTerms() < 0) {
            ROS_ERROR("failed to reinitialize qsDrivetrain");
            return NULL;
        }
    }

    return aiocbp;

}

/* 
 * Read data
 * Mostly stolen from the SICKLMS200 driver.
 * The code isn't /great/, but it's what I want rather than blocking
 */
int ReadLine(int fd, char *buff, ssize_t maxlen, int timeout) {
    int len = 0;
    int bytes = 0;
    // If the timeout is infinite,
    // go to blocking io
    //

    // tcflush(fd_left, TCIOFLUSH); /* Maybe i need this ? */
    if (timeout < 0) {
        //PLAYER_MSG0(2, "using blocking io");
        int flags = ::fcntl(fd, F_GETFL);
        if (flags < 0) {
            ROS_ERROR("unable to get device flags");
            return 0;
        }
        if (::fcntl(fd, F_SETFL, flags & (~O_NONBLOCK)) < 0) {
            ROS_ERROR("unable to set device flags");
            return 0;
        }
    }
    //
    // Otherwise, use non-blocking io
    //
    else {
        //ROS_MSG0(2, "using non-blocking io");
        int flags = ::fcntl(fd, F_GETFL);
        if (flags < 0) {
            ROS_ERROR("unable to get device flags");
            return 0;
        }
        if (::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
            ROS_ERROR("unable to set device flags");
            return 0;
        }
    }

    int64_t start_time = GetTime();
    int64_t stop_time = start_time + timeout;

    //PLAYER_MSG2(2, "%Ld %Ld", start_time, stop_time);

    // Read until we get a valid header
    // or we timeout
    //
    while (true) {
        if (timeout >= 0)
            usleep(1000);

        //printf("reading %d\n", timeout); fflush(stdout);
        bytes = ::read(fd, buff + len, 1);
        if (bytes != -1) {
            len = len + bytes;
            if (buff[len - 1] == '\r' || buff[len - 1] == '\n' || len == maxlen)
            {
                buff[len - 1] = '\0';
                break;
            }
        }

        if (timeout >= 0 && GetTime() >= stop_time) {
            //PLAYER_MSG2(2, "%Ld %Ld", GetTime(), stop_time);
            ROS_WARN("timeout on read (1)");
            return 0;
        }
    }
    
#ifdef DEBUGLOG
    write(fd_debuglog, buff, len);
#endif
    return len;
}

inline int32_t MPSToSVU(double mps, double wc) {
    /* mps/wc = RPS */
    /* From SilverLode User Guide: 2^31 SVU/4000 RPM * 60RPM/RPS
           = 32,212,254.705 SVU/RPS */
#define POWER_2_31 2147483648.0 /* (double) 2 ^ 31 */

    double svu;
    svu = (POWER_2_31 * mps) / ( motor_max_rpm * wc ) *
           60.0 * motor_gearbox_ratio;
    //printf("[MPSTOSVU]  svu %f mps %f wc %f\n",svu, mps, wc);
    return (int32_t) rint(svu);
}

inline double TicsToMeters(int tics, double wc, double tpr)
/* Encoder Tics, Wheel Circumference, Tics Per Rev */
{
    double meters;
    meters = (tics * wc)/tpr;
    return meters;
}

/*
 * Gets "tic" data from motor driver?
 */
int GetTics(int32_t &tics_left, int32_t &tics_right, double &tics_time) {
    char cmd_left[25], cmd_right[25];
    char buff_left[25], buff_right[25];
    int len_left, len_right;
    int status;


    tcflush(fd_left, TCIOFLUSH);
    tcflush(fd_right, TCIOFLUSH);
    
    len_left = snprintf(cmd_left, 25, "@%d %d %d\r", 
                        qs_left_id, RRG, QS_REG_ENCODER);
    len_right = snprintf(cmd_right, 25, "@%d %d %d\r", 
                         qs_right_id, RRG, QS_REG_ENCODER);

    //printf("[GetTicks]Command out %s ; %s\n",cmd_left,cmd_right);
    
    tics_time = GetTime();
    status = qsQuery(cmd_left, len_left, buff_left,
                     cmd_right, len_right, buff_right, 25);
    if (status == 0) { /* No errors */
        uint16_t tics_low;
        uint16_t tics_high;

        /* Line should read as: # 01 000C   0000      0001 */
        //                        buf com  hiByte    LowByte
        /* All numbers are hex:
           # (Ack)
           01 (ID Number of motor)
           000C (Register that was read)
           0000
           0001 (The value from the register)
        */

        /* The 10 comes from strlen("# 01 000C ") */
        
        
       
        /*sscanf(buff_left + 10, "%4x %4x",
               (unsigned int *) &tics_high, (unsigned int *) &tics_low);*/
               
                sscanf(buff_left + 10, "%4x %4x",
               (uint16_t *) &tics_high, (uint16_t *) &tics_low);
                //printf("[GetTicks]  left high tics %x; right low tics %x\n", tics_high, tics_low);
       // tics_left = (((uint32_t) tics_high) << 16) + ((uint32_t) tics_low);
       // uint32_t tics_left_unsigned;
        // = (((uint32_t) tics_high) << 16) + ((uint32_t) tics_low);
        char tempBf[9];
        memcpy(tempBf, buff_left+10,4);
        memcpy(tempBf+4, buff_left+15,4);
        sscanf(tempBf, "%8x", &tics_left);
        
        //memcpy(&tics_left + 16, &tics_low, 16);
        //printf("[GetTics] tics_left signed %x\n", tics_left);
        /*sscanf(buff_right + 10, "%4x %4x",
               (unsigned int *) &tics_high, (unsigned int *) &tics_low);*/
        sscanf(buff_right + 10, "%4x %4x",
               (uint16_t *) &tics_high, (uint16_t *) &tics_low);       
        //printf("[GetTicks]  right high tics %x; right low tics %x\n", tics_high, tics_low);       
       // tics_right = (((uint32_t) tics_high) << 16) + ((uint32_t) tics_low);
       // uint32_t tics_right_unsigned;
        //memcpy(&tics_right, buff_right+10,4);
        //memcpy(&tics_right + 16, &tics_low, 16);
       // char tempBf[9];
        memcpy(tempBf, buff_right+10,4);
        memcpy(tempBf+4, buff_right+15,4);
        sscanf(tempBf, "%8x", &tics_right);
        
        
        
    //printf("[GetTicks] resopnse buffer %s ; %s\n",buff_left,buff_right);
    //printf("[GetTicks] left tics %d; right tics %d\n", tics_left, tics_right);
   
    //printf("[GetTicks] Signed hex: left tics %x; right tics %x\n", tics_left, tics_right);
  //  printf("[GetTicks] unsigned hex: left tics %x; right tics %x\n", tics_left_unsigned, tics_right_unsigned);

    } else {
        /* We got an error getting encoder data. This is bad. */
        ROS_WARN("qsDrivetrain: Error getting encoder data.");
        tics_left = 0;
        tics_right = 0;
        tics_time = 0;
    }
    return status;

}


/* 
 * Function allowing querying of two motors at virtually the same time assuming
 * two serial ports are used. If only a single port is used, 
 * the code still works, just with added latency
 */
int qsQuery(char *cmd_left, int cmd_len_left, char * output_left,
                      char *cmd_right, int cmd_len_right, char * output_right,
                      int  maxlen)
{
    int output_len_left, output_len_right;
    struct aiocb *aiocbps[2];
    int8_t left_status = 0;
    int8_t right_status = 0;
    /* Basic idea is if we have two serial ports:
       Async write command 1
       Async write command 2
       Wait for 1 to finish and read response.
       Wait for 2 to finish and read response.

       This way things like speed changes and encoder reads happen at
       Virtually the same time, rather than the left always responding quicker.
       
       However for tristate mode no such sequence is possible so we do:
       Async write command 1
       Wait for 1 to finish and read response.
       Async write command 2
       Wait for 2 to finish and read response.
       Async isn't needed for this case, but the code is built for two port mode,
       With single port mode only supported because at the time of writing I don't
       have two port hardware ready.
    */


    if (cmd_len_left > 0) {
        aiocbps[0] = AioWriteBuf(fd_left, cmd_left, cmd_len_left);
        if (serial_single_port) {
            /* Single port mode. Read left respones before writing right command */
            aio_suspend(aiocbps, 1, 0); /* Wait for write to finish */
            aio_return(aiocbps[0]);
            output_len_left = ReadLine(fd_left, output_left, maxlen, READ_TIMEOUT);
            /* Then attempt to read */
        }
    }

    if (cmd_len_right > 0) {
        /* Now write to the Right motor */
        aiocbps[1] = AioWriteBuf(fd_right, cmd_right, cmd_len_right);
        /* And wait for both to finish */
    }

    if (! serial_single_port && cmd_len_left > 0) {
        /* Haven't read from left yet, and we sent something to left */
        aio_suspend(aiocbps, 1, 0); /* Wait for write to finish */
        aio_return(aiocbps[0]);
        output_len_left = ReadLine(fd_left, output_left, maxlen, READ_TIMEOUT);
        /* Then attempt to read */
    }

    if (cmd_len_right > 0) {
        /* Sent something to right, so read it */
        aio_suspend(aiocbps+1, 1, 0); /* Wait for write to finish */
        aio_return(aiocbps[1]);
        output_len_right = ReadLine(fd_right, output_right, maxlen, READ_TIMEOUT);
        /* Then attempt to read */
    }

    /* Error checking */
    left_status = CheckResponseCode(output_len_left, output_left);
    right_status = CheckResponseCode(output_len_right, output_right);

    return left_status + (right_status << 4);

}

inline int CheckResponseCode(int len, char *buff) {
    int status = 0;
    if (len > 0) { /* Got a response */
        switch ( buff[0] ) {
            case '#':
            case '*':
                status = 0;
                /* Good ! */
                break;
            case '!':
            default:
                status = 1;
                /* Crap */
                break;
        }
    } else {
        status = 1;
    }
    return status;
}

/* 
 * SetSpeed( Linear Meters Per Second, Rotational Radians Per Second )
 * Returns 1 on success, 0 otherwise
 */
int SetSpeed(double trans_vel, double yaw_rad_per_sec) {
    char cmd_left[25], cmd_right[25];
    char buff_left[25], buff_right[25];
    int len_left, len_right;
    //printf("[setSpeed]  trans_vel %f, yaw_rad_per_sec %f\n",trans_vel, yaw_rad_per_sec);
    double vel_left, vel_right;

    double bias = yaw_rad_per_sec * wheelbase;
    /* Sanity check on data. Make sure we're not too fast */
    if (trans_vel > max_trans_vel)
        trans_vel = max_trans_vel;
    else if (trans_vel < -max_trans_vel)
        trans_vel = -max_trans_vel;

    if (bias > max_bias_vel)
        bias = max_bias_vel;
    else if (bias < -max_bias_vel)
        bias = -max_bias_vel;

    /* After sanity check, calculate the individual motor velocities */
    vel_left = trans_vel - bias/2;
    vel_right = trans_vel + bias/2;
    //printf("[setSpeed]  left velocity %f, right velocity %f\n",vel_left, vel_right);
    /* Convert into "counts" to send to the motor */
    int32_t svu_left, svu_right;
    /* The motors don't just use tics per second, they have a scale factor too,
       That's why this is called Counts not tics */
    svu_left = MPSToSVU(vel_left, wheelcircumference);
    svu_right = MPSToSVU(vel_right, wheelcircumference);


    /* Generate the command and send it */
    len_left = snprintf(cmd_left, 25, "@%d %d %d %d\r",
                        qs_left_id, WRI,
                        QS_REG_PC_MODE_VELOCITY, svu_left);

    len_right = snprintf(cmd_right, 25, "@%d %d %d %d\r",
                         qs_right_id, WRI,
                         QS_REG_PC_MODE_VELOCITY, svu_right);
/*
    printf("[Set Speed] ID  %x %x \n",  qs_left_id , qs_right_id );
    printf("[Set Speed] SVU %d %d \n",  svu_left   , svu_right );
    printf("[Set Speed] @%d %d %d %d\n",  qs_left_id , WRI, QS_REG_PC_MODE_VELOCITY, svu_left);
    printf("[Set Speed] @%d %d %d %d\n",  qs_right_id, WRI, QS_REG_PC_MODE_VELOCITY, svu_right);
    printf("[Set Speed] Send L: %s\n [Set Speed] Send R: %s\n", cmd_left, cmd_right);
*/
    int status = qsQuery(cmd_left, len_left, buff_left,
                         cmd_right, len_right, buff_right, 25);

    stall = false;
    //printf("[Set Speed] Got Status: %d and Buffer:L [%s] and R [%s]\n", status, buff_left, buff_right);
    return status;
}

int PetWatchDog() {
    char cmd_left[25], cmd_right[25];
    char buff_left[25], buff_right[25];
    int len_left, len_right;

    //printf("[Watchdog] Pet the dog\t");
    len_left  = snprintf(cmd_left, 25, "@%d %d %d %d\r", qs_left_id, WRI,
                         QS_REG_WATCHDOG, 0);
    len_right = snprintf(cmd_right, 25, "@%d %d %d %d\r", qs_right_id, WRI,
                         QS_REG_WATCHDOG, 0);

    return  qsQuery(cmd_left, len_left, buff_left,
                    cmd_right, len_right, buff_right,
                    25);
}

/* Returns current time in miliseconds */
int64_t GetTime() {
    ros::Time now = ros::Time::now();
    return (int64_t) now.sec * 1000 + (int64_t) now.nsec / 1000000;
}

/*
 * Driver shutdown
 */
int Shutdown() {
    ROS_INFO("Shutting qsDrivetrain driver down");
    //SetSpeed(0, 0); /* Count on the watchdog to stop it. */
    /* 
    Assumed not in use in this config
    if(use_compass)
        ShutdownCompass();
    */ 

    // Here you would shut the device down by, for example, closing a
    // serial port.

    ROS_INFO("qsDrivetrain driver has been shutdown");

    return(0);
}


/*
 * Callback for incoming velocity messages
 */
void twistCallback(const geometry_msgs::Twist& msg) {

   // TODO check max speed , max turn rate here
   //  limits(max_turn_rate, -max_turn_rate,msg.angular.z);
    /* Sanity check on data. Make sure we're not too fast */
   double Va,Vx;

   Vx=limits(max_trans_vel, min_trans_vel, msg.linear.x  );
   Va=limits(max_turn_rate,-max_turn_rate, msg.angular.z );
/*
   if (turn_rate_in > max_turn_rate)
        turn_rate_in = max_turn_rate;
    else if (turn_rate_in < -max_turn_rate)
        turn_rate_in = -max_turn_rate;
*/

    SetSpeed(Vx,Va);
    last_command_time=GetTime();
    trans_vel_last_sent = Vx;
    rot_vel_last_sent   = Va;
    //printf("[TwistCallback] got Vx:%f Va:%f\n",Vx, Va);
    return;
}

double limits(double Max,double Min, double data){
    if 		(data > Max)        return Max;
    else if (data < Min)        return Min;
    else    					return data;
	}


/* 
 * Produces odometry output
 * Used to update posdata.px, posdata.py, posdata.pa and posdata.vx, posdata.va
 */
int UpdateOdom(double pyaw) {
    int32_t ltics,rtics;
    int ltics_delta, rtics_delta;
    double Lmeters, Rmeters;
    double tics_time_delta, tics_time;
    double trans_delta, angle_delta;

    //double pa;  // Temporary value in rad (to be converted to quats)

   /* Get the current tics from the encoder */
   if (GetTics(ltics, rtics, tics_time)) {
       /* GetTics failed */
       /* Wait and try again */
       usleep(5000);
       if (GetTics(ltics, rtics, tics_time)) {
           /* Failed again. We're screwed. */
           return -1;
       }
   }

///ROS_WARN("[UpdateOdom][remove the comment in UpdateOdom after debugging!]\n");



    /*
    Player-specific? If this is to be converted, could get broadcast as another topic or
    pushed into the parameter server
    LeftTics.SetValue( (int) ltics);
    RightTics.SetValue( (int) rtics);
    */

    /* Find the delta and save the current tics (for next cycle) */
    tics_time_delta = tics_time - tics_time_last;
    ltics_delta = ltics - ltics_last;
    rtics_delta = rtics - rtics_last;
/*
    printf("[UpdateOdom] ltics %d litcs_last %d Ltics_delta %d\n]", ltics, ltics_last, ltics_delta);
    printf("[UpdateOdom] rtics %d ritcs_last %d rtics_delta %d\n]", rtics, rtics_last, rtics_delta);
*/
    tics_time_last = tics_time;
    ltics_last = ltics;
    rtics_last = rtics;

    /* Convert to meters */
    
    Lmeters = TicsToMeters(ltics_delta, wheelcircumference,
                           tics_per_rev * motor_gearbox_ratio);
    Rmeters = TicsToMeters(rtics_delta, wheelcircumference,
                           tics_per_rev * motor_gearbox_ratio);
/*
    printf("[UpdateOdom] LeftMeters: %f ,RightMeters: %f Meters\n",Lmeters,Rmeters);
*/
/// Debug Odom start 
///    Lmeters = 0.1;
///		Rmeters = 0.2;
///    printf("[UpdateOdom] Fake Data L: %f ,R: %f Meters\n",Lmeters,Rmeters);
		
///ROS_WARN("[UpdateOdom][remove the Debug Odom in UpdateOdom after debugging!]\n");
/// Debug Odom end



///  No idea why the heading is store in odom_msg , so I use global variable pa to store heading - Steven
///    /* Convert old orientation to radians */
///    tf::Quaternion quat;
///    tf::quaternionMsgToTF(odom_msg.pose.pose.orientation, quat);
///    double temp; // Don't care about roll and pitch
///    tf::Matrix3x3(quat).getRPY(temp,temp,pa);

// got it from my ROSJAUS code
///  tf::Pose pose;
///  tf::poseMsgToTF(odom_msg.pose.pose, pose);
///	pa = tf::getYaw(pose.getRotation());

/*
    printf("[UpdateOdom] Output odom_msg xyzw: x: %f ,y: %f , z: %f ,w: %f ,pa: %f\n",
    odom_msg.pose.pose.orientation.x,
    odom_msg.pose.pose.orientation.y,
    odom_msg.pose.pose.orientation.z,
    odom_msg.pose.pose.orientation.w, pa);
*/



    /* Convert to vehicle values, not wheel values */
    trans_delta = (Lmeters + Rmeters)/2;
    /* Translational distance (in x/forward direction) traveled this cycle */
    if(isnan(pyaw))
        angle_delta = (Rmeters - Lmeters)/wheelbase;
    else
        angle_delta = NORMALIZE(pyaw - pa);
/*
    printf("[UpdateOdom] trans_delta: %f ,angle_delta: %f \n",trans_delta,angle_delta);
        
    printf("[UpdateOdom] pyaw: %f ,pa: %f \n",pyaw,pa);
*/        
        
    /* Angular change (in yaw) this cycle */

    /* Compute Velocities */
    odom_msg.twist.twist.linear.x = trans_delta / (tics_time_delta/1000); // convert ms to sec
    /* Linear velocity */

    odom_msg.twist.twist.angular.z = angle_delta / (tics_time_delta/1000); 
    /* Angular velocity */

    /* Add to our current pose */
    odom_msg.pose.pose.position.x += trans_delta * cos(pa + angle_delta/2);
    odom_msg.pose.pose.position.y += trans_delta * sin(pa + angle_delta/2);
/*
    printf("[UpdateOdom] Vx: %f ,Va: %f , x:%f y:%f\n",
    odom_msg.twist.twist.linear.x,
    odom_msg.twist.twist.angular.z,
    odom_msg.pose.pose.position.x,
    odom_msg.pose.pose.position.y);
*/
    if(!isnan(pyaw))
        pa = pyaw;
    else
        pa = NORMALIZE(pa + angle_delta);
        
//    printf("[UpdateOdom] pa: %f ,angle_delta: %f \n",pa,angle_delta);

    // Convert orientation back to quaternion
    tf::Quaternion quat;
    quat.setRPY(0, 0, NORMALIZE(pa)); //  Steven fix this during IGVC2013, normalize first, make it -pi~pi ,
    
/*    printf("[UpdateOdom]quat.setRPY xyzw: x: %f ,y: %f , z: %f ,w: %f\n",
    quat[0],quat[1],quat[2],quat[3]);
*/
    tf::quaternionTFToMsg(quat, odom_msg.pose.pose.orientation);


/*
    double temp1,temp2,temp3; //  roll and pitch yaw
    tf::Matrix3x3(quat).getRPY(temp1,temp2,temp3);
    printf("[UpdateOdom] R: %f ,P: %f , Y%f\n",temp1,temp2,temp3);
    printf("[UpdateOdom] Output odom_msg xyzw: x: %f ,y: %f , z: %f ,w: %f\n",
    odom_msg.pose.pose.orientation.x,
    odom_msg.pose.pose.orientation.y,
    odom_msg.pose.pose.orientation.z,
    odom_msg.pose.pose.orientation.w);
*/

 
    /* Constant covariances (not properly calculated)
     * Note: Check Husky dead-reckoning code for a few simple cases
     * which should be taken care of separately (ex. both motors stopped, same speed)
     */
    boost::array<double,36> covar = {{1e-3, 0, 0, 0, 0, 0,
                                      0, 1e-3, 0, 0, 0, 0,
                                      0, 0, 1e-3, 0, 0, 0,
                                      0, 0, 0, 1e-6, 0, 0,
                                      0, 0, 0, 0, 1e-6, 0,
                                      0, 0, 0, 0, 0, 1e-6}};
    odom_msg.pose.covariance = covar;
    odom_msg.twist.covariance = covar;

    /* Update header */
    odom_msg.header.frame_id = "odom";
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.child_frame_id = "base_link";
    /* Publish! */
    odom_pub.publish(odom_msg);
    return 0;
}

/*
 * Publishes stall data
 */
int UpdateStall() {
    std_msgs::Bool msg;
    msg.data = stall;
    stall_pub.publish(msg);
    return 0;
}

/*
 * Node entry point
 */
int main (int argc, char **argv) {

    // Basic setup - publishing, subscribing, etc
    ros::init(argc, argv, "qsNode");
    ros::NodeHandle n=ros::NodeHandle("~");
    //ros::NodeHandle node_handle_private = ros::NodeHandle("~");
    // Create drivetrain object
    qsDrivetrain(n);
   // Set up driver
    Setup();

    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
    stall_pub = n.advertise<std_msgs::Bool>("stall", 10);
    twist_sub = n.subscribe("cmd_vel", 1, &twistCallback);    

    // Main loop
    int64_t time_last_watchdog = GetTime();
    int64_t time_cur;
    // Run spin() in different thread
    //boost::thread spin_thread = boost::thread(boost::bind(&spinThread));

	ros::Rate loop_rate(cmd_UpdateRate);

    while (ros::ok()) {
        ros::spinOnce();
        time_cur = GetTime();
        /* Hardware Watchdog */
        /* This is to pet the hardware watchdog */
        if (watchdog_hardware_time != 0) {
            if (time_cur - time_last_watchdog >= watchdog_hardware_time)
                /* Obviously this time should be set to less than the actual
                   hardware time */
            {
                PetWatchDog();
                time_last_watchdog = time_cur;
            }
        }
        
        
        //printf("[main]Software command watch dog\n");

        /* Software watchdog */
        /* This is our own watchdog for a bad client program */
/*
         printf("[Watchdog] current time: %ld, last command time: %ld  ,diff: %ld , watchdog time %ld",
                          time_cur,last_command_time,time_cur-last_command_time,watchdog_software_time);
*/
       if (watchdog_software_time != 0) {
            if ( (trans_vel_last_sent != 0 || rot_vel_last_sent != 0) &&
                    /* We sent a speed */
                 (time_cur - last_command_time >= watchdog_software_time) )
                    /* A long time ago */
            { /* It's been too long since our last velocity command.
                 Phil probably pulled the ethernet cable again.
                 Stop the motors before we hit someone */
        // printf("[Watchdog] current time: %ld last command time: %ld",time_cur,time_last_watchdog);
         
               ROS_WARN("qsDrivetrain Software Watchdog: %dms. Halting.",
                             watchdog_software_time);
                SetSpeed(0, 0);
                trans_vel_last_sent = 0;
                rot_vel_last_sent = 0;

                stall = true;
            }
        }


        // Check the encoders, do the math, and publish result
        /*
        Assumed compass not in use in this config
        if(use_compass)
            usleep(20000);
        else
            RefreshData();
        */
        //printf("[main]Update Odom & Stall\n");
        UpdateOdom();
        UpdateStall();

        // Sleep (you might, for example, block on a read() instead)
       // usleep(10000);
       
		// must put some delay in for useful odom resolution
    		loop_rate.sleep();
    }

    // Shut down driver
    Shutdown();
    // stop the thread for ros::spin()
		//spin_thread.join();

    return 0;
}

// Use different Thread to spin(); (get message) forever
void spinThread()
{
    ros::spin();
}


