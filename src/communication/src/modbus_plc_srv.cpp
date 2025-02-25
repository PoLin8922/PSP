﻿#include <stdio.h>
#include <stdlib.h>
#include <modbus/modbus.h>
#include <errno.h>
#include <unistd.h>
#include "ros/ros.h"
#include "communication/ModbusPLC.h" 
#include <std_msgs/Int32.h>

/* PLC */
#define SERVER_ADDRESS "192.168.50.30"  
#define SERVER_PORT 501  
#define SLAVE_ID 1       
#define QUANTITY 64  
#define START_ADDRESS 51 

modbus_t* ctx; 
bool semaphore = false;

void modnus_restore(){
    // printf("restore modbus\n");

    /* reset modbus tcp */
    modbus_close(ctx);
    modbus_free(ctx);

    ctx = modbus_new_tcp(SERVER_ADDRESS, SERVER_PORT);
    if (ctx == NULL) {
        fprintf(stderr, "modbus_new_tcp error\n");
        exit(1);
    }

    modbus_set_slave(ctx, SLAVE_ID);

    // Connect to manipulator
    int retry_count = 5;
    while (retry_count > 0) {
        if (modbus_connect(ctx) == -1) {
            // fprintf(stderr, "modbus_connect error: %s\n", modbus_strerror(errno));
            retry_count--;
            usleep(1000 * 1000);  // wait 1 second before retrying
        } else {
            break;
        }
    }

    if (retry_count == 0) {
        fprintf(stderr, "modbus_connect failed after multiple attempts\n");
        modbus_free(ctx);
        exit(1);
    }

    // printf("restore success\n");
}

void Set_val(modbus_t* ctx, uint16_t address, uint16_t val) {
    uint16_t output_data[QUANTITY] = { 0 };
    output_data[0] = val;
    int rc = modbus_write_registers(ctx, address, QUANTITY, output_data);
    if (rc == -1) {
        fprintf(stderr, "modbus_write_registers error: %s\n", modbus_strerror(errno));
        modbus_close(ctx);
        modbus_free(ctx);
        exit(1);
    }
}

int Read_val(modbus_t* ctx, uint16_t address) {
    uint16_t read_data[QUANTITY] = { 0 };
    usleep(500);
    int rc = modbus_read_registers(ctx, address, QUANTITY, read_data);
    // std::cout<<"rc = "<<rc<<std::endl;
    if (rc == -1) {
        fprintf(stderr, "modbus_plc modbus_read_registers error: %s\n", modbus_strerror(errno));
        modnus_restore();
        // modbus_close(ctx);
        // modbus_free(ctx);
        // exit(1);
    }

    // printf("Register Address : %d, Value: %d\n", address, read_data[0]);
    return read_data[0];
}

bool modbusControlCallback(communication::ModbusPLC::Request &req,
                          communication::ModbusPLC::Response &res) {
    semaphore = false;
    usleep(500);
    if(req.setnum >= 0){
        Set_val(ctx, START_ADDRESS, req.setnum);
        res.success = true;
    }
    else {
        res.success = false;
    }

    semaphore = true;
    usleep(500);

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "modbus_plc_control_node");
    ros::NodeHandle nh;

    /* set modbus tcp */
    ctx = modbus_new_tcp(SERVER_ADDRESS, SERVER_PORT);
    if (ctx == NULL) {
        fprintf(stderr, "modbus_new_tcp error\n");
        exit(1);
    }

    modbus_set_slave(ctx, SLAVE_ID);

    // Connect to manipulator
    if (modbus_connect(ctx) == -1) {
        fprintf(stderr, "modbus_connect error\n");
        modbus_free(ctx);
        exit(1);
    }

    // Create a ROS service for modbus control
    ros::ServiceServer service = nh.advertiseService("modbus_plc_control", modbusControlCallback);
    ros::Publisher plc_state_pub = nh.advertise<std_msgs::Int32>("plc_state", 10);
    ROS_INFO("Ready to control PLC via Modbus.");

    // Spin ROS node

    semaphore = true;
    usleep(500);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        if(semaphore){
    usleep(500);

            std_msgs::Int32 msg;
            msg.data = Read_val(ctx, START_ADDRESS);
            plc_state_pub.publish(msg);
            // ROS_INFO("Published: %d", msg.data);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Close modbus connection and free context
    modbus_close(ctx);
    modbus_free(ctx);

    return 0;
}
