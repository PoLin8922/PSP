#include <stdio.h>
#include <stdlib.h>
#include <std_msgs/Int32.h>
#include <modbus/modbus.h>
#include <errno.h>
#include <unistd.h>
#include "ros/ros.h"
#include "communication/ModbusPLC.h" 

/* PLC */
#define SERVER_ADDRESS "192.168.50.30"  
#define SERVER_PORT 501  
#define SLAVE_ID 1       
#define QUANTITY 64  
#define START_ADDRESS 51 

modbus_t* ctx; 

int Read_val(modbus_t* ctx, uint16_t address) {
    uint16_t read_data[QUANTITY] = { 0 };
    int rc = modbus_read_registers(ctx, address, QUANTITY, read_data);
    if (rc == -1) {
        fprintf(stderr, "modbus_read_registers error: %s\n", modbus_strerror(errno));
        modbus_close(ctx);
        modbus_free(ctx);
        exit(1);
    }

    // printf("Register Address : %d, Value: %d\n", address, read_data[0]);
    return read_data[0];
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "plc_state_publisher");
    ros::NodeHandle nh;

    /* Node publisher*/
    ros::Publisher plc_state_pub = nh.advertise<std_msgs::Int32>("plc_state", 10);

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

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        std_msgs::Int32 msg;
        msg.data = Read_val(ctx, START_ADDRESS);
        plc_state_pub.publish(msg);
        ROS_INFO("Published: %d", msg.data);
        loop_rate.sleep();
    }

    // Close modbus connection and free context
    modbus_close(ctx);
    modbus_free(ctx);

    return 0;
}
