#include <stdio.h>
#include <stdlib.h>
#include <modbus/modbus.h>
#include <errno.h>
#include <unistd.h>

/* FUNUC ROBOT */
#define SERVER_ADDRESS "192.168.255.200"  

/* ROBOGUIDE */
//#define SERVER_ADDRESS "127.0.0.1" 
// #define SERVER_ADDRESS "192.168.255.200" 
#define SERVER_PORT 502 
#define SLAVE_ID 1       
#define QUANTITY 4
#define START_ADDRESS 0

// /* PLC */
// #define SERVER_ADDRESS "192.168.50.30"  
// #define SERVER_PORT 501  
// #define SLAVE_ID 1       
// #define QUANTITY 64  
// #define START_ADDRESS 51
   
bool execute = false;


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

void Read_val(modbus_t* ctx, uint16_t address) {
    uint16_t read_data[QUANTITY] = { 0 };
    int rc = modbus_read_registers(ctx, address, QUANTITY, read_data);
    if (rc == -1) {
        fprintf(stderr, "modbus.cpp modbus_read_registers error: %s\n", modbus_strerror(errno));
        modbus_close(ctx);
        modbus_free(ctx);
        exit(1);
    }

    printf("Register Address : %d, Value: %d\n", address, read_data[0]);
}

int main() {
    /* set modbus tcp */
    modbus_t* ctx = modbus_new_tcp(SERVER_ADDRESS, SERVER_PORT);
    if (ctx == NULL) {
        fprintf(stderr, "modbus_new_tcp error\n");
        exit(1);
    }

    modbus_set_slave(ctx, SLAVE_ID);

    /* connect manipulator */
    if (modbus_connect(ctx) == -1) {
        fprintf(stderr, "modbus_connect error\n");
        modbus_free(ctx);
        exit(1);
    } 

    /* Simple test*/
    Set_val(ctx, START_ADDRESS, 0);
    usleep(500);
    Read_val(ctx, START_ADDRESS);

    modbus_close(ctx);
    modbus_free(ctx); 

    return 0;
}


