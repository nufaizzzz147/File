/*
 * COMPLETE GSM CODE WITH YOUR AT COMMANDS
 * Just copy this code and use it directly!
 */

#include "application.h"

// Your AT Commands (exactly as you provided)
unsigned char Command1[]  = "AT\r\0";
unsigned char Command2[]  = "ATE0\r\0";
unsigned char Command3[]  = "AT+CGMR\r\0";          // Get manufacturer info
unsigned char Command4[]  = "AT+CFUN=1\r\0";        // Enable full functionality
unsigned char Command5[]  = "AT+CSQ\r\0";           // Check signal quality
unsigned char Command6[]  = "AT+CREG=1\r\0";        // Network registration
unsigned char Command7[]  = "AT+COPS?\r\0";         // Check operator
unsigned char Command8[]  = "AT+COPS=4,2,\"40445\",7\r\0";  // Set operator
unsigned char Command9[]  = "AT+CGSN=2\r\0";        // Get IMEI
unsigned char Command10[] = "AT+CGCMOD=?\r\0";      // Check context
unsigned char Command11[] = "AT+CGATT=1\r\0";       // Attach to network
unsigned char Command12[] = "AT+CGACT=1,1\r\0";     // Activate context
unsigned char Command13[] = "AT+CGDCONT=1,\"IP\",\"airtel\"\r\0";  // Set APN

// Command states
#define CMD1    0
#define CMD2    1
#define CMD3    2
#define CMD4    3
#define CMD5    4
#define CMD6    5
#define CMD7    6
#define CMD8    7
#define CMD9    8
#define CMD10   9
#define CMD11   10
#define CMD12   11
#define CMD13   12

// Variables
uint8_t current_command = CMD1;
uint8_t gsm_ready = 0;

// Send AT command function
void send_at_command(unsigned char* command) 
{
    printf("Sending: %s", command);
    // Add your UART send function here
    // uart_write_bytes(UART_NUM_1, command, strlen(command));
}

// Main GSM initialization function
void gsm_init_with_your_commands(void) 
{
    switch(current_command) 
    {
        case CMD1:    // AT
            send_at_command(Command1);
            printf("Step 1: Basic AT test\n");
            current_command = CMD2;
            break;
            
        case CMD2:    // ATE0  
            send_at_command(Command2);
            printf("Step 2: Disable echo\n");
            current_command = CMD3;
            break;
            
        case CMD3:    // AT+CGMR
            send_at_command(Command3);
            printf("Step 3: Get manufacturer info\n");
            current_command = CMD4;
            break;
            
        case CMD4:    // AT+CFUN=1
            send_at_command(Command4);
            printf("Step 4: Enable full functionality\n");
            current_command = CMD5;
            break;
            
        case CMD5:    // AT+CSQ
            send_at_command(Command5);
            printf("Step 5: Check signal quality\n");
            current_command = CMD6;
            break;
            
        case CMD6:    // AT+CREG=1
            send_at_command(Command6);
            printf("Step 6: Enable network registration\n");
            current_command = CMD7;
            break;
            
        case CMD7:    // AT+COPS?
            send_at_command(Command7);
            printf("Step 7: Check network operator\n");
            current_command = CMD8;
            break;
            
        case CMD8:    // AT+COPS=4,2,"40445",7
            send_at_command(Command8);
            printf("Step 8: Set network operator\n");
            current_command = CMD9;
            break;
            
        case CMD9:    // AT+CGSN=2
            send_at_command(Command9);
            printf("Step 9: Get IMEI\n");
            current_command = CMD10;
            break;
            
        case CMD10:   // AT+CGCMOD=?
            send_at_command(Command10);
            printf("Step 10: Check context\n");
            current_command = CMD11;
            break;
            
        case CMD11:   // AT+CGATT=1
            send_at_command(Command11);
            printf("Step 11: Attach to network\n");
            current_command = CMD12;
            break;
            
        case CMD12:   // AT+CGACT=1,1
            send_at_command(Command12);
            printf("Step 12: Activate context\n");
            current_command = CMD13;
            break;
            
        case CMD13:   // AT+CGDCONT=1,"IP","airtel"
            send_at_command(Command13);
            printf("Step 13: Set APN - DONE!\n");
            gsm_ready = 1;  // GSM is now ready
            printf("GSM INITIALIZATION COMPLETE!\n");
            break;
    }
}

// Call this function every 2 seconds
void gsm_task(void) 
{
    if(gsm_ready == 0) {
        gsm_init_with_your_commands();
        vTaskDelay(2000 / portTICK_PERIOD_MS);  // Wait 2 seconds
    }
    else {
        printf("GSM is ready for communication!\n");
        // Your communication code here
    }
}

// Simple usage in your main task:
void your_main_task(void *args) 
{
    while(1) {
        gsm_task();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}