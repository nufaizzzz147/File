#include <stdio.h>
#include <stdint.h>

// Forward declaration of the gsm_engine function
void gsm_engine(void);

// External variables
extern uint8_t gsm_device_state;
extern uint32_t gsm_reset_count;

int main() {
    printf("GSM Engine Test Program\n");
    printf("=======================\n");
    
    printf("Initial GSM device state: %d (GSM_RESTART)\n", gsm_device_state);
    printf("Initial reset count: %d\n\n", gsm_reset_count);
    
    printf("Running GSM engine cycles to demonstrate state transitions:\n");
    
    // Run the GSM engine for several cycles to demonstrate the state machine
    for(int i = 0; i < 300; i++) {
        gsm_engine();
        
        // Print state changes
        static uint8_t last_state = 255;
        if(gsm_device_state != last_state) {
            const char* state_names[] = {
                "GSM_ON", "GSM_RST", "GSM_INIT", "GSM_DATA_TXRX", 
                "GSM_GET_TIME", "GSM_FWVR_UP", "GSM_RESTART", "GSM_IDLE",
                "GSM_TCP_COMM", "GSM_GPS", "GSM_TX_SMS", "GSM_RX_SMS"
            };
            
            printf("Cycle %d: State changed to %s (%d), reset_count=%d\n", 
                   i, (gsm_device_state < 12) ? state_names[gsm_device_state] : "UNKNOWN", 
                   gsm_device_state, gsm_reset_count);
            last_state = gsm_device_state;
        }
        
        // Show progress for GSM_ON state counting
        if(gsm_device_state == 0 && gsm_reset_count > 0 && gsm_reset_count % 50 == 0) {
            printf("  GSM_ON state: reset_count=%d (waiting for 150)\n", gsm_reset_count);
        }
        
        // Break if we've completed both sequences
        if(gsm_device_state == 2 && gsm_reset_count == 0) { // GSM_INIT state reached
            printf("GSM engine has transitioned to GSM_INIT state after GSM_ON sequence\n");
            break;
        }
    }
    
    printf("\nTest completed!\n");
    return 0;
}