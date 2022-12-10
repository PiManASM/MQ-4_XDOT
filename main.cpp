#include "dot_util.h"
#include "RadioEvent.h"

#if ACTIVE_EXAMPLE == OTA_EXAMPLE

/////////////////////////////////////////////////////////////////////////////
// -------------------- DOT LIBRARY REQUIRED ------------------------------//
// * Because these example programs can be used for both mDot and xDot     //
//     devices, the LoRa stack is not included. The libmDot library should //
//     be imported if building for mDot devices. The libxDot library       //
//     should be imported if building for xDot devices.                    //
// * https://developer.mbed.org/teams/MultiTech/code/libmDot-dev-mbed5/    //
// * https://developer.mbed.org/teams/MultiTech/code/libmDot-mbed5/        //
// * https://developer.mbed.org/teams/MultiTech/code/libxDot-dev-mbed5/    //
// * https://developer.mbed.org/teams/MultiTech/code/libxDot-mbed5/        //
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////
// * these options must match the settings on your gateway //
// * edit their values to match your configuration         //
// * frequency sub band is only relevant for the 915 bands //
// * either the network name and passphrase can be used or //
//     the network ID (8 bytes) and KEY (16 bytes)         //
/////////////////////////////////////////////////////////////

#define TIME_BETWEEN_MEASUREMENTS 500s

static std::string network_name = "MTCDT-20314488";
static std::string network_passphrase = "password";

static uint8_t network_id[] = { 0x6C, 0x4E, 0xEF, 0x66, 0xF4, 0x79, 0x86, 0xA6 };
static uint8_t network_key[] = { 0x1F, 0x33, 0xA1, 0x70, 0xA5, 0xF1, 0xFD, 0xA0, 0xAB, 0x69, 0x7A, 0xAE, 0x2B, 0x95, 0x91, 0x6B };
static uint8_t frequency_sub_band = 7;
static lora::NetworkType network_type = lora::PUBLIC_LORAWAN;
static uint8_t join_delay = 5;
static uint8_t ack = 0;
static bool adr = true;

#define BETA 3975

// deepsleep consumes slightly less current than sleep
// in sleep mode, IO state is maintained, RAM is retained, and application will resume after waking up
// in deepsleep mode, IOs float, RAM is lost, and application will start from beginning after waking up
// if deep_sleep == true, device will enter deepsleep mode
static bool deep_sleep = false;

mDot* dot = NULL;
lora::ChannelPlan* plan = NULL;

mbed::UnbufferedSerial pc(USBTX, USBRX);


AnalogIn sensor(PB_15);
DigitalOut sensor_power(GPIO3);
 
double Thermistor(int RawADC) { 
    double Temp;
    Temp = (RawADC * 5)/1024;
    Temp = (Temp);
    //Temp= (float) 10000.0 * ((65536.0 / RawADC) - 1.0);
    //Temp = (1/((log(Temp/10000.0)/BETA) + (1.0/298.15)));
    //Temp = Temp - 273.15; // Convert Kelvin to Celcius
    //Temp = (Temp * 9.0)/ 5.0 + 32.0; // Convert C to F
    return Temp;
}

int main() {
    // Custom event handler for automatically displaying RX data
    RadioEvent events;
    DigitalOut led1(LED1);
    pc.baud(115200);

    mts::MTSLog::setLogLevel(mts::MTSLog::TRACE_LEVEL);

    // Create channel plan
    plan = create_channel_plan();
    assert(plan);

    dot = mDot::getInstance(plan);
    assert(dot);

    // attach the custom events handler
    dot->setEvents(&events);

    if (!dot->getStandbyFlag() && !dot->getPreserveSession()) {
        logInfo("mbed-os library version: %d.%d.%d", MBED_MAJOR_VERSION, MBED_MINOR_VERSION, MBED_PATCH_VERSION);

        // start from a well-known state
        logInfo("defaulting Dot configuration");
        dot->resetConfig();
        dot->resetNetworkSession();

        // make sure library logging is turned on
        dot->setLogLevel(mts::MTSLog::INFO_LEVEL);

        // update configuration if necessary
        if (dot->getJoinMode() != mDot::OTA) {
            logInfo("changing network join mode to OTA");
            if (dot->setJoinMode(mDot::OTA) != mDot::MDOT_OK) {
                logError("failed to set network join mode to OTA");
            }
        }

        // To preserve session over power-off or reset enable this flag
        // dot->setPreserveSession(true);

        // in OTA and AUTO_OTA join modes, the credentials can be passed to the library as a name and passphrase or an ID and KEY
        // only one method or the other should be used!
        // network ID = crc64(network name)
        // network KEY = cmac(network passphrase)
        update_ota_config_name_phrase(network_name, network_passphrase, frequency_sub_band, network_type, ack);
        //update_ota_config_id_key(network_id, network_key, frequency_sub_band, network_type, ack);

        // configure network link checks
        // network link checks are a good alternative to requiring the gateway to ACK every packet and should allow a single gateway to handle more Dots
        // check the link every count packets
        // declare the Dot disconnected after threshold failed link checks
        // for count = 3 and threshold = 5, the Dot will ask for a link check response every 5 packets and will consider the connection lost if it fails to receive 3 responses in a row
        update_network_link_check_config(3, 5);

        // enable or disable Adaptive Data Rate
        dot->setAdr(adr);

        // Configure the join delay
        dot->setJoinDelay(join_delay);

        // save changes to configuration
        logInfo("saving configuration");
        if (!dot->saveConfig()) {
            logError("failed to save configuration");
        }

        // display configuration
        display_config();
    } else {
        // restore the saved session if the dot woke from deepsleep mode
        // useful to use with deepsleep because session info is otherwise lost when the dot enters deepsleep
        logInfo("restoring network session from NVM");
        dot->restoreNetworkSession();
    }


    char data[5];

    while (true) {
        
        sensor_power = true;
        led1 = true;

        ThisThread::sleep_for(15s); //wait for sensor to warm up

        unsigned int val = sensor.read_u16();
        double temp = Thermistor(val);
        printf("%f\n",temp);
        std::vector<uint8_t> tx_data;

        // join network if not joined
        if (!dot->getNetworkJoinStatus()) {
            join_network();
        }
        
        snprintf(data, 5, "%f", temp);
        for(int i=0; i<sizeof(data)-1; i++){
            tx_data.push_back(data[i]);
        }
        

        send_data(tx_data);
        led1 = false;
        sensor_power = false;
        ThisThread::sleep_for(TIME_BETWEEN_MEASUREMENTS); // sleep until next measurement 


    }

    return 0;
}

#endif
