#ifndef LIBREALSENSE2_CS_EVENT_CAPTURE_H
#define LIBREALSENSE2_CS_EVENT_CAPTURE_H



#include <list>



namespace librealsense
{



#define GVCP_EVENTDATA_CMD              (0x00C2)
#define EVENTDATA_MAX_PACKET_SIZE       (576)
#define EVENTDATA_MAX_DATA_SIZE         (524)


#define EVENTDATA_ACC_XOS_VALUE_OFFSET  (0)
#define EVENTDATA_ACC_YOS_VALUE_OFFSET  (4)
#define EVENTDATA_ACC_ZOS_VALUE_OFFSET  (8)
#define EVENTDATA_ACC_XOS_VALUE_SIZE    (4)
#define EVENTDATA_ACC_YOS_VALUE_SIZE    (4)
#define EVENTDATA_ACC_ZOS_VALUE_SIZE    (4)

#define EVENTDATA_GYRO_XOS_VALUE_OFFSET  (0)
#define EVENTDATA_GYRO_YOS_VALUE_OFFSET  (4)
#define EVENTDATA_GYRO_ZOS_VALUE_OFFSET  (8)
#define EVENTDATA_GYRO_XOS_VALUE_SIZE    (4)
#define EVENTDATA_GYRO_YOS_VALUE_SIZE    (4)
#define EVENTDATA_GYRO_ZOS_VALUE_SIZE    (4)

#define EVENT_CAPTURE_SOCKET_RECV_TIMOUT_S (5)



#define GEV_MCP_REG_ADDR    (0x0B00)    // Message Channel Port register.
#define GEV_MCDA_REG_ADDR   (0x0B10)    // Message Channel Destination Address register.
#define GEV_MCTT_REG_ADDR   (0x0B14)    // Message Channel Transmission Timeout in ms register.
#define GEV_MCRC_REG_ADDR   (0x0B18)    // Message Channel Retry Count register.
#define GEV_MCSP_REG_ADDR   (0x0B1C)    // Message Channel Source Port register.



/** Event Selector strings defined in camera xml file.
 */
const std::string XML_EVENT_SELECTOR_ACC = "Accelerometer";
const std::string XML_EVENT_SELECTOR_GYRO = "Gyroscope";



/** Event Notification strings defined in camera xml file.
 */
const std::string XML_EVENT_NOTIFICATION_OFF = "Off";
const std::string XML_EVENT_NOTIFICATION_ON = "On";



/** Event Accelerometer datarate strings defined in camera xml file.
 */
const std::string XML_EVENT_ACC_DATARATE_7_81_HZ = "Rate_7_81_Hz";
const std::string XML_EVENT_ACC_DATARATE_15_63_HZ = "Rate_15_63_Hz";
const std::string XML_EVENT_ACC_DATARATE_31_25_HZ = "Rate_31_25_Hz";
const std::string XML_EVENT_ACC_DATARATE_62_50_HZ = "Rate_62_50_Hz";
const std::string XML_EVENT_ACC_DATARATE_125_HZ = "Rate_125_Hz";
const std::string XML_EVENT_ACC_DATARATE_250_HZ = "Rate_250_Hz";
const std::string XML_EVENT_ACC_DATARATE_500_HZ = "Rate_500_Hz";
const std::string XML_EVENT_ACC_DATARATE_1000_HZ = "Rate_1000_Hz";

const std::string XML_EVENT_ACC_DATARATE_12_5_HZ = "Rate_12_5_Hz";
const std::string XML_EVENT_ACC_DATARATE_25_HZ = "Rate_25_Hz";
const std::string XML_EVENT_ACC_DATARATE_50_HZ = "Rate_50_Hz";
const std::string XML_EVENT_ACC_DATARATE_100_HZ = "Rate_100_Hz";
const std::string XML_EVENT_ACC_DATARATE_200_HZ = "Rate_200_Hz";
const std::string XML_EVENT_ACC_DATARATE_400_HZ = "Rate_400_Hz";
const std::string XML_EVENT_ACC_DATARATE_800_HZ = "Rate_800_Hz";
const std::string XML_EVENT_ACC_DATARATE_1600_HZ = "Rate_1600_Hz";



/** Event Gyroscope datarate strings defined in camera xml file.
 */
const std::string XML_EVENT_GYRO_DATARATE_100_HZ = "Rate_100_Hz";
const std::string XML_EVENT_GYRO_DATARATE_200_HZ = "Rate_200_Hz";
const std::string XML_EVENT_GYRO_DATARATE_400_HZ = "Rate_400_Hz";
const std::string XML_EVENT_GYRO_DATARATE_1000_HZ = "Rate_1000_Hz";
const std::string XML_EVENT_GYRO_DATARATE_2000_HZ = "Rate_2000_Hz";



/** From ds5-motion.h
 */
const uint32_t EVENT_ACC_DATARATE_62_50_TO_RS = 63;
const uint32_t EVENT_ACC_DATARATE_250_TO_RS = 250;
const uint32_t EVENT_ACC_DATARATE_100_TO_RS = 100;
const uint32_t EVENT_ACC_DATARATE_200_TO_RS = 200;
const uint32_t EVENT_GYRO_DATARATE_200_TO_RS = 200;
const uint32_t EVENT_GYRO_DATARATE_400_TO_RS = 400;



const std::string EVENT_NAME_ACC = "Accl";
const std::string EVENT_NAME_GYRO = "Gyro";



struct gvcp_header
{
    uint8_t	key_field;
    uint8_t	flag;
    uint16_t command;
    uint16_t length;
    uint16_t req_id;
};



struct eventdata_cmd_message
{
    gvcp_header header_gvcp;
    uint16_t reserved;
    uint16_t event_identifier;
    uint16_t event_stream_ch_id;
    uint16_t event_block_id;
    uint32_t event_timestamp_high;
    uint32_t event_timestamp_low;
    uint8_t event_data[EVENTDATA_MAX_DATA_SIZE];
};



struct cfg_event_stream
{
    cfg_event_stream()
    {
        clear();
    }

    void clear()
    {
        event_selector = "";
        event_data_rate = "";
        is_valid = false;
    }

    std::string event_selector;
    std::string event_data_rate;
    bool is_valid;
};

typedef std::list<cfg_event_stream> cfg_event_stream_list;
typedef std::list<cfg_event_stream>::iterator cfg_event_stream_list_it;
typedef std::list<cfg_event_stream>::const_iterator cfg_event_stream_list_const_it;



struct event_data
{
    event_data()
    {
        clear();
    }

    void clear()
    {
        device_serial = "";
        event_name = "";
        event_id = 0;
        event_hw_timestamp = 0;
        xOs = 0.0f;
        yOs = 0.0f;
        zOs = 0.0f;
        is_valid = false;
    }

    std::string device_serial;
    std::string event_name;
    uint32_t event_id;
    uint64_t event_hw_timestamp;
    float xOs;
    float yOs;
    float zOs;
    bool is_valid;
};

typedef std::list<event_data> event_data_list;
typedef std::list<event_data>::iterator event_data_list_it;
typedef std::list<event_data>::const_iterator event_data_list_const_it;



} // ! namespace librealsense



#endif //!LIBREALSENSE2_CS_EVENT_CAPTURE_H
