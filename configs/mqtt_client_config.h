/******************************************************************************
 * File Name: mqtt_client_config.h
 *
 * Description: Configuration for connecting an Infineon device to Ubidots
 *              using MQTT protocol.
 *
 ******************************************************************************/
#ifndef MQTT_CLIENT_CONFIG_H_
#define MQTT_CLIENT_CONFIG_H_

#include "cy_mqtt_api.h"

/*******************************************************************************
* Macros
********************************************************************************/
/* MQTT Broker/Server address and port used for the MQTT connection. */
#define MQTT_BROKER_ADDRESS               "mqtt.thingsboard.cloud"
#define MQTT_PORT                         1883

/* Set this macro to 1 if a secure (TLS) connection to the MQTT Broker is
 * required to be established, else 0. Note: Ubidots can work with or without TLS.
 */
#define MQTT_SECURE_CONNECTION            ( 0 )

/* Configure the user credentials to be sent as part of MQTT CONNECT packet */
/* Replace the token below with your Ubidots account token. */
#define MQTT_USERNAME                     ""
#define MQTT_PASSWORD                     ""

/********************* MQTT MESSAGE CONFIGURATION MACROS **********************/
/* The MQTT topic for publishing data. */
#define MQTT_PUB_TOPIC                    "v1/devices/me/telemetry"
#define MQTT_TOPIC_LEN (sizeof(MQTT_PUB_TOPIC) - 1)

/* Set the QoS that is associated with the MQTT publish message.
 * Valid choices are 0, 1, and 2. Ubidots recommends QoS 1 for reliability.
 */
#define MQTT_MESSAGES_QOS                 ( 1 )

/* Configuration for the 'Last Will and Testament (LWT)'. */
#define ENABLE_LWT_MESSAGE                ( 0 )
#if ENABLE_LWT_MESSAGE
    #define MQTT_WILL_MESSAGE             ("MQTT client unexpectedly disconnected!")
#endif

/******************* OTHER MQTT CLIENT CONFIGURATION MACROS *******************/
/* A unique client identifier to be used for every MQTT connection. */
#define MQTT_CLIENT_IDENTIFIER            "8415bc10"

/* The timeout in milliseconds for MQTT operations in this example. */
#define MQTT_TIMEOUT_MS                   ( 5000 )

/* The keep-alive interval in seconds used for MQTT ping request. */
#define MQTT_KEEP_ALIVE_SECONDS           ( 60 )

/* Every active MQTT connection must have a unique client identifier. */
#define GENERATE_UNIQUE_CLIENT_ID         ( 1 )

/* The longest client identifier that an MQTT server must accept. */
#define MQTT_CLIENT_IDENTIFIER_MAX_LEN    ( 23 )

/* A Network buffer is allocated for sending and receiving MQTT packets over
 * the network. Specify the size of this buffer using this macro.
 */
#define MQTT_NETWORK_BUFFER_SIZE          ( 2 * CY_MQTT_MIN_NETWORK_BUFFER_SIZE )

/* Maximum MQTT connection re-connection limit. */
#define MAX_MQTT_CONN_RETRIES            (150u)

/* MQTT re-connection time interval in milliseconds. */
#define MQTT_CONN_RETRY_INTERVAL_MS      (2000)

/**************** MQTT CLIENT CERTIFICATE CONFIGURATION MACROS ****************/

/* Configure the below credentials in case of a secure MQTT connection. */
/* TLS is optional for Ubidots. If not needed, leave these macros empty. */
#define CLIENT_CERTIFICATE      ""
#define CLIENT_PRIVATE_KEY      ""
#define ROOT_CA_CERTIFICATE     ""

/******************************************************************************
* Global Variables
*******************************************************************************/
extern cy_mqtt_broker_info_t broker_info;
extern cy_awsport_ssl_credentials_t  *security_info;
extern cy_mqtt_connect_info_t connection_info;

#endif /* MQTT_CLIENT_CONFIG_H_ */
