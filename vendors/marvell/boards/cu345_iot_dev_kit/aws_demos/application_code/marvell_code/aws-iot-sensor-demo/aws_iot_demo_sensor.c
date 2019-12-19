/*
 * Amazon FreeRTOS V201910.00
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */

/**
 * @file aws_iot_demo_sensor.c
 * @brief Demonstrates usage of the Thing Shadow library.
 *
 * This program demonstrates the using Shadow documents to toggle a state called
 * "powerOn" in a remote device.
 */

/* The config header is always included first. */
#include "iot_config.h"

/* Standard includes. */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Set up logging for this demo. */
#include "iot_demo_logging.h"

/* Platform layer includes. */
#include "platform/iot_clock.h"
#include "platform/iot_threads.h"

/* MQTT include. */
#include "iot_mqtt.h"

/* Shadow include. */
#include "aws_iot_shadow.h"

/* JSON utilities include. */
#include "iot_json_utils.h"

#include "aws_utils.h"
#include "queue.h"
#include <wmstdio.h>
#include <wm_os.h>

#ifdef SENSOR_HUMIDITY_TEMPERATURE_SI7021
#include "si7021.h"
#endif /* SENSOR_HUMIDITY_TEMPERATURE_SI7021 */

#ifdef SENSOR_AMBIENT_LIGHT_APDS9301
#include "apds9301.h"
#endif /* SENSOR_AMBIENT_LIGHT_APDS9301 */

#ifdef SENSOR_DIGITAL_PRESSURE_BMP180
#include "bmp180.h"
#endif /* SENSOR_DIGITAL_PRESSURE_BMP180 */

/**
 * @brief The keep-alive interval used for this demo.
 *
 * An MQTT ping request will be sent periodically at this interval.
 */
#define KEEP_ALIVE_SECONDS    ( 60 )

/**
 * @brief The timeout for Shadow and MQTT operations in this demo.
 */
#define TIMEOUT_MS            ( 5000 )

/**
 * @brief The Last Will and Testament topic name in this demo.
 *
 * The MQTT server will publish a message to this topic name if this client is
 * unexpectedly disconnected.
 */
#define WILL_TOPIC_NAME         "iot-sensor-demo/%s/update"

/**
 * @brief The message to publish to #WILL_TOPIC_NAME.
 */
#define WILL_MESSAGE            \
    "{"                         \
    "\"state\":{"               \
    "\"reported\":{"            \
    "\"connected\":false"       \
    "}},"                       \
    "\"clientToken\":\"123456\"" \
    "}"

/**
 * @brief The length of #WILL_MESSAGE.
 */
#define WILL_MESSAGE_LENGTH     ((size_t)(sizeof(WILL_MESSAGE) - 1))

#if defined(SENSOR_HUMIDITY_TEMPERATURE_SI7021)
    #define SENSOR_NAME           "si7021"
#elif defined(SENSOR_AMBIENT_LIGHT_APDS9301)
    #define SENSOR_NAME           "apds9301"
#elif defined(SENSOR_DIGITAL_PRESSURE_BMP180)
    #define SENSOR_NAME           "bmp180"
#else
    #define SENSOR_NAME           "unknown"
#endif

#define IDENTIFIER_LENGTH_LIMIT     23

static uint32_t uPollingInterval    = 0;
static uint32_t uDesiredInterval    = democonfigDEMO_POLLING_INTERVAL;

static const char SensorIdentifier[IDENTIFIER_LENGTH_LIMIT + 1];

/**
 * @brief Format string representing a Shadow document with a "desired" state.
 *
 * Note the client token, which is required for all Shadow updates. The client
 * token must be unique at any given time, but may be reused once the update is
 * completed. For this demo, a timestamp is used for a client token.
 */
#define SHADOW_DESIRED_INTERVAL_JSON    \
    "{"                                 \
    "\"state\":{"                       \
    "\"desired\":{"                     \
    "\"interval\":%d"                   \
    "}},"                               \
    "\"clientToken\":\"%06lu\""         \
    "}"

/**
 * @brief The expected size of #SHADOW_DESIRED_INTERVAL_JSON.
 *
 * Because all the format specifiers in #SHADOW_DESIRED_INTERVAL_JSON include a length,
 * its full size is known at compile-time.
 */
#define EXPECTED_DESIRED_INTERVAL_JSON_SIZE    ( sizeof( SHADOW_DESIRED_INTERVAL_JSON ) + 6 )

/**
 * @brief Format string representing a Shadow document with a "reported" state.
 *
 * Note the client token, which is required for all Shadow updates. The client
 * token must be unique at any given time, but may be reused once the update is
 * completed. For this demo, a timestamp is used for a client token.
 */
#if defined(SENSOR_HUMIDITY_TEMPERATURE_SI7021)

#define SHADOW_REPORTED_JSON    \
    "{"                         \
    "\"state\":{"               \
    "\"reported\":{"            \
    "\"humidity\":%d,"          \
    "\"temperature\":%d,"       \
    "\"connected\":true"        \
    "}},"                       \
    "\"clientToken\":\"%06lu\"" \
    "}"

#elif defined(SENSOR_AMBIENT_LIGHT_APDS9301)

#define SHADOW_REPORTED_JSON    \
    "{"                         \
    "\"state\":{"               \
    "\"reported\":{"            \
    "\"lux\":%d,"                \
    "\"connected\":true"        \
    "}},"                       \
    "\"clientToken\":\"%06lu\"" \
    "}"

#elif defined(SENSOR_DIGITAL_PRESSURE_BMP180)

#define SHADOW_REPORTED_JSON    \
    "{"                         \
    "\"state\":{"               \
    "\"reported\":{"            \
    "\"pressure\":%d,"          \
    "\"temperature\":%d,"       \
    "\"connected\":true"        \
    "}},"                       \
    "\"clientToken\":\"%06lu\"" \
    "}"

#else

#define SHADOW_REPORTED_JSON    \
    "{"                         \
    "\"state\":{"               \
    "\"reported\":{"            \
    "\"xxx\":%d,"               \
    "\"connected\":true"        \
    "}},"                       \
    "\"clientToken\":\"%06lu\"" \
    "}"
#endif

/**
 * @brief The expected size of #SHADOW_REPORTED_JSON.
 *
 * Because all the format specifiers in #SHADOW_REPORTED_JSON include a length,
 * its full size is known at compile-time.
 */
#define EXPECTED_REPORTED_JSON_SIZE (sizeof(SHADOW_REPORTED_JSON) + 10)

/**
 * @brief Format string representing a Shadow document with a "desired" state.
 *
 * Note the client token, which is required for all Shadow updates. The client
 * token must be unique at any given time, but may be reused once the update is
 * completed. For this demo, a timestamp is used for a client token.
 */
#define SHADOW_REPORTED_INTERVAL_JSON    \
    "{"                                 \
    "\"state\":{"                       \
    "\"reported\":{"                     \
    "\"interval\":%d"                   \
    "},\"desired\": null},"             \
    "\"clientToken\":\"%06lu\""         \
    "}"

/**
 * @brief The expected size of #SHADOW_DESIRED_JSON.
 *
 * Because all the format specifiers in #SHADOW_DESIRED_JSON include a length,
 * its full size is known at compile-time.
 */
#define EXPECTED_REPORTED_INTERVAL_JSON_SIZE    ( sizeof( SHADOW_REPORTED_INTERVAL_JSON ) + 6 )

/*-----------------------------------------------------------*/

/* Declaration of demo function. */
int RunSensorDemo( bool awsIotMqttMode,
                   const char * pIdentifier,
                   void * pNetworkServerInfo,
                   void * pNetworkCredentialInfo,
                   const IotNetworkInterface_t * pNetworkInterface );

static int _updateShadowInterval(IotMqttConnection_t mqttConnection,
                                 const char * pThingName,
                                 uint32_t interval);

static bool _getShadowDocumentKeyValue(const char * pJsonDocument,
                                       const char * pType,
                                       const char * pKey,
                                       const char ** pValue,
                                       size_t * pValueLength);
static bool _getDeltaDocumentKeyValue(const char * pJsonDocument,
                                      const char * pKey,
                                      const char ** pValue,
                                      size_t * pValueLength);
/*-----------------------------------------------------------*/
/**
 * @brief Shadow delta callback, invoked when the desired and updates Shadow
 * states differ.
 *
 * This function simulates a device updating its state in response to a Shadow.
 *
 * @param[in] pCallbackContext Not used.
 * @param[in] pCallbackParam The received Shadow delta document.
 */
static void _shadowDeltaCallback(void * pCallbackContext,
                                 AwsIotShadowCallbackParam_t * pCallbackParam )
{
    bool found;
    int status;
    const char * pInterval;
    size_t uIntervalLength = 0;
    uint32_t uInterval = 0;
    IotSemaphore_t * pDeltaSemaphore = pCallbackContext;

    /* parese configuration */
    found = _getDeltaDocumentKeyValue(pCallbackParam->u.callback.pDocument,
                                      "interval",
                                      (const char **)&pInterval,
                                      &uIntervalLength);
    if (found) {
        uInterval = strtol(pInterval, NULL, 10);
        if (uInterval != uPollingInterval) {
            status = _updateShadowInterval(pCallbackParam->mqttConnection,
                                           pCallbackParam->pThingName,
                                           uInterval);
            if (status == EXIT_SUCCESS) {
                uPollingInterval = uInterval;
                uDesiredInterval = uInterval;
            }
        }
    }
    IotSemaphore_Post(pDeltaSemaphore);
}

/*-----------------------------------------------------------*/

/**
 * @brief Initialize the the MQTT library and the Shadow library.
 *
 * @return `EXIT_SUCCESS` if all libraries were successfully initialized;
 * `EXIT_FAILURE` otherwise.
 */
static int _initializeDemo( void )
{
    int status = EXIT_SUCCESS;
    IotMqttError_t mqttInitStatus = IOT_MQTT_SUCCESS;
    AwsIotShadowError_t shadowInitStatus = AWS_IOT_SHADOW_SUCCESS;
    /* Flags to track cleanup on error. */
    bool mqttInitialized = false;

    /* Initialize the MQTT library. */
    mqttInitStatus = IotMqtt_Init();

    if (mqttInitStatus == IOT_MQTT_SUCCESS) {
        mqttInitialized = true;
    } else {
        status = EXIT_FAILURE;
    }

    /* Initialize the Shadow library. */
    if (status == EXIT_SUCCESS) {
        /* Use the default MQTT timeout. */
        shadowInitStatus = AwsIotShadow_Init(0);

        if (shadowInitStatus != AWS_IOT_SHADOW_SUCCESS) {
            status = EXIT_FAILURE;
        }
    }

    /* Clean up on error. */
    if (status == EXIT_FAILURE) {
        if (mqttInitialized) {
            IotMqtt_Cleanup();
        }
    }
    return status;
}

/*-----------------------------------------------------------*/

/**
 * @brief Clean up the the MQTT library and the Shadow library.
 */
static void _cleanupDemo( void )
{
    AwsIotShadow_Cleanup();
    IotMqtt_Cleanup();
}

/*-----------------------------------------------------------*/

/**
 * @brief Establish a new connection to the MQTT server for the Shadow demo.
 *
 * @param[in] pIdentifier NULL-terminated MQTT client identifier. The Shadow
 * demo will use the Thing Name as the client identifier.
 * @param[in] pNetworkServerInfo Passed to the MQTT connect function when
 * establishing the MQTT connection.
 * @param[in] pNetworkCredentialInfo Passed to the MQTT connect function when
 * establishing the MQTT connection.
 * @param[in] pNetworkInterface The network interface to use for this demo.
 * @param[out] pMqttConnection Set to the handle to the new MQTT connection.
 *
 * @return `EXIT_SUCCESS` if the connection is successfully established; `EXIT_FAILURE`
 * otherwise.
 */
static int _establishMqttConnection( const char * pIdentifier,
                                     void * pNetworkServerInfo,
                                     void * pNetworkCredentialInfo,
                                     const IotNetworkInterface_t * pNetworkInterface,
                                     IotMqttConnection_t * pMqttConnection )
{
    int status = EXIT_SUCCESS;
    IotMqttError_t connectStatus = IOT_MQTT_STATUS_PENDING;
    IotMqttNetworkInfo_t networkInfo = IOT_MQTT_NETWORK_INFO_INITIALIZER;
    IotMqttConnectInfo_t connectInfo = IOT_MQTT_CONNECT_INFO_INITIALIZER;
    IotMqttPublishInfo_t willInfo = IOT_MQTT_PUBLISH_INFO_INITIALIZER;
    static char willTopicName[64] = { 0 };

    if (pIdentifier == NULL) {
        IotLogError( "Shadow Thing Name must be provided.\r\n" );
        status = EXIT_FAILURE;
    }

    if (status == EXIT_SUCCESS) {
        /* Set the members of the network info not set by the initializer. This
         * struct provided information on the transport layer to the MQTT connection. */
        networkInfo.createNetworkConnection = true;
        networkInfo.u.setup.pNetworkServerInfo = pNetworkServerInfo;
        networkInfo.u.setup.pNetworkCredentialInfo = pNetworkCredentialInfo;
        networkInfo.pNetworkInterface = pNetworkInterface;

        #if ( IOT_MQTT_ENABLE_SERIALIZER_OVERRIDES == 1 ) && defined( IOT_DEMO_MQTT_SERIALIZER )
            networkInfo.pMqttSerializer = IOT_DEMO_MQTT_SERIALIZER;
        #endif

        /* Set the members of the connection info not set by the initializer. */
        connectInfo.awsIotMqttMode = true;
        connectInfo.cleanSession = true;
        connectInfo.keepAliveSeconds = KEEP_ALIVE_SECONDS;
        connectInfo.pWillInfo = &willInfo;

        /* Set the members of the Last Will and Testament (LWT) message info. The
         * MQTT server will publish the LWT message if this client disconnects
         * unexpectedly. */
        snprintf(willTopicName, 64, WILL_TOPIC_NAME, pIdentifier);
        willInfo.pTopicName = (const char *) willTopicName;
        willInfo.topicNameLength = strlen(willTopicName);
        willInfo.pPayload = WILL_MESSAGE;
        willInfo.payloadLength = WILL_MESSAGE_LENGTH;

        /* AWS IoT recommends the use of the Thing Name as the MQTT client ID. */
        connectInfo.pClientIdentifier = pIdentifier;
        connectInfo.clientIdentifierLength = ( uint16_t ) strlen( pIdentifier );

        IotLogInfo( "Shadow Thing Name is %.*s (length %hu).\r\n",
                    connectInfo.clientIdentifierLength,
                    connectInfo.pClientIdentifier,
                    connectInfo.clientIdentifierLength );

        /* Establish the MQTT connection. */
        connectStatus = IotMqtt_Connect( &networkInfo,
                                         &connectInfo,
                                         TIMEOUT_MS,
                                         pMqttConnection );

        if (connectStatus != IOT_MQTT_SUCCESS) {
            IotLogError( "MQTT CONNECT returned error %s.\r\n",
                         IotMqtt_strerror( connectStatus ) );
            status = EXIT_FAILURE;
        }
    }

    return status;
}

/*-----------------------------------------------------------*/
/**
 * @brief Set the Shadow callback functions used in this demo.
 *
 * @param[in] pDeltaSemaphore Used to synchronize Shadow updates with the delta
 * callback.
 * @param[in] mqttConnection The MQTT connection used for Shadows.
 * @param[in] pThingName The Thing Name for Shadows in this demo.
 *
 * @return `EXIT_SUCCESS` if all Shadow callbacks were set; `EXIT_FAILURE`
 * otherwise.
 */
static int _setShadowCallbacks(IotSemaphore_t * pDeltaSemaphore,
                               IotMqttConnection_t mqttConnection,
                               const char * pThingName)
{
    int status = EXIT_SUCCESS;
    AwsIotShadowError_t callbackStatus = AWS_IOT_SHADOW_STATUS_PENDING;
    AwsIotShadowCallbackInfo_t deltaCallback =
                               AWS_IOT_SHADOW_CALLBACK_INFO_INITIALIZER;

    /* Set the functions for callbacks. */
    deltaCallback.pCallbackContext = pDeltaSemaphore;
    deltaCallback.function = _shadowDeltaCallback;

    /* Set the delta callback, which notifies of different desired and
     * reported Shadow states. */
    callbackStatus = AwsIotShadow_SetDeltaCallback( mqttConnection,
                                                    pThingName,
                                                    strlen(pThingName),
                                                    0,
                                                    &deltaCallback );

    if (callbackStatus != AWS_IOT_SHADOW_SUCCESS) {
        IotLogError( "Failed to set demo shadow callback, error %s.",
                     AwsIotShadow_strerror( callbackStatus ) );
        status = EXIT_FAILURE;
    }

    return status;
}

/*-----------------------------------------------------------*/
static int _getShadowDocument(IotMqttConnection_t mqttConnection,
                              const char * pThingName,
                              const char ** pShadowDocument,
                              size_t * pShadowDocumentLength)
{
    int status = EXIT_SUCCESS;
    AwsIotShadowDocumentInfo_t documentInfo =
                            AWS_IOT_SHADOW_DOCUMENT_INFO_INITIALIZER;

    /* Set the common members of the Shadow get document info. */
    documentInfo.pThingName = pThingName;
    documentInfo.thingNameLength = strlen(pThingName);
    /* Set the members of the Shadow document info for GET. */
    documentInfo.u.get.mallocDocument = IotDemo_Malloc;

    /* Retrieve the Shadow document. */
    status = AwsIotShadow_TimedGet(mqttConnection,
                                   &documentInfo,
                                   0,
                                   TIMEOUT_MS,
                                   pShadowDocument,
                                   pShadowDocumentLength);
    return status;
}
/*-----------------------------------------------------------*/

/**
 * @brief Send the Shadow updates that will update the interval value
 *
 * @param[in] pDeltaSemaphore Used to synchronize Shadow updates with the delta
 * callback.
 * @param[in] mqttConnection The MQTT connection used for Shadows.
 * @param[in] pThingName The Thing Name for Shadows in this demo.
 * @param[in] interval The sensor polling interval value.
 *
 * @return `EXIT_SUCCESS` if all Shadow updates were sent; `EXIT_FAILURE`
 * otherwise.
 */
static int _requireUpdateShadowInterval(IotSemaphore_t * pDeltaSemaphore,
                                        IotMqttConnection_t mqttConnection,
                                        const char * pThingName,
                                        uint32_t interval)
{
    int status = EXIT_SUCCESS, len = 0;
    AwsIotShadowError_t updateStatus = AWS_IOT_SHADOW_STATUS_PENDING;
    AwsIotShadowDocumentInfo_t updateDocument =
                            AWS_IOT_SHADOW_DOCUMENT_INFO_INITIALIZER;

    /* A buffer containing the update document. It has static duration to prevent
     * it from being placed on the call stack. */
    static char pUpdateDocument[ EXPECTED_DESIRED_INTERVAL_JSON_SIZE + 1 ] = { 0 };

    /* Set the common members of the Shadow update document info. */
    updateDocument.pThingName = pThingName;
    updateDocument.thingNameLength = strlen(pThingName);
    updateDocument.u.update.pUpdateDocument = pUpdateDocument;
    updateDocument.u.update.updateDocumentLength =
                                    EXPECTED_DESIRED_INTERVAL_JSON_SIZE;

    len = snprintf(pUpdateDocument,
                   EXPECTED_DESIRED_INTERVAL_JSON_SIZE + 1,
                   SHADOW_DESIRED_INTERVAL_JSON,
                   (int) interval,
                   (uint32_t) (IotClock_GetTimeMs() % 1000000));
    updateDocument.u.update.updateDocumentLength = len;

    updateStatus = AwsIotShadow_TimedUpdate(mqttConnection,
                                            &updateDocument,
                                            AWS_IOT_SHADOW_FLAG_KEEP_SUBSCRIPTIONS,
                                            TIMEOUT_MS);

    if (updateStatus != AWS_IOT_SHADOW_SUCCESS) {
        status = EXIT_FAILURE;
        wm_printf("Failed to update interval value, err %s\r\n",
                  AwsIotShadow_strerror(updateStatus));
    } else {
        if (!IotSemaphore_TimedWait(pDeltaSemaphore, TIMEOUT_MS)) {
            status = EXIT_FAILURE;
        }
    }
    return status;
}

static int _updateShadowInterval(IotMqttConnection_t mqttConnection,
                                 const char * pThingName,
                                 uint32_t interval)
{
    int status = EXIT_SUCCESS, len = 0;
    AwsIotShadowError_t updateStatus = AWS_IOT_SHADOW_STATUS_PENDING;
    AwsIotShadowDocumentInfo_t updateDocument =
                            AWS_IOT_SHADOW_DOCUMENT_INFO_INITIALIZER;

    /* A buffer containing the update document. It has static duration to prevent
     * it from being placed on the call stack. */
    static char pUpdateDocument[ EXPECTED_REPORTED_INTERVAL_JSON_SIZE + 1 ] = { 0 };

    /* Set the common members of the Shadow update document info. */
    updateDocument.pThingName = pThingName;
    updateDocument.thingNameLength = strlen(pThingName);
    updateDocument.u.update.pUpdateDocument = pUpdateDocument;
    updateDocument.u.update.updateDocumentLength =
                                    EXPECTED_REPORTED_INTERVAL_JSON_SIZE;

    len = snprintf(pUpdateDocument,
                   EXPECTED_REPORTED_INTERVAL_JSON_SIZE + 1,
                   SHADOW_REPORTED_INTERVAL_JSON,
                   (int) interval,
                   (uint32_t) (IotClock_GetTimeMs() % 1000000));
    updateDocument.u.update.updateDocumentLength = len;

    updateStatus = AwsIotShadow_TimedUpdate(mqttConnection,
                                            &updateDocument,
                                            AWS_IOT_SHADOW_FLAG_KEEP_SUBSCRIPTIONS,
                                            TIMEOUT_MS);

    if (updateStatus != AWS_IOT_SHADOW_SUCCESS) {
        status = EXIT_FAILURE;
    }
    return status;
}

/*-----------------------------------------------------------*/
static bool _getDeltaDocumentKeyValue(const char * pJsonDocument,
                                      const char * pKey,
                                      const char ** pValue,
                                      size_t * pValueLength)
{
    const char * pSection;
    size_t SectionLength = 0;
    bool found = false;

    found = IotJsonUtils_FindJsonValue(pJsonDocument,
                                       strlen(pJsonDocument),
                                       "state",
                                       5,
                                       &pSection,
                                       &SectionLength);
    if (found) {
        found = IotJsonUtils_FindJsonValue(pSection,
                                           SectionLength,
                                           pKey,
                                           strlen(pKey),
                                           pValue,
                                           pValueLength);
    }
    return found;
}

static bool _getShadowDocumentKeyValue(const char * pJsonDocument,
                                       const char * pType,
                                       const char * pKey,
                                       const char ** pValue,
                                       size_t * pValueLength )
{
    const char * pState;
    const char * pDocumentType;
    size_t uStateLength = 0;
    size_t uDocumentTypeLength = 0;
    bool found = false;

    /* get state part */
    found = IotJsonUtils_FindJsonValue(pJsonDocument,
                                       strlen(pJsonDocument),
                                       "state",
                                       5,
                                       &pState,
                                       &uStateLength);
    if (found) {
        /* get type part: reported | desired */
        found = IotJsonUtils_FindJsonValue(pState,
                                           uStateLength,
                                           pType,
                                           strlen(pType),
                                           &pDocumentType,
                                           &uDocumentTypeLength);
    }

    if (found) {
        /* get key part */
        found = IotJsonUtils_FindJsonValue(pDocumentType,
                                           uDocumentTypeLength,
                                           pKey,
                                           strlen(pKey),
                                           pValue,
                                           pValueLength);
    }
    return found;
}

/*-----------------------------------------------------------*/

int _makeThingName(char *pIdentifier, size_t IdentifierLength)
{
    uint8_t mac[6];
    char type[3];

    if (read_aws_device_mac(mac) < 0) {
        return EXIT_FAILURE;
    }

#if defined(SENSOR_HUMIDITY_TEMPERATURE_SI7021)
    strncpy(type, "ht", sizeof(type));
#elif defined(SENSOR_AMBIENT_LIGHT_APDS9301)
    strncpy(type, "lu", sizeof(type));
#elif defined(SENSOR_DIGITAL_PRESSURE_BMP180)
    strncpy(type, "pt", sizeof(type));
#else
    strncpy(type, "na", sizeof(type));
#endif

    snprintf(pIdentifier, IdentifierLength,
            "sensor-%s-%02x%02x%02x%02x%02x%02x",
            type, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    wm_printf("ThingName: %s\r\n", pIdentifier);
    return EXIT_SUCCESS;
}

/*-----------------------------------------------------------*/

/**
 * @brief Send the Shadow updates that will update the sensor data
 *
 * @param[in] pDeltaSemaphore Used to synchronize Shadow updates with the delta
 * callback.
 * @param[in] mqttConnection The MQTT connection used for Shadows.
 * @param[in] pThingName The Thing Name for Shadows in this demo.
 * @param[in] thingNameLength The length of `pThingName`.
 *
 * @return `EXIT_SUCCESS` if all Shadow updates were sent; `EXIT_FAILURE`
 * otherwise.
 */
#if defined(SENSOR_HUMIDITY_TEMPERATURE_SI7021)
static int _sendShadowUpdates(IotMqttConnection_t mqttConnection,
                              const char * const pThingName,
                              int humidity,
                              int temperature)
{
    int status = EXIT_SUCCESS, len = 0;
    AwsIotShadowError_t updateStatus = AWS_IOT_SHADOW_STATUS_PENDING;
    AwsIotShadowDocumentInfo_t updateDocument = AWS_IOT_SHADOW_DOCUMENT_INFO_INITIALIZER;

    /* A buffer containing the update document. It has static duration to prevent
     * it from being placed on the call stack. */
    static char pUpdateDocument[EXPECTED_REPORTED_JSON_SIZE + 1] = { 0 };

    /* Set the common members of the Shadow update document info. */
    updateDocument.pThingName = pThingName;
    updateDocument.thingNameLength = strlen(pThingName);
    updateDocument.u.update.pUpdateDocument = pUpdateDocument;
    updateDocument.u.update.updateDocumentLength = EXPECTED_REPORTED_JSON_SIZE;

    len = snprintf(pUpdateDocument,
                   EXPECTED_REPORTED_JSON_SIZE + 1,
                   SHADOW_REPORTED_JSON,
                   humidity,
                   temperature,
                   ( long unsigned ) ( IotClock_GetTimeMs() % 1000000 ));

    updateDocument.u.update.updateDocumentLength = len;
    updateStatus = AwsIotShadow_TimedUpdate(mqttConnection,
                                            &updateDocument,
                                            AWS_IOT_SHADOW_FLAG_KEEP_SUBSCRIPTIONS,
                                            TIMEOUT_MS);
    if (updateStatus != AWS_IOT_SHADOW_SUCCESS) {
        status = EXIT_FAILURE;
        wm_printf("shadow update fail, Err:%d\r\n", updateStatus);
    }
    return status;
}

int _updateSensorData(IotSemaphore_t * pDeltaSemaphore,
                      IotMqttConnection_t mqttConnection,
                      const char * pThingName)
{
    mdev_t *pSensorDev = NULL;
    IotSemaphore_t sensorSemaphore;
    bool bSensorInitialized = false;
    bool bSensorOpened = false;
    bool bSemaphoreCreated = false;
    bool status = false;
    int ret = 0;
    int humidity = 0, last_humidity = 0;
    int temperature = 0, last_temperature = 0;
    int numRetry = 5;
    uint64_t last_time = 0;
    uint32_t timeout = 0, t;

    if (si7021_drv_init(board_peripheral_i2c_id()) < 0) {
        wm_printf("[%s] failed to init driver\r\n", SENSOR_NAME);
        goto f_exit;
    }
    bSensorInitialized = true;

    pSensorDev = si7021_drv_open(MDEV_SI7021);
    if (!pSensorDev) {
        wm_printf("[%s] failed to open driver\r\n", SENSOR_NAME);
        goto f_exit;
    }
    bSensorOpened = true;

    bSemaphoreCreated = IotSemaphore_Create(&sensorSemaphore, 0, 1);
    if (!bSemaphoreCreated) {
        goto f_exit;
    }

    last_time = 0;
    while (1) {
        t = (uint32_t)(IotClock_GetTimeMs() - last_time);
        if ((uPollingInterval == uDesiredInterval) &&
            (uPollingInterval > t) &&
            (last_time != 0)) {
            timeout = uPollingInterval - t;
            status = IotSemaphore_TimedWait(&sensorSemaphore, timeout);
        } else {
            status = true;
        }

        if (uPollingInterval != uDesiredInterval) {
            ret = _requireUpdateShadowInterval(pDeltaSemaphore,
                                               mqttConnection,
                                               pThingName,
                                               uDesiredInterval);
            if (ret == EXIT_FAILURE) {
                wm_printf("Failed to update interval data.\r\n");
                break;
            }
        }
        last_time = IotClock_GetTimeMs();

        /* get humidity & temperature from sensor */
        ret = si7021_getHumidityAndTemperature(pSensorDev,
                                                  &humidity,
                                                  &temperature);
        /* failed to get sensor data */
        if (ret < 0) {
            numRetry--;
            if (numRetry >= 0) {
                continue;
            }
            wm_printf("Failed to read data from sensor.\r\n");
            ret = EXIT_FAILURE;
            break;
        } else {
            numRetry = 5;
        }
        wm_printf("humidity:%d%%, temperature:%d\r\n", humidity,
                  temperature);
        if ((humidity != last_humidity) ||
            ((temperature / 10) != last_temperature)) {

            ret = _sendShadowUpdates(mqttConnection,
                                     pThingName,
                                     humidity,
                                     temperature / 10);
            if (ret == EXIT_SUCCESS) {
                last_humidity = humidity;
                last_temperature = temperature / 10;
            } else {
                wm_printf("Failed to update sensor data.\r\n");
                break;
            }
        }
    }
f_exit:
    if (bSemaphoreCreated) {
        IotSemaphore_Destroy(&sensorSemaphore);
    }
    if (bSensorOpened) {
        si7021_drv_close(pSensorDev);
    }
    if (bSensorInitialized) {
        si7021_drv_deinit(board_peripheral_i2c_id());
    }
    return ret;
}
/*-----------------------------------------------------------*/

#elif defined(SENSOR_AMBIENT_LIGHT_APDS9301)
static int _sendShadowUpdates(IotMqttConnection_t mqttConnection,
                              const char * const pThingName,
                              int lux)
{
    int status = EXIT_SUCCESS, len = 0;
    AwsIotShadowError_t updateStatus = AWS_IOT_SHADOW_STATUS_PENDING;
    AwsIotShadowDocumentInfo_t updateDocument = AWS_IOT_SHADOW_DOCUMENT_INFO_INITIALIZER;

    /* A buffer containing the update document. It has static duration to prevent
     * it from being placed on the call stack. */
    static char pUpdateDocument[EXPECTED_REPORTED_JSON_SIZE + 1] = { 0 };

    /* Set the common members of the Shadow update document info. */
    updateDocument.pThingName = pThingName;
    updateDocument.thingNameLength = strlen(pThingName);
    updateDocument.u.update.pUpdateDocument = pUpdateDocument;
    updateDocument.u.update.updateDocumentLength = EXPECTED_REPORTED_JSON_SIZE;

    len = snprintf(pUpdateDocument,
                   EXPECTED_REPORTED_JSON_SIZE + 1,
                   SHADOW_REPORTED_JSON,
                   lux,
                   ( long unsigned ) ( IotClock_GetTimeMs() % 1000000 ));

    updateDocument.u.update.updateDocumentLength = len;
    updateStatus = AwsIotShadow_TimedUpdate(mqttConnection,
                                            &updateDocument,
                                            AWS_IOT_SHADOW_FLAG_KEEP_SUBSCRIPTIONS,
                                            TIMEOUT_MS);
    if (updateStatus != AWS_IOT_SHADOW_SUCCESS) {
        status = EXIT_FAILURE;
        wm_printf("shadow update fail, Err:%d\r\n", updateStatus);
    }
    return status;
}

int _updateSensorData(IotSemaphore_t * pDeltaSemaphore,
                      IotMqttConnection_t mqttConnection,
                      const char * pThingName)
{
    mdev_t *pSensorDev = NULL;
    IotSemaphore_t sensorSemaphore;
    bool bSensorInitialized = false;
    bool bSensorOpened = false;
    bool bSemaphoreCreated = false;
    bool status = false;
    int ret = 0;
    float value = 0;
    int lux = 0, last_lux = 0;
    int numRetry = 5;
    uint64_t last_time = 0;
    uint32_t timeout = 0, t;

    if (apds9301_drv_init(board_peripheral_i2c_id(),
                          democonfigSENSOR_I2C_ADDR,
                          democonfigSENSOR_INT_GPIO) < 0) {
        wm_printf("[%s] failed to init driver\r\n", SENSOR_NAME);
        goto f_exit;
    }
    bSensorInitialized = true;

    pSensorDev = apds9301_drv_open(MDEV_APDS9301);
    if (!pSensorDev) {
        wm_printf("[%s] failed to open driver\r\n", SENSOR_NAME);
        goto f_exit;
    }

    /* power on sensor */
    apds9301_powerOn(pSensorDev);
    /* disable sensor threshold interrupt */
    apds9301_setInterruptOff(pSensorDev);
    /* set high gain */
    apds9301_setADCGain(pSensorDev, 1);
    /* set high integration time */
    apds9301_setIntegrationTime(pSensorDev, APDS9301_INTEG_HIGH);

    bSensorOpened = true;

    bSemaphoreCreated = IotSemaphore_Create(&sensorSemaphore, 0, 1);
    if (!bSemaphoreCreated) {
        goto f_exit;
    }

    last_time = 0;
    while (1) {
        t = (uint32_t)(IotClock_GetTimeMs() - last_time);
        if ((uPollingInterval == uDesiredInterval) &&
            (uPollingInterval > t) &&
            (last_time != 0)) {
            timeout = uPollingInterval - t;
            status = IotSemaphore_TimedWait(&sensorSemaphore, timeout);
        } else {
            status = true;
        }

        if (uPollingInterval != uDesiredInterval) {
            ret = _requireUpdateShadowInterval(pDeltaSemaphore,
                                               mqttConnection,
                                               pThingName,
                                               uDesiredInterval);
            if (ret == EXIT_FAILURE) {
                wm_printf("Failed to update interval data.\r\n");
                break;
            }
        }
        last_time = IotClock_GetTimeMs();

        /* get lux from sensor */
        value = apds9301_getLux(pSensorDev);

        /* failed to get sensor data */
        if (value < 0) {
            numRetry--;
            if (numRetry >= 0) {
                continue;
            }
            wm_printf("Failed to read data from sensor.\r\n");
            ret = EXIT_FAILURE;
            break;
        } else {
            numRetry = 5;
        }

        lux = (int)(value * 10);
        wm_printf("lux:%f\r\n", value);

        if (lux != last_lux) {

            ret = _sendShadowUpdates(mqttConnection,
                                     pThingName,
                                     lux);
            if (ret == EXIT_SUCCESS) {
                last_lux = lux;
            } else {
                wm_printf("Failed to update sensor data.\r\n");
                break;
            }
        }
    }
f_exit:
    if (bSemaphoreCreated) {
        IotSemaphore_Destroy(&sensorSemaphore);
    }
    if (bSensorOpened) {
        apds9301_powerOff(pSensorDev);
        apds9301_drv_close(pSensorDev);
    }
    if (bSensorInitialized) {
        apds9301_drv_deinit(board_peripheral_i2c_id());
    }
    return ret;
}
/*-----------------------------------------------------------*/

#elif defined(SENSOR_DIGITAL_PRESSURE_BMP180)
static int _sendShadowUpdates(IotMqttConnection_t mqttConnection,
                              const char * const pThingName,
                              int pressure,
                              int temperature)
{
    int status = EXIT_SUCCESS, len = 0;
    AwsIotShadowError_t updateStatus = AWS_IOT_SHADOW_STATUS_PENDING;
    AwsIotShadowDocumentInfo_t updateDocument = AWS_IOT_SHADOW_DOCUMENT_INFO_INITIALIZER;

    /* A buffer containing the update document. It has static duration to prevent
     * it from being placed on the call stack. */
    static char pUpdateDocument[EXPECTED_REPORTED_JSON_SIZE + 1] = { 0 };

    /* Set the common members of the Shadow update document info. */
    updateDocument.pThingName = pThingName;
    updateDocument.thingNameLength = strlen(pThingName);
    updateDocument.u.update.pUpdateDocument = pUpdateDocument;
    updateDocument.u.update.updateDocumentLength = EXPECTED_REPORTED_JSON_SIZE;

    len = snprintf(pUpdateDocument,
                   EXPECTED_REPORTED_JSON_SIZE + 1,
                   SHADOW_REPORTED_JSON,
                   pressure,
                   temperature,
                   ( long unsigned ) ( IotClock_GetTimeMs() % 1000000 ));

    updateDocument.u.update.updateDocumentLength = len;
    updateStatus = AwsIotShadow_TimedUpdate(mqttConnection,
                                            &updateDocument,
                                            AWS_IOT_SHADOW_FLAG_KEEP_SUBSCRIPTIONS,
                                            TIMEOUT_MS);
    if (updateStatus != AWS_IOT_SHADOW_SUCCESS) {
        status = EXIT_FAILURE;
        wm_printf("shadow update fail, Err:%d\r\n", updateStatus);
    }
    return status;
}

int _updateSensorData(IotSemaphore_t * pDeltaSemaphore,
                      IotMqttConnection_t mqttConnection,
                      const char * pThingName)
{
    mdev_t *pSensorDev = NULL;
    IotSemaphore_t sensorSemaphore;
    bool bSensorInitialized = false;
    bool bSensorOpened = false;
    bool bSemaphoreCreated = false;
    bool status = false;
    int ret = 0;
    int pressure = 0, last_pressure = 0;
    int temperature = 0, last_temperature = 0;
    int numRetry = 5;
    uint64_t last_time = 0;
    uint32_t timeout = 0, t;

    if (bmp180_drv_init(board_peripheral_i2c_id()) < 0) {
        wm_printf("[%s] failed to init driver\r\n", SENSOR_NAME);
        goto f_exit;
    }
    bSensorInitialized = true;

    pSensorDev = bmp180_drv_open(MDEV_BMP180);
    if (!pSensorDev) {
        wm_printf("[%s] failed to open driver\r\n", SENSOR_NAME);
        goto f_exit;
    }
    bSensorOpened = true;

    bmp180_setOversamplingSetting(pSensorDev, 3);

    bSemaphoreCreated = IotSemaphore_Create(&sensorSemaphore, 0, 1);
    if (!bSemaphoreCreated) {
        goto f_exit;
    }

    last_time = 0;
    while (1) {
        t = (uint32_t)(IotClock_GetTimeMs() - last_time);
        if ((uPollingInterval == uDesiredInterval) &&
            (uPollingInterval > t) &&
            (last_time != 0)) {
            timeout = uPollingInterval - t;
            status = IotSemaphore_TimedWait(&sensorSemaphore, timeout);
        } else {
            status = true;
        }

        if (uPollingInterval != uDesiredInterval) {
            ret = _requireUpdateShadowInterval(pDeltaSemaphore,
                                               mqttConnection,
                                               pThingName,
                                               uDesiredInterval);
            if (ret == EXIT_FAILURE) {
                wm_printf("Failed to update interval data.\r\n");
                break;
            }
        }
        last_time = IotClock_GetTimeMs();

        /* get pressure & temperature from sensor */
        ret = bmp180_getTemperature(pSensorDev, &temperature);
        if (ret == WM_SUCCESS) {
            ret = bmp180_getPressure(pSensorDev, &pressure);
        }

        /* failed to get sensor data */
        if (ret < 0) {
            numRetry--;
            if (numRetry >= 0) {
                continue;
            }
            wm_printf("Failed to read data from sensor.\r\n");
            ret = EXIT_FAILURE;
            break;
        } else {
            numRetry = 5;
        }
        wm_printf("pressure:%dPa, temperature:%d\r\n", pressure,
                  temperature);
        if ((pressure != last_pressure) ||
            (temperature != last_temperature)) {

            ret = _sendShadowUpdates(mqttConnection,
                                     pThingName,
                                     pressure,
                                     temperature);
            if (ret == EXIT_SUCCESS) {
                last_pressure = pressure;
                last_temperature = temperature;
            } else {
                wm_printf("Failed to update sensor data.\r\n");
                break;
            }
        }
    }
f_exit:
    if (bSemaphoreCreated) {
        IotSemaphore_Destroy(&sensorSemaphore);
    }
    if (bSensorOpened) {
        bmp180_drv_close(pSensorDev);
    }
    if (bSensorInitialized) {
        bmp180_drv_deinit(board_peripheral_i2c_id());
    }
    return ret;
}
/*-----------------------------------------------------------*/
#else
int _updateSensorData(IotSemaphore_t * pDeltaSemaphore,
                      IotMqttConnection_t mqttConnection,
                      const char * pThingName)
{
    return EXIT_SUCCESS;
}
#endif

/*-----------------------------------------------------------*/
int IotSensorDemo( bool awsIotMqttMode,
                   const char * pIdentifier,
                   void * pNetworkServerInfo,
                   void * pNetworkCredentialInfo,
                   const IotNetworkInterface_t * pNetworkInterface )
{
    int status = 0;
    IotMqttConnection_t mqttConnection = IOT_MQTT_CONNECTION_INITIALIZER;
    IotSemaphore_t deltaSemaphore;
    bool librariesInitialized = false;
    bool connectionEstablished = false;
    bool deltaSemaphoreCreated = false;
    char *pThingName = (char *)SensorIdentifier;
    char *pShadowDocument = NULL;
    size_t uShadowDocumentLength = 0;
    const char * pInterval;
    size_t uIntervalLength = 0;

    status = _makeThingName(pThingName, sizeof(SensorIdentifier));
    if (status == EXIT_SUCCESS) {
        /* Initialize the libraries required for this demo. */
        status = _initializeDemo();
    }

    if( status == EXIT_SUCCESS ) {
        /* Mark the libraries as initialized. */
        librariesInitialized = true;

        /* Establish a new MQTT connection. */
        status = _establishMqttConnection((const char *)pThingName,
                                          pNetworkServerInfo,
                                          pNetworkCredentialInfo,
                                          pNetworkInterface,
                                          &mqttConnection);
    }

    if( status == EXIT_SUCCESS ) {
        connectionEstablished = true;
        /* Create the semaphore that synchronizes with the delta callback. */
        deltaSemaphoreCreated = IotSemaphore_Create(&deltaSemaphore, 0, 1);
        if (deltaSemaphoreCreated == false) {
            status = EXIT_FAILURE;
        }
    }

    if (status == EXIT_SUCCESS) {
        /* Set the Shadow callbacks for this demo. */
        status = _setShadowCallbacks(&deltaSemaphore,
                                     mqttConnection,
                                     (const char *)pThingName);
    }

    if (status == EXIT_SUCCESS) {
        /* get shadow document to get last configuration */
        status = _getShadowDocument(mqttConnection,
                                    (const char *)pThingName,
                                    (const char **)&pShadowDocument,
                                    &uShadowDocumentLength);
        if (status == EXIT_SUCCESS) {
            /* parese configuration */
            if (_getShadowDocumentKeyValue(pShadowDocument,
                                           "reported",
                                           "interval",
                                           (const char **)&pInterval,
                                           &uIntervalLength)) {
                /* load interval value from aws shadow */
                uDesiredInterval = strtol(pInterval, NULL, 10);
                uPollingInterval = uDesiredInterval;
                wm_printf("uPollingInterval = %d\r\n", uPollingInterval);
            }
        } else if (status == AWS_IOT_SHADOW_NOT_FOUND) {
            status = EXIT_SUCCESS;
        }
    }

    if (status == EXIT_SUCCESS) {
        status = _updateSensorData(&deltaSemaphore,
                                   mqttConnection,
                                   (const char *)pThingName);
    }

    /* Disconnect the MQTT connection if it was established. */
    if (connectionEstablished) {
        IotMqtt_Disconnect(mqttConnection, 0);
    }

    /* Clean up libraries if they were initialized. */
    if (librariesInitialized) {
        _cleanupDemo();
    }

    if (deltaSemaphoreCreated) {
        IotSemaphore_Destroy(&deltaSemaphore);
    }
    return status;
}
/*-----------------------------------------------------------*/

/**
 * @brief The function that runs the Sensor demo, called by the demo runner.
 *
 * @param[in] awsIotMqttMode Ignored for the Sensor demo.
 * @param[in] pIdentifier NULL-terminated Shadow Thing Name.
 * @param[in] pNetworkServerInfo Passed to the MQTT connect function when
 * establishing the MQTT connection for Shadows.
 * @param[in] pNetworkCredentialInfo Passed to the MQTT connect function when
 * establishing the MQTT connection for Shadows.
 * @param[in] pNetworkInterface The network interface to use for this demo.
 *
 * @return `EXIT_SUCCESS` if the demo completes successfully; `EXIT_FAILURE` otherwise.
 */
int RunSensorDemo( bool awsIotMqttMode,
                   const char * pIdentifier,
                   void * pNetworkServerInfo,
                   void * pNetworkCredentialInfo,
                   const IotNetworkInterface_t * pNetworkInterface )
{
    int status = 0;

    while (1) {
        status = IotSensorDemo(awsIotMqttMode,
                               pIdentifier,
                               pNetworkServerInfo,
                               pNetworkCredentialInfo,
                               pNetworkInterface);
        wm_printf("restart iot sensor demo again...\r\n");
    }
    return status;
}

/*-----------------------------------------------------------*/
