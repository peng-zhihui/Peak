#include "HAL/HAL.h"
#include "NimBLEDevice.h"

void notifyCB(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify)
{
    std::string str = isNotify ? "Notification" : "Indication";
    str += " from ";
    /** NimBLEAddress and NimBLEUUID have std::string operators */
    str += std::string(pRemoteCharacteristic->getRemoteService()->getClient()->getPeerAddress());
    str += ": Service = " + std::string(pRemoteCharacteristic->getRemoteService()->getUUID());
    str += ", Characteristic = " + std::string(pRemoteCharacteristic->getUUID());
    str += ", Value = ";
    Serial.print(str.c_str());
    Serial.println(pRemoteCharacteristic->getValue<float>());
}
void scanEndedCB(NimBLEScanResults results)
{
    Serial.println("Scan Ended");
}

static NimBLEAdvertisedDevice* advDevice;
static bool doConnect = false;
static bool isConnected = false;

class ClientCallbacks : public NimBLEClientCallbacks
{
    void onConnect(NimBLEClient* pClient)
    {
        Serial.println("Connected");
        /** After connection we should change the parameters if we don't need fast response times.
         *  These settings are 150ms interval, 0 latency, 450ms timout.
         *  Timeout should be a multiple of the interval, minimum is 100ms.
         *  I find a multiple of 3-5 * the interval works best for quick response/reconnect.
         *  Min interval: 120 * 1.25ms = 150, Max interval: 120 * 1.25ms = 150, 0 latency, 60 * 10ms = 600ms timeout
         */
        pClient->updateConnParams(120, 120, 0, 300);
        isConnected = true;
    };

    void onDisconnect(NimBLEClient* pClient)
    {
        Serial.print(pClient->getPeerAddress().toString().c_str());
        Serial.println(" Disconnected - Starting scan");
        NimBLEDevice::getScan()->start(0, scanEndedCB);
        isConnected = false;
    };

    /** Called when the peripheral requests a change to the connection parameters.
     *  Return true to accept and apply them or false to reject and keep
     *  the currently used parameters. Default will return true.
     */
    bool onConnParamsUpdateRequest(NimBLEClient* pClient, const ble_gap_upd_params* params)
    {
        if (params->itvl_min < 24)
        { /** 1.25ms units */
            return false;
        } else if (params->itvl_max > 40)
        { /** 1.25ms units */
            return false;
        } else if (params->latency > 2)
        { /** Number of intervals allowed to skip */
            return false;
        } else if (params->supervision_timeout > 100)
        { /** 10ms units */
            return false;
        }

        return true;
    };

    /********************* Security handled here **********************
    ****** Note: these are the same return values as defaults ********/
    uint32_t onPassKeyRequest()
    {
        Serial.println("Client Passkey Request");
        /** return the passkey to send to the server */
        return 123456;
    };

    bool onConfirmPIN(uint32_t pass_key)
    {
        Serial.print("The passkey YES/NO number: ");
        Serial.println(pass_key);
        /** Return false if passkeys don't match. */
        return true;
    };

    /** Pairing process complete, we can check the results in ble_gap_conn_desc */
    void onAuthenticationComplete(ble_gap_conn_desc* desc)
    {
        if (!desc->sec_state.encrypted)
        {
            Serial.println("Encrypt connection failed - disconnecting");
            /** Find the client with the connection handle provided in desc */
            NimBLEDevice::getClientByID(desc->conn_handle)->disconnect();
            return;
        }
    };
};

class AdvertisedDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks
{
    void onResult(NimBLEAdvertisedDevice* advertisedDevice)
    {
//        Serial.print("Advertised Device found: ");
//        Serial.println(advertisedDevice->toString().c_str());
        if (advertisedDevice->isAdvertisingService(NimBLEUUID("0001")))
//            if (advertisedDevice->getAddress() == NimBLEAddress("50:02:91:ac:b0:6a"))
        {
            Serial.println("Found Dummy-Robot!");
            /** stop scan before connecting */
            NimBLEDevice::getScan()->stop();
            /** Save the device reference in a global for the client to use*/
            advDevice = advertisedDevice;
            /** Ready to connect now */
            doConnect = true;
        }
    };
};

static ClientCallbacks clientCB;

bool connectToServer()
{
    NimBLEClient* pClient = nullptr;

    /** Check if we have a client we should reuse first **/
    if (NimBLEDevice::getClientListSize())
    {
        /** Special case when we already know this device, we send false as the
         *  second argument in connect() to prevent refreshing the service database.
         *  This saves considerable time and power.
         */
        pClient = NimBLEDevice::getClientByPeerAddress(advDevice->getAddress());
        if (pClient)
        {
            if (!pClient->connect(advDevice, false))
            {
                Serial.println("Reconnect failed");
                return false;
            }
            Serial.println("Reconnected client");
        }
            /** We don't already have a client that knows this device,
             *  we will check for a client that is disconnected that we can use.
             */
        else
        {
            pClient = NimBLEDevice::getDisconnectedClient();
        }
    }

    /** No client to reuse? Create a new one. */
    if (!pClient)
    {
        if (NimBLEDevice::getClientListSize() >= NIMBLE_MAX_CONNECTIONS)
        {
            Serial.println("Max clients reached - no more connections available");
            return false;
        }

        pClient = NimBLEDevice::createClient();

        Serial.println("New client created");

        pClient->setClientCallbacks(&clientCB, false);
        /** Set initial connection parameters: These settings are 15ms interval, 0 latency, 120ms timout.
         *  These settings are safe for 3 clients to connect reliably, can go faster if you have less
         *  connections. Timeout should be a multiple of the interval, minimum is 100ms.
         *  Min interval: 12 * 1.25ms = 15, Max interval: 12 * 1.25ms = 15, 0 latency, 51 * 10ms = 510ms timeout
         */
        pClient->setConnectionParams(12, 12, 0, 51);
        /** Set how long we are willing to wait for the connection to complete (seconds), default is 30. */
        pClient->setConnectTimeout(5);


        if (!pClient->connect(advDevice))
        {
            /** Created a client but failed to connect, don't need to keep it as it has no data */
            NimBLEDevice::deleteClient(pClient);
            Serial.println("Failed to connect, deleted client");
            return false;
        }
    }

    if (!pClient->isConnected())
    {
        if (!pClient->connect(advDevice))
        {
            Serial.println("Failed to connect");
            return false;
        }
    }

    Serial.print("Connected to: ");
    Serial.println(pClient->getPeerAddress().toString().c_str());
    Serial.print("RSSI: ");
    Serial.println(pClient->getRssi());

    /** Now we can read/write/subscribe the charateristics of the services we are interested in */
    NimBLERemoteService* pSvc = nullptr;
    NimBLERemoteCharacteristic* pChr = nullptr;
    NimBLERemoteDescriptor* pDsc = nullptr;

    pSvc = pClient->getService("0001");
    if (pSvc)
    {     /** make sure it's not null */
        pChr = pSvc->getCharacteristic("1001");

        if (pChr)
        {     /** make sure it's not null */
            if (pChr->canRead())
            {
                Serial.print(pChr->getUUID().toString().c_str());
                Serial.print(" Value: ");
                Serial.println(*(float*) (pChr->readValue().c_str()));
            }

//            if (pChr->canWrite())
//            {
//                if (pChr->writeValue("Tasty"))
//                {
//                    Serial.print("Wrote new value to: ");
//                    Serial.println(pChr->getUUID().toString().c_str());
//                } else
//                {
//                    /** Disconnect if write failed */
//                    pClient->disconnect();
//                    return false;
//                }
//
//                if (pChr->canRead())
//                {
//                    Serial.print("The value of: ");
//                    Serial.print(pChr->getUUID().toString().c_str());
//                    Serial.print(" is now: ");
//                    Serial.println(pChr->readValue().c_str());
//                }
//            }

            /** registerForNotify() has been deprecated and replaced with subscribe() / unsubscribe().
             *  Subscribe parameter defaults are: notifications=true, notifyCallback=nullptr, response=false.
             *  Unsubscribe parameter defaults are: response=false.
             */
            if (pChr->canNotify())
            {
                //if(!pChr->registerForNotify(notifyCB)) {
                if (!pChr->subscribe(true, notifyCB))
                {
                    /** Disconnect if subscribe failed */
                    pClient->disconnect();
                    return false;
                }
            }
//            else if (pChr->canIndicate())
//            {
//                /** Send false as first argument to subscribe to indications instead of notifications */
//                //if(!pChr->registerForNotify(notifyCB, false)) {
//                if (!pChr->subscribe(false, notifyCB))
//                {
//                    /** Disconnect if subscribe failed */
//                    pClient->disconnect();
//                    return false;
//                }
//            }
        }

        pChr = pSvc->getCharacteristic("1002");
        if (pChr)
        {     /** make sure it's not null */
            if (pChr->canNotify())
            {
                //if(!pChr->registerForNotify(notifyCB)) {
                if (!pChr->subscribe(true, notifyCB))
                {
                    /** Disconnect if subscribe failed */
                    pClient->disconnect();
                    return false;
                }
            }
        }

        pChr = pSvc->getCharacteristic("1003");
        if (pChr)
        {     /** make sure it's not null */
            if (pChr->canNotify())
            {
                //if(!pChr->registerForNotify(notifyCB)) {
                if (!pChr->subscribe(true, notifyCB))
                {
                    /** Disconnect if subscribe failed */
                    pClient->disconnect();
                    return false;
                }
            }
        }

        pChr = pSvc->getCharacteristic("1004");
        if (pChr)
        {     /** make sure it's not null */
            if (pChr->canNotify())
            {
                //if(!pChr->registerForNotify(notifyCB)) {
                if (!pChr->subscribe(true, notifyCB))
                {
                    /** Disconnect if subscribe failed */
                    pClient->disconnect();
                    return false;
                }
            }
        }

        pChr = pSvc->getCharacteristic("1005");
        if (pChr)
        {     /** make sure it's not null */
            if (pChr->canNotify())
            {
                //if(!pChr->registerForNotify(notifyCB)) {
                if (!pChr->subscribe(true, notifyCB))
                {
                    /** Disconnect if subscribe failed */
                    pClient->disconnect();
                    return false;
                }
            }
        }

        pChr = pSvc->getCharacteristic("1006");
        if (pChr)
        {     /** make sure it's not null */
            if (pChr->canNotify())
            {
                //if(!pChr->registerForNotify(notifyCB)) {
                if (!pChr->subscribe(true, notifyCB))
                {
                    /** Disconnect if subscribe failed */
                    pClient->disconnect();
                    return false;
                }
            }
        }
    } else
    {
        Serial.println("0001 service not found.");
    }

    Serial.println("Done with this device!");
    return true;
}


bool HAL::BT_Init()
{
    Serial.println("Starting Peak-BLE Client");

    NimBLEDevice::init("Peak-BLE");

    NimBLEDevice::setSecurityAuth(false, false, true);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9); /** +9db */

    /** create new scan */
    NimBLEScan* pScan = NimBLEDevice::getScan();
    /** create a callback that gets called when advertisers are found */
    pScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
    /** Set scan interval (how often) and window (how long) in milliseconds */
    pScan->setInterval(20);
    pScan->setWindow(15);
    /** Active scan will gather scan response data from advertisers
     *  but will use more energy from both devices
     */
    pScan->setActiveScan(true);
    /** Start scanning for advertisers for the scan time specified (in seconds) 0 = forever
     *  Optional callback for when scanning stops.
     */
    pScan->start(0, scanEndedCB);

    return true;
}

void HAL::BT_Update()
{
    if (doConnect)
    {
        doConnect = false;

        /** Found a device we want to connect to, do it now */
        if (connectToServer())
        {
            Serial.println("Success! we should now be getting notifications, scanning for more!");
        } else
        {
            Serial.println("Failed to connect, starting scan");
            NimBLEDevice::getScan()->start(0, scanEndedCB);
        }
    }
}

bool HAL::BluetoothConnected()
{
    return isConnected;
}
