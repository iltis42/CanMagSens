/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "DeviceMgr.h"

#include "InterfaceCtrl.h"
#include "comm/Messages.h"
#include "comm/CanBus.h"
#include "protocol/ProtocolItf.h"

#include "sensor.h"
#include "logdef.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <deque>

// global variables
DeviceManager* DEVMAN = nullptr; // singleton like
QueueHandle_t ItfSendQueue = 0;
MessagePool MP;

// static vars
static TaskHandle_t SendTask = nullptr;

// static routing table on device level
static RoutingMap Routes = {
    { {FLARM_DEV, 0}, {{NAVI_DEV, 0}, {XCVARIOCLIENT_DEV, 20}} },
    { {NAVI_DEV, 0}, {{FLARM_DEV, 0}} }
};

// a dummy interface
class DmyItf final : public InterfaceCtrl
{
public:
    const char* getStringId() const override { return "DMY"; }
    void ConfigureIntf(int cfg) override {}
    int Send(const char *msg, int &len, int port=0) { return 0; }
};
static DmyItf dummy_itf;

// generic transmitter grabbing messages from a queue
void TransmitTask(void *arg)
{
    QueueHandle_t queue = (QueueHandle_t)arg;
    Message* msg;

    while (true)
    {
        // sleep until the queue gives us something to do
        if ( xQueueReceive(queue, &msg, portMAX_DELAY) ) {

            if ( msg == nullptr ) { break; } // termination signal

            int len = msg->buffer.length();
            int port = msg->port;
            InterfaceCtrl *itf = DEVMAN->getIntf(msg->target_id);
            ESP_LOGD(FNAME, "send to dev %d %s/%d NMEA len %d, msg: %s", msg->target_id, itf->getStringId(), port, len, msg->buffer.c_str());
            if ( ! itf || (itf->Send(msg->buffer.c_str(), len, port) != 0) ) {
                ESP_LOGE(FNAME, "failed sending");
            }
            DEV::relMessage(msg);
        }

    }

    vTaskDelete(NULL);
}

DeviceManager::DeviceManager()
{
    ItfSendQueue = xQueueCreate( 20, sizeof(Message*) );
    
}

DeviceManager::~DeviceManager()
{
    xQueueSend( ItfSendQueue, nullptr, 0 ); // for transmitter
}

// The device manager
DeviceManager* DeviceManager::Instance()
{
    if ( ! DEVMAN ) {
        DEVMAN = new DeviceManager();
    }
    return DEVMAN;
}

// Do all it needs to prepare comm with device and route data
// It returns the pointer to the protocol as handle to send messages
ProtocolItf* DeviceManager::addDevice(DeviceId did, ProtocolType proto, int listen_port, int send_port, InterfaceId iid)
{
    // On first device a send task needs to be created
    if ( ! SendTask ) {
        xTaskCreate(TransmitTask, "genTx", 3000, ItfSendQueue, 90, &SendTask);
    }
    InterfaceCtrl *itf = &dummy_itf;
    if ( iid == CAN_BUS ) {
        if ( CAN && CAN->isInitialized() ) {
            itf = CAN;
        }
    }
    else if ( iid == NO_PHY ) {
        // Device might exist already
        itf = getIntf(did);
    }
    bool is_new = false;
    Device *dev = getDevice(did);
    if ( ! dev ) {
        dev = new Device(did);
        is_new = true;
    }
    // Retrieve, or create a new data link
    DataLink *dl = itf->newDataLink(listen_port);
    ProtocolItf* ret = dl->addProtocol(proto, did, send_port); // Add proto, if not yet there

    if ( is_new ) {
        // Add only if new
        _device_map[did] = dev; // and add, in case this dev is new
        dev->_itf = itf;
    }
    dev->_dlink.insert(dl);
    refreshRouteCache();

    ESP_LOGI(FNAME, "After add device %d.", did);
    dumpMap();

    return ret;
}

Device *DeviceManager::getDevice(DeviceId did)
{
    DevMap::iterator it = _device_map.find(did);
    if ( it != _device_map.end() ) {
        return it->second;
    }
    return nullptr;
}

ProtocolItf *DeviceManager::getProtocol(DeviceId did, ProtocolType proto)
{
    Device *d = getDevice(did);
    if ( d ) {
        return d->getProtocol(proto);
    }
    return nullptr;
}

// Remove device from map, delete device and all resources
void DeviceManager::removeDevice(DeviceId did)
{
    DevMap::iterator it = _device_map.find(did);
    Device* dev = nullptr;
    if ( it != _device_map.end() ) {
        dev = it->second;
        ESP_LOGI(FNAME, "Delete device %d", did);
        _device_map.erase(it);
        InterfaceCtrl *itf = dev->_itf;
        delete dev;
        // is it the last device on this interface
        if ( itf->getNrDLinks() == 0 ) {
            if ( itf->getId() == CAN_BUS ) {
                CAN->stop();
            }
        }
    }
}

// routing lookup table
InterfaceCtrl* DeviceManager::getIntf(DeviceId did)
{
    DevMap::iterator it = _device_map.find(did);
    if ( it != _device_map.end() ) {
        return it->second->_itf;
    }
    ESP_LOGW(FNAME, "No device %d found", did);
    return nullptr;
}

// Result should be cashed for performance purpose. Todo dirty flag mechanism
RoutingList DeviceManager::getRouting(RoutingTarget target)
{
    RoutingMap::iterator route_it = Routes.find(target);
    if ( route_it != Routes.end() )
    {
        // remove not existing devices from the routing list
        RoutingList res = route_it->second;
        for (RoutingList::iterator tit=res.begin(); tit != res.end(); ) 
        {
            // is dev configured
            if ( _device_map.find(tit->did) == _device_map.end() ) {
                ESP_LOGD(FNAME, "remove %d from routing list.", tit->did);
                tit = res.erase(tit);
            }
            else {
                tit++;
            }
        }
        return std::move(res);
    }
    else {
        return RoutingList();
    }
}

// Start a binary data route
DataLink *DeviceManager::getFlarmBinPeer()
{
    for ( auto &d : _device_map ) {
        DataLink *dl = d.second->getDLforProtocol(FLARMHOST_P);
        if ( dl ) return dl;
    }
    return nullptr;
}

// Recover all Flarm data routes back to NMEA mode
void DeviceManager::resetFlarmModeToNmea()
{

}

// prio - 0,..,4 0:low prio data stream, .. 5: important high prio commanding
// return an id chunk of four to use (0..0x3fc), use then +0, +1, +2, +3
// else -1 in case there is no chunk left
int DeviceManager::getFreeCANId(int prio)
{
    static int nxt_free_slot[5] = { 0x400, 0x200, 0x100, 0x80, 0x40 }; // 5 prio slots of 15 chucks of 4 ids
    if ( prio > 4 ) {
        prio = 4;
    }
    if ( prio < 0 ) {
        prio = 0;
    }
    if ( (nxt_free_slot[prio] & 0x3c) != 0x3c ) { // chunk mask b0011 1100
        int slot = nxt_free_slot[prio];
        nxt_free_slot[prio] += 4; // next chunk of four
        return slot;
    }
    return -1;
}

void DeviceManager::dumpMap() const
{
    ESP_LOGI(FNAME, "Device map dump:");
    for ( auto &it : _device_map ) {
        Device* d = it.second;
        ESP_LOGI(FNAME, "%d: %p (did%d/iid%s/dl*%d)", it.first, d, d->_id, d->_itf->getStringId(), d->_dlink.size());
        for ( auto &l : d->_dlink ) {
            l->dumpProto();
        }
    }
}

// Resolve the existance of a device
Device::~Device()
{
    ESP_LOGI(FNAME, "Delete device %d.", _id);
    // Detach data links from interface
    for (DataLink* dl : _dlink) {
        _itf->DeleteDataLink(dl->getPort());
    }
    // Clear the set
    _dlink.clear();
}

ProtocolItf *Device::getProtocol(ProtocolType p) const
{
    // Find protocol
    for (DataLink* dl : _dlink) {
        ProtocolItf *tmp = dl->getProtocol(p);
        if ( tmp ) {
            return tmp;
        }
    }
    return nullptr;
}

DataLink *Device::getDLforProtocol(ProtocolType p) const
{
    // Find protocol
    for (DataLink* dl : _dlink) {
        if ( dl->getProtocol(p) ) {
            return dl;
        }
    }
    return nullptr;
}

int Device::getSendPort(ProtocolType p) const
{
    // get port for protocol
    ProtocolItf *tmp = getProtocol(p);
    if ( tmp ) {
        return tmp->getSendPort();
    }

    return 0;
}



// Some global routines from the router, they might move elswhere
namespace DEV {

Message* acqMessage(DeviceId target_id, int port)
{
    Message* m = MP.getOne();
    m->target_id = target_id;
    m->port = port;
    return m;
}

void relMessage(Message *msg)
{
    MP.recycleMsg(msg);
}

bool Send(Message* msg)
{
    if ( ItfSendQueue ) {
        if ( pdTRUE != xQueueSend( ItfSendQueue, (void * ) &msg, (TickType_t)0 ) ) {
            // drop it
            ESP_LOGW(FNAME, "Dropped message to %d", msg->target_id);
            MP.recycleMsg(msg);
        }
        return true;
    }
    return false;
}

} // namespace
