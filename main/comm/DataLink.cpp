/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "DataLink.h"

#include "protocol/RegQuery.h"
#include "protocol/MagSens.h"
#include "Messages.h"
#include "DeviceMgr.h"

#include "logdef.h"

DataLink::~DataLink()
{
    for (auto *it : _all_p) {
        ESP_LOGI(FNAME, "delete  prtcl %p", it);
        delete it;
    }
    _all_p.clear();
}

// protocol factory
ProtocolItf* DataLink::addProtocol(ProtocolType ptyp, DeviceId did, int sendport)
{

    // Check if already there
    bool foundit = false;
    for (auto *it : _all_p) {
        if ( (*it).getProtocolId() == ptyp 
            && (*it).getSendPort() == sendport ) {
            foundit = true;
        }
    }
    if ( ! foundit ) {
        // Create a new one
        ProtocolItf *tmp = nullptr;
        switch (ptyp)
        {
        case REGISTRATION_P:
            ESP_LOGI(FNAME, "New RegQuery");
            tmp = new RegQuery(sendport, _sm, *this);
            break;
        case MAGSENS_P:
            ESP_LOGI(FNAME, "New MagSens");
            tmp = new MagSens(sendport, _sm, *this);
            break;
        default:
            break;
        }
        ESP_LOGI(FNAME, "On send port %d", sendport);

        if ( tmp ) {
            // Check device id is equal to all others
            if ( _did == NO_DEVICE ) {
                _did = tmp->getDeviceId();
            } else {
                for ( auto it : _all_p ) {
                    if ( (*it).getDeviceId() != _did ) {
                        ESP_LOGW(FNAME, "DevId missmatch in protocol list for Itf/port %d/%d %d: %d.", 
                            _itf_id.iid, _itf_id.port, _did, (*it).getDeviceId());
                    }
                }
            }
            _all_p.push_back(tmp);
            if ( _all_p.size() == 1 && tmp->isBinary() ) {
                _binary = tmp;
            }
            else if ( getNumNMEA() == 1 ) {
                _nmea = tmp;
            }
            return tmp;
        }
    }
    else {
        ESP_LOGW(FNAME, "Double insertion of device/protocol %d/%d.", did, ptyp);
    }
    return nullptr;
}

ProtocolItf* DataLink::getProtocol(ProtocolType ptyp) const
{
    if ( ptyp == NO_ONE ) {
        if ( ! _all_p.empty() ) {
            return _all_p.front();
        }
    }
    else {
        for (ProtocolItf *it : _all_p) {
            if ( (*it).getProtocolId() == ptyp ) {
                return it;
            }
        }
    }
    return nullptr;
}

bool DataLink::hasProtocol(ProtocolType ptyp) const
{
    for (ProtocolItf *it : _all_p) {
        if ( (*it).getProtocolId() == ptyp ) {
            return true;
        }
    }
    return false;
}

void DataLink::deleteProtocol(ProtocolItf *proto)
{
    for (auto it = _all_p.begin(); it != _all_p.end(); it++) {
        if (*it == proto) {
            delete *it;
            _all_p.erase(it);
            return;
        }
    }
}

void DataLink::process(const char *packet, int len)
{
    if ( _all_p.empty() ) {
        return;
    }

    datalink_action_t action = NOACTION;

    if (packet == nullptr)
    {
        // Special use, "no data" timeout, might be expected and normal
        // We just reset the protocol state machine then
        goNMEA();
        ESP_LOGW(FNAME, "timeout Itf/Port %d/%d", _itf_id.iid, _itf_id.port);
        return;
    }

    if ( _binary ) {
        ESP_LOGD(FNAME, "%d procB %dchar: %c", _itf_id.iid, len, *packet);
        if ( _binary->nextStreamChunk(packet, len) == GO_NMEA ) {
            goNMEA();
        }
    }
    else if ( _nmea )
    {
        ESP_LOGD(FNAME, "%d procN %dchar: %s", _itf_id.iid, len, packet);
        // most likely case, only one protocol to parse
        // process every frame byte through state machine
        ProtocolItf *prtcl = _all_p.front();
        for (int i = 0; i < len; i++) {
            if ( _sm.push(packet[i]) ) {
                action = prtcl->nextByte(packet[i]);
                if ( action )
                {
                    if ( action & FORWARD_BIT ) {
                        forwardMsg(prtcl->getDeviceId());
                    }
                    if ( action == NXT_PROTO ) {
                        _sm.reset();
                        break; // end loop imidiately
                    }
                    else if ( action == GO_NMEA ) {
                        goNMEA();
                    }

                }
                ESP_LOGD(FNAME, "crc := %d", _sm._crc);
            } else {
                _sm.reset();
            }
        }
    }
}

ProtocolItf* DataLink::goBIN()
{
    ProtocolItf *bin = getBinary();
    if ( bin ) {
        _binary = bin;
        return bin;
    }
    return nullptr;
}

void DataLink::goNMEA()
{
    _binary = nullptr;
    _sm.reset();
}

int DataLink::getNumNMEA() const
{
    int count = 0;
    for (ProtocolItf *it : _all_p) {
        if ( (*it).isBinary() ) {
            continue;
        }
        count++;
    }
    return count;
}

ProtocolItf* DataLink::getBinary() const
{
    for (ProtocolItf *it : _all_p) {
        if ( (*it).isBinary() ) {
            return it;
        }
    }
    return nullptr;
}

void DataLink::updateRoutes()
{
    ESP_LOGD(FNAME, "get routing for %d/%d", _did, _itf_id.port);
    _routes = DEVMAN->getRouting(RoutingTarget(_did, _itf_id.port));
}

void DataLink::dumpProto()
{
    for (ProtocolItf *it : _all_p)
    {
        ESP_LOGI(FNAME, "    lp%d: did%d\tpid%d\tsp%d", getPort(), (*it).getDeviceId(), (*it).getProtocolId(), (*it).getSendPort());
    }
}

void DataLink::forwardMsg(DeviceId src_dev)
{
    // consider forwarding
    for ( auto &target : _routes ) {
        Message* msg = DEV::acqMessage(target.did, target.port);
        ESP_LOGI(FNAME, "route %d/%d to %d/%d", src_dev, _itf_id.port, msg->target_id, target.port);
        msg->buffer = _sm._frame;
        DEV::Send(msg);
    }
}
