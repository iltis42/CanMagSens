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
            _all_p.push_back(tmp);
            if ( getNumNMEA() == 1 ) {
                _nmea = tmp;
            }
            return tmp;
        }
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
        // _sm.reset();
        // _binary_mode = false;
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
        ESP_LOGI(FNAME, "%d procN %dchar: %s", _itf_id.iid, len, packet);
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

                }
                ESP_LOGD(FNAME, "crc := %d", _sm._crc);
            } else {
                _sm.reset();
            }
        }
    }
}

bool DataLink::goBIN(ProtocolItf *peer_prto)
{
    // FlarmBinary *bin = static_cast<FlarmBinary*>(getProtocol(FLARMBIN_P));
    // if ( bin ) {
    //     bin->setPeer(peer_prto);
    //     _binary = bin;
    //     return true;
    // }
    return false;
}

void DataLink::goNMEA()
{
    if ( _binary ) {
        _binary = nullptr;
    }
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

void DataLink::dumpProto()
{
    for (ProtocolItf *it : _all_p)
    {
        ESP_LOGI(FNAME, "  lp%d: did%d\tpid%d\tsp%d", getPort(), (*it).getDeviceId(), (*it).getProtocolId(), (*it).getSendPort());
    }
}

void DataLink::forwardMsg(DeviceId src_dev)
{
    // consider forwarding
    ESP_LOGD(FNAME, "get routing for %d/%d", src_dev, _itf_id.port);
    RoutingList routes = DEVMAN->getRouting(RoutingTarget(src_dev, _itf_id.port)); // todo cache the route
    ESP_LOGD(FNAME, "routings %d", routes.size());
    for ( auto &target : routes ) {
        Message* msg = DEV::acqMessage(target.did, target.port);
        ESP_LOGI(FNAME, "route %d/%d to %d/%d", src_dev, _itf_id.port, msg->target_id, target.port);
        msg->buffer = _sm._frame;
        DEV::Send(msg);
    }
}
