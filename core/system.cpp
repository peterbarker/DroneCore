#include "system.h"
#include "global_include.h"
#include "dronecore_impl.h"
#include "mavlink_include.h"
#include "mavlink_system.h"
#include "plugin_impl_base.h"
#include <functional>
#include <algorithm>
#include "px4_custom_mode.h"

// Set to 1 to log incoming/outgoing mavlink messages.
#define MESSAGE_DEBUGGING 0

namespace dronecore {

using namespace std::placeholders; // for `_1`

System::System(DroneCoreImpl &parent,
               uint8_t system_id,
               uint8_t component_id) :
    _system_id(system_id),
    _component_id(component_id),
    _parent(parent)
{
}

System::~System()
{
}

bool System::is_standalone() const
{
    if (_mavlink_system == nullptr) {
        return false;
    }
    return _mavlink_system->is_standalone();
}

bool System::has_autopilot() const
{
    if (_mavlink_system == nullptr) {
        return false;
    }
    return _mavlink_system->has_autopilot();
}

bool System::has_camera(int camera_id) const
{
    if (_mavlink_system == nullptr) {
        return false;
    }
    return _mavlink_system->has_camera(camera_id);
}

bool System::has_gimbal() const
{
    if (_mavlink_system == nullptr) {
        return false;
    }
    return _mavlink_system->has_gimbal();
}

void System::add_new_component(uint8_t component_id)
{
    if (_mavlink_system == nullptr) {
        return;
    }
    return _mavlink_system->add_new_component(component_id);
}

void System::process_mavlink_message(const mavlink_message_t &message)
{
    if (_mavlink_system == nullptr) {
        if (message.msgid != MAVLINK_MSG_ID_HEARTBEAT) {
            return;
        }
        mavlink_heartbeat_t hbeat;
        mavlink_msg_heartbeat_decode(&message, &hbeat);
        _autopilot_type = (MAV_AUTOPILOT)hbeat.autopilot;
        switch (hbeat.autopilot) {
        case MAV_AUTOPILOT_PX4:
            _mavlink_system = std::make_shared<MAVLinkSystem>(_parent,
                                                              _system_id,
                                                              _component_id);
            break;
        case MAV_AUTOPILOT_ARDUPILOTMEGA:
            _mavlink_system = std::make_shared<APMAVLinkSystem>(_parent,
                                                                 _system_id,
                                                                 _component_id);
            break;
        }
    }
    return _mavlink_system->process_mavlink_message(message);
}

void System::set_system_id(uint8_t system_id)
{
    _system_id = system_id;
    if (_mavlink_system == nullptr) {
        return;
    }
    return _mavlink_system->set_system_id(system_id);
}

bool System::is_connected() const
{
    if (_mavlink_system == nullptr) {
        return false;
    }
    return _mavlink_system->is_connected();
}

uint64_t System::get_uuid() const
{
    // We want to support UUIDs if the autopilot tells us.
    if (_mavlink_system == nullptr) {
        return (uint64_t)-1;
    }
    return _mavlink_system->get_uuid();
}

uint8_t System::get_system_id() const
{
    return _system_id;
}

} // namespace dronecore
