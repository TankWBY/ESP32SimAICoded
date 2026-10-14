#include "host/serial_host.hpp"

namespace esp32sim {

bool SerialHost::poll(SerialEvent& ev) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) return false;
    ev = std::move(queue_.front());
    queue_.pop_front();
    return true;
}

void SerialHost::push(SerialEvent ev) {
    std::lock_guard<std::mutex> lock(mutex_);
    // Coalesce consecutive data events to keep the queue short.
    if (ev.kind == SerialEvent::Kind::Data && !queue_.empty() && queue_.back().kind == SerialEvent::Kind::Data) {
        auto& d = queue_.back().data;
        d.insert(d.end(), ev.data.begin(), ev.data.end());
        return;
    }
    queue_.push_back(std::move(ev));
}

}  // namespace esp32sim
