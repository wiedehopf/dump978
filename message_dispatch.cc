// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#include "message_dispatch.h"

#include <boost/thread/locks.hpp>
#include <mutex>

using namespace flightaware::uat;

MessageDispatch::MessageDispatch() : next_handle_(0), busy_(0) {}

MessageDispatch::Handle MessageDispatch::AddClient(MessageHandler handler) {
    std::unique_lock<std::recursive_mutex> lock(mutex_);

    Handle h = next_handle_++;
    clients_[h] = {handler, false};
    return h;
}

void MessageDispatch::RemoveClient(Handle h) {
    std::unique_lock<std::recursive_mutex> lock(mutex_);

    auto i = clients_.find(h);
    if (i == clients_.end())
        return;

    i->second.deleted = true;
    PurgeDeadClients();
}

template <typename T> class BusyCounter {
  public:
    BusyCounter(T &var) : var_(var), owned_(true) { ++var_; }

    BusyCounter(const T &) = delete;
    BusyCounter &operator=(const T &) = delete;

    ~BusyCounter() { release(); }

    void release() {
        if (owned_) {
            --var_;
            owned_ = false;
        }
    }

  private:
    T &var_;
    bool owned_;
};

void MessageDispatch::Dispatch(SharedMessageVector messages) {
    std::unique_lock<std::recursive_mutex> lock(mutex_);

    BusyCounter<unsigned> counter(busy_);
    for (auto i = clients_.begin(); i != clients_.end(); ++i) {
        Client &c = i->second;
        if (!c.deleted)
            c.handler(messages);
    }
    counter.release();

    PurgeDeadClients();
}

void MessageDispatch::PurgeDeadClients() {
    // caller must lock!
    if (busy_)
        return;

    for (auto i = clients_.begin(); i != clients_.end();) {
        Client &c = i->second;
        if (c.deleted)
            clients_.erase(i++);
        else
            ++i;
    }
}
