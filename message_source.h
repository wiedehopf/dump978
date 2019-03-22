// -*- c++ -*-

// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#ifndef DUMP978_MESSAGE_SOURCE_H
#define DUMP978_MESSAGE_SOURCE_H

#include "uat_message.h"

namespace flightaware::uat {
    class MessageSource {
      public:
        typedef std::function<void(SharedMessageVector)> Consumer;

        virtual ~MessageSource() {}

        void SetConsumer(Consumer consumer) { consumer_ = consumer; }

      protected:
        void DispatchMessages(SharedMessageVector messages) {
            if (consumer_) {
                consumer_(messages);
            }
        }

      private:
        Consumer consumer_;
    };
}; // namespace flightaware::uat

#endif
