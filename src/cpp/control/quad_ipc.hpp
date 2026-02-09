#pragma once

#include "iox2/iceoryx2.hpp"

template <typename T>
class QuadIpcPublisher {
    public:
        QuadIpcPublisher(const char* node_name, const char* service_name) :

            _node (iox2::NodeBuilder().name(iox2::NodeName::create(node_name).value())
                                       .create<iox2::ServiceType::Ipc>().value()),

            _service (_node.service_builder(iox2::ServiceName::create(service_name).value())
                            .publish_subscribe<T>()
                            .open_or_create()
                            .value()),
            
            _publisher (_service.publisher_builder().create().value())
        {}

        bool wait(int milli) {
            return _node.wait(iox2::bb::Duration::from_millis(milli)).has_value();
        }

        void send(const T& payload) {
            auto sample = _publisher.loan_uninit().value();
            auto initialized_payload = sample.write_payload(payload);

            iox2::send(std::move(initialized_payload)).value();
        }

    private:
        iox2::Node<iox2::ServiceType::Ipc> _node;
        iox2::PortFactoryPublishSubscribe<iox2::ServiceType::Ipc, T, void> _service;
        iox2::Publisher<iox2::ServiceType::Ipc, T, void> _publisher;
};
