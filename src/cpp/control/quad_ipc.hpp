#pragma once

#include "iox2/iceoryx2.hpp"

// TODO: Rewrite

/*
*Cannot create 1 class for each publisher-subscribe paradigm
*maintaining zero copying when subscribing to service
*multi-service subscribing
*static initialization of services

*more simple main.cpp API
*Handle IPC errors within the IPC send() or receive() calls
*

*/



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


template <typename T>
class QuadIpcSubscriber {
    public:
        QuadIpcSubscriber(const char* node_name, const char* service_name) :

            _node (iox2::NodeBuilder().name(iox2::NodeName::create(node_name).value())
                                       .create<iox2::ServiceType::Ipc>().value()),

            _service (_node.service_builder(iox2::ServiceName::create(service_name).value())
                            .publish_subscribe<T>()
                            .open_or_create()
                            .value()),
            
            _subscriber (_service.subscriber_builder().create().value())
        {}

        bool wait(int milli) {
            return _node.wait(iox2::bb::Duration::from_millis(milli)).has_value();
        }


        // Note: must assign and check if sample_opt has value
        auto receive() -> decltype(std::move(_subscriber.receive().value())) {
            auto receive_result = _subscriber.receive();
            if (receive_result.has_value()) {
                return std::move(receive_result.value());
            }
            return decltype(std::move(_subscriber.receive().value())){};
        }

    private:
        iox2::Node<iox2::ServiceType::Ipc> _node;
        iox2::PortFactoryPublishSubscribe<iox2::ServiceType::Ipc, T, void> _service;
        iox2::Subscriber<iox2::ServiceType::Ipc, T, void> _subscriber;
};


template <typename TReceive, typename TSend>
class QuadIpcPublisherSubscriber {
    public:
        QuadIpcPublisherSubscriber(const char* node_name, 
            const char* subscriber_service, const char* publisher_service) :

            _node (iox2::NodeBuilder().name(iox2::NodeName::create(node_name).value())
                                       .create<iox2::ServiceType::Ipc>().value()),

            _subscriber_service (_node.service_builder(iox2::ServiceName::create(subscriber_service).value())
                            .publish_subscribe<TReceive>()
                            .open_or_create()
                            .value()),
            
            _subscriber (_subscriber_service.subscriber_builder().create().value()),

            _publisher_service (_node.service_builder(iox2::ServiceName::create(publisher_service).value())
                            .publish_subscribe<TSend>()
                            .open_or_create()
                            .value()),
            
            _publisher (_publisher_service.publisher_builder().create().value())
        {}

        bool wait(int milli) {
            return _node.wait(iox2::bb::Duration::from_millis(milli)).has_value();
        }

        // Note: must assign and check if sample_opt has value
        auto receive() -> decltype(std::move(_subscriber.receive().value())) {
            auto receive_result = _subscriber.receive();
            if (receive_result.has_value()) {
                return std::move(receive_result.value());
            }
            return decltype(std::move(_subscriber.receive().value())){};
        }
const 

        void send(const TSend& payload) {
            auto sample = _publisher.loan_uninit().value();
            auto initialized_payload = sample.write_payload(payload);
            iox2::send(std::move(initialized_payload)).value();
        }

    private:
        iox2::Node<iox2::ServiceType::Ipc> _node;
        iox2::PortFactoryPublishSubscribe<iox2::ServiceType::Ipc, TReceive, void> _subscriber_service;
        iox2::Subscriber<iox2::ServiceType::Ipc, TReceive, void> _subscriber;
        iox2::PortFactoryPublishSubscribe<iox2::ServiceType::Ipc, TSend, void> _publisher_service;
        iox2::Publisher<iox2::ServiceType::Ipc, TSend, void> _publisher;
};


inline iox2::Node<iox2::ServiceType::Ipc> make_node(const char* node_name) {
    return iox2::NodeBuilder().name(iox2::NodeName::create(node_name).value())
                                    .create<iox2::ServiceType::Ipc>().value();
}

template<typename T>
inline iox2::PortFactoryPublishSubscribe<iox2::ServiceType::Ipc, T, void>
    make_service(const char* service_name, iox2::Node<iox2::ServiceType::Ipc> node) {
        return node.service_builder(ServiceName::create(service_name).value())
                    .publish_subscribe<T>()
                    .open_or_create()
                    .value();
    }

template<typename T>
inline iox2::Subscriber<iox2::ServiceType::Ipc, T, void>
    make_subscriber(const char* subscriber_name, iox2::PortFactoryPublishSubscribe<iox2::ServiceType::Ipc, T, void> service_name) {
        return service_name.subscriber_builder().create().value();
    }

template<typename T>
inline iox2::Subscriber<iox2::ServiceType::Ipc, T, void>
    make_publisher(const char* publisher_name, iox2::PortFactoryPublishSubscribe<iox2::ServiceType::Ipc, T, void> service_name) {
        return service_name.publisher_builder().create().value();
    }

// Receives a [Sample] from [Publisher]. If no sample could be
// received [None] is returned. If a failure occurs [ReceiveError] is returned.
template<typename T>
bool receive_sample(iox2::Subscriber<iox2::ServiceType::Ipc, T, void> subscriber, T& data_out) {
        auto receive_result = subscriber.receive();
        if (!receive_result.has_value()) {
            std::cerr << "IPC Error: " << static_cast<int>(receive_result.error()) << "\n";
            return false; 
        }
        auto sample_opt = std::move(receive_result.value());
    }


inline auto iox2::Subscriber<iox2::ServiceType::Ipc, T, void>::receive() const->iox2::bb::Expected<iox2::bb::Optional<iox2::Sample<iox2::ServiceType::Ipc, T, void>>, iox2::ReceiveError>

std::remove_reference<iox2::bb::Optional<iox2::Sample<iox2::ServiceType::Ipc, T, void>> &>::type = iox2::bb::Optional<iox2::Sample<iox2::ServiceType::Ipc, T, void>> 

// Todo: add inline function checking for receive and sending, including errors
