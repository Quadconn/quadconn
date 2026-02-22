#include "iox2/iceoryx2.hpp"
#include <optional>

// Make Node (Returns by value via copy elision/move semantics)
inline iox2::Node<iox2::ServiceType::Ipc> make_node(const char* node_name) {
    return iox2::NodeBuilder().name(iox2::NodeName::create(node_name).value())
                              .create<iox2::ServiceType::Ipc>().value();
}

// Make Service (Pass Node by lvalue reference '&' to avoid copying)
template<typename T>
inline iox2::PortFactoryPublishSubscribe<iox2::ServiceType::Ipc, T, void>
    make_service(const char* service_name, iox2::Node<iox2::ServiceType::Ipc>& node) {
        return node.service_builder(iox2::ServiceName::create(service_name).value())
                    .publish_subscribe<T>()
                    .open_or_create()
                    .value();
    }

//  Make Subscriber (Pass Service by rvalue reference '&&' to support inline chaining)
template<typename T>
inline iox2::Subscriber<iox2::ServiceType::Ipc, T, void>
    make_subscriber(iox2::PortFactoryPublishSubscribe<iox2::ServiceType::Ipc, T, void>&& service) {
        return service.subscriber_builder().create().value();
    }

//  Make Publisher (Pass Service by rvalue reference '&&' to support inline chaining)
template<typename T>
inline iox2::Publisher<iox2::ServiceType::Ipc, T, void>
    make_publisher(iox2::PortFactoryPublishSubscribe<iox2::ServiceType::Ipc, T, void>&& service) {
        return service.publisher_builder().create().value();
    }

// wrapper for waiting in node for a specifies millisecond duration. Must pass both milliseconds as double
// and node as reference
inline bool loop_waitms(double ms, iox2::Node<iox2::ServiceType::Ipc>& node) {
    return node.wait(iox2::bb::Duration::from_millis(ms)).has_value();
}

// simple wrapper for sending data using a publisher. Note: this will pass the data as value, leaving 
// the local copy of data the same
template<typename T> 
inline void ipc_send(const T& data, iox2::Publisher<iox2::ServiceType::Ipc, T, void>& publisher) {
    auto sample = publisher.loan_uninit().value();
    auto initialized_sample =
        sample.write_payload(data); 
    iox2::send(std::move(initialized_sample)).value();
}


// simple wrapper for receiving data. Safely unwraps the nested iceoryx2 types.
template<typename T>
inline std::optional<T> ipc_receive(iox2::Subscriber<iox2::ServiceType::Ipc, T, void>& subscriber) {
    auto receive_result = subscriber.receive();
    
    // Check if the IPC mechanism threw an error
    if (receive_result.has_error()) {
        return std::nullopt;
    }
    
    auto sample_opt = receive_result.value();
    
    // Check if a new message was actually in the queue
    if (sample_opt.has_value()) {
        // Extract the payload and copy it into the std::optional
        return std::optional<T>{sample_opt.value().payload()};
    }
    // Queue was empty
    return std::nullopt;
}
