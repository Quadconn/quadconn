// Source - https://stackoverflow.com/a/24560310
// Posted by selbie, modified by community. See post 'Timeline' for change history
// Retrieved 2026-02-06, License - CC BY-SA 4.0

#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <memory.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <errno.h>
#include <stdlib.h>
#include <iostream>

#include "motor_diagnostics.hpp"
#include "../common/quad_ipc.hpp"

const char* OPERATOR_IP = "100.119.158.85";
const char* OPERATOR_PORT = "808";
constexpr iox2::bb::Duration UPDATE_RATE = iox2::bb::Duration::from_millis(1000);

float randomFloat()
{
    return (float)(rand()) / (float)(rand());
}


int resolvehelper(const char* hostname, int family, const char* service, sockaddr_storage* pAddr)
{
    int result;
    addrinfo* result_list = NULL;
    addrinfo hints = {};
    hints.ai_family = family;
    hints.ai_socktype = SOCK_DGRAM; // without this flag, getaddrinfo will return 3x the number of addresses (one for each socket type).
    result = getaddrinfo(hostname, service, &hints, &result_list);
    if (result == 0)
    {
        //ASSERT(result_list->ai_addrlen <= sizeof(sockaddr_in));
        memcpy(pAddr, result_list->ai_addr, result_list->ai_addrlen);
        freeaddrinfo(result_list);
    }

    return result;
}


int main()
{
    using namespace iox2;
    auto node = NodeBuilder()
            .name(NodeName::create("udp_sender")
            .value()).create<ServiceType::Ipc>().value();
    
    int result = 0;
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    char szIP[100];

    sockaddr_in addrListen = {}; // zero-int, sin_port is 0, which picks a random port for bind.
    addrListen.sin_family = AF_INET;
    result = bind(sock, (sockaddr*)&addrListen, sizeof(addrListen));
    if (result == -1)
    {
       int lasterror = errno;
       std::cout << "error: " << lasterror;
       exit(1);
    }


    sockaddr_storage addrDest = {};
    
    // just hardcoded a tailscale IP because I cannot be bothered 
    result = resolvehelper(OPERATOR_IP, AF_INET, OPERATOR_PORT, &addrDest);
    if (result != 0)
    {
       int lasterror = errno;
       std::cout << "error: " << lasterror;
       exit(1);
    }

    while (node.wait(UPDATE_RATE).has_value()) {


        MotorDiagnosticsArray dummy_data = {};
        for (int i = 0; i < MOTOR_COUNT; i++) {
            dummy_data.motor_instance[i].abs_position = randomFloat();
            dummy_data.motor_instance[i].d_current = randomFloat();
            dummy_data.motor_instance[i].fault = rand();
            dummy_data.motor_instance[i].mode = rand();
            dummy_data.motor_instance[i].motor_temperature = randomFloat();
            dummy_data.motor_instance[i].position = randomFloat();
            dummy_data.motor_instance[i].power = randomFloat();
            dummy_data.motor_instance[i].q_current = randomFloat();
            dummy_data.motor_instance[i].temperature = randomFloat();
            dummy_data.motor_instance[i].torque = randomFloat();
            dummy_data.motor_instance[i].trajectory_complete = rand() % 2;
            dummy_data.motor_instance[i].velocity = randomFloat();
            dummy_data.motor_instance[i].voltage = randomFloat();
        }

        ssize_t sent_bytes = sendto(sock, &dummy_data, sizeof(dummy_data), 0, 
                                (struct sockaddr*)&addrDest, sizeof(addrDest));

        std::cout << sent_bytes << " bytes sent" << std::endl;
    }

    
    return 0;

}
