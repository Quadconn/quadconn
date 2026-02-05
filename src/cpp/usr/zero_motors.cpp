#include <iostream>
#include <unistd.h>
#include <vector>
#include <memory>
#include <chrono>
#include <cmath>
#include <map>

#include "moteus.h"
#include "motor_diagnostics.hpp"



// Convert radians to turns
double rad2turns(double radians) {
    return 9*(radians / (2.0 * M_PI));
}

int main(int argc, char** argv) {
    using namespace mjbots;

    // Define initial positions, indexed as N+1 as nodes
    std::array<double, MOTOR_COUNT> zero_positions = 
    {rad2turns(0.0),               rad2turns(0.0),      rad2turns(M_PI / 2),
     rad2turns(M_PI),              rad2turns(0.0),      rad2turns(0.0),
     rad2turns(0.0),               rad2turns(0.0),      rad2turns(0.0),
     rad2turns(0.0),               rad2turns(0.0),      rad2turns(0.0)};

    // TODO: define once across motor interface and zeroing cpp files
    auto bus_a = std::make_shared<moteus::Fdcanusb>("/dev/serial/by-id/usb-mjbots_fdcanusb_188998B3-if00");
    auto bus_b = std::make_shared<moteus::Fdcanusb>("/dev/serial/by-id/usb-mjbots_fdcanusb_9C92C905-if00");
    std::map<int, std::shared_ptr<moteus::Fdcanusb>> id_to_bus = {
        {4, std::move(bus_a)},
        {3, std::move(bus_b)}
    };

    // Initialize Controllers
    std::vector<std::shared_ptr<moteus::Controller>> controllers;
    for (auto const& [id, bus] : id_to_bus) {
        moteus::Controller::Options options{};
        options.id = id;
        options.transport = bus; 
        controllers.push_back(std::make_shared<moteus::Controller>(options));
    }

    // Stop everything to clear faults first
    for (auto& c : controllers) { c->SetStop(); }

    // Execution loop
    std::cout << "starting calibration sequence\n";
        
    for (auto& c : controllers) {
        // retrieve CAN id and corresponding physical angle position
        int id = c->options().id;
        int idx = id - 1;


        // 2. Format the diagnostic strings
        // d exact: sets the current runtime position to the target
        std::string exact_cmd = "d exact " + std::to_string(zero_positions[idx]);
        // d cfg-set-output: updates the 'motor_position.sources.0.offset' config
        std::string sync_cmd = "d cfg-set-output " + std::to_string(zero_positions[idx]);
        
        try {
            std::cout << "Calibrating Motor " << id << " to " << zero_positions[idx] << " turns...\n";
            
            // Execute the sequence
            c->DiagnosticCommand(exact_cmd);    // Set runtime position
            c->DiagnosticCommand(sync_cmd);     // Sync config offset
            c->DiagnosticCommand("conf write"); // Commit to non-volatile flash
            
            std::cout << "Motor " << id << " saved successfully.\n";
        } catch (const std::exception& e) {
            std::cerr << "Failed to calibrate motor " << id << ": " << e.what() << "\n";
        }
    }
            
    // End safely
    for (auto& c : controllers) { c->SetStop(); }
    return 0;
}

