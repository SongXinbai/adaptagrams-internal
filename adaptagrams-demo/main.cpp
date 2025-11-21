#define SVG_OUTPUT

#include <iostream>
#include "adaptagrams.h"

int main() {
    std::cout << "Starting the Adaptagrams demo application..." << std::endl;

    // Initialize and run the demo from adaptagrams_demo.cpp
    adaptagrams::Adaptagram demo;
    std::vector<dialect::Node_SP> nodes;
    std::vector<std::pair<unsigned, unsigned>> edges;
    demo.generateLayout(nodes, edges);
    demo.exportToSVG("adaptagrams_demo_output.svg");

    std::cout << "Demo completed." << std::endl;
    return 0;
}