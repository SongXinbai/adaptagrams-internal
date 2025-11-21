#ifndef ADAPTAGRAMS_H
#define ADAPTAGRAMS_H

#include <vector>
#include <memory>
#include "libavoid/libavoid.h"
#include "libcola/cola.h"
#include "libdialect/libdialect.h"
#include "libtopology/cola_topology_addon.h"
#include "libvpsc/rectangle.h"

namespace adaptagrams {

class Adaptagram {
public:
    Adaptagram();
    void generateLayout(const std::vector<dialect::Node_SP>& nodes, const std::vector<std::pair<unsigned, unsigned>>& edges);
    void exportToSVG(const std::string& filename);

private:
    std::vector<std::unique_ptr<vpsc::Rectangle>> nodeRectangles;
    std::vector<cola::Edge> colaEdges;
    topology::ColaTopologyAddon topologyAddon;

    Avoid::Router router;
    void convertNodesToRectangles(const std::vector<dialect::Node_SP>& nodes);
    void createEdges(const std::vector<std::pair<unsigned, unsigned>>& edges);
};

} // namespace adaptagrams

#endif // ADAPTAGRAMS_H