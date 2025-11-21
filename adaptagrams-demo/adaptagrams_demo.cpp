#include <iostream>
#include <vector>
#include <memory>
#include <filesystem>

#include "libavoid/libavoid.h"
#include "libcola/cola.h"
#include "libdialect/libdialect.h"
#include "libtopology/cola_topology_addon.h"
#include "libvpsc/rectangle.h"

namespace {
Avoid::Polygon rectangleToPolygon(const vpsc::Rectangle &rect)
{
    Avoid::Polygon polygon(4);
    polygon.ps[0] = Avoid::Point(rect.getMinX(), rect.getMinY());
    polygon.ps[1] = Avoid::Point(rect.getMaxX(), rect.getMinY());
    polygon.ps[2] = Avoid::Point(rect.getMaxX(), rect.getMaxY());
    polygon.ps[3] = Avoid::Point(rect.getMinX(), rect.getMaxY());
    return polygon;
}

vpsc::Rectangle toRectangleFromDialect(const dialect::Node &node)
{
    const auto dims = node.getDimensions();
    const auto centre = node.getCentre();
    const double halfW = dims.first / 2.0;
    const double halfH = dims.second / 2.0;
    return vpsc::Rectangle(centre.x - halfW, centre.x + halfW, centre.y - halfH, centre.y + halfH);
}
}

int main()
{
    dialect::Graph g;
    std::vector<dialect::Node_SP> dialectNodes;
    dialectNodes.reserve(5);

    for (int i = 0; i < 5; ++i)
    {
        auto n = dialect::Node::allocate(26.0 + i * 2, 18.0 + i);
        n->setCentre(i * 40.0, i * 5.0);
        g.addNode(n);
        dialectNodes.push_back(n);
    }

    const std::vector<std::pair<unsigned, unsigned>> dialectEdges = {
        {0, 1}, {1, 2}, {2, 3}, {1, 4}
    };
    for (const auto &e : dialectEdges)
    {
        g.addEdge(dialectNodes[e.first], dialectNodes[e.second]);
    }

    dialect::doHOLA(g);

    std::cout << "[libdialect] HOLA layout node centers:" << std::endl;
    for (size_t i = 0; i < dialectNodes.size(); ++i)
    {
        auto centre = dialectNodes[i]->getCentre();
        std::cout << "  Node " << i << ": (" << centre.x << ", " << centre.y << ")" << std::endl;
    }

    std::vector<std::unique_ptr<vpsc::Rectangle>> nodes;
    nodes.reserve(dialectNodes.size());
    for (const auto &n : dialectNodes)
    {
        nodes.emplace_back(std::make_unique<vpsc::Rectangle>(toRectangleFromDialect(*n)));
    }

    std::vector<vpsc::Rectangle*> nodeRefs;
    nodeRefs.reserve(nodes.size());
    for (const auto &n : nodes)
    {
        nodeRefs.push_back(n.get());
    }

    std::vector<cola::Edge> edges;
    edges.reserve(dialectEdges.size());
    for (const auto &e : dialectEdges)
    {
        edges.emplace_back(e.first, e.second);
    }

    topology::ColaTopologyAddon topologyAddon;

    cola::DesiredPositions locks;
    // DesiredPosition is an aggregate with fields {id, x, y, weight}
    locks.push_back({3u,
                     dialectNodes[3]->getCentre().x + 120.0,
                     dialectNodes[3]->getCentre().y - 40.0,
                     1.0});

    const double idealLength = 90.0;
    cola::ConstrainedFDLayout layout(nodeRefs, edges, idealLength, cola::StandardEdgeLengths, nullptr, nullptr);
    layout.setDesiredPositions(&locks);
    layout.setTopology(&topologyAddon);
    layout.run();

    std::cout << "[libcola + libtopology] Topology-preserved node centers:" << std::endl;
    for (size_t i = 0; i < nodes.size(); ++i)
    {
        std::cout << "  Node " << i << ": ("
                  << nodes[i]->getCentreX() << ", "
                  << nodes[i]->getCentreY() << ")" << std::endl;
    }

    Avoid::Router router(Avoid::PolyLineRouting);
    std::vector<Avoid::ShapeRef*> shapes;
    shapes.reserve(nodes.size());
       for (const auto &rect : nodes)
       {
           Avoid::Polygon shapePoly = rectangleToPolygon(*rect);
           Avoid::ShapeRef *s = new Avoid::ShapeRef(&router, shapePoly);
           shapes.push_back(s);
    }

    std::vector<Avoid::ConnRef*> connectors;
    connectors.reserve(edges.size());
    for (const auto &edge : edges)
    {
        const auto &srcRect = nodes[edge.first];
        const auto &dstRect = nodes[edge.second];
        Avoid::Point src(srcRect->getCentreX(), srcRect->getCentreY());
        Avoid::Point dst(dstRect->getCentreX(), dstRect->getCentreY());
           Avoid::ConnRef *c = new Avoid::ConnRef(&router, src, dst);
           connectors.push_back(c);
    }

    router.processTransaction();

    std::filesystem::create_directories("output");
    router.outputDiagram("output/adaptagrams_demo");
    std::cout << "Generated output/adaptagrams_demo.svg and .txt" << std::endl;

    return 0;
}