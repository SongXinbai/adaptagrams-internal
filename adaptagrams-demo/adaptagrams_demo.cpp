#include <iostream>
#include <vector>
#include <memory>
#include <filesystem>
#include <string>
#include <tuple>

#include "libavoid/libavoid.h"
#include "libcola/cola.h"
#include "libdialect/libdialect.h"
#include "libtopology/cola_topology_addon.h"
#include "libvpsc/rectangle.h"

namespace {
// Custom deleters so unique_ptr doesn't call the object destructor directly.
// The Avoid::Router owns ShapeRef and ConnRef objects; they must be removed
// via Router::deleteShape and Router::deleteConnector.
struct ShapeRefDeleter {
    Avoid::Router *router{};
    void operator()(Avoid::ShapeRef *s) const {
        if (router && s) {
            router->deleteShape(s);
        }
    }
};

struct ConnRefDeleter {
    Avoid::Router *router{};
    void operator()(Avoid::ConnRef *c) const {
        if (router && c) {
            router->deleteConnector(c);
        }
    }
};

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

std::unique_ptr<vpsc::Rectangle> cloneRectangle(const vpsc::Rectangle &rect)
{
    return std::make_unique<vpsc::Rectangle>(rect.getMinX(), rect.getMaxX(), rect.getMinY(), rect.getMaxY());
}

std::vector<std::unique_ptr<vpsc::Rectangle>> toRectangles(const std::vector<dialect::Node_SP> &nodes)
{
    std::vector<std::unique_ptr<vpsc::Rectangle>> rectangles;
    rectangles.reserve(nodes.size());
    for (const auto &node : nodes)
    {
        rectangles.emplace_back(std::make_unique<vpsc::Rectangle>(toRectangleFromDialect(*node)));
    }
    return rectangles;
}

void exportDiagram(const std::vector<std::unique_ptr<vpsc::Rectangle>> &rectangles,
                   const std::vector<cola::Edge> &edges,
                   const std::string &filename)
{
    Avoid::Router router(Avoid::OrthogonalRouting);
    std::vector<std::unique_ptr<Avoid::ShapeRef, ShapeRefDeleter>> shapes;
    shapes.reserve(rectangles.size());
    for (const auto &rect : rectangles)
    {
        Avoid::Polygon shapePoly = rectangleToPolygon(*rect);
        shapes.emplace_back(std::unique_ptr<Avoid::ShapeRef, ShapeRefDeleter>(
            new Avoid::ShapeRef(&router, shapePoly), ShapeRefDeleter{&router}));
    }

    std::vector<std::unique_ptr<Avoid::ConnRef, ConnRefDeleter>> connectors;
    connectors.reserve(edges.size());
    for (const auto &edge : edges)
    {
        const auto &srcRect = rectangles[edge.first];
        const auto &dstRect = rectangles[edge.second];
        Avoid::Point src(srcRect->getCentreX(), srcRect->getCentreY());
        Avoid::Point dst(dstRect->getCentreX(), dstRect->getCentreY());
        connectors.emplace_back(std::unique_ptr<Avoid::ConnRef, ConnRefDeleter>(
            new Avoid::ConnRef(&router, src, dst), ConnRefDeleter{&router}));
    }

    router.processTransaction();

    std::filesystem::create_directories("output");
    router.outputDiagram("output/" + filename);
    std::cout << "Generated output/" << filename << ".svg and .txt" << std::endl;
}
}

int main()
{
    dialect::Graph g;
    std::vector<dialect::Node_SP> dialectNodes;
    dialectNodes.reserve(12);

    const std::vector<std::tuple<double, double, double, double>> seeds = {
        {-220.0, -120.0, 34.0, 18.0}, {-120.0, -120.0, 36.0, 20.0}, {0.0, -120.0, 38.0, 18.0}, {140.0, -110.0, 30.0, 22.0},
        {-220.0, -20.0, 32.0, 18.0}, {-120.0, -30.0, 34.0, 20.0}, {0.0, -10.0, 36.0, 18.0}, {160.0, -10.0, 40.0, 24.0},
        {-220.0, 90.0, 32.0, 18.0}, {-120.0, 70.0, 34.0, 19.0}, {10.0, 80.0, 36.0, 20.0}, {170.0, 95.0, 42.0, 22.0}
    };

    for (size_t i = 0; i < seeds.size(); ++i)
    {
        double cx, cy, w, h;
        std::tie(cx, cy, w, h) = seeds[i];
        auto n = dialect::Node::allocate(w, h);
        n->setCentre(cx, cy);
        g.addNode(n);
        dialectNodes.push_back(n);
    }

    const std::vector<std::pair<unsigned, unsigned>> dialectEdges = {
        {0, 1}, {1, 2}, {2, 3},
        {0, 4}, {1, 5}, {2, 6}, {3, 7},
        {4, 5}, {5, 6}, {6, 7},
        {4, 8}, {5, 9}, {6, 10}, {7, 11},
        {8, 9}, {9, 10}, {10, 11},
        {2, 5}, {6, 3}, {1, 9}
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

    auto holaRectangles = toRectangles(dialectNodes);
    std::vector<std::unique_ptr<vpsc::Rectangle>> colaRectangles;
    colaRectangles.reserve(holaRectangles.size());
    for (const auto &rect : holaRectangles)
    {
        colaRectangles.emplace_back(cloneRectangle(*rect));
    }

    exportDiagram(holaRectangles, dialectEdges, "adaptagrams_hola_layout");

    std::vector<vpsc::Rectangle*> nodeRefs;
    nodeRefs.reserve(colaRectangles.size());
    for (const auto &n : colaRectangles)
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
    locks.push_back({6u,
                     dialectNodes[6]->getCentre().x + 80.0,
                     dialectNodes[6]->getCentre().y - 60.0,
                     1.0});

    std::vector<std::unique_ptr<cola::CompoundConstraint>> ownedConstraints;
    cola::CompoundConstraints constraints;

    auto leftColumn = std::make_unique<cola::AlignmentConstraint>(vpsc::XDIM, -220.0);
    leftColumn->addShape(0, 0.0);
    leftColumn->addShape(4, 0.0);
    leftColumn->addShape(8, 0.0);
    cola::AlignmentConstraint *leftColumnPtr = leftColumn.get();
    ownedConstraints.push_back(std::move(leftColumn));
    constraints.push_back(leftColumnPtr);

    auto middleColumn = std::make_unique<cola::AlignmentConstraint>(vpsc::XDIM, -30.0);
    middleColumn->addShape(1, 0.0);
    middleColumn->addShape(5, 0.0);
    middleColumn->addShape(9, 0.0);
    middleColumn->addShape(10, 0.0);
    cola::AlignmentConstraint *middleColumnPtr = middleColumn.get();
    ownedConstraints.push_back(std::move(middleColumn));
    constraints.push_back(middleColumnPtr);

    auto rightColumn = std::make_unique<cola::AlignmentConstraint>(vpsc::XDIM, 200.0);
    rightColumn->addShape(2, 0.0);
    rightColumn->addShape(3, 0.0);
    rightColumn->addShape(6, 0.0);
    rightColumn->addShape(7, 0.0);
    rightColumn->addShape(11, 0.0);
    cola::AlignmentConstraint *rightColumnPtr = rightColumn.get();
    ownedConstraints.push_back(std::move(rightColumn));
    constraints.push_back(rightColumnPtr);

    auto leftToMiddle = std::make_unique<cola::SeparationConstraint>(vpsc::XDIM, leftColumnPtr, middleColumnPtr, 160.0);
    constraints.push_back(leftToMiddle.get());
    ownedConstraints.push_back(std::move(leftToMiddle));

    auto middleToRight = std::make_unique<cola::SeparationConstraint>(vpsc::XDIM, middleColumnPtr, rightColumnPtr, 170.0);
    constraints.push_back(middleToRight.get());
    ownedConstraints.push_back(std::move(middleToRight));

    const std::vector<std::pair<unsigned, unsigned>> verticalOrdering = {
        {0, 4}, {4, 8},
        {1, 5}, {5, 9},
        {2, 6}, {6, 10},
        {3, 7}, {7, 11}
    };
    for (const auto &pair : verticalOrdering)
    {
        auto constraint = std::make_unique<cola::SeparationConstraint>(vpsc::YDIM, pair.first, pair.second, 70.0);
        constraints.push_back(constraint.get());
        ownedConstraints.push_back(std::move(constraint));
    }

    const double idealLength = 110.0;
    cola::ConstrainedFDLayout layout(nodeRefs, edges, idealLength, cola::StandardEdgeLengths, nullptr, nullptr);
    layout.setAvoidNodeOverlaps(true);
    layout.setConstraints(constraints);
    layout.setDesiredPositions(&locks);
    layout.setTopology(&topologyAddon);
    layout.run();

    std::cout << "[libcola + libtopology] Topology-preserved node centers:" << std::endl;
    for (size_t i = 0; i < colaRectangles.size(); ++i)
    {
        std::cout << "  Node " << i << ": ("
                  << colaRectangles[i]->getCentreX() << ", "
                  << colaRectangles[i]->getCentreY() << ")" << std::endl;
    }

    exportDiagram(colaRectangles, edges, "adaptagrams_constrained_layout");

    return 0;
}
