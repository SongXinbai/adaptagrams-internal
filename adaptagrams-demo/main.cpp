#include <iostream>
#include <vector>
#include <unordered_map>
#include <memory>
#include <cmath>
#include <fstream>
#include <algorithm>

#include <libcola/cola.h>
#include <libavoid/router.h>
#include <libavoid/connector.h>
#include <libavoid/connend.h>
#include <libavoid/connectionpin.h>
#include <libvpsc/rectangle.h>

// Simple demo of using Adaptagrams (libcola + libavoid) to layout
// and route a tiny orthogonal diagram. The goal is readability rather
// than raw performance or exhaustive API coverage.  We intentionally
// use libcola instead of libdialect's HOLA because this example leans on
// ideal-position hints and explicit separation constraints; HOLA is
// excellent for quick, small topologies without positional guidance, but
// cola gives us the soft constraints we need here.

enum class NodeType { BigRect, SmallRect, Square };
enum class EdgeType { Overhead, Normal };

struct Node {
    int id;
    NodeType type;
    double width;
    double height;
    bool hasIdealPos;
    double idealX;
    double idealY;
    double x = 0.0; // computed centre X
    double y = 0.0; // computed centre Y
};

struct Edge {
    int id;
    int sourceId;
    int targetId;
    EdgeType type;
};

struct Port {
    int id;
    int nodeId;
    unsigned classId; // Avoid pin class
    Avoid::Point position; // absolute position for SVG markers
};

struct RoutedEdge {
    int edgeId;
    EdgeType type;
    std::vector<std::pair<double, double>> points;
};

using NodeIndexMap = std::unordered_map<int, std::size_t>;

NodeIndexMap buildNodeIndexMap(const std::vector<Node>& nodes) {
    NodeIndexMap map;
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        map[nodes[i].id] = i;
    }
    return map;
}

std::vector<Edge> buildSampleGraph(std::vector<Node>& nodes) {
    nodes.clear();
    int id = 0;
    // One big hub with ideal position.
    nodes.push_back({id++, NodeType::BigRect, 140.0, 90.0, true, 0.0, 0.0});
    // Small rectangles.
    nodes.push_back({id++, NodeType::SmallRect, 90.0, 50.0, true, 220.0, 10.0});
    nodes.push_back({id++, NodeType::SmallRect, 90.0, 50.0, false, 0.0, 0.0});
    nodes.push_back({id++, NodeType::SmallRect, 90.0, 50.0, true, -200.0, -40.0});
    // Squares.
    nodes.push_back({id++, NodeType::Square, 60.0, 60.0, true, -40.0, 220.0});
    nodes.push_back({id++, NodeType::Square, 60.0, 60.0, false, 0.0, 0.0});
    nodes.push_back({id++, NodeType::Square, 60.0, 60.0, true, 260.0, 220.0});

    std::vector<Edge> edges;
    int eid = 0;
    // Overhead edges try to avoid crossings strongly.
    edges.push_back({eid++, nodes[0].id, nodes[1].id, EdgeType::Overhead});
    edges.push_back({eid++, nodes[0].id, nodes[4].id, EdgeType::Overhead});

    // Normal edges fill in the rest.
    edges.push_back({eid++, nodes[1].id, nodes[2].id, EdgeType::Normal});
    edges.push_back({eid++, nodes[1].id, nodes[3].id, EdgeType::Normal});
    edges.push_back({eid++, nodes[2].id, nodes[5].id, EdgeType::Normal});
    edges.push_back({eid++, nodes[3].id, nodes[4].id, EdgeType::Normal});
    edges.push_back({eid++, nodes[4].id, nodes[6].id, EdgeType::Normal});
    edges.push_back({eid++, nodes[0].id, nodes[6].id, EdgeType::Normal});
    return edges;
}

void layoutNodesWithCola(std::vector<Node>& nodes, const std::vector<Edge>& edges) {
    const double minSpacing = 30.0;
    NodeIndexMap indexMap = buildNodeIndexMap(nodes);

    std::vector<std::unique_ptr<vpsc::Rectangle>> storage;
    std::vector<vpsc::Rectangle*> rectRefs;
    storage.reserve(nodes.size());
    rectRefs.reserve(nodes.size());

    for (const auto& n : nodes) {
        const double cx = n.hasIdealPos ? n.idealX : 0.0;
        const double cy = n.hasIdealPos ? n.idealY : 0.0;
        const double halfW = (n.width + minSpacing) / 2.0;
        const double halfH = (n.height + minSpacing) / 2.0;
        storage.emplace_back(std::make_unique<vpsc::Rectangle>(
            cx - halfW, cx + halfW, cy - halfH, cy + halfH));
        rectRefs.push_back(storage.back().get());
    }

    std::vector<cola::Edge> colaEdges;
    colaEdges.reserve(edges.size());
    for (const auto& e : edges) {
        colaEdges.emplace_back(indexMap[e.sourceId], indexMap[e.targetId]);
    }

    cola::DesiredPositions desiredPositions;
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        if (nodes[i].hasIdealPos) {
            desiredPositions.push_back({static_cast<unsigned>(i), nodes[i].idealX, nodes[i].idealY, 1.0});
        }
    }

    cola::CompoundConstraints rawConstraints;
    std::vector<std::unique_ptr<cola::CompoundConstraint>> constraintStorage;

    // Preserve rough left/right/up/down directions relative to the hub (node 0) if hints are present.
    const std::size_t hubIndex = 0;
    for (std::size_t i = 1; i < nodes.size(); ++i) {
        if (!nodes[i].hasIdealPos || !nodes[hubIndex].hasIdealPos) {
            continue;
        }
        const double dx = nodes[i].idealX - nodes[hubIndex].idealX;
        const double dy = nodes[i].idealY - nodes[hubIndex].idealY;
        if (dx > 0.0) {
            constraintStorage.emplace_back(std::make_unique<cola::SeparationConstraint>(
                vpsc::XDIM, static_cast<unsigned>(hubIndex), static_cast<unsigned>(i), minSpacing));
            rawConstraints.push_back(constraintStorage.back().get());
        } else if (dx < 0.0) {
            constraintStorage.emplace_back(std::make_unique<cola::SeparationConstraint>(
                vpsc::XDIM, static_cast<unsigned>(i), static_cast<unsigned>(hubIndex), minSpacing));
            rawConstraints.push_back(constraintStorage.back().get());
        }
        if (dy > 0.0) {
            constraintStorage.emplace_back(std::make_unique<cola::SeparationConstraint>(
                vpsc::YDIM, static_cast<unsigned>(hubIndex), static_cast<unsigned>(i), minSpacing));
            rawConstraints.push_back(constraintStorage.back().get());
        } else if (dy < 0.0) {
            constraintStorage.emplace_back(std::make_unique<cola::SeparationConstraint>(
                vpsc::YDIM, static_cast<unsigned>(i), static_cast<unsigned>(hubIndex), minSpacing));
            rawConstraints.push_back(constraintStorage.back().get());
        }
    }

    const double idealEdgeLength = 140.0;
    cola::ConstrainedFDLayout layout(rectRefs, colaEdges, idealEdgeLength);
    layout.setDesiredPositions(&desiredPositions);
    layout.setConstraints(rawConstraints);
    layout.setAvoidNodeOverlaps(true);
    layout.run();

    for (std::size_t i = 0; i < nodes.size(); ++i) {
        nodes[i].x = rectRefs[i]->getCentreX();
        nodes[i].y = rectRefs[i]->getCentreY();
    }
}

std::vector<Port> createPortsForNodes(const std::vector<Node>& nodes,
                                      const std::vector<Edge>& edges,
                                      Avoid::Router& router,
                                      std::vector<Avoid::ShapeRef*>& shapes) {
    std::unordered_map<int, int> degree;
    for (const auto& e : edges) {
        degree[e.sourceId]++;
        degree[e.targetId]++;
    }

    std::vector<Port> ports;
    unsigned nextClassId = 1; // Avoid pins must be non-zero

    shapes.reserve(nodes.size());
    for (const auto& node : nodes) {
        const double left = node.x - node.width / 2.0;
        const double right = node.x + node.width / 2.0;
        const double top = node.y - node.height / 2.0;
        const double bottom = node.y + node.height / 2.0;

        Avoid::Polygon poly(4);
        poly.ps[0] = Avoid::Point(left, top);
        poly.ps[1] = Avoid::Point(right, top);
        poly.ps[2] = Avoid::Point(right, bottom);
        poly.ps[3] = Avoid::Point(left, bottom);

        auto* shapeRef = new Avoid::ShapeRef(&router, poly, static_cast<unsigned>(node.id + 1));
        shapes.push_back(shapeRef);

        const int count = std::max(2, degree[node.id]);
        for (int i = 0; i < count; ++i) {
            double xPortion = 0.5;
            double yPortion = 0.5;
            switch (node.type) {
            case NodeType::BigRect:
                xPortion = 1.0; // right side
                yPortion = (i + 1.0) / (count + 1.0);
                break;
            case NodeType::SmallRect:
                xPortion = (i + 1.0) / (count + 1.0);
                yPortion = 0.0; // top side
                break;
            case NodeType::Square:
                xPortion = 0.0; // left side
                yPortion = (i + 1.0) / (count + 1.0);
                break;
            }
            auto* pin = new Avoid::ShapeConnectionPin(
                shapeRef, nextClassId, xPortion, yPortion, true, 0.0,
                Avoid::ConnDirAll);

            const double px = left + xPortion * node.width;
            const double py = top + yPortion * node.height;
            ports.push_back({static_cast<int>(ports.size()), node.id, nextClassId, Avoid::Point(px, py)});
            ++nextClassId;
            (void)pin; // Pins are owned by the router/shape, no extra bookkeeping needed.
        }
    }

    return ports;
}

std::vector<RoutedEdge> routeEdgesWithAvoid(const std::vector<Node>& nodes,
                                            const std::vector<Edge>& edges,
                                            const std::vector<Port>& ports,
                                            const std::vector<Avoid::ShapeRef*>& shapes,
                                            Avoid::Router& router) {
    NodeIndexMap indexMap = buildNodeIndexMap(nodes);

    // Build lookup from nodeId -> list of port class IDs for round-robin use.
    std::unordered_map<int, std::vector<unsigned>> portsPerNode;
    for (const auto& p : ports) {
        portsPerNode[p.nodeId].push_back(p.classId);
    }
    std::unordered_map<int, std::size_t> portOffsets;

    auto nextPortFor = [&](int nodeId) {
        auto& list = portsPerNode[nodeId];
        auto& idx = portOffsets[nodeId];
        if (list.empty()) {
            return 0u;
        }
        unsigned id = list[idx % list.size()];
        ++idx;
        return id;
    };

    std::vector<std::pair<int, Avoid::ConnRef*>> overheadConns;
    std::vector<std::pair<int, Avoid::ConnRef*>> normalConns;

    router.setRoutingParameter(Avoid::segmentPenalty, 30.0);
    router.setRoutingParameter(Avoid::idealNudgingDistance, 10.0);
    router.setRoutingParameter(Avoid::shapeBufferDistance, 6.0);

    // Phase 1: overhead edges with stronger crossing penalty.
    router.setRoutingParameter(Avoid::crossingPenalty, 600.0);
    for (const auto& e : edges) {
        if (e.type != EdgeType::Overhead) continue;
        unsigned srcClass = nextPortFor(e.sourceId);
        unsigned tgtClass = nextPortFor(e.targetId);
        Avoid::ConnEnd src(shapes[indexMap[e.sourceId]], srcClass);
        Avoid::ConnEnd dst(shapes[indexMap[e.targetId]], tgtClass);
        auto* conn = new Avoid::ConnRef(&router, src, dst, static_cast<unsigned>(e.id + 100));
        overheadConns.push_back({e.id, conn});
    }
    router.processTransaction();

    // Phase 2: normal edges with balanced penalties.
    router.setRoutingParameter(Avoid::crossingPenalty, 200.0);
    for (const auto& e : edges) {
        if (e.type != EdgeType::Normal) continue;
        unsigned srcClass = nextPortFor(e.sourceId);
        unsigned tgtClass = nextPortFor(e.targetId);
        Avoid::ConnEnd src(shapes[indexMap[e.sourceId]], srcClass);
        Avoid::ConnEnd dst(shapes[indexMap[e.targetId]], tgtClass);
        auto* conn = new Avoid::ConnRef(&router, src, dst, static_cast<unsigned>(e.id + 200));
        normalConns.push_back({e.id, conn});
    }
    router.processTransaction();

    std::vector<RoutedEdge> routed;
    routed.reserve(edges.size());
    auto extractPoints = [](const Avoid::PolyLine& line) {
        std::vector<std::pair<double, double>> pts;
        pts.reserve(line.size());
        for (const auto& p : line.ps) {
            pts.emplace_back(p.x, p.y);
        }
        return pts;
    };

    auto recordRoutes = [&](const std::vector<std::pair<int, Avoid::ConnRef*>>& conns, EdgeType type) {
        for (const auto& pair : conns) {
            routed.push_back({pair.first, type, extractPoints(pair.second->displayRoute())});
        }
    };

    recordRoutes(overheadConns, EdgeType::Overhead);
    recordRoutes(normalConns, EdgeType::Normal);
    return routed;
}

static bool segmentsIntersect(const std::pair<double, double>& a1,
                              const std::pair<double, double>& a2,
                              const std::pair<double, double>& b1,
                              const std::pair<double, double>& b2) {
    auto cross = [](double x1, double y1, double x2, double y2) {
        return x1 * y2 - y1 * x2;
    };
    const double dxa = a2.first - a1.first;
    const double dya = a2.second - a1.second;
    const double dxb = b2.first - b1.first;
    const double dyb = b2.second - b1.second;

    const double denom = cross(dxa, dya, dxb, dyb);
    if (std::fabs(denom) < 1e-9) return false; // Parallel

    const double dx = b1.first - a1.first;
    const double dy = b1.second - a1.second;
    const double t = cross(dx, dy, dxb, dyb) / denom;
    const double u = cross(dx, dy, dxa, dya) / denom;
    return t > 0.0 && t < 1.0 && u > 0.0 && u < 1.0;
}

double computeLayoutScore(const std::vector<RoutedEdge>& routedEdges) {
    double totalLength = 0.0;
    std::size_t totalBends = 0;
    std::size_t crossings = 0;

    for (const auto& e : routedEdges) {
        for (std::size_t i = 1; i < e.points.size(); ++i) {
            const auto dx = e.points[i].first - e.points[i - 1].first;
            const auto dy = e.points[i].second - e.points[i - 1].second;
            totalLength += std::hypot(dx, dy);
        }
        if (e.points.size() > 2) {
            totalBends += e.points.size() - 2;
        }
    }

    for (std::size_t i = 0; i < routedEdges.size(); ++i) {
        for (std::size_t j = i + 1; j < routedEdges.size(); ++j) {
            const auto& a = routedEdges[i].points;
            const auto& b = routedEdges[j].points;
            for (std::size_t p = 1; p < a.size(); ++p) {
                for (std::size_t q = 1; q < b.size(); ++q) {
                    if (segmentsIntersect(a[p - 1], a[p], b[q - 1], b[q])) {
                        ++crossings;
                    }
                }
            }
        }
    }

    // Simple weighted sum: shorter + fewer bends/crossings is better.
    return totalLength + totalBends * 10.0 + crossings * 50.0;
}

void exportToSvg(const std::string& filename,
                 const std::vector<Node>& nodes,
                 const std::vector<Port>& ports,
                 const std::vector<RoutedEdge>& routedEdges) {
    double minX = 1e9, minY = 1e9, maxX = -1e9, maxY = -1e9;
    auto updateBounds = [&](double x, double y) {
        minX = std::min(minX, x);
        minY = std::min(minY, y);
        maxX = std::max(maxX, x);
        maxY = std::max(maxY, y);
    };

    for (const auto& n : nodes) {
        updateBounds(n.x - n.width / 2.0 - 10.0, n.y - n.height / 2.0 - 10.0);
        updateBounds(n.x + n.width / 2.0 + 10.0, n.y + n.height / 2.0 + 10.0);
    }
    for (const auto& e : routedEdges) {
        for (const auto& p : e.points) {
            updateBounds(p.first, p.second);
        }
    }

    const double width = maxX - minX + 40.0;
    const double height = maxY - minY + 40.0;
    const double offsetX = -minX + 20.0;
    const double offsetY = -minY + 20.0;

    std::ofstream svg(filename);
    svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width
        << "\" height=\"" << height << "\" viewBox=\"0 0 " << width
        << ' ' << height << "\">\n";
    svg << "<defs><style> .overhead { stroke: #d1495b; stroke-width: 3; fill: none; stroke-dasharray: 8 4; }"
           " .normal { stroke: #22577a; stroke-width: 2; fill: none; }"
           " .port { fill: #333; }"
           " .big { fill: #cdeac0; stroke: #4f772d; }"
           " .small { fill: #fef9c3; stroke: #cc7722; }"
           " .square { fill: #cddafd; stroke: #364fc7; }"</style></defs>\n";

    for (const auto& n : nodes) {
        double x = n.x - n.width / 2.0 + offsetX;
        double y = n.y - n.height / 2.0 + offsetY;
        const char* klass = (n.type == NodeType::BigRect)
                                ? "big"
                                : (n.type == NodeType::SmallRect ? "small" : "square");
        svg << "<rect class=\"" << klass << "\" x=\"" << x << "\" y=\"" << y
            << "\" width=\"" << n.width << "\" height=\"" << n.height
            << "\" stroke-width=\"2\" />\n";
    }

    for (const auto& e : routedEdges) {
        const char* klass = (e.type == EdgeType::Overhead) ? "overhead" : "normal";
        svg << "<polyline class=\"" << klass << "\" points=\"";
        for (const auto& p : e.points) {
            svg << (p.first + offsetX) << ',' << (p.second + offsetY) << ' ';
        }
        svg << "\" />\n";
    }

    for (const auto& p : ports) {
        svg << "<circle class=\"port\" cx=\"" << (p.position.x + offsetX) << "\" cy=\""
            << (p.position.y + offsetY) << "\" r=\"3\" />\n";
    }

    svg << "</svg>\n";
}

int main() {
    std::vector<Node> nodes;
    std::vector<Edge> edges = buildSampleGraph(nodes);

    layoutNodesWithCola(nodes, edges);

    Avoid::Router router(Avoid::PolyLineRouting | Avoid::OrthogonalRouting);
    std::vector<Avoid::ShapeRef*> shapes;
    auto ports = createPortsForNodes(nodes, edges, router, shapes);

    auto routedEdges = routeEdgesWithAvoid(nodes, edges, ports, shapes, router);
    const double score = computeLayoutScore(routedEdges);

    exportToSvg("output.svg", nodes, ports, routedEdges);

    std::cout << "Simple score (length + penalties): " << score << "\n";
    std::cout << "Wrote output.svg – open it in a browser." << std::endl;
    return 0;
}
