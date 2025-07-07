// NodeEdgeProcessor.cpp
#include "org_graph_outside_NativeLibrary.h"
#include <jni.h>
#include <vector>
#include <iostream>
#include "libdialect/commontypes.h"
#include "libdialect/graphs.h"
#include "libdialect/hola.h"
#include "libdialect/opts.h"

// 在这里定义跟 Java 层 nativeNode/nativeEdge 一一对应的 C++ 结构
struct CppNode {
    int    index;
    double x;
    double y;
    double    width;
    double    height;
    // TODO: 如果 Java 侧还有其他字段，继续在这里添加
};

struct CppEdge {
    int from;
    int to;
    // TODO: 如果 Java 侧有权重、类型等字段，继续在这里添加
};

JNIEXPORT void JNICALL
Java_org_graph_outside_NativeLibrary_generateLayout(JNIEnv* env, jobject nativeLibraryObject) {
    // ——1. 获取 nodeList/edgeList 字段引用——
    jclass clsLib     = env->GetObjectClass(nativeLibraryObject);
    jfieldID fidNodes = env->GetFieldID(clsLib, "nodeList", "Ljava/util/List;");
    jfieldID fidEdges = env->GetFieldID(clsLib, "edgeList", "Ljava/util/List;");

    jobject jNodeList = env->GetObjectField(nativeLibraryObject, fidNodes);
    jobject jEdgeList = env->GetObjectField(nativeLibraryObject, fidEdges);

    // ——2. 拿到 java.util.List 的 size 和 get 方法——
    jclass clsList      = env->FindClass("java/util/List");
    jmethodID midSize   = env->GetMethodID(clsList, "size", "()I");
    jmethodID midGet    = env->GetMethodID(clsList, "get", "(I)Ljava/lang/Object;");

    // ——3. 读出所有 nativeNode ——
    jint nodeCount = env->CallIntMethod(jNodeList, midSize);
    std::vector<CppNode> nodes;
    nodes.reserve(nodeCount);

    // inner class JNI 名称：外部类$内部类
    jclass clsNode     = env->FindClass("org/graph/outside/NativeLibrary$NativeNode");
    jfieldID fidIndex  = env->GetFieldID(clsNode, "index", "I");
    jfieldID fidX      = env->GetFieldID(clsNode, "x",     "D");
    jfieldID fidY      = env->GetFieldID(clsNode, "y",     "D");
    jfieldID fidWidth  = env->GetFieldID(clsNode, "width", "D");
    jfieldID fidHeight = env->GetFieldID(clsNode, "height","D");

    // Integer.valueOf(int)
    jclass clsInteger = env->FindClass("java/lang/Integer");
    jmethodID midIntegerValueOf = env->GetStaticMethodID(
        clsInteger, "valueOf", "(I)Ljava/lang/Integer;"
    );

    for (jint i = 0; i < nodeCount; ++i) {
        jobject jNode = env->CallObjectMethod(jNodeList, midGet, i);
        CppNode n;
        n.index  = env->GetIntField(jNode, fidIndex);
        n.x      = env->GetDoubleField(jNode, fidX);
        n.y      = env->GetDoubleField(jNode, fidY);
        n.width  = env->GetDoubleField(jNode, fidWidth);
        n.height = env->GetDoubleField(jNode, fidHeight);
        // TODO: 读取其他字段

        nodes.push_back(n);
        env->DeleteLocalRef(jNode);
    }

    // ——4. 读出所有 nativeEdge ——
    jint edgeCount = env->CallIntMethod(jEdgeList, midSize);
    std::vector<CppEdge> edges;
    edges.reserve(edgeCount);

    jclass clsEdge    = env->FindClass("org/graph/outside/NativeLibrary$NativeEdge");
    jfieldID fidFrom  = env->GetFieldID(clsEdge, "from", "I");
    jfieldID fidTo    = env->GetFieldID(clsEdge, "to",   "I");
    // TODO: 如果还有其它字段，继续 GetFieldID

    for (jint i = 0; i < edgeCount; ++i) {
        jobject jEdge = env->CallObjectMethod(jEdgeList, midGet, i);
        CppEdge e;
        e.from = env->GetIntField(jEdge, fidFrom);
        e.to   = env->GetIntField(jEdge, fidTo);
        // TODO: 读取其他字段

        edges.push_back(e);
        env->DeleteLocalRef(jEdge);
    }

    // ——5. 在这里对 nodes 和 edges 做任何你想要的 C++ 处理 ——
    // 例如：简单打印出来
    dialect::Graph_SP graph = std::make_shared<dialect::Graph>();
    dialect::Node_SP node;
    dialect::Edge_SP edge;
    dialect::NodesById nodesByExternalId;

    for (auto & n : nodes) {
        node = dialect::Node::allocate();
        node->setExternalId(n.index);
        node->setCentre(n.x, n.y);
        node->setDims(n.width, n.height);
        graph->addNode(node);
        nodesByExternalId.insert({n.index, node});
    }

    for (auto & e : edges) {
        edge = dialect::Edge::allocate(nodesByExternalId[e.from], nodesByExternalId[e.to]);
        graph->addEdge(edge);
    }
    dialect::HolaOpts opts;
    try {
        dialect::doHOLA(*graph, opts);
    } catch (std::exception &e) {
        std::cout << e.what() << std::endl;
    }


    // 4. 遍历 list
    jint count = env->CallIntMethod(jNodeList, midSize);
    for (jint i = 0; i < count; ++i) {
        // 4.1 取出第 i 个 nativeNode 对象
        jobject jNode = env->CallObjectMethod(jNodeList, midGet, i);

        // 4.2 读出它的 id
        jint nodeId = env->GetIntField(jNode, fidIndex);

        dialect::Node_SP node = nodesByExternalId[nodeId];

        // 4.4 就地写回同一个对象
        env->SetDoubleField(jNode, fidX, node->getCentre().x);
        env->SetDoubleField(jNode, fidY, node->getCentre().y);

        // 4.5 释放局部引用
        env->DeleteLocalRef(jNode);
    }
    // 清理
    env->DeleteLocalRef(jNodeList);
    env->DeleteLocalRef(jEdgeList);
    env->DeleteLocalRef(clsList);
    env->DeleteLocalRef(clsNode);
    env->DeleteLocalRef(clsEdge);
    env->DeleteLocalRef(clsLib);
}