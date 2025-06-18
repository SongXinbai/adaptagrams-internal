//
// Created by ersong2 on 2025/4/23.
//
#include "org_graph_outside_NativeLibrary.h"
#include <jni.h>
#include <iostream>
#include <vector>
#include "libdialect/commontypes.h"
#include "libdialect/graphs.h"
#include "libdialect/hola.h"
#include "libdialect/io.h"
#include "libdialect/opts.h"

using namespace std;

// 一个小工具函数，把 Java List<Integer> 取到 C++ vector<int>
vector<int> getNodeIdList(JNIEnv* env, jobject nativeLibraryObject) {
    // 获取 class
    jclass cls = env->GetObjectClass(nativeLibraryObject);

    // 获取 field id
    jfieldID fid = env->GetFieldID(cls, "nodeIdList", "Ljava/util/List;");
    jobject nodeIdListObj = env->GetObjectField(nativeLibraryObject, fid);
    // List类和方法
    jclass listCls = env->GetObjectClass(nodeIdListObj);
    jmethodID sizeMethod = env->GetMethodID(listCls, "size", "()I");
    jmethodID getMethod = env->GetMethodID(listCls, "get", "(I)Ljava/lang/Object;");

    jint size = env->CallIntMethod(nodeIdListObj, sizeMethod);
    vector<int> nodeIds;

    for (jint i = 0; i < size; i++) {
        jobject integerObj = env->CallObjectMethod(nodeIdListObj, getMethod, i);
        // Integer.intValue()
        jclass integerCls = env->GetObjectClass(integerObj);
        jmethodID intValueMethod = env->GetMethodID(integerCls, "intValue", "()I");
        jint value = env->CallIntMethod(integerObj, intValueMethod);

        nodeIds.push_back((int)value);

        env->DeleteLocalRef(integerObj);
        env->DeleteLocalRef(integerCls);
    }

    env->DeleteLocalRef(listCls);
    return nodeIds;
}

// 取 edgeList 同理
vector<string> getEdgeList(JNIEnv* env, jobject nativeLibraryObject) {
    jclass cls = env->GetObjectClass(nativeLibraryObject);
    jfieldID fid = env->GetFieldID(cls, "edgeList", "Ljava/util/List;");
    jobject edgeListObj = env->GetObjectField(nativeLibraryObject, fid);

    jclass listCls = env->GetObjectClass(edgeListObj);
    jmethodID sizeMethod = env->GetMethodID(listCls, "size", "()I");
    jmethodID getMethod = env->GetMethodID(listCls, "get", "(I)Ljava/lang/Object;");

    jint size = env->CallIntMethod(edgeListObj, sizeMethod);
    vector<string> edges;

    for (jint i = 0; i < size; i++) {
        jobject strObj = env->CallObjectMethod(edgeListObj, getMethod, i);
        const char* str = env->GetStringUTFChars((jstring)strObj, 0);

        edges.push_back(string(str));

        env->ReleaseStringUTFChars((jstring)strObj, str);
        env->DeleteLocalRef(strObj);
    }

    env->DeleteLocalRef(listCls);
    return edges;
}

// 填充 processedNodeList
void fillProcessedNodeList(JNIEnv* env, jobject nativeLibraryObject, vector<int>& nodeIds, vector<string>& edges, int width, int height) {


    dialect::Graph_SP graph = make_shared<dialect::Graph>();
    dialect::Node_SP node;
    dialect::Edge_SP edge;
    dialect::NodesById nodesByExternalId;

    for (int i=0;i<nodeIds.size();++i) {
        node = dialect::Node::allocate();
        node->setExternalId(nodeIds.at(i));
        node->setDims(width, height);
        graph->addNode(node);
        nodesByExternalId.insert({nodeIds.at(i), node});
    }

    for (int i=0;i<edges.size();++i) {
        int fnode, tnode;
        std::stringstream ss(edges.at(i));
        std::string token;
        // 先取第一个数字
        if (std::getline(ss, token, '-')) {
            fnode = std::stoi(token);
        }
        // 再取第二个数字
        if (std::getline(ss, token, '-')) {
            tnode = std::stoi(token);
        }
        edge = dialect::Edge::allocate(nodesByExternalId[fnode], nodesByExternalId[tnode]);
        graph->addEdge(edge);
    }
    dialect::HolaOpts opts;
    try {
        dialect::doHOLA(*graph, opts);
    } catch (std::exception &e) {
        std::cout << e.what() << std::endl;
    }

    // 拿 NativeLibrary class
    jclass nativeCls = env->GetObjectClass(nativeLibraryObject);

    // 拿 processedNodeList field
    jfieldID processedListFid = env->GetFieldID(nativeCls, "processedNodeList", "Ljava/util/List;");
    jobject processedListObj = env->GetObjectField(nativeLibraryObject, processedListFid);

    // 拿 List的 add 方法
    if (processedListObj == NULL) {
        std::cerr << "Error: processedListObj is NULL!" << std::endl;
        return;  // 或者适当的错误处理
    }

    jclass listCls = env->GetObjectClass(processedListObj);
    if (listCls == NULL) {
        std::cerr << "Error: Could not get class of the object!" << std::endl;
        return;  // 或者适当的错误处理
    }
    jmethodID addMethod = env->GetMethodID(listCls, "add", "(Ljava/lang/Object;)Z");

    // 拿 ProcessedNode class
    jclass processedNodeCls = env->FindClass("org/graph/outside/NativeLibrary$ProcessedNode");
    jmethodID processedNodeCtor = env->GetMethodID(processedNodeCls, "<init>", "(Lorg/graph/outside/NativeLibrary;)V");


    for (int id : nodeIds) {

        dialect::Node_SP node = nodesByExternalId[id];

        // 创建 ProcessedNode 实例
        jobject nodeObj = env->NewObject(processedNodeCls, processedNodeCtor, nativeLibraryObject);

        // 设置 nodeId、x、y
        jfieldID nodeIdFid = env->GetFieldID(processedNodeCls, "nodeId", "Ljava/lang/Integer;");
        jfieldID xFid = env->GetFieldID(processedNodeCls, "x", "Ljava/lang/Double;");
        jfieldID yFid = env->GetFieldID(processedNodeCls, "y", "Ljava/lang/Double;");

        // 创建 Integer和Double对象
        jclass integerCls = env->FindClass("java/lang/Integer");
        jmethodID integerCtor = env->GetMethodID(integerCls, "<init>", "(I)V");
        jobject nodeIdObj = env->NewObject(integerCls, integerCtor, id);

        jclass doubleCls = env->FindClass("java/lang/Double");
        jmethodID doubleCtor = env->GetMethodID(doubleCls, "<init>", "(D)V");
        jobject xObj = env->NewObject(doubleCls, doubleCtor, node->getCentre().x); // just for example
        jobject yObj = env->NewObject(doubleCls, doubleCtor, node->getCentre().y); // just for example

        // 设置字段
        env->SetObjectField(nodeObj, nodeIdFid, nodeIdObj);
        env->SetObjectField(nodeObj, xFid, xObj);
        env->SetObjectField(nodeObj, yFid, yObj);

        // 加入 processedNodeList
        env->CallBooleanMethod(processedListObj, addMethod, nodeObj);

        // 回收局部引用
        env->DeleteLocalRef(nodeIdObj);
        env->DeleteLocalRef(xObj);
        env->DeleteLocalRef(yObj);
        env->DeleteLocalRef(nodeObj);
    }

    env->DeleteLocalRef(listCls);
}

JNIEXPORT void JNICALL Java_org_graph_outside_NativeLibrary_generateLayout(JNIEnv* env, jobject obj, jint width, jint height) {
    vector<int> nodeIds = getNodeIdList(env, obj);
    vector<string> edges = getEdgeList(env, obj);

    // 这里可以做你的C++算法处理，比如layout计算
    // 你也可以显式转成 int 用
    int w = static_cast<int>(width);
    int h = static_cast<int>(height);
    // 处理完后写回processedNodeList
    fillProcessedNodeList(env, obj, nodeIds, edges, w, h);
}