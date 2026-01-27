/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#ifndef amr_COMMON_INCLUDE_amr_COMMON_amr_NODE_NAME_H_
#define amr_COMMON_INCLUDE_amr_COMMON_amr_NODE_NAME_H_

// 所有节点名常量移步于此
namespace amr_node {

constexpr int kErrorExitCode = -1;

// 节点名
constexpr char kEmbeddedNode[] = "amr_embedded_node";
constexpr char kCommunicateNode[] = "amr_communicate_node";
constexpr char kDecisionMakerNode[] = "decision_maker_node";
constexpr char kNavigationNode[] = "amr_navigation_node";
constexpr char kLocalizerNode[] = "amr_localizer_node";
constexpr char kReflectorNode[] = "amr_reflector_localizer_node";
constexpr char kActionNode[] = "amr_action_node";
constexpr char kDiagnosticNode[] = "amr_diagnostic_node";
constexpr char kAvoidNode[] = "amr_avoid_node";
constexpr char kStorageNode[] = "amr_storage_node";
constexpr char kFakeServerNode[] = "faker_server";
constexpr char kFakeVisualNode[] = "amr_visual_node";
}  // namespace amr_node

#endif  // amr_COMMON_INCLUDE_amr_COMMON_amr_NODE_NAME_H_
