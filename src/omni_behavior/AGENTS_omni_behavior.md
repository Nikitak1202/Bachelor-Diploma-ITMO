# AGENTS: omni_behavior

## Role

Small ament_cmake package exporting a Nav2 Behavior Tree plugin: `TargetVisibleCondition`, which reads `std_msgs/Bool` on `/target_visible` (latched) and returns SUCCESS while true.

## Current use

Default chase stack uses Python `target_nav_bridge` in `omni_robot`, not this BT plugin. The package remains for optional BT-based trees (`omni_robot/behavior_trees/chase_target.xml`) if you re-enable them in Nav2 params.

## Files

- `include/.../target_visible_condition.hpp`, `src/target_visible_condition.cpp` — BT node implementation.
- `plugin.xml` — library name `omni_target_visible_condition_bt_node` must match `bt_navigator.plugin_lib_names` in `omni_robot/config/omni_nav2_params.yaml` when the plugin is linked.

## Rules for changes

New BT nodes: add sources, export in `plugin.xml`, register in `CMakeLists.txt`, and document the library name in Nav2 YAML. Renaming the shared library requires the same string in YAML and XML.

Back: `../../AGENTS.md`
