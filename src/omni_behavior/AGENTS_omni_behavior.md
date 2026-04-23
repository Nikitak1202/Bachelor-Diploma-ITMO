# AGENTS Guide: omni_behavior

## Scope
`omni_behavior` is a minimal C++ ament_cmake package that exports custom
Nav2 Behavior Tree plugins used by the target-chase pipeline.

Current runtime in `omni_robot` uses Python `target_detector` + `target_nav_bridge`
as the primary chase pipeline. This package is currently optional/legacy unless
custom BT-based chase behavior is explicitly enabled again.

Currently it exports one plugin:
- `TargetVisibleCondition` - BT condition node that returns `SUCCESS` while
  `/target_visible` (std_msgs/Bool, latched) is true, else `FAILURE`.

Back to root guide: [`../../AGENTS.md`](../../AGENTS.md)

## Main Components
- `include/omni_behavior/target_visible_condition.hpp` + `src/target_visible_condition.cpp`
  - ConditionNode using a dedicated callback group so it does not depend on
    the BT Navigator spinning the subscription.
- `plugin.xml`
  - pluginlib export; library name: `omni_target_visible_condition_bt_node`.
- `CMakeLists.txt`, `package.xml`
  - library target name is referenced both by the BT Navigator
    `plugin_lib_names` list and by `nav2_behavior_tree`.

## Integration Contract
- Library name `omni_target_visible_condition_bt_node` must appear in
  `bt_navigator.plugin_lib_names` in
  [`omni_robot/config/omni_nav2_params.yaml`](../omni_robot/config/omni_nav2_params.yaml).
- BT XML nodes using this plugin reference `<TargetVisibleCondition
  topic="/target_visible"/>`; see
  [`omni_robot/behavior_trees/chase_target.xml`](../omni_robot/behavior_trees/chase_target.xml).

## Change Guidelines
- Add new BT plugins only when behavior cannot be expressed with built-in
  Nav2 BT nodes.
- Keep each plugin self-contained (its own `.hpp`/`.cpp`) and update
  `plugin.xml` + `CMakeLists.txt` accordingly.
- Preserve the library name exported in `plugin.xml`; renaming it requires
  updating the Nav2 params file.
