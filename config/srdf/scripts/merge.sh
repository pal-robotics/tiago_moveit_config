#!/bin/bash
set -e
source "$(rospack find pal_moveit_config_generator)/srdf_utils.sh" "$(dirname "${BASH_SOURCE[0]}")/../tiago.srdf.xacro"

ref=${1:-HEAD}

for f in "$srdf_folder"/end_effectors/*.srdf.xacro; do
    end_effector=$(basename "$f" .srdf.xacro)
    end_effector_name=$(grep -Po '<xacro:property name="end_effector_name" value="\K[^"]+' "$f" || echo "gripper")
    for tiago in tiago tiago_omni; do
        add_diff_matrix_to_xacro_from_ref "$ref" "${tiago}_${end_effector}.srdf" "$end_effector_name" "disable_collisions/$end_effector.srdf.xacro"
    done
done

add_diff_matrix_to_xacro_from_ref "$ref" "tiago_no-arm.srdf" "" "disable_collisions/tiago_no-arm.srdf.xacro"

for f in "$srdf_folder"/disable_collisions/*.srdf.xacro; do
    p=${f#"$srdf_folder"/}
    add_diff_matrix_to_xacro_from_ref "$ref" "$p" "" "$p"
done
