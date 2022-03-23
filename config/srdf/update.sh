#!/bin/bash
set -e
ulimit -m 8048000
this_folder=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source "$this_folder/generate_srdf.sh" "$this_folder/tiago.srdf.xacro"
echo "this folder = $this_folder"
echo "$srdf_folder"/end_effectors/*.srdf.xacro
# crawl all end effectors and generate the corresponding subtree SRDF
for end_effector_file in "$srdf_folder"/end_effectors/*.srdf.xacro; do
    end_effector=$(basename "$end_effector_file" .srdf.xacro)
    for ft_sensor in false schunk-ft; do
        args=(arm:=true "ft_sensor:=$ft_sensor" end_effector:="$end_effector")
        if [ "$ft_sensor" != false ]; then
            generate_disable_collisions_subtree arm_tool_link "${end_effector}_${ft_sensor}" "${end_effector}" "${args[@]}"
        else
            generate_disable_collisions_subtree arm_tool_link "${end_effector}"  "" "${args[@]}"
        fi
    done
done

# for base_type in pmb2 omni_base; do
#     # Generate base disable collision pairs
#     prefix="${robot}"
#     if [ "$base_type" = "omni_base" ]; then
#         prefix="${robot}_omni"
#     fi
#     generate_srdf "${prefix}_no-arm" "" base_type:="$base_type" arm:=false ft_sensor:=false end_effector:=false # base & torso only
#     generate_disable_collisions "${prefix}_no-ee" "${prefix}_no-arm" base_type:="$base_type" arm:=true ft_sensor:=false end_effector:=false # plus arm
#     generate_disable_collisions "${prefix}_with-arm_schunk-ft" "${prefix}_no-ee" base_type:="$base_type" arm:=true ft_sensor:=schunk-ft end_effector:=false # plus FT sensor

#     # crawl all end effectors and generate the corresponding SRDF
#     for end_effector_file in "$srdf_folder"/end_effectors/*.srdf.xacro; do
#         end_effector=$(basename "$end_effector_file" .srdf.xacro)
#         for ft_sensor in false schunk-ft; do
#             args=(base_type:="$base_type" arm:=true "ft_sensor:=$ft_sensor" end_effector:="$end_effector")
#             if [ "$ft_sensor" != false ]; then
#                 generate_srdf "${prefix}_${end_effector}_${ft_sensor}" "${prefix}_no-ee_${ft_sensor}:${end_effector}_${ft_sensor}" "${args[@]}"
#             else
#                 generate_srdf "${prefix}_${end_effector}" "${prefix}_with-arm:${end_effector}" "${args[@]}"
#             fi
#         done
#     done
# done
