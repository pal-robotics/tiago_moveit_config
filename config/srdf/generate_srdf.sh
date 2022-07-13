#!/bin/bash
this_folder="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
main_srdf_xacro=${1?please provide path to main srdf.xacro}
robot=$(basename "$main_srdf_xacro" ".srdf.xacro")
srdf_folder=$(dirname "$main_srdf_xacro")
srdf_pkg="${robot}_moveit_config"
trials=${trials-100000}

function get_main_urdf_xacro() {
    if [ -z "$main_urdf_xacro" ]; then
        description_folder=$(rospack find "$robot"_description)
        main_urdf_xacro="$description_folder/robots/${robot}.urdf.xacro"
    fi
    echo "$main_urdf_xacro"
}

function find_file() {
    if [[ $1 =~ '$(find '(.*)')/'(.*)$ ]]; then
        if [ "${BASH_REMATCH[1]}" = "$srdf_pkg" ]; then
            echo "$srdf_folder/../../${BASH_REMATCH[2]}"
        else
            echo "$(rospack find "${BASH_REMATCH[1]}")/${BASH_REMATCH[2]}"
        fi
    else
        echo "$1"
    fi
}

function ref_file() {
    if [[ "$1" == "\$("* ]]; then
        echo "$1"
    else
        path="config/srdf/disable_collisions/$1.srdf.xacro"; shift
        pkg="${1-$srdf_pkg}"
        echo "\$(find $pkg)/${path}"
    fi
}

function comm_lines() {
    comm "$@" | sed -e 's/^\w+/\n  /g' | extract_matrix | sort -u
}

function extract_matrix() {
    grep "<disable_collision" | sed -e 's/^    /  /' -e 's/^\t//' -e 's/" \/>/"\/>/' | sort
}

function make_xacro() {
    echo '<?xml version="1.0" ?>'
    echo "<robot name=\"$robot\" xmlns:xacro=\"http://ros.org/wiki/xacro\">"
    for include in "$@"; do
        echo "  <xacro:include filename=\"$include\" />"
    done
    cat
    echo "</robot>"
}

function filter_urdf() {
    "python$ROS_PYTHON_VERSION" "$this_folder/urdf_filter.py" /dev/stdin "$@"
}

function extract_clean_matrix() {
    sort_links | extract_matrix | sed -e 's/( *)</  </'
}


function make_mandatory_disable_collisions() {
    local mandatory=""
    mandatory="$("python$ROS_PYTHON_VERSION" "$this_folder/urdf_to_disable_collisions.py" /dev/stdin 2> /dev/null | extract_matrix)"
    comm_lines -23 <(echo "$mandatory") <(grep Adjacent <<< "$mandatory" | sed -e 's/Adjacent/Never/' | sort -u)
}

function filter_matrix() {
    grep "link1=\"$1.*link2=\"$1" || true
}

function diff_matrix() {
    local new; new=$(extract_clean_matrix < "$3" | filter_matrix "$4")
    comm_lines $1 <(extract_clean_matrix < "$2" | filter_matrix "$4") <(echo -n "$new") | grep -v -F -f <(sed 's;reason=.*;;' <<< "$new") || true
}

function add_matrix_to_xacro() {
    local orig
    orig=$(grep -v "</robot>" $1)
    local diff
    diff=$(diff_matrix -23 "$2" <(echo "$orig"))
    if [ -n "$diff" ]; then
        echo "$1"
        echo "${diff}"
        { echo "${orig}"; echo "${diff}"; echo "</robot>"; } | sort_srdf > "$1"
    fi
}

function add_diff_matrix_to_xacro_from_ref() {
    local from_git
    from_git=$(cd "$srdf_folder" && git cat-file -p "$1:./$2")
    if [ -n "$from_git" ]; then
        echo "Updating $4 from $2 ($1)"
        add_matrix_to_xacro "$srdf_folder/$4" <(diff_matrix -23 <(echo "$from_git") "$srdf_folder/$2" "$3")
    fi
}

function resolve_includes() {
    while IFS= read -r line; do
        if [[ $line =~ '<xacro'.'include filename="'(.*)\" ]]; then
            { grep disable_collisions "$(find_file "${BASH_REMATCH[1]}")" || true; } | resolve_includes
        else
            echo "$line"
        fi
    done
}

function fast_xacro() {
    local input=$1; shift
    sed -e 's;<xacro:include filename="\(.*config/srdf/disable_collisions/.*\)";<xacro_include filename="\1";' "$input" \
    | rosrun xacro xacro /dev/stdin "$@" \
    | sed -e 's/ encoding="utf-8"//' \
    | resolve_includes
}

function parse_matrix() {
    resolve_includes < "$1" | extract_matrix
}

function sort_links() {
    while IFS= read -r line; do
        if [[ $line =~ (.*)'<disable_collisions link1="'(.*)'" link2="'(.*)'" reason="'(.*)'"'\ ?'/>'(.*) ]]; then
            link1=${BASH_REMATCH[2]}
            link2=${BASH_REMATCH[3]}
            if [[ "$link1" > "$link2" ]]; then
                link1=${BASH_REMATCH[3]}
                link2=${BASH_REMATCH[2]}
            fi
            echo "${BASH_REMATCH[1]}<disable_collisions link1=\"$link1\" link2=\"$link2\" reason=\"${BASH_REMATCH[4]}\" />${BASH_REMATCH[5]}"
        else
            echo "$line"
        fi
    done
}

function sort_srdf() {
    local disable_collisions=""
    while IFS= read -r line; do
        if [[ $line == *'<disable_collisions'* ]]; then
            disable_collisions+=$line$'\n'
        else
            if [ -n "$disable_collisions" ]; then
                echo -n "$disable_collisions" | sort -u
                disable_collisions=""
            fi
            echo "$line"
        fi
    done
    echo -n "$disable_collisions" | sort -u
}

function make_srdf() {
    fast_xacro "$@" | sort_links | sort_srdf | sed -E -e 's/( *)</\1\1</' -e 's;"(.)>;" \1>;'
}

## generate disable_collisions for given variant from URDF
##
## @param urdf: path to urdf/xacro
## @param name: suffix of variant
## @param from: base variant (emptry string disables the overlay)
## @param xacro_args: xacro arguments
##
function generate_disable_collisions_from_urdf() {
    local urdf_xacro=$1; shift
    local name=$1; shift
    IFS=':' read -r -a from <<< "$1"; shift
    local dc_name="disable_collisions/${name}.srdf.xacro"
    dc="$srdf_folder/$dc_name"

    if [ ! -e "$dc" ]; then
        srdf="${dc%.xacro}"
        urdf="${srdf%.srdf}.urdf"
        mkdir -p "$(dirname "$dc")"

        echo
        echo "Creating '$dc_name'..."

        local bases=()
        for f in "${from[@]}"; do
            if [ -n "$f" ]; then
                bases+=("$(ref_file "$f")")
            fi
        done

        local base_matrix=""
        base_matrix="$(for base in "${bases[@]}"; do parse_matrix "$(find_file "$base")"; done | sort -u)"

        rosrun xacro xacro "$@" "$urdf_xacro" > "$urdf"
        local mandatory_matrix="" # disable mandatory link collisions
        mandatory_matrix="$(comm_lines -23 <(make_mandatory_disable_collisions < "$urdf") <(echo "$base_matrix"))"

        local pass=0
        while true; do
            local old_matrix=""
            pass=$((pass+1))
            echo
            echo "Pass $pass with $trials trials for '$dc_name'"

           if [ -e "$dc" ]; then
                old_matrix="$(extract_matrix < "$dc")"
            fi
            # Create initial SRDF from URDF
            { echo "$mandatory_matrix"; echo "$base_matrix"; } | make_xacro  > "$srdf"

            # Optimize SRDF by disabling link pairs, which collide "never"
            rosrun moveit_setup_assistant collisions_updater \
                --trials "$trials" \
                --urdf "$urdf" \
                --srdf "$srdf" \
                --default \
                --keep \
                --out "$srdf"

            # If variant is derived from another one, only keep the additional link pairs
            local target_matrix=""
            target_matrix="$(comm_lines -23 <(extract_matrix < "$srdf") <(echo "$base_matrix"))"

            if [ -n "$old_matrix" ]; then
                # keep only pairs which are in old and new matrix (truly never colliding)
                target_matrix=$(comm_lines -12 <(echo "$target_matrix") <(echo "$old_matrix"))
            fi
            echo "$target_matrix" | make_xacro "${bases[@]}"  > "$dc"

            if [ -n "$old_matrix" ] || [ -z "$target_matrix" ]; then

                local diff=""
                diff="$(comm_lines -23 <(echo "$old_matrix") <(echo "$target_matrix") | sed -e 's/^  / -/')"
                if [ -n "$diff" ]; then
                    echo "Difference:"
                    echo "$diff"
                    continue
                else
                    break
                fi
            fi
        done
        rm "$urdf" "$srdf"
    fi
}

## generate disable_collisions for given variant
##
## @param name: suffix of variant
## @param from: base variant (emptry string disables the overlay)
## @param start_link: start link of subtree
## @param xacro_args: xacro arguments
##
function generate_disable_collisions_subtree() {
    local start_link=$1; shift
    local name=$1; shift
    local from=$1; shift
    local target="$srdf_folder/disable_collisions/${name}.srdf.xacro"

    if [ ! -e "$srdf_folder/disable_collisions/${name}.srdf.xacro" ]; then
        rosrun xacro xacro "$(get_main_urdf_xacro)" "$@" | filter_urdf "$start_link" > "/tmp/$name.urdf"
        generate_disable_collisions_from_urdf "/tmp/$name.urdf" "$name" "$from" "$@"
        echo "$(grep -v grep -v "\"$start_link\"") "$target")" > "$target"
        rm -rf "/tmp/$name.urdf"
    fi
}

## generate disable_collisions for given variant
##
## @param name: suffix of variant
## @param from: base variant (emptry string disables the overlay)
## @param xacro_args: xacro arguments
##
function generate_disable_collisions() {
    generate_disable_collisions_from_urdf "$(get_main_urdf_xacro)" "$@"
}

## generate SRDF for given variant
##
## @param expand: create expanded SRDF
## @param name: suffix of variant
## @param from: base variant (emptry string disables the overlay)
## @param xacro_args: xacro arguments
##
function generate_srdf() {
    local name=$1; shift
    local from=$1; shift

    generate_disable_collisions "$name" "$from" "$@"

    # Expand SRDF from xacro to speed up launch
    echo
    echo "Writing '${name}.srdf'..."
    make_srdf "$main_srdf_xacro" "$@" > "$srdf_folder/${name}.srdf"
}
