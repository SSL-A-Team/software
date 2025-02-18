#! /bin/bash

# This script finds which packages have been changed relative to some git reference

git_base_ref=$1
working_dir=$2

pushd $working_dir > /dev/null

changed_files=$(git diff --name-only "$git_base_ref"...HEAD)

changed_packages=()

for file in $changed_files; do
  while read package_info; do
    package_path=$(echo "$package_info" | awk '{ print $2 }')
    package_name=$(echo "$package_info" | awk '{ print $1 }')
    if [[ ${file} == ${package_path}* ]]; then
      changed_packages+=( "$package_name" )
      break
    fi
  done < <(colcon --log-base /dev/null list)
done

printf "%s " "${changed_packages[*]}"

popd > /dev/null
