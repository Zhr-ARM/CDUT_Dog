#!/usr/bin/env bash

# Keep this workspace on the system ROS/Python toolchain even if conda base is
# activated by the user's global shell startup files.
__cdut_remove_path_entry() {
  local needle="$1"
  local old_ifs="${IFS}"
  local entry
  local rebuilt=""

  IFS=":"
  for entry in ${PATH}; do
    if [[ -z "${entry}" || "${entry}" == "${needle}"* ]]; then
      continue
    fi
    if [[ -z "${rebuilt}" ]]; then
      rebuilt="${entry}"
    else
      rebuilt="${rebuilt}:${entry}"
    fi
  done
  IFS="${old_ifs}"
  PATH="${rebuilt}"
}

if command -v conda >/dev/null 2>&1; then
  while [[ "${CONDA_SHLVL:-0}" =~ ^[0-9]+$ && "${CONDA_SHLVL:-0}" -gt 0 ]]; do
    conda deactivate >/dev/null 2>&1 || break
  done
fi

unset CONDA_DEFAULT_ENV
unset CONDA_EXE
unset CONDA_PREFIX
unset CONDA_PROMPT_MODIFIER
unset CONDA_PYTHON_EXE
unset CONDA_SHLVL
unset _CONDA_EXE
unset _CONDA_ROOT

__cdut_remove_path_entry "${HOME}/miniconda3"
__cdut_remove_path_entry "${HOME}/anaconda3"

export PATH
export CMAKE_IGNORE_PREFIX_PATH="${HOME}/miniconda3${CMAKE_IGNORE_PREFIX_PATH:+:${CMAKE_IGNORE_PREFIX_PATH}}"
export Python3_EXECUTABLE="/usr/bin/python3"
export PYTHON_EXECUTABLE="/usr/bin/python3"

unset -f __cdut_remove_path_entry
