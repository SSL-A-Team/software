"""Git helper functions for the radio bridge tests."""

import clang.cindex
import subprocess
import pathlib

def get_ateam_radio_msgs_prefix():
    """Get the path prefix for the ateam_radio_msgs package."""
    result = subprocess.run(
        ["ros2", "pkg", "prefix", "ateam_radio_msgs"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(f"Failed to get package prefix: {result.stderr}")
    return pathlib.Path(result.stdout.strip())

def find_version_header():
    """Return the path to the version header file."""
    return get_ateam_radio_msgs_prefix() / "include" / "ateam_radio_msgs" / "ateam_radio_msgs" / "version.hpp"

def find_constant_value(symbol_name):
    """Find the value of a symbol in the version header."""
    index = clang.cindex.Index.create()
    tu = index.parse(str(find_version_header()), args=["-Xclang","-ast-dump"])
    tokens = list(tu.cursor.get_tokens())
    for token_index in range(len(tokens)):
        if tokens[token_index].spelling == symbol_name and tokens[token_index + 1].spelling == "=":
            return tokens[token_index + 2].spelling
    raise RuntimeError(f"Failed to find constant {symbol_name} in version.hpp")

def get_coms_submodule_hash():
    """Get the current commit hash of the software communication submodule."""
    return int(find_constant_value("kComsHash"))

def get_coms_submodule_dirty():
    """Check if the software communication submodule has uncommitted changes."""
    return bool(find_constant_value("kComsDirty"))
