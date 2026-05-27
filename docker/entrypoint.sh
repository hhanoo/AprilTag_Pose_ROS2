#!/bin/bash
set -e

# ===== Workspace =====
WORKSPACE="/ros2_ws"

# ===== Functions =====
restore_ownership() {
    # Restore host file ownership
    if [ -n "$HOST_UID" ] && [ -n "$HOST_GID" ] && [ -d "$WORKSPACE" ]; then
        chown -R "$HOST_UID:$HOST_GID" "$WORKSPACE"
    fi
}

# ===== Trap =====
trap restore_ownership EXIT

# ===== Default Command =====
if [ "$#" -eq 0 ]; then
    set -- /bin/bash
fi

# ===== Execute Command =====
"$@"
