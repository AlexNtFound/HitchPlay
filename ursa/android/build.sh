#!/usr/bin/env bash
# ----------------------------------------------------------------------
#  Ursa build entry point (macOS / Linux).
#
#  Auto-detects a JDK so collaborators don't need to set JAVA_HOME by hand,
#  then forwards all args to gradlew. Resolution order:
#    1) JAVA_HOME, if already set and valid
#    2) macOS java_home utility
#    3) Android Studio's bundled JBR (standard install paths)
#    4) System `java` on PATH
#    5) Bail with an actionable message
# ----------------------------------------------------------------------
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

valid_java() {
    [[ -n "${1:-}" && -x "$1/bin/java" ]]
}

# 1) Honor an already-valid JAVA_HOME.
if valid_java "${JAVA_HOME:-}"; then
    :
# 2) macOS: use the system java_home helper.
elif [[ "$(uname -s)" == "Darwin" ]] && command -v /usr/libexec/java_home >/dev/null 2>&1; then
    JAVA_HOME="$(/usr/libexec/java_home -v 17 2>/dev/null || /usr/libexec/java_home 2>/dev/null || true)"
fi

# 3) Try common Android Studio JBR locations.
if ! valid_java "${JAVA_HOME:-}"; then
    for c in \
        "/Applications/Android Studio.app/Contents/jbr/Contents/Home" \
        "$HOME/Library/Android/sdk/jbr" \
        "/opt/android-studio/jbr" \
        "$HOME/android-studio/jbr" \
        "/usr/lib/jvm/java-17-openjdk-amd64" \
        "/usr/lib/jvm/default-java"; do
        if valid_java "$c"; then
            JAVA_HOME="$c"
            break
        fi
    done
fi

# 4) Last resort: derive from `java` on PATH.
if ! valid_java "${JAVA_HOME:-}" && command -v java >/dev/null 2>&1; then
    java_bin="$(command -v java)"
    java_real="$(readlink -f "$java_bin" 2>/dev/null || echo "$java_bin")"
    JAVA_HOME="$(dirname "$(dirname "$java_real")")"
fi

if ! valid_java "${JAVA_HOME:-}"; then
    cat <<EOF >&2

[Ursa] Could not find a JDK to run the Gradle wrapper.

Easy fix: install Android Studio (its bundled JBR is auto-detected), or
install OpenJDK 17 from https://adoptium.net/temurin/releases/?version=17
and set JAVA_HOME to point at it.

EOF
    exit 1
fi

echo "[Ursa] Using JAVA_HOME=$JAVA_HOME"
export JAVA_HOME
exec "$SCRIPT_DIR/gradlew" "$@"
