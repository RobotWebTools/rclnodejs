#!/bin/bash
# Run tests with AddressSanitizer (ASan/LSan) to detect memory leaks and errors
# in the native N-API addon.
#
# Requires: ROS2 sourced, g++ with libasan
#
# Usage:
#   bash scripts/run_asan_test.sh                        # rebuild + run all tests
#   bash scripts/run_asan_test.sh test/test-node.js      # rebuild + run specific test(s)
#   bash scripts/run_asan_test.sh --no-build test/test-node.js  # skip rebuild
#   bash scripts/run_asan_test.sh --exclude test/test-serialization.js  # exclude specific test(s)

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_DIR"

# Parse flags
DO_BUILD=1
ARGS=()
EXCLUDES=()
NEXT_IS_EXCLUDE=0
for arg in "$@"; do
  if [[ $NEXT_IS_EXCLUDE -eq 1 ]]; then
    EXCLUDES+=("$arg")
    NEXT_IS_EXCLUDE=0
  elif [[ "$arg" == "--no-build" ]]; then
    DO_BUILD=0
  elif [[ "$arg" == "--exclude" ]]; then
    NEXT_IS_EXCLUDE=1
  else
    ARGS+=("$arg")
  fi
done

if [[ $NEXT_IS_EXCLUDE -eq 1 ]]; then
  echo "Error: --exclude requires a test file argument"
  echo "Usage: bash scripts/run_asan_test.sh --exclude test/test-foo.js"
  exit 1
fi

# Step 1: Build with ASan
if [[ $DO_BUILD -eq 1 ]]; then
  echo "=== Building with AddressSanitizer ==="
  CXXFLAGS="-fsanitize=address" npx node-gyp -j 16 rebuild --debug
fi

# Step 2: Locate libasan
LIBASAN=$(g++ -print-file-name=libasan.so)
if [[ "$LIBASAN" == "libasan.so" ]]; then
  # g++ returned just the name, try to find the full path
  LIBASAN=$(find /usr/lib* -name "libasan.so*" -type f 2>/dev/null | head -1)
fi

if [[ -z "$LIBASAN" || ! -f "$LIBASAN" ]]; then
  echo "Warning: Could not find libasan.so, proceeding without LD_PRELOAD"
  echo "If you see 'ASan not the first DSO' errors, install libasan: sudo apt install libasan6"
  LIBASAN=""
fi

# Step 3: Set up environment
export LSAN_OPTIONS="suppressions=$PROJECT_DIR/suppr.txt"
if [[ -n "$LIBASAN" ]]; then
  export LD_PRELOAD="$LIBASAN"
fi

# Step 3.5: Temporarily hide prebuilds so the debug build is loaded
PREBUILDS_DIR="$PROJECT_DIR/prebuilds"
PREBUILDS_BAK="$PROJECT_DIR/.prebuilds_asan_bak"
MOVED_PREBUILDS=0
if [[ -d "$PREBUILDS_DIR" ]]; then
  if [[ -d "$PREBUILDS_BAK" ]]; then
    echo "Error: $PREBUILDS_BAK already exists (previous interrupted run?)."
    echo "Please remove it manually and retry: rm -rf $PREBUILDS_BAK"
    exit 1
  fi
  mv "$PREBUILDS_DIR" "$PREBUILDS_BAK"
  MOVED_PREBUILDS=1
fi

# Restore prebuilds on exit (even on failure)
cleanup() {
  if [[ $MOVED_PREBUILDS -eq 1 && -d "$PREBUILDS_BAK" ]]; then
    mv "$PREBUILDS_BAK" "$PREBUILDS_DIR"
  fi
}
trap cleanup EXIT

# Step 4: Build mocha exclude args from --exclude flags, blocklist.json, and asan_blocklist.json
MOCHA_EXCLUDES=()
for exc in "${EXCLUDES[@]}"; do
  MOCHA_EXCLUDES+=(--ignore "$exc")
done

# Auto-exclude tests from blocklist.json (Linux entries) and asan_blocklist.json
for blocklist in "$PROJECT_DIR/test/blocklist.json" "$PROJECT_DIR/test/asan_blocklist.json"; do
  if [[ -f "$blocklist" ]]; then
    while IFS= read -r test_file; do
      MOCHA_EXCLUDES+=(--ignore "test/$test_file")
    done < <(node -e "
      const bl = require('$blocklist');
      const entries = bl.Linux || bl;
      if (Array.isArray(entries)) entries.forEach(t => console.log(t));
    " 2>/dev/null)
  fi
done

# Step 5: Run tests
if [[ ${#ARGS[@]} -gt 0 ]]; then
  echo "=== Running ASan test: ${ARGS[*]} ==="
  node --expose-gc node_modules/.bin/mocha -r test/gc-on-exit.js "${MOCHA_EXCLUDES[@]}" "${ARGS[@]}"
else
  echo "=== Running all ASan tests ==="
  node --expose-gc node_modules/.bin/mocha -r test/gc-on-exit.js "${MOCHA_EXCLUDES[@]}" 'test/test-*.js'
fi

echo "=== ASan test complete ==="
