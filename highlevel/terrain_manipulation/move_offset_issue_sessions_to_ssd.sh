#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_ROOT="${SCRIPT_DIR}/data"
DEST_ROOT="/media/parnia/Extreme SSD/density experiments backup - with offset issue"

SESSIONS=(
  "session_20260314_122949"
  "session_20260314_123416"
  "session_20260314_123744"
  "session_20260314_124126"
  "session_20260314_124658"
  "session_20260314_125646"
  "session_20260316_132543"
  "session_20260316_142942"
  "session_20260316_144047"
  "session_20260316_145733"
  "session_20260316_150341"
  "session_20260316_152118"
  "session_20260316_152712"
)

missing_sources=()
existing_destinations=()

for session in "${SESSIONS[@]}"; do
  src="${SRC_ROOT}/${session}"
  dst="${DEST_ROOT}/${session}"
  if [[ ! -d "${src}" ]]; then
    missing_sources+=("${src}")
  fi
  if [[ -e "${dst}" ]]; then
    existing_destinations+=("${dst}")
  fi
done

if (( ${#missing_sources[@]} > 0 )); then
  echo "Error: missing source session folder(s):" >&2
  for p in "${missing_sources[@]}"; do
    echo "  - ${p}" >&2
  done
fi

if (( ${#existing_destinations[@]} > 0 )); then
  echo "Error: destination already exists for session folder(s):" >&2
  for p in "${existing_destinations[@]}"; do
    echo "  - ${p}" >&2
  done
fi

if (( ${#missing_sources[@]} > 0 || ${#existing_destinations[@]} > 0 )); then
  exit 1
fi

mkdir -p "${DEST_ROOT}"

for session in "${SESSIONS[@]}"; do
  src="${SRC_ROOT}/${session}"
  echo "Moving: ${src} -> ${DEST_ROOT}/"
  mv "${src}" "${DEST_ROOT}/"
done

echo "Done. Moved ${#SESSIONS[@]} session folder(s) to:"
echo "  ${DEST_ROOT}"
