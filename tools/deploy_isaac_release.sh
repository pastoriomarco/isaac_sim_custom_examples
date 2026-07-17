#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: deploy_isaac_release.sh --host HOST [--user USER] [--identity PATH]
                               [--release-id ID] [--remote-root PATH]

Validates the USD authority contract, copies the complete asset tree into an
isolated incoming directory, and atomically promotes it to an immutable release.
EOF
}

asset_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
host=""
user="${USER}"
identity="${HOME}/.ssh/manyforge_isaac_laptop_ed25519"
revision=$(git -C "${asset_root}" rev-parse --short HEAD)
if [[ -n "$(git -C "${asset_root}" status --porcelain)" ]]; then
  revision="${revision}-dirty"
fi
release_id="$(date -u +%Y%m%dT%H%M%SZ)-${revision}"
remote_root=""

while (($#)); do
  case "$1" in
    --host) host="$2"; shift 2 ;;
    --user) user="$2"; shift 2 ;;
    --identity) identity="$2"; shift 2 ;;
    --release-id) release_id="$2"; shift 2 ;;
    --remote-root) remote_root="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -n "${host}" ]] || { echo "--host is required" >&2; exit 2; }
[[ -f "${identity}" ]] || { echo "SSH identity not found: ${identity}" >&2; exit 2; }
remote_root="${remote_root:-/home/${user}/manyforge_isaac_asset_runs}"
[[ "${host}" =~ ^[A-Za-z0-9][A-Za-z0-9.-]*$ ]] || {
  echo "Unsafe host: ${host}" >&2
  exit 2
}
[[ "${user}" =~ ^[A-Za-z_][A-Za-z0-9_-]*$ ]] || {
  echo "Unsafe user: ${user}" >&2
  exit 2
}
[[ "${identity}" =~ ^/[A-Za-z0-9._/+:-]+$ ]] || {
  echo "Unsafe identity path: ${identity}" >&2
  exit 2
}
[[ "${remote_root}" =~ ^/[A-Za-z0-9._/-]+$ ]] \
  && [[ "${remote_root}" != *"/../"* ]] \
  && [[ "${remote_root}" != */.. ]] \
  && [[ "${remote_root}" != *"//"* ]] || {
  echo "Unsafe remote root: ${remote_root}" >&2
  exit 2
}
[[ "${release_id}" =~ ^[A-Za-z0-9._-]+$ ]] || {
  echo "Unsafe release id: ${release_id}" >&2
  exit 2
}

if docker inspect manyforge-usd-tools >/dev/null 2>&1; then
  docker exec manyforge-usd-tools \
    python3 /repo/tools/validate_ur10e_robotiq_contract.py --asset-root /repo
else
  python3 "${asset_root}/tools/validate_ur10e_robotiq_contract.py" \
    --asset-root "${asset_root}"
fi

ssh_opts=(
  -o BatchMode=yes
  -o IdentitiesOnly=yes
  -i "${identity}"
)
remote="${user}@${host}"
incoming="${remote_root}/incoming/${release_id}"
release="${remote_root}/releases/${release_id}"
manifest=$(mktemp)

(
  cd "${asset_root}"
  find . -type f \
    ! -path './.git/*' \
    ! -path '*/__pycache__/*' \
    ! -name '*.pyc' \
    ! -name '.manyforge_release_manifest.sha256' \
    -print0 \
    | sort -z \
    | xargs -0 sha256sum >"${manifest}"
)

ssh "${ssh_opts[@]}" "${remote}" \
  "test ! -e '${incoming}' && test ! -e '${release}' && mkdir -p '${incoming}' '${remote_root}/releases'"

cleanup() {
  rm -f "${manifest}"
  ssh "${ssh_opts[@]}" "${remote}" "rm -rf '${incoming}'" >/dev/null 2>&1 || true
}
trap cleanup EXIT

rsync -a --delete --delay-updates \
  --exclude=.git/ \
  --exclude=__pycache__/ \
  --exclude='*.pyc' \
  -e "ssh -o BatchMode=yes -o IdentitiesOnly=yes -i ${identity}" \
  "${asset_root}/" "${remote}:${incoming}/"

rsync -a \
  -e "ssh -o BatchMode=yes -o IdentitiesOnly=yes -i ${identity}" \
  "${manifest}" "${remote}:${incoming}/.manyforge_release_manifest.sha256"

ssh "${ssh_opts[@]}" "${remote}" \
  "cd '${incoming}' && sha256sum --check --strict .manyforge_release_manifest.sha256 >/dev/null && mv '${incoming}' '${release}'"
rm -f "${manifest}"
trap - EXIT

printf '%s\n' "${release}"
