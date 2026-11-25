#!/usr/bin/env bash
set -euo pipefail
A6Y_VR_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# -ne 1 ]]; then echo '用法: scripts/create_vr_certificate.bash <主机局域网IPv4地址>' >&2; exit 2; fi
A6Y_VR_IP="$(/usr/bin/python3 -c 'import ipaddress,sys; print(ipaddress.IPv4Address(sys.argv[1]))' "$1")"
A6Y_VR_TLS="$A6Y_VR_ROOT/validation/vr_rviz/tls"
mkdir -p "$A6Y_VR_TLS"
if [[ -e "$A6Y_VR_TLS/key.pem" || -e "$A6Y_VR_TLS/cert.pem" ]]; then
  echo '证书已存在；请先另存旧文件再生成新证书，避免覆盖。' >&2; exit 2
fi
umask 077
openssl req -x509 -newkey rsa:2048 -sha256 -days 365 -nodes \
  -keyout "$A6Y_VR_TLS/key.pem" -out "$A6Y_VR_TLS/cert.pem" \
  -subj '/CN=A5 VR Local Development' \
  -addext "subjectAltName=IP:$A6Y_VR_IP,IP:127.0.0.1,DNS:localhost"
echo "已生成本地开发证书：$A6Y_VR_TLS/cert.pem"
