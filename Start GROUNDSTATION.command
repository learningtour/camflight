#!/bin/zsh
# Double-click this file: starts the GROUNDSTATION server and opens the browser.
# If the server is already running, it just opens the browser.
cd "$(dirname "$0")"
PORT=8765

if nc -z 127.0.0.1 $PORT >/dev/null 2>&1; then
  open "http://localhost:$PORT"
  echo "GROUNDSTATION was already running — browser opened."
  exit 0
fi

# open the browser as soon as the server responds
(
  for i in {1..80}; do
    if nc -z 127.0.0.1 $PORT >/dev/null 2>&1; then
      open "http://localhost:$PORT"
      exit 0
    fi
    sleep 0.25
  done
) &

exec python3 server.py $PORT
