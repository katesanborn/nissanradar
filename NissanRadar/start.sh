#!/bin/bash

echo "=========================="
echo "Starting App nissanradar for {APP_PRETTY_NAME}"


systemctl start nissanradar
systemctl start rosnodeChecker
