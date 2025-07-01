#!/bin/bash

echo "=========================="
echo "Starting App NissanRadar for {APP_PRETTY_NAME}"


systemctl start NissanRadar
systemctl start rosnodeChecker
