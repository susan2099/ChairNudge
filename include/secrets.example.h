// Copy this file to secrets.h and fill in your values.
// secrets.h is in .gitignore, so it never gets committed.
#pragma once

#define WIFI_SSID     "your-wifi-or-hotspot-name"
#define WIFI_PASSWORD "your-wifi-password"

// Grafana Cloud: My Account -> stack Details -> Prometheus card -> Details
// Remote Write Endpoint with /api/prom/push replaced by /api/v1/push/influx/write
#define GRAFANA_URL   "https://prometheus-prod-XX-prod-us-west-0.grafana.net/api/v1/push/influx/write"
#define GRAFANA_USER  "1234567"        // the Username / Instance ID number
#define GRAFANA_TOKEN "glc_..."        // access policy token with metrics:write

#define DEVICE_ID     "chair01"
