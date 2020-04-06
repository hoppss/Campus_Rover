# PEV Networking
## tcp_server.py
Let PEV as server, our APP will be client. This configuration only use for local network.

## tcp_client.py
This is TCP client setting and base on JSON RPC 2.0. In this configuration, both PEV and APP have to connect to the server (ie. AWS or server in Taipei Tech).

## tcp_client_v2.py - Currently for Demo
This setting is base on original design (no Json RPC 2.0). Both PEV and app connect to server.
For running demo: ```rosrun pev_network tcp_client_v2.py```

AWS Server IP: 18.188.72.131
