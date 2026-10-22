#!/usr/bin/env python3
"""LAN-restricted UDP plus same-host SHM; preserve automatic data locators.

An explicit UDP-only defaultUnicastLocatorList suppresses automatic SHM data
locators. Discovery stays unicast to Main; UDP interfaces remain whitelisted.
"""
import argparse
import os
from pathlib import Path
import tempfile

PEERS = tuple(f'192.168.3.{n}' for n in (161, 162, 163, 164, 166))


def profile_xml(address, server=False):
    if address not in PEERS or (server and address != PEERS[-1]):
        raise ValueError('DDS requires an Edge .161-.164 or the main server .166')
    discovery = ('<discovery_config><discoveryProtocol>SERVER</discoveryProtocol></discovery_config>' if server else '')
    port = '<port>11811</port>' if server else ''
    guid = '<prefix>44.53.00.5f.45.50.52.4f.53.49.4d.41</prefix>' if server else ''
    return f'''<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>sirius_lan</transport_id><type>UDPv4</type>
        <maxMessageSize>1400</maxMessageSize>
        <sendBufferSize>16777216</sendBufferSize>
        <receiveBufferSize>16777216</receiveBufferSize>
        <interfaceWhiteList><address>{address}</address></interfaceWhiteList>
      </transport_descriptor>
      <transport_descriptor>
        <transport_id>sirius_local_shm</transport_id><type>SHM</type>
        <maxMessageSize>8388608</maxMessageSize>
        <segment_size>134217728</segment_size>
        <port_queue_capacity>8192</port_queue_capacity>
      </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="sirius_lan" is_default_profile="true">
      <rtps>
        {guid}
        <builtin>
          {discovery}
          <metatrafficUnicastLocatorList><locator><udpv4><address>{address}</address>{port}</udpv4></locator></metatrafficUnicastLocatorList>
          <initialPeersList><locator><udpv4><address>192.168.3.166</address><port>11811</port></udpv4></locator></initialPeersList>
        </builtin>
        <userTransports><transport_id>sirius_lan</transport_id><transport_id>sirius_local_shm</transport_id></userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
'''


def write_profile(address, destination, server=False):
    content = profile_xml(address, server)
    path = Path(destination).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as output:
        temporary = Path(output.name)
        output.write(content)
    try:
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)
    return path


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ip', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--server', action='store_true')
    args = parser.parse_args()
    print(write_profile(args.ip, args.output, args.server))
