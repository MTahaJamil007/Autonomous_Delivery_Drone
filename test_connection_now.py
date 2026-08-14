#!/usr/bin/env python3
"""
Quick MAVSDK connection test to verify correct port
"""

import asyncio
import sys
from mavsdk import System

async def test_connection(port):
    """Test connection to specific port"""
    print(f"Testing connection to udpin://0.0.0.0:{port}...")
    
    drone = System()
    await drone.connect(system_address=f"udpin://0.0.0.0:{port}")
    
    # Wait up to 5 seconds for connection
    connected = False
    for attempt in range(10):
        try:
            async for state in drone.core.connection_state():
                if state.is_connected:
                    connected = True
                    print(f"✅ Successfully connected on port {port}!")
                    return True
                break
        except:
            pass
        await asyncio.sleep(0.5)
    
    print(f"❌ Connection failed on port {port}")
    return False

async def main():
    print("=" * 60)
    print("MAVSDK Connection Test")
    print("=" * 60)
    print()
    
    # Test both common ports
    ports_to_test = [14540, 14580]
    
    for port in ports_to_test:
        result = await test_connection(port)
        if result:
            print()
            print(f"🎯 PX4 is listening on port {port}")
            print(f"   Update config.py: MAVSDK_PORT_BASE = {port}")
            sys.exit(0)
        print()
    
    print("❌ Could not connect to PX4 on any port!")
    print()
    print("Check:")
    print("  1. PX4 SITL is running")
    print("  2. You see 'INFO [commander] Ready for takeoff!'")
    print("  3. Check PX4 output for 'INFO [mavlink] mode: Onboard' line")
    print("     It shows which port PX4 is listening on")
    sys.exit(1)

if __name__ == "__main__":
    asyncio.run(main())
