#!/usr/bin/env python3
import subprocess
import time
import webbrowser
import sys

def main():
    # Clean up DDS environment variables to ensure container loads standard multicast settings
    import os
    env = os.environ.copy()
    env.pop('CYCLONEDDS_URI', None)
    env.pop('RMW_IMPLEMENTATION', None)

    # Grant local container access to host X server for GUI applications (like RViz)
    try:
        subprocess.run(["xhost", "+local:root"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass

    # Start container in detached mode
    subprocess.run(["docker", "compose", "up", "-d"], env=env)
    
    print("Waiting for web server to initialize...")
    time.sleep(2.0)
    
    print("Opening HMI dashboard in default web browser...")
    webbrowser.open("http://localhost:8000")
    
    print("\n--- Streaming Workspace Logs (Press Ctrl+C to exit logs, container will keep running) ---")
    try:
        # Stream the logs to the terminal
        subprocess.run(["docker", "compose", "logs", "-f"])
    except KeyboardInterrupt:
        print("\nExited log stream. Container is still running. Run 'docker compose down' to stop it.")

if __name__ == "__main__":
    main()
