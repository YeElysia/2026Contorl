#!/usr/bin/env python3
"""一次性读取串口数据 - 用于快速查看输出"""
import sys
import time

try:
    import serial
except ImportError:
    print("需要安装pyserial: pip3 install --user pyserial")
    sys.exit(1)

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem5078740666043"
duration = int(sys.argv[2]) if len(sys.argv) > 2 else 5

try:
    ser = serial.Serial(port, 115200, timeout=0.1)
    print(f"读取 {port} 共 {duration} 秒...")
    print("=" * 60)
    
    start = time.time()
    while time.time() - start < duration:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            text = data.decode('utf-8', errors='replace')
            print(text, end='', flush=True)
        time.sleep(0.01)
    
    print("\n" + "=" * 60)
    print("读取完成")
    ser.close()
    
except Exception as e:
    print(f"错误: {e}")
    sys.exit(1)
