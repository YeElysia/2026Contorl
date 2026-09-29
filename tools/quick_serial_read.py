#!/usr/bin/env python3
"""快速串口读取工具 - 用于临时调试"""
import sys
import time

try:
    import serial
except ImportError:
    print("错误: 缺少pyserial库")
    print("请运行: pip3 install pyserial")
    sys.exit(1)

if len(sys.argv) < 2:
    print("用法: python3 quick_serial_read.py <串口设备>")
    sys.exit(1)

port = sys.argv[1]
baudrate = 115200

try:
    ser = serial.Serial(port, baudrate, timeout=0.1)
    print(f"已连接到 {port} @ {baudrate}")
    print("=" * 60)
    
    while True:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            try:
                text = data.decode('utf-8', errors='replace')
                print(text, end='', flush=True)
            except Exception as e:
                print(f"\n[解码错误: {e}]")
        time.sleep(0.01)
        
except KeyboardInterrupt:
    print("\n\n程序已停止")
except Exception as e:
    print(f"错误: {e}")
finally:
    if 'ser' in locals():
        ser.close()
