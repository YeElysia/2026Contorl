#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
串口日志记录工具
自动读取串口数据并保存到带时间戳的日志文件
支持实时显示、关键词高亮、数据统计
"""

import serial
import serial.tools.list_ports
import argparse
import sys
import os
from datetime import datetime
import re
import signal

class SerialLogger:
    def __init__(self, port, baudrate, output_dir="logs"):
        self.port = port
        self.baudrate = baudrate
        self.output_dir = output_dir
        self.serial_conn = None
        self.log_file = None
        self.running = True
        
        # 统计数据
        self.stats = {
            'total_lines': 0,
            'errors': 0,
            'warnings': 0,
            'faults': 0,
            'movements': 0,
            'alignments': 0,
        }
        
        # ANSI颜色码
        self.colors = {
            'reset': '\033[0m',
            'red': '\033[91m',
            'green': '\033[92m',
            'yellow': '\033[93m',
            'blue': '\033[94m',
            'magenta': '\033[95m',
            'cyan': '\033[96m',
            'white': '\033[97m',
            'bold': '\033[1m',
        }
        
        # 关键词匹配规则
        self.patterns = {
            'error': (r'(?i)(error|fail|fault)', 'red'),
            'warning': (r'(?i)(warning|caution|⚠️)', 'yellow'),
            'success': (r'(?i)(success|complete|✅|done)', 'green'),
            'movement': (r'(?i)(move|rotate|translate|chassis)', 'cyan'),
            'vision': (r'(?i)(vision|camera|align|maix|ring)', 'magenta'),
            'state': (r'(?i)(state|status|pose|position)', 'blue'),
        }
    
    def list_ports(self):
        """列出所有可用串口"""
        ports = serial.tools.list_ports.comports()
        if not ports:
            print("未找到可用串口")
            return []
        
        print("\n可用串口列表：")
        print("-" * 60)
        for i, port in enumerate(ports):
            print(f"{i+1}. {port.device}")
            print(f"   描述: {port.description}")
            print(f"   硬件ID: {port.hwid}")
            print()
        return [p.device for p in ports]
    
    def connect(self):
        """连接串口"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            print(f"{self.colors['green']}✓ 已连接到 {self.port} @ {self.baudrate} baud{self.colors['reset']}")
            return True
        except serial.SerialException as e:
            print(f"{self.colors['red']}✗ 串口连接失败: {e}{self.colors['reset']}")
            return False
    
    def create_log_file(self):
        """创建日志文件"""
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_filename = os.path.join(self.output_dir, f"serial_log_{timestamp}.txt")
        
        try:
            self.log_file = open(log_filename, 'w', encoding='utf-8')
            header = f"""
{'='*80}
串口日志记录
{'='*80}
时间: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
串口: {self.port}
波特率: {self.baudrate}
{'='*80}

"""
            self.log_file.write(header)
            self.log_file.flush()
            print(f"{self.colors['green']}✓ 日志文件: {log_filename}{self.colors['reset']}")
            return True
        except Exception as e:
            print(f"{self.colors['red']}✗ 创建日志文件失败: {e}{self.colors['reset']}")
            return False
    
    def colorize_line(self, line):
        """为行添加颜色（仅终端显示）"""
        colored_line = line
        for pattern_name, (pattern, color) in self.patterns.items():
            if re.search(pattern, line):
                colored_line = f"{self.colors[color]}{line}{self.colors['reset']}"
                break
        return colored_line
    
    def update_stats(self, line):
        """更新统计数据"""
        self.stats['total_lines'] += 1
        
        line_lower = line.lower()
        if 'error' in line_lower or 'fault' in line_lower:
            self.stats['errors'] += 1
        if 'warning' in line_lower or '⚠️' in line:
            self.stats['warnings'] += 1
        if 'move' in line_lower or 'rotate' in line_lower:
            self.stats['movements'] += 1
        if 'align' in line_lower or 'vision' in line_lower:
            self.stats['alignments'] += 1
    
    def print_stats(self):
        """打印统计信息"""
        print(f"\n{self.colors['bold']}{'='*60}{self.colors['reset']}")
        print(f"{self.colors['bold']}统计信息{self.colors['reset']}")
        print(f"{self.colors['bold']}{'='*60}{self.colors['reset']}")
        print(f"总行数: {self.stats['total_lines']}")
        print(f"{self.colors['red']}错误: {self.stats['errors']}{self.colors['reset']}")
        print(f"{self.colors['yellow']}警告: {self.stats['warnings']}{self.colors['reset']}")
        print(f"{self.colors['cyan']}运动: {self.stats['movements']}{self.colors['reset']}")
        print(f"{self.colors['magenta']}视觉: {self.stats['alignments']}{self.colors['reset']}")
        print(f"{self.colors['bold']}{'='*60}{self.colors['reset']}\n")
    
    def run(self):
        """主循环"""
        if not self.connect():
            return
        
        if not self.create_log_file():
            return
        
        print(f"\n{self.colors['green']}开始记录日志... (按 Ctrl+C 停止){self.colors['reset']}\n")
        print(f"{self.colors['bold']}{'='*80}{self.colors['reset']}")
        
        try:
            while self.running:
                if self.serial_conn.in_waiting > 0:
                    try:
                        # 读取一行数据
                        raw_data = self.serial_conn.readline()
                        line = raw_data.decode('utf-8', errors='ignore').rstrip()
                        
                        if line:
                            # 添加时间戳
                            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                            log_line = f"[{timestamp}] {line}"
                            
                            # 写入日志文件
                            self.log_file.write(log_line + '\n')
                            self.log_file.flush()
                            
                            # 终端显示（带颜色）
                            colored_line = self.colorize_line(line)
                            print(f"[{self.colors['white']}{timestamp}{self.colors['reset']}] {colored_line}")
                            
                            # 更新统计
                            self.update_stats(line)
                    
                    except UnicodeDecodeError:
                        print(f"{self.colors['yellow']}[警告] 无法解码数据{self.colors['reset']}")
                        continue
        
        except KeyboardInterrupt:
            print(f"\n\n{self.colors['yellow']}收到中断信号，正在停止...{self.colors['reset']}")
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        self.running = False
        
        if self.log_file:
            self.log_file.write(f"\n{'='*80}\n")
            self.log_file.write(f"日志结束时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            self.log_file.write(f"{'='*80}\n")
            self.log_file.close()
            print(f"{self.colors['green']}✓ 日志文件已保存{self.colors['reset']}")
        
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print(f"{self.colors['green']}✓ 串口已关闭{self.colors['reset']}")
        
        self.print_stats()


def main():
    parser = argparse.ArgumentParser(
        description='串口日志记录工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python serial_logger.py --list                    # 列出所有可用串口
  python serial_logger.py -p /dev/cu.usbserial-1    # macOS
  python serial_logger.py -p COM3                   # Windows
  python serial_logger.py -p /dev/ttyUSB0           # Linux
  python serial_logger.py -p COM3 -b 9600           # 自定义波特率
  python serial_logger.py -p COM3 -o my_logs        # 自定义输出目录
        """
    )
    
    parser.add_argument('-p', '--port', type=str, help='串口名称 (如 COM3 或 /dev/ttyUSB0)')
    parser.add_argument('-b', '--baudrate', type=int, default=115200, help='波特率 (默认: 115200)')
    parser.add_argument('-o', '--output', type=str, default='logs', help='日志输出目录 (默认: logs)')
    parser.add_argument('--list', action='store_true', help='列出所有可用串口')
    
    args = parser.parse_args()
    
    logger = SerialLogger('', 115200, args.output)
    
    # 列出串口
    if args.list or not args.port:
        ports = logger.list_ports()
        if not args.port and ports:
            try:
                choice = input("\n请选择串口编号 (或按 Enter 退出): ").strip()
                if choice.isdigit() and 1 <= int(choice) <= len(ports):
                    args.port = ports[int(choice) - 1]
                else:
                    print("已取消")
                    return
            except (KeyboardInterrupt, EOFError):
                print("\n已取消")
                return
        elif not ports:
            return
    
    if not args.port:
        parser.print_help()
        return
    
    # 创建日志记录器并运行
    logger = SerialLogger(args.port, args.baudrate, args.output)
    
    # 设置信号处理
    signal.signal(signal.SIGINT, lambda s, f: logger.cleanup())
    
    logger.run()


if __name__ == '__main__':
    main()
